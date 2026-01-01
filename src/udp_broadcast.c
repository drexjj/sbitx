#include "udp_broadcast.h"
#include "sdr_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <errno.h>
#include <time.h>

/* WSJT-X UDP Protocol Constants */
#define WSJTX_MAGIC 0xadbccbda
#define WSJTX_SCHEMA 2  /* WSJT-X uses Schema 2, not 3 */

/* Message Types */
#define WSJTX_MSG_HEARTBEAT 0
#define WSJTX_MSG_STATUS 1
#define WSJTX_MSG_DECODE 2

/* Maximum message buffer size */
#define MAX_BUFFER_SIZE 2048

/* Static variables for socket management */
static int broadcast_socket = -1;
static struct sockaddr_in broadcast_addr;
static char last_ip[20] = "";
static int last_port = 0;

/* Unique ID for this application */
static const char *WSJTX_ID = "sBitx";

/* Version information - extract version from VER_STR "sbitx v5.2" */
/* Skip "sbitx " to get just "v5.2" */
static const char *WSJTX_VERSION = VER_STR + 6;
static const char *WSJTX_REVISION = "0b453a3";

/* Buffer for building messages */
static unsigned char msg_buffer[MAX_BUFFER_SIZE];
static int msg_pos = 0;

/**
 * Binary encoding helper functions
 * These implement Qt QDataStream encoding with big-endian byte order
 */

/* Write a 32-bit unsigned integer in big-endian format */
static void encode_quint32(uint32_t value) {
    msg_buffer[msg_pos++] = (value >> 24) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 16) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 8) & 0xFF;
    msg_buffer[msg_pos++] = value & 0xFF;
}

/* Write a 64-bit unsigned integer in big-endian format */
static void encode_quint64(uint64_t value) {
    msg_buffer[msg_pos++] = (value >> 56) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 48) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 40) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 32) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 24) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 16) & 0xFF;
    msg_buffer[msg_pos++] = (value >> 8) & 0xFF;
    msg_buffer[msg_pos++] = value & 0xFF;
}

/* Write a 32-bit signed integer in big-endian format */
static void encode_qint32(int32_t value) {
    encode_quint32((uint32_t)value);
}

/* Write a boolean value */
static void encode_bool(bool value) {
    msg_buffer[msg_pos++] = value ? 1 : 0;
}

/* Write a UTF-8 string with length prefix */
static void encode_utf8(const char *str) {
    if (str == NULL) {
        str = "";
    }
    uint32_t len = strlen(str);
    encode_quint32(len);
    memcpy(&msg_buffer[msg_pos], str, len);
    msg_pos += len;
}

/* Write a QTime value (milliseconds since midnight) */
static void encode_qtime(uint32_t ms) {
    encode_quint32(ms);
}

/* Write a double value (IEEE 754 64-bit, big-endian) */
static void encode_double(double value) {
    union {
        double d;
        uint64_t i;
    } u;
    u.d = value;
    encode_quint64(u.i);
}

/* Write message header (magic, schema, type) */
static void encode_header(uint32_t msg_type) {
    msg_pos = 0;
    encode_quint32(WSJTX_MAGIC);
    encode_quint32(WSJTX_SCHEMA);
    encode_quint32(msg_type);
}

/**
 * Initialize the WSJT-X broadcast socket
 */
int udp_broadcast_init(void) {
    const char *enabled = field_str("UDP_BROADCAST");

    if (enabled == NULL || strcmp(enabled, "ON") != 0) {
        return 0; /* Not enabled, not an error */
    }

    /* Get IP and port - these settings use # prefix so label matches */
    const char *ip = field_str("UDP_IP");
    if (ip == NULL || !ip[0]) {
        ip = "127.0.0.1";
    }

    const char *port_str = field_str("UDP_PORT");
    int port = 2237; /* Default port */
    if (port_str != NULL && port_str[0]) {
        port = atoi(port_str);
    }
    if (port < 1024 || port > 65535) {
        fprintf(stderr, "WSJTX: Invalid port %d, using default 2237\n", port);
        port = 2237;
    }

    /* Check if we need to recreate the socket */
    if (broadcast_socket >= 0) {
        if (strcmp(last_ip, ip) == 0 && last_port == port) {
            return 0; /* Already initialized with same settings */
        }
        /* Settings changed, close old socket */
        close(broadcast_socket);
        broadcast_socket = -1;
    }

    /* Create UDP socket */
    broadcast_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (broadcast_socket < 0) {
        fprintf(stderr, "WSJTX: Failed to create socket: %s\n", strerror(errno));
        return -1;
    }

    /* Set non-blocking mode */
    int flags = fcntl(broadcast_socket, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(broadcast_socket, F_SETFL, flags | O_NONBLOCK);
    }

    /* Configure destination address */
    memset(&broadcast_addr, 0, sizeof(broadcast_addr));
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(port);
    if (inet_aton(ip, &broadcast_addr.sin_addr) == 0) {
        fprintf(stderr, "WSJTX: Invalid IP address: %s\n", ip);
        close(broadcast_socket);
        broadcast_socket = -1;
        return -1;
    }

    /* Save settings */
    strncpy(last_ip, ip, sizeof(last_ip) - 1);
    last_ip[sizeof(last_ip) - 1] = '\0';
    last_port = port;

    return 0;
}

/**
 * Close the WSJT-X broadcast socket
 */
void udp_broadcast_close(void) {
    if (broadcast_socket >= 0) {
        close(broadcast_socket);
        broadcast_socket = -1;
    }
}

/**
 * Send the current message buffer via UDP
 */
static int send_message(void) {
    if (broadcast_socket < 0) {
        if (udp_broadcast_init() < 0) {
            return -1;
        }
    }

    const char *enabled = field_str("UDP_BROADCAST");
    if (enabled == NULL || strcmp(enabled, "ON") != 0) {
        return 0; /* Not enabled */
    }

    ssize_t sent = sendto(broadcast_socket, msg_buffer, msg_pos, 0,
                          (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

    if (sent < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            fprintf(stderr, "WSJTX: Failed to send message: %s\n", strerror(errno));
            return -1;
        }
    }

    return 0;
}

/**
 * Send a Heartbeat message (Type 0)
 */
int udp_broadcast_heartbeat(void) {
    encode_header(WSJTX_MSG_HEARTBEAT);
    encode_utf8(WSJTX_ID);
    encode_quint32(WSJTX_SCHEMA);
    encode_utf8(WSJTX_VERSION);
    encode_utf8(WSJTX_REVISION);

    return send_message();
}

/**
 * Send a Status message (Type 1)
 */
int udp_broadcast_status(
    uint64_t frequency,
    const char *mode,
    const char *dx_call,
    const char *report,
    bool tx_enabled,
    bool transmitting,
    bool decoding,
    uint32_t rx_df,
    uint32_t tx_df,
    const char *de_call,
    const char *de_grid,
    const char *dx_grid)
{
    encode_header(WSJTX_MSG_STATUS);

    /* Id (unique key) */
    encode_utf8(WSJTX_ID);

    /* Dial Frequency (Hz) */
    encode_quint64(frequency);

    /* Mode */
    encode_utf8(mode);

    /* DX call */
    encode_utf8(dx_call);

    /* Report */
    encode_utf8(report);

    /* Tx Mode */
    encode_utf8(mode);

    /* Tx Enabled */
    encode_bool(tx_enabled);

    /* Transmitting */
    encode_bool(transmitting);

    /* Decoding */
    encode_bool(decoding);

    /* Rx DF */
    encode_quint32(rx_df);

    /* Tx DF */
    encode_quint32(tx_df);

    /* DE call */
    encode_utf8(de_call);

    /* DE grid */
    encode_utf8(de_grid);

    /* DX grid */
    encode_utf8(dx_grid);

    /* Tx Watchdog */
    encode_bool(false);

    /* Sub-mode */
    encode_utf8("");

    /* Fast mode */
    encode_bool(false);

    /* Special Operation Mode (0 = NONE) */
    msg_buffer[msg_pos++] = 0;

    /* Frequency Tolerance */
    encode_quint32(20);

    /* T/R Period */
    uint32_t tr_period = (strcmp(mode, "FT4") == 0) ? 7 : 15;
    encode_quint32(tr_period);

    /* Configuration Name */
    encode_utf8("");

    /* Tx Message */
    encode_utf8("");

    return send_message();
}

/**
 * Send a Decode message (Type 2)
 */
int udp_broadcast_decode(
    uint32_t time_ms,
    int32_t snr,
    double delta_time,
    uint32_t delta_freq,
    const char *mode,
    const char *message,
    bool low_confidence,
    bool off_air)
{
    encode_header(WSJTX_MSG_DECODE);

    /* Id (unique key) */
    encode_utf8(WSJTX_ID);

    /* New */
    encode_bool(true);

    /* Time (QTime - milliseconds since midnight) */
    encode_qtime(time_ms);

    /* SNR */
    encode_qint32(snr);

    /* Delta time (seconds) */
    encode_double(delta_time);

    /* Delta frequency (Hz) */
    encode_quint32(delta_freq);

    /* Mode */
    encode_utf8(mode);

    /* Message */
    encode_utf8(message);

    /* Low confidence */
    encode_bool(low_confidence);

    /* Off air */
    encode_bool(off_air);

    return send_message();
}

/**
 * Helper: Convert HH:MM:SS timestamp to milliseconds since midnight
 */
uint32_t udp_timestamp_to_ms(const char *timestamp) {
    if (timestamp == NULL) {
        return 0;
    }

    int hours = 0, minutes = 0, seconds = 0;

    /* Try HH:MM:SS format */
    if (sscanf(timestamp, "%d:%d:%d", &hours, &minutes, &seconds) == 3) {
        return (hours * 3600 + minutes * 60 + seconds) * 1000;
    }

    /* Try HHMMSS format */
    if (sscanf(timestamp, "%2d%2d%2d", &hours, &minutes, &seconds) == 3) {
        return (hours * 3600 + minutes * 60 + seconds) * 1000;
    }

    /* Fall back to current time */
    time_t now = time(NULL);
    struct tm *tm_info = gmtime(&now);
    return (tm_info->tm_hour * 3600 + tm_info->tm_min * 60 + tm_info->tm_sec) * 1000;
}

/**
 * Send a Status message using current radio state from fields
 */
int udp_broadcast_status_auto(void) {
    /* Check if broadcasting is enabled */
    const char *enabled = field_str("UDP_BROADCAST");
    if (enabled == NULL || strcmp(enabled, "ON") != 0) {
        return 0;
    }

    /* Get frequency - field_str() looks up by LABEL not cmd */
    const char *freq_str = field_str("FREQ");  /* Label for r1:freq */
    uint64_t frequency = 14074000; /* Default to 20m FT8 */
    if (freq_str != NULL && freq_str[0]) {
        frequency = (uint64_t)atol(freq_str);
    }

    /* Get mode */
    const char *mode = field_str("MODE");  /* Label for r1:mode */
    if (mode == NULL || (strcmp(mode, "FT8") != 0 && strcmp(mode, "FT4") != 0)) {
        mode = "FT8"; /* Default */
    }

    /* Get callsigns and grids - use LABEL not cmd */
    const char *de_call = field_str("MYCALLSIGN");  /* Label for #mycallsign */
    if (de_call == NULL || !de_call[0]) {
        de_call = "N0CALL";
    }

    const char *de_grid = field_str("MYGRID");  /* Label for #mygrid */
    if (de_grid == NULL) {
        de_grid = "";
    }

    const char *dx_call = field_str("CALL");  /* Label for #contact_callsign */
    if (dx_call == NULL) {
        dx_call = "";
    }

    const char *dx_grid = field_str("EXCH");  /* Label for #exchange_received */
    if (dx_grid == NULL) {
        dx_grid = "";
    }

    const char *report = field_str("SENT");  /* Label for #rst_sent */
    if (report == NULL) {
        report = "";
    }

    /* Get TX/RX state */
    const char *tx_state = field_str("TX");  /* Need to find correct label */
    bool transmitting = (tx_state != NULL && strcmp(tx_state, "ON") == 0);
    bool tx_enabled = true; /* Assume TX is always enabled */
    bool decoding = !transmitting; /* If not transmitting, we're decoding */

    /* Get frequency offsets */
    const char *rx_df_str = field_str("FTX_RX_PITCH");  /* Label for ftx_rx_pitch */
    uint32_t rx_df = (rx_df_str && rx_df_str[0]) ? (uint32_t)atoi(rx_df_str) : 0;
    uint32_t tx_df = rx_df; /* TX pitch = RX pitch (no separate TX pitch field) */

    return udp_broadcast_status(frequency, mode, dx_call, report,
                                 tx_enabled, transmitting, decoding,
                                 rx_df, tx_df, de_call, de_grid, dx_grid);
}
