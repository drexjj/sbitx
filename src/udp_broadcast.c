#include "udp_broadcast.h"
#include "sdr_ui.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
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

/* Multi-destination support */
#define MAX_DESTINATIONS 10
#define MAX_DEST_STRING 256

/* Destination structure */
struct destination {
    char host[256];  /* IP address or hostname */
    int port;
    int socket;
    struct sockaddr_in addr;
    int initialized;
};

/* Static variables for socket management */
static struct destination destinations[MAX_DESTINATIONS];
static int num_destinations = 0;
static char last_destinations_str[MAX_DEST_STRING] = "";

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
 * Parse semicolon-delimited destination string
 * Format: "IP:port;IP:port;..."
 * Returns: Number of valid destinations parsed
 */
static int parse_destinations(const char *dest_str, struct destination *dests, int max_dests) {
    if (dest_str == NULL || dest_str[0] == '\0') {
        return 0;
    }

    char buffer[MAX_DEST_STRING];
    strncpy(buffer, dest_str, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    int count = 0;
    char *saveptr;
    char *token = strtok_r(buffer, ";", &saveptr);

    while (token != NULL && count < max_dests) {
        /* Trim leading whitespace */
        while (*token == ' ' || *token == '\t') {
            token++;
        }

        /* Skip empty tokens */
        if (*token == '\0') {
            token = strtok_r(NULL, ";", &saveptr);
            continue;
        }

        /* Find colon separator */
        char *colon = strchr(token, ':');
        if (colon == NULL) {
            fprintf(stderr, "UDP: Malformed destination (no colon): %s\n", token);
            token = strtok_r(NULL, ";", &saveptr);
            continue;
        }

        /* Split IP and port */
        *colon = '\0';
        char *ip = token;
        char *port_str = colon + 1;

        /* Trim trailing whitespace from IP */
        char *end = colon - 1;
        while (end > ip && (*end == ' ' || *end == '\t')) {
            *end = '\0';
            end--;
        }

        /* Trim whitespace from port */
        while (*port_str == ' ' || *port_str == '\t') {
            port_str++;
        }

        /* Validate hostname/IP is not empty */
        if (ip[0] == '\0') {
            fprintf(stderr, "UDP: Empty hostname/IP in destination\n");
            token = strtok_r(NULL, ";", &saveptr);
            continue;
        }

        /* Validate port */
        int port = atoi(port_str);
        if (port < 1024 || port > 65535) {
            fprintf(stderr, "UDP: Invalid port %d (must be 1024-65535): %s:%s\n", port, ip, port_str);
            token = strtok_r(NULL, ";", &saveptr);
            continue;
        }

        /* Store parsed destination (hostname or IP) */
        strncpy(dests[count].host, ip, sizeof(dests[count].host) - 1);
        dests[count].host[sizeof(dests[count].host) - 1] = '\0';
        dests[count].port = port;
        dests[count].socket = -1;
        dests[count].initialized = 0;
        count++;

        token = strtok_r(NULL, ";", &saveptr);
    }

    if (count >= max_dests && token != NULL) {
        fprintf(stderr, "UDP: Warning - maximum %d destinations supported, ignoring extras\n", max_dests);
    }

    return count;
}

/**
 * Initialize a single destination socket
 */
static int init_destination(struct destination *dest) {
    /* Resolve hostname/IP to address */
    struct addrinfo hints, *result, *rp;
    char port_str[8];

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;        /* IPv4 */
    hints.ai_socktype = SOCK_DGRAM;   /* UDP */
    hints.ai_flags = 0;
    hints.ai_protocol = IPPROTO_UDP;

    snprintf(port_str, sizeof(port_str), "%d", dest->port);

    int gai_error = getaddrinfo(dest->host, port_str, &hints, &result);
    if (gai_error != 0) {
        fprintf(stderr, "UDP: Failed to resolve %s:%d: %s\n",
                dest->host, dest->port, gai_strerror(gai_error));
        return -1;
    }

    /* Try each address until we successfully create a socket */
    int sock = -1;
    for (rp = result; rp != NULL; rp = rp->ai_next) {
        sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
        if (sock >= 0) {
            /* Copy the resolved address */
            memcpy(&dest->addr, rp->ai_addr, sizeof(dest->addr));
            break;
        }
    }

    freeaddrinfo(result);

    if (sock < 0) {
        fprintf(stderr, "UDP: Failed to create socket for %s:%d: %s\n",
                dest->host, dest->port, strerror(errno));
        return -1;
    }

    dest->socket = sock;

    /* Set non-blocking mode */
    int flags = fcntl(dest->socket, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(dest->socket, F_SETFL, flags | O_NONBLOCK);
    }

    dest->initialized = 1;
    return 0;
}

/**
 * Close all destination sockets
 */
static void close_all_destinations(void) {
    for (int i = 0; i < num_destinations; i++) {
        if (destinations[i].socket >= 0) {
            close(destinations[i].socket);
            destinations[i].socket = -1;
        }
        destinations[i].initialized = 0;
    }
    num_destinations = 0;
}

/**
 * Initialize the WSJT-X broadcast socket(s)
 */
int udp_broadcast_init(void) {
    const char *enabled = field_str("UDP_BROADCAST");

    if (enabled == NULL || strcmp(enabled, "ON") != 0) {
        return 0; /* Not enabled, not an error */
    }

    /* Get destinations string */
    const char *dest_str = field_str("UDP_DESTINATIONS");
    if (dest_str == NULL || dest_str[0] == '\0') {
        dest_str = "127.0.0.1:2237"; /* Default */
    }

    /* Check if destinations changed */
    if (strcmp(last_destinations_str, dest_str) == 0 && num_destinations > 0) {
        return 0; /* Already initialized with same settings */
    }

    /* Close all old sockets */
    close_all_destinations();

    /* Parse new destinations */
    num_destinations = parse_destinations(dest_str, destinations, MAX_DESTINATIONS);

    if (num_destinations == 0) {
        fprintf(stderr, "UDP: No valid destinations, trying default 127.0.0.1:2237\n");
        /* Try default as fallback */
        num_destinations = parse_destinations("127.0.0.1:2237", destinations, MAX_DESTINATIONS);
        if (num_destinations == 0) {
            return -1;
        }
    }

    /* Initialize each destination socket */
    int success_count = 0;
    for (int i = 0; i < num_destinations; i++) {
        if (init_destination(&destinations[i]) == 0) {
            success_count++;
        }
    }

    if (success_count == 0) {
        fprintf(stderr, "UDP: Failed to initialize any destinations\n");
        return -1;
    }

    /* Save current configuration */
    strncpy(last_destinations_str, dest_str, sizeof(last_destinations_str) - 1);
    last_destinations_str[sizeof(last_destinations_str) - 1] = '\0';

    fprintf(stderr, "UDP: Initialized %d of %d destinations\n", success_count, num_destinations);
    return 0;
}

/**
 * Close the WSJT-X broadcast sockets
 */
void udp_broadcast_close(void) {
    close_all_destinations();
    last_destinations_str[0] = '\0';
}

/**
 * Send the current message buffer via UDP to all destinations
 */
static int send_message(void) {
    /* Initialize if needed */
    if (num_destinations == 0) {
        if (udp_broadcast_init() < 0) {
            return -1;
        }
    }

    /* Check if still enabled */
    const char *enabled = field_str("UDP_BROADCAST");
    if (enabled == NULL || strcmp(enabled, "ON") != 0) {
        return 0; /* Not enabled */
    }

    int success_count = 0;
    int error_count = 0;

    /* Send to all destinations */
    for (int i = 0; i < num_destinations; i++) {
        if (!destinations[i].initialized || destinations[i].socket < 0) {
            continue;
        }

        ssize_t sent = sendto(destinations[i].socket, msg_buffer, msg_pos, 0,
                             (struct sockaddr *)&destinations[i].addr,
                             sizeof(destinations[i].addr));

        if (sent < 0) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                fprintf(stderr, "UDP: Failed to send to %s:%d: %s\n",
                       destinations[i].host, destinations[i].port, strerror(errno));
                error_count++;
            }
        } else {
            success_count++;
        }
    }

    /* Return success if at least one destination succeeded */
    return (success_count > 0) ? 0 : -1;
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
