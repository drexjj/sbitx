#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <errno.h>
#include <fcntl.h>
#include <complex.h>
#include <fftw3.h>
#include <sqlite3.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "logbook.h"
#include "adif_broadcast.h"

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
			printf("ADIF: Malformed destination (no colon): %s\n", token);
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
			printf("ADIF: Empty hostname/IP in destination\n");
			token = strtok_r(NULL, ";", &saveptr);
			continue;
		}

		/* Validate port */
		int port = atoi(port_str);
		if (port < 1024 || port > 65535) {
			printf("ADIF: Invalid port %d (must be 1024-65535): %s:%s\n", port, ip, port_str);
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
		printf("ADIF: Warning - maximum %d destinations supported, ignoring extras\n", max_dests);
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
		printf("ADIF: Failed to resolve %s:%d: %s\n",
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
		printf("ADIF: Failed to create socket for %s:%d: %s\n",
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
 * Initialize the ADIF broadcast socket(s)
 */
int adif_broadcast_init(void) {
	const char *enabled = field_str("ADIF_BROADCAST");

	if (enabled == NULL || strcmp(enabled, "ON") != 0) {
		return 0; /* Not enabled, not an error */
	}

	/* Get destinations string */
	const char *dest_str = field_str("ADIF_DESTINATIONS");
	if (dest_str == NULL || dest_str[0] == '\0') {
		dest_str = "127.0.0.1:12060"; /* Default */
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
		printf("ADIF: No valid destinations, trying default 127.0.0.1:12060\n");
		/* Try default as fallback */
		num_destinations = parse_destinations("127.0.0.1:12060", destinations, MAX_DESTINATIONS);
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
		printf("ADIF: Failed to initialize any destinations\n");
		return -1;
	}

	/* Save current configuration */
	strncpy(last_destinations_str, dest_str, sizeof(last_destinations_str) - 1);
	last_destinations_str[sizeof(last_destinations_str) - 1] = '\0';

	printf("ADIF: Initialized %d of %d destinations\n", success_count, num_destinations);
	return 0;
}

// Format ADIF record for ft8battle.com (lowercase fields, 6-digit time)
static int format_adif_for_broadcast(sqlite3_stmt *stmt, char *buf, int buf_size) {
	// Column indices from query
	// 0=id, 1=mode, 2=freq, 3=qso_date, 4=qso_time, 5=callsign_sent, 6=rst_sent, 7=exch_sent,
	// 8=callsign_recv, 9=rst_recv, 10=exch_recv, 11=tx_id, 12=comments, 13=tx_power

	const char *mode = (const char*)sqlite3_column_text(stmt, 1);
	const char *freq_str = (const char*)sqlite3_column_text(stmt, 2);
	const char *qso_date = (const char*)sqlite3_column_text(stmt, 3);
	const char *qso_time = (const char*)sqlite3_column_text(stmt, 4);
	const char *my_call = (const char*)sqlite3_column_text(stmt, 5);
	const char *rst_sent = (const char*)sqlite3_column_text(stmt, 6);
	const char *my_grid = (const char*)sqlite3_column_text(stmt, 7);
	const char *their_call = (const char*)sqlite3_column_text(stmt, 8);
	const char *rst_rcvd = (const char*)sqlite3_column_text(stmt, 9);
	const char *their_grid = (const char*)sqlite3_column_text(stmt, 10);
	const char *comments = (const char*)sqlite3_column_text(stmt, 12);
	const char *tx_power = (const char*)sqlite3_column_text(stmt, 13);

	// Convert frequency from Hz to MHz
	long freq_hz = atol(freq_str ? freq_str : "0");
	float freq_mhz = freq_hz / 1000000.0;

	// Determine band from frequency
	const char *band = "?";
	long freq_khz = freq_hz / 1000;
	if (freq_khz >= 28000 && freq_khz <= 29700) band = "10m";
	else if (freq_khz >= 24890 && freq_khz <= 24990) band = "12m";
	else if (freq_khz >= 21000 && freq_khz <= 21450) band = "15m";
	else if (freq_khz >= 18068 && freq_khz <= 18168) band = "17m";
	else if (freq_khz >= 14000 && freq_khz <= 14350) band = "20m";
	else if (freq_khz >= 10100 && freq_khz <= 10150) band = "30m";
	else if (freq_khz >= 7000 && freq_khz <= 7300) band = "40m";
	else if (freq_khz >= 5351 && freq_khz <= 5367) band = "60m";
	else if (freq_khz >= 3500 && freq_khz <= 4000) band = "80m";
	else if (freq_khz >= 1800 && freq_khz <= 2000) band = "160m";

	// Format date as YYYYMMDD (remove dashes)
	char date_clean[16] = "";
	if (qso_date) {
		const char *p = qso_date;
		char *d = date_clean;
		while (*p) {
			if (*p != '-') *d++ = *p;
			p++;
		}
		*d = '\0';
	}

	// Format time as HHMMSS (add seconds "00")
	char time_full[8] = "";
	if (qso_time) {
		snprintf(time_full, sizeof(time_full), "%s00", qso_time);
	}

	// Determine submode (FT8, FT4, etc.)
	const char *submode = mode;
	const char *mode_for_adif = "MFSK"; // FT8/FT4 are MFSK modes

	// Build ADIF record (lowercase field names)
	int offset = 0;

	// Call
	if (their_call && their_call[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<call:%d>%s ", (int)strlen(their_call), their_call);
	}

	// Their grid
	if (their_grid && their_grid[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<gridsquare:%d>%s ", (int)strlen(their_grid), their_grid);
	} else {
		offset += snprintf(buf + offset, buf_size - offset, "<gridsquare:0> ");
	}

	// Mode and submode
	if (mode && mode[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<mode:%d>%s <submode:%d>%s ",
			(int)strlen(mode_for_adif), mode_for_adif,
			(int)strlen(submode), submode);
	}

	// RST sent/rcvd
	if (rst_sent && rst_sent[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<rst_sent:%d>%s ", (int)strlen(rst_sent), rst_sent);
	}
	if (rst_rcvd && rst_rcvd[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<rst_rcvd:%d>%s ", (int)strlen(rst_rcvd), rst_rcvd);
	}

	// Date and time
	if (date_clean[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<qso_date:%d>%s ", (int)strlen(date_clean), date_clean);
	}
	if (time_full[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<time_on:%d>%s ", (int)strlen(time_full), time_full);
		// Also add time_off (same as time_on typically)
		offset += snprintf(buf + offset, buf_size - offset,
			"<qso_date_off:%d>%s <time_off:%d>%s ",
			(int)strlen(date_clean), date_clean,
			(int)strlen(time_full), time_full);
	}

	// Band and frequency
	offset += snprintf(buf + offset, buf_size - offset,
		"<band:%d>%s ", (int)strlen(band), band);
	char freq_formatted[16];
	snprintf(freq_formatted, sizeof(freq_formatted), "%.6f", freq_mhz);
	offset += snprintf(buf + offset, buf_size - offset,
		"<freq:%d>%s ", (int)strlen(freq_formatted), freq_formatted);

	// Station callsign (my call)
	if (my_call && my_call[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<station_callsign:%d>%s ", (int)strlen(my_call), my_call);
	}

	// My gridsquare
	if (my_grid && my_grid[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<my_gridsquare:%d>%s ", (int)strlen(my_grid), my_grid);
	}

	// TX power
	if (tx_power && tx_power[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<tx_pwr:%d>%s ", (int)strlen(tx_power), tx_power);
	}

	// Comment (optional)
	if (comments && comments[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<comment:%d>%s ", (int)strlen(comments), comments);
	}

	// End of record (lowercase!)
	offset += snprintf(buf + offset, buf_size - offset, "<eor>");

	return offset;
}

static sqlite3_stmt* get_last_qso(sqlite3 *db) {
	if (!db)
		return NULL;

	sqlite3_stmt *stmt = NULL;
	const char *query =
		"SELECT id,mode,freq,qso_date,qso_time,callsign_sent,rst_sent,exch_sent,"
		"callsign_recv,rst_recv,exch_recv,tx_id,comments,tx_power,vswr,xota,xota_loc "
		"FROM logbook ORDER BY id DESC LIMIT 1";

	if (sqlite3_prepare_v2(db, query, -1, &stmt, NULL) != SQLITE_OK) {
		printf("ADIF Broadcast: Failed to query last QSO: %s\n", sqlite3_errmsg(db));
		return NULL;
	}

	if (sqlite3_step(stmt) != SQLITE_ROW) {
		sqlite3_finalize(stmt);
		return NULL;
	}

	return stmt;
}

int adif_broadcast_qso(void) {
	// Check if broadcasting is enabled
	const char *enabled = field_str("ADIF_BROADCAST");
	if (!enabled || strcmp(enabled, "ON") != 0) {
		printf("ADIF Broadcast: Disabled (setting: %s)\n", enabled ? enabled : "NULL");
		return 0;
	}

	printf("ADIF Broadcast: Enabled, preparing to send...\n");

	// Initialize sockets if needed (lazy init)
	if (num_destinations == 0) {
		if (adif_broadcast_init() < 0)
			return -1;
	}

	// Open database connection
	sqlite3 *db = NULL;
	char db_path[512];
	snprintf(db_path, sizeof(db_path), "%s/sbitx/data/sbitx.db", getenv("HOME"));

	if (sqlite3_open(db_path, &db) != SQLITE_OK) {
		printf("ADIF Broadcast: Failed to open database: %s\n", sqlite3_errmsg(db));
		return -1;
	}

	// Get last QSO from database
	sqlite3_stmt *stmt = get_last_qso(db);
	if (!stmt) {
		sqlite3_close(db);
		return -1;
	}

	// Format as ADIF record (ft8battle.com format: lowercase, 6-digit time)
	char adif_buffer[2048];
	int len = format_adif_for_broadcast(stmt, adif_buffer, sizeof(adif_buffer));
	sqlite3_finalize(stmt);
	sqlite3_close(db);

	if (len <= 0) {
		printf("ADIF Broadcast: Failed to format ADIF record\n");
		return -1;
	}

	printf("ADIF Broadcast: Formatted record: %s\n", adif_buffer);

	// Send to all destinations
	int success_count = 0;
	for (int i = 0; i < num_destinations; i++) {
		if (!destinations[i].initialized || destinations[i].socket < 0) {
			continue;
		}

		int sent = sendto(destinations[i].socket, adif_buffer, len, 0,
						 (struct sockaddr*)&destinations[i].addr,
						 sizeof(destinations[i].addr));

		if (sent < 0) {
			printf("ADIF Broadcast: sendto %s:%d failed: %s\n",
				   destinations[i].host, destinations[i].port, strerror(errno));
		} else {
			printf("ADIF Broadcast: Sent %d bytes to %s:%d\n",
				   sent, destinations[i].host, destinations[i].port);
			success_count++;
		}
	}

	return (success_count > 0) ? 0 : -1;
}

void adif_broadcast_close(void) {
	close_all_destinations();
	last_destinations_str[0] = '\0';
	printf("ADIF Broadcast: Sockets closed\n");
}
