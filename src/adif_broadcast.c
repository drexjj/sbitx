#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <complex.h>
#include <fftw3.h>
#include <sqlite3.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "logbook.h"
#include "adif_broadcast.h"

static int broadcast_socket = -1;
static struct sockaddr_in broadcast_addr;
static char last_ip[20] = "";
static int last_port = 0;

int adif_broadcast_init(const char *ip, int port) {
	// Close existing socket if IP/port changed
	if (broadcast_socket >= 0 &&
		(strcmp(ip, last_ip) != 0 || port != last_port)) {
		close(broadcast_socket);
		broadcast_socket = -1;
	}

	// Already initialized with same settings
	if (broadcast_socket >= 0)
		return 0;

	// Create UDP socket
	broadcast_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if (broadcast_socket < 0) {
		printf("ADIF Broadcast: Failed to create socket: %s\n", strerror(errno));
		return -1;
	}

	// Set non-blocking
	int flags = fcntl(broadcast_socket, F_GETFL, 0);
	fcntl(broadcast_socket, F_SETFL, flags | O_NONBLOCK);

	// Configure destination address
	memset(&broadcast_addr, 0, sizeof(broadcast_addr));
	broadcast_addr.sin_family = AF_INET;
	broadcast_addr.sin_port = htons(port);

	if (inet_aton(ip, &broadcast_addr.sin_addr) == 0) {
		printf("ADIF Broadcast: Invalid IP address: %s\n", ip);
		close(broadcast_socket);
		broadcast_socket = -1;
		return -1;
	}

	strncpy(last_ip, ip, sizeof(last_ip) - 1);
	last_port = port;

	printf("ADIF Broadcast: Initialized UDP to %s:%d\n", ip, port);
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
	char comment[128];
	snprintf(comment, sizeof(comment), "%s  Sent: %s  Rcvd: %s",
		submode ? submode : "",
		rst_sent ? rst_sent : "",
		rst_rcvd ? rst_rcvd : "");
	if (comment[0]) {
		offset += snprintf(buf + offset, buf_size - offset,
			"<comment:%d>%s ", (int)strlen(comment), comment);
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

	// Get IP and port from settings
	const char *ip = field_str("ADIF_IP");
	if (!ip || !ip[0])
		ip = "127.0.0.1";

	int port = field_int("ADIF_PORT");
	if (port < 1024 || port > 65535)
		port = 12060;

	// Initialize socket if needed
	if (adif_broadcast_init(ip, port) < 0)
		return -1;

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

	// Send via UDP
	int sent = sendto(broadcast_socket, adif_buffer, len, 0,
					  (struct sockaddr*)&broadcast_addr, sizeof(broadcast_addr));

	if (sent < 0) {
		printf("ADIF Broadcast: sendto failed: %s\n", strerror(errno));
		return -1;
	}

	printf("ADIF Broadcast: Sent %d bytes to %s:%d\n", sent, ip, port);
	return 0;
}

void adif_broadcast_close(void) {
	if (broadcast_socket >= 0) {
		close(broadcast_socket);
		broadcast_socket = -1;
		printf("ADIF Broadcast: Socket closed\n");
	}
}
