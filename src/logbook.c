#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <linux/types.h>
#include <stdint.h>
#include <pthread.h>
#include <stdbool.h>
#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <gtk/gtk.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "logbook.h"
#include "adif_broadcast.h"

#include <sqlite3.h>

static int rc;
static sqlite3* db = NULL;
GtkListStore* list_store = NULL;
GtkTreeSelection* selection = NULL;
GtkWidget* logbook_window = NULL;
GtkWidget* tree_view = NULL;

int logbook_fill(int from_id, int count, const char* query);
void logbook_refill(const char* query);
void clear_tree(GtkListStore* list_store);

int logbook_has_power_swr_xota() {
	static int ret = -1;
	if (ret < 0) {
		sqlite3_stmt *stmt;
		if (db == NULL)
			logbook_open();
		// https://stackoverflow.com/a/30348775
		// it might be called sqlite_schema in newer versions
		sqlite3_prepare_v2(db, "select sql from sqlite_master where name='logbook';", -1, &stmt, NULL);
		assert(sqlite3_step(stmt) == SQLITE_ROW);
		assert(sqlite3_column_count(stmt));
		assert(sqlite3_column_type(stmt, 0) == SQLITE3_TEXT);
		const char *sql = sqlite3_column_text(stmt, 0); // a CREATE TABLE command
		ret = (strstr(sql, "tx_power") && strstr(sql, "vswr") && strstr(sql, "xota"));
		sqlite3_finalize(stmt);
	}
	return ret;
}

/*!
	Writes the output into a temporary file and copies the file path into
	\a result_file, a string with length \a result_file_len.
	(Currently the file path is always /tmp/sbitx_result_rows.txt, but don't count on it.)
	The file format is strings separated by `|` characters (psv, pipe-separated values).
	- If \a from_id is negative: writes the next 50 records with a higher id than that
	- If \a from_id is positive: writes 50 records prior to it (those with a lower id)
	- If \a from_id is \c 0: writes the most-recent 50 records
*/
int logbook_query(char* query, int from_id, char* result_file, int result_file_len)
{
	sqlite3_stmt* stmt;
	char statement[200], param[2000];

	if (db == NULL)
		logbook_open();

	// add to the bottom of the logbook
	if (from_id > 0) {
		if (query)
			snprintf(statement, sizeof(statement), "select * from logbook "
							   "where (callsign_recv LIKE '%s%%' AND id < %d) ",
				query, from_id);
		else
			snprintf(statement, sizeof(statement), "select * from logbook where id < %d ", from_id);
	}
	// last 50 QSOs
	else if (from_id == 0) {
		if (query)
			snprintf(statement, sizeof(statement), "select * from logbook "
							   "where callsign_recv LIKE '%s%%' ",
				query);
		else
			strncpy(statement, "select * from logbook ", sizeof(statement));
	}
	// latest QSOs after from_id (top of the log)
	else {
		if (query)
			snprintf(statement, sizeof(statement), "select * from logbook "
							   "where (callsign_recv LIKE '%s%%' AND id > %d) ",
				query, -from_id);
		else
			snprintf(statement, sizeof(statement), "select * from logbook where id > %d ", -from_id);
	}
	strcat(statement, "ORDER BY id DESC LIMIT 50;");

	// printf("[%s]\n", statement);
	sqlite3_prepare_v2(db, statement, -1, &stmt, NULL);

 	const char *output_path = "/tmp/sbitx_result_rows.txt";
	strncpy(result_file, output_path, result_file_len);

	FILE* pf = fopen(output_path, "w");
	if (!pf)
		return -1;

	int rec = 0;
	while (sqlite3_step(stmt) == SQLITE_ROW) {
		int i;
		int num_cols = sqlite3_column_count(stmt);
		for (i = 0; i < num_cols; i++) {
			switch (sqlite3_column_type(stmt, i)) {
			case (SQLITE3_TEXT):
				strncpy(param, sqlite3_column_text(stmt, i), sizeof(param));
				break;
			case (SQLITE_INTEGER):
				snprintf(param, sizeof(param), "%d", sqlite3_column_int(stmt, i));
				break;
			case (SQLITE_FLOAT):
				snprintf(param, sizeof(param), "%g", sqlite3_column_double(stmt, i));
				break;
			case (SQLITE_NULL):
				break;
			default:
				snprintf(param, sizeof(param), "%d", sqlite3_column_type(stmt, i));
				break;
			}
			// printf("%s|", param);
			fprintf(pf, "%s|", param);
		}
		// printf("\n");
		fprintf(pf, "\n");
	}
	sqlite3_finalize(stmt);
	fclose(pf);
	return rec;
}

int logbook_count_dup(const char* callsign, int last_seconds)
{
	char date_str[100], time_str[100], statement[1000];
	sqlite3_stmt* stmt;

	time_t log_time = time_sbitx() - last_seconds;
	struct tm* tmp = gmtime(&log_time);
	snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", tmp->tm_year + 1900, tmp->tm_mon + 1, tmp->tm_mday);
	snprintf(time_str, sizeof(time_str), "%02d%02d", tmp->tm_hour, tmp->tm_min);

	snprintf(statement, sizeof(statement), "select * from logbook where "
					   "callsign_recv=\"%s\" AND qso_date >= \"%s\" AND qso_time >= \"%s\"",
		callsign, date_str, time_str);

	sqlite3_prepare_v2(db, statement, -1, &stmt, NULL);
	int rec = 0;
	while (sqlite3_step(stmt) == SQLITE_ROW) {
		rec++;
	}
	sqlite3_finalize(stmt);
	return rec;
}

int logbook_get_grids(void (*f)(char*, int))
{
	sqlite3_stmt* stmt;
	char* statement = "SELECT exch_recv, COUNT(*) AS n FROM logbook "
					  "GROUP BY exch_recv order by exch_recv";
	int res = sqlite3_prepare_v2(db, statement, -1, &stmt, NULL);
	int cnt = 0;
	char grid[20];
	int n = 0;
	while (sqlite3_step(stmt) == SQLITE_ROW) {
		int num_cols = sqlite3_column_count(stmt);
		for (int i = 0; i < num_cols; i++) {
			char const* col_name = sqlite3_column_name(stmt, i);
			if (!strcmp(col_name, "exch_recv")) {
				strncpy(grid, sqlite3_column_text(stmt, i), sizeof(grid));
			} else if (!strcmp(col_name, "n")) {
				n = sqlite3_column_int(stmt, i);
			}
		}
		f(grid, n);
		cnt++;
	}
	sqlite3_finalize(stmt);
	return cnt;
}

static sqlite3_stmt* logbook_last_qso_stmt = NULL;

/*!
	If the \a callsign is found in the logbook, return the timestamp of the most recent QSO.
	If not, return \c 0.
	\note callsign may end with a space character, null, etc.; \a len tells where
*/
time_t logbook_last_qso(const char* callsign, int len)
{
	time_t ret = 0;
	if (!logbook_last_qso_stmt) {
		const char* statement = "SELECT unixepoch(qso_date ||' ' || substr(qso_time, 1, 2) || ':' || substr(qso_time, 3, 2)) FROM logbook WHERE callsign_recv=?"
			" ORDER BY unixepoch(qso_date ||' ' || substr(qso_time, 1, 2) || ':' || substr(qso_time, 3, 2)) DESC";
		int zErrMsg = sqlite3_prepare_v2(db, statement, -1, &logbook_last_qso_stmt, NULL);
		if (zErrMsg != SQLITE_OK) {
			fprintf(stderr, "Failed to query logbook for last QSO. SQL error: %d\n", zErrMsg);
			return ret;
		}
	}
	int zErrMsg = sqlite3_bind_text(logbook_last_qso_stmt, 1, callsign, len, SQLITE_STATIC);
	if (zErrMsg == SQLITE_OK && sqlite3_step(logbook_last_qso_stmt) == SQLITE_ROW) {
		ret = sqlite3_column_int64(logbook_last_qso_stmt, 0);
		//~ printf("last QSO with %s was at timestamp %lld\n", callsign, ret);
	}
	// release binding of \a callsign, and be ready to reuse logbook_last_qso_stmt next time
	sqlite3_reset(logbook_last_qso_stmt);
	return ret;
}

static sqlite3_stmt* logbook_grid_last_qso_stmt = NULL;

/*!
	If the grid \a id is found in the logbook, return the timestamp of the most recent QSO.
	If not, return \c 0.
	\note \a id may end with a space character, null, etc.; \a len tells where
*/
time_t logbook_grid_last_qso(const char *id, int len)
{
	time_t ret = 0;
	if (len < 4 || !strncmp(id, "RR73", 4) || !strncmp(id, "R+", 2) || !strncmp(id, "R-", 2) || !(isalpha(id[0]) && isalpha(id[1])))
		return ret; // that's not a grid
	if (!logbook_grid_last_qso_stmt) {
		const char* statement = "SELECT unixepoch(qso_date ||' ' || substr(qso_time, 1, 2) || ':' || substr(qso_time, 3, 2)) FROM logbook WHERE exch_recv=?"
			" ORDER BY unixepoch(qso_date ||' ' || substr(qso_time, 1, 2) || ':' || substr(qso_time, 3, 2)) DESC";
		int zErrMsg = sqlite3_prepare_v2(db, statement, -1, &logbook_grid_last_qso_stmt, NULL);
		if (zErrMsg != SQLITE_OK) {
			fprintf(stderr, "Failed to query logbook for last QSO. SQL error: %d\n", zErrMsg);
			return ret;
		}
	}
	int zErrMsg = sqlite3_bind_text(logbook_grid_last_qso_stmt, 1, id, len, SQLITE_STATIC);
	if (zErrMsg == SQLITE_OK && sqlite3_step(logbook_grid_last_qso_stmt) == SQLITE_ROW) {
		ret = sqlite3_column_int64(logbook_grid_last_qso_stmt, 0);
		//~ printf("last QSO with %s was at timestamp %lld\n", id, ret);
	}
	// release binding of \a callsign, and be ready to reuse logbook_grid_last_qso_stmt next time
	sqlite3_reset(logbook_grid_last_qso_stmt);
	return ret;
}

// TODO unsafe API: result_len must be given
int logbook_prev_log(const char* callsign, char* result)
{
	char statement[1000], param[2000];
	sqlite3_stmt* stmt;
	snprintf(statement, sizeof(statement), "select * from logbook where "
					   "callsign_recv=\"%s\" ORDER BY id DESC",
		callsign);
	strcpy(result, callsign);
	strcat(result, ": ");
	int res = sqlite3_prepare_v2(db, statement, -1, &stmt, NULL);
	int rec = 0;
	while (sqlite3_step(stmt) == SQLITE_ROW) {
		int i;
		int num_cols = sqlite3_column_count(stmt);
		if (rec == 0) {
			for (i = 0; i < num_cols; i++) {
				char const* col_name = sqlite3_column_name(stmt, i);
				if (!strcmp(col_name, "id")) {
					continue;
				}
				if (!strcmp(col_name, "callsign_recv")) {
					continue;
				}
				switch (sqlite3_column_type(stmt, i)) {
				case (SQLITE3_TEXT):
					strncpy(param, sqlite3_column_text(stmt, i), sizeof(param));
					break;
				case (SQLITE_INTEGER):
					snprintf(param, sizeof(param), "%d", sqlite3_column_int(stmt, i));
					break;
				case (SQLITE_FLOAT):
					snprintf(param, sizeof(param), "%g", sqlite3_column_double(stmt, i));
					break;
				case (SQLITE_NULL):
					break;
				default:
					snprintf(param, sizeof(param), "%d", sqlite3_column_type(stmt, i));
					break;
				}
				strcat(result, param);
				if (!strcmp(col_name, "qso_date"))
					strcat(result, "_");
				else
					strcat(result, " ");
			}
		}
		rec++;
	}
	sqlite3_finalize(stmt);
	snprintf(param, sizeof(param), ": %d", rec);
	strcat(result, param);
	return rec;
}

int row_count_callback(void* data, int argc, char** argv, char** azColName)
{
	int* count = (int*)data;
	(*count)++;
	return 0;
}

void logbook_open()
{
	char db_path[PATH_MAX];
	char* zErrMsg = 0;
	snprintf(db_path, sizeof(db_path), "%s/sbitx/data/sbitx.db", getenv("HOME"));

	rc = sqlite3_open(db_path, &db);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "Failed to open logbook. SQL error: %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
		return;
	}
	char* sql = "SELECT name FROM sqlite_master WHERE type='index' AND tbl_name='logbook' AND name IN ('gridIx', 'callIx');";
	int index_count = 0;
	rc = sqlite3_exec(db, sql, row_count_callback, &index_count, &zErrMsg);
	if (rc != SQLITE_OK) {
		fprintf(stderr, "logbook index check failed: %s\n", zErrMsg);
		sqlite3_free(zErrMsg);
		return;
	}
	if (index_count == 2) {
		printf("Logbook indexes are OK.\n");
	} else {
		char* sql1 = "CREATE INDEX gridIx ON logbook (exch_recv);";
		char* sql2 = "CREATE INDEX callIx ON logbook (callsign_recv);";
		rc = sqlite3_exec(db, sql1, NULL, NULL, &zErrMsg);
		if (rc != SQLITE_OK) {
			fprintf(stderr, "SQL-error creating gridIx: %s\n", zErrMsg);
			sqlite3_free(zErrMsg);
			return;
		}
		rc = sqlite3_exec(db, sql2, NULL, NULL, &zErrMsg);
		if (rc != SQLITE_OK) {
			fprintf(stderr, "SQL-error creating callIx: %s\n", zErrMsg);
			sqlite3_free(zErrMsg);
			return;
		}
		printf("Logbook indexes created.\n");
	}
}

void logbook_add(const char* contact_callsign, const char* rst_sent, const char* exchange_sent,
	const char* rst_recv, const char* exchange_recv, int tx_power, int tx_vswr,
	const char* xota, const char* xota_loc, const char* comments)
{
	char statement[1000], *err_msg, date_str[11], time_str[5];
	char log_freq[12], mode[10], mycallsign[12];

	time_t log_time = time_sbitx();
	struct tm* tmp = gmtime(&log_time);
	get_field_value("r1:mode", mode);
	const bool ftx = !strncmp(mode, "FT", 2);
	int freq = field_int("FREQ");
	// For FT8/FT4 we log dial frequency + audio frequency
	if (ftx) {
		const int pitch = field_int("FTX_RX_PITCH");
		printf("FTx: freq %d + pitch %d\n", freq, pitch);
		freq += pitch;
	}
	get_field_value("#mycallsign", mycallsign);

	snprintf(log_freq, sizeof(log_freq), "%d", freq);
	snprintf(date_str, sizeof(date_str), "%04d-%02d-%02d", tmp->tm_year + 1900, tmp->tm_mon + 1, tmp->tm_mday);
	snprintf(time_str, sizeof(time_str), "%02d%02d", tmp->tm_hour, tmp->tm_min);

	if (logbook_has_power_swr_xota()) {
		snprintf(statement, sizeof(statement),
			"INSERT INTO logbook (freq, mode, qso_date, qso_time, callsign_sent,"
			"rst_sent, exch_sent, callsign_recv, rst_recv, exch_recv, tx_power, vswr, xota, xota_loc, comments) "
			"VALUES('%s', '%s', '%s', '%s',  '%s','%s','%s',  '%s','%s','%s','%d.%d','%d.%d','%s','%s','%s');",
				log_freq, mode, date_str, time_str, mycallsign,
				rst_sent, exchange_sent, contact_callsign, rst_recv, exchange_recv,
				tx_power / 10, tx_power % 10, tx_vswr / 10, tx_vswr % 10, xota, xota_loc, comments);
	} else {
		snprintf(statement, sizeof(statement),
			"INSERT INTO logbook (freq, mode, qso_date, qso_time, callsign_sent,"
			"rst_sent, exch_sent, callsign_recv, rst_recv, exch_recv, comments) "
			"VALUES('%s', '%s', '%s', '%s',  '%s','%s','%s',  '%s','%s','%s','%s');",
				log_freq, mode, date_str, time_str, mycallsign,
				rst_sent, exchange_sent, contact_callsign, rst_recv, exchange_recv, comments);
	}

	if (db == NULL)
		logbook_open();

	sqlite3_exec(db, statement, 0, 0, &err_msg);

	// Broadcast ADIF record via UDP if enabled
	adif_broadcast_qso();

	logbook_refill(NULL);
}

void logbook_refill(const char* query)
{
	// refresh/refill the list if opened
	if (list_store) {
		/* Detach model from view */
		gtk_tree_view_set_model(GTK_TREE_VIEW(tree_view), NULL);

		clear_tree(list_store);
		logbook_fill(0, 10000, query);

		/* Re-attach model to view */
		gtk_tree_view_set_model(GTK_TREE_VIEW(tree_view), GTK_TREE_MODEL(list_store));
	}
}

/*
void import_logs(char *filename){
	char entry_text[1000], statement[1000];
	char freq[10], mode[10], date_str[10], time_str[10], mycall[10], rst_sent[10],
	exchange_sent[10], contact_callsign[12], rst_recv[10], exchange_recv[10];

	FILE *pf = fopen(filename, "r");
	while(fgets(entry_text, sizeof(entry_text), pf)){
		char *p = strtok(entry_text, "\t ");
		strcpy(freq, p);
		strcpy(mode, strtok(NULL, "\t "));
		strcpy(date_str, strtok(NULL, "\t "));
		strcpy(time_str, strtok(NULL, "\t "));
		strcpy(mycall, strtok(NULL, "\t "));
		strcpy(rst_sent, strtok(NULL, "\t "));
		strcpy(exchange_sent, strtok(NULL, "\t "));
		strcpy(contact_callsign, strtok(NULL, "\t "));
		strcpy(rst_recv, strtok(NULL, "\t "));
		strcpy(exchange_recv, strtok(NULL, "\t\n"));
		sprintf(statement,
		"INSERT INTO logbook (freq, mode, qso_date, qso_time, callsign_sent,"
		"rst_sent, exch_sent, callsign_recv, rst_recv, exch_recv) "
		"VALUES('%s', '%s', '%s', '%s',  '%s','%s','%s',  '%s','%s','%s');",
			freq, mode, date_str, time_str,
			 mycall, rst_sent, exchange_sent,
			contact_callsign, rst_recv, exchange_recv);

		puts(statement);
	}
	fclose(pf);
}
*/

// ADIF field headers in the order requested in prepare_query_by_date()
const static char* adif_names[] = { "ID", "MODE", "FREQ", "QSO_DATE", "TIME_ON", "OPERATOR",
		"RST_SENT", "STX_String", "CALL", "RST_RCVD", "SRX_String", "STX", "COMMENTS",
		"TX_PWR", "MY_SIG", "MY_SOTA_REF"};

struct band_name {
	char* name;
	int from, to;
} bands[] = {
	{ "160M", 1800, 2000 },
	{ "80M", 3500, 4000 },
	{ "60M", 5000, 5500 },
	{ "40M", 7000, 7300 },
	{ "30M", 10000, 10150 },
	{ "20M", 14000, 14350 },
	{ "17M", 18000, 18200 },
	{ "15M", 21000, 21450 },
	{ "12M", 24800, 25000 },
	{ "10M", 28000, 29700 },
};

static void strip_chr(char* str, const char to_remove)
{
	int i, j, len;

	len = strlen(str);
	for (i = 0; i < len; i++) {
		if (str[i] == to_remove) {
			for (j = i; j < len; j++)
				str[j] = str[j + 1];
			len--;
			i--;
		}
	}
}

void *prepare_query_by_date(const char *start_date, const char *end_date)
{
	char statement[250];
	sqlite3_stmt *ret = NULL;
	if (logbook_has_power_swr_xota()) {
		if (start_date && start_date[0]) {
			snprintf(statement, sizeof(statement),
					"select id,mode,freq,qso_date,qso_time,callsign_sent,rst_sent,exch_sent,callsign_recv,rst_recv,exch_recv,tx_id,comments,tx_power,xota,xota_loc "
					" from logbook where (qso_date >= '%s' AND qso_date <= '%s') ORDER BY id DESC;",
					start_date, end_date);
		} else {
			strncpy(statement,
					"select id,mode,freq,qso_date,qso_time,callsign_sent,rst_sent,exch_sent,callsign_recv,rst_recv,exch_recv,tx_id,comments,tx_power,xota,xota_loc "
					" from logbook ORDER BY id DESC;", sizeof(statement));
		}
	} else {
		snprintf(statement, sizeof(statement),
				"select id,mode,freq,qso_date,qso_time,callsign_sent,rst_sent,exch_sent,callsign_recv,rst_recv,exch_recv,tx_id,comments "
				" from logbook where (qso_date >= '%s' AND qso_date <= '%s') ORDER BY id DESC;",
				start_date, end_date);
	}

	int r = sqlite3_prepare_v2(db, statement, -1, &ret, NULL);
	if (r != SQLITE_OK) {
		printf("problem with query: %s\n", statement);
		ret = NULL;
	}
	return ret;
}

int write_adif_header(char *buf, int len, const char *source) {
	return snprintf(buf,  len, "/ADIF file\n"
		"generated from sBITX log db by %s\n"
		"<adif version:5>3.1.4\n"
		"<EOH>\n", source);
}

/*!
	Write out a single record from results returned from prepare_query_by_date().
	The ADIF field names come from adif_names, but we make some substitutions:
	- MY_GRIDSQUARE/GRIDSQUARE instead of STX_String/SRX_String for FTx
	- MY_SIG_INFO instead of MY_SOTA_REF for POTA
	- IOTA instead of MY_SOTA_REF for IOTA
*/
int write_adif_record(void *stmt, char *buf, int len) {
	char field_value[2000]; // big enough for a comment, hopefully; the rest are much smaller
	int num_cols = sqlite3_column_count(stmt);
	bool is_ftx = false;

	int buf_offset = 0;
	char sig[5]; // IOTA/SOTA/POTA
	for (int i = 1; i < num_cols; i++) {
		switch (sqlite3_column_type(stmt, i))
		{
		case (SQLITE3_TEXT):
			strncpy(field_value, sqlite3_column_text(stmt, i), sizeof(field_value));
			break;
		case (SQLITE_INTEGER):
			snprintf(field_value, sizeof(field_value), "%d", sqlite3_column_int(stmt, i));
			break;
		case (SQLITE_FLOAT):
			snprintf(field_value, sizeof(field_value), "%g", sqlite3_column_double(stmt, i));
			break;
		case (SQLITE_NULL):
			field_value[0] = 0; // field empty, nothing to export
			break;
		default:
			snprintf(field_value, sizeof(field_value), "%d", sqlite3_column_type(stmt, i));
			break;
		}
		//~ printf("col %d of %d type %d: ADIF %s value '%s'\n",
			//~ i, num_cols, sqlite3_column_type(stmt, i), adif_names[i], field_value);

		int field_len = strlen(field_value);
		bool output_done = false;
		// Columns are in the order requested in prepare_query_by_date().
		// First, take care of special cases for certain columns:
		switch (i) {
		case 1: // mode
			// If mode is FT8/FT4, remember to use gridsquare instead of stx/srx fields - n1qm & k7ihz
			is_ftx = (!strncmp("FT", field_value, 2));
			break;
		case 2: { // freq
			long hz = atoi(field_value);
			if (hz > 100000) {
				// big number: it must really be in hz (new log entry)
				float mhz = hz / 1000000.0;
				snprintf(field_value, sizeof(field_value), "%.6f", mhz); // write out with 6 decimal digits
			} else {
				// assume it's in khz (old log entry)
				hz *= 1000;
				float mhz = hz / 1000000.0;
				snprintf(field_value, sizeof(field_value), "%.3f", mhz); // write out only the 3 digits we know
			}
			const long khz = hz / 1000;
			for (int j = 0 ; j < sizeof(bands)/sizeof(struct band_name); j++)
				if (bands[j].from <= khz && khz <= bands[j].to)
					buf_offset += snprintf(buf + buf_offset, len - buf_offset,
						"<BAND:%d>%s ", strlen(bands[j].name), bands[j].name);
		} break;
		case 3: // qso_date
			strip_chr(field_value, '-');
			field_len = strlen(field_value);
			break;
		case 7: // exch_sent
			if (is_ftx)
				buf_offset += snprintf(buf + buf_offset, len - buf_offset,
					"<MY_GRIDSQUARE:%d>%s ", field_len, field_value);
			else
				buf_offset += snprintf(buf + buf_offset, len - buf_offset,
					"<%s:%d>%s ", adif_names[i], field_len, field_value);
			output_done = true;
			break;
		case 10: // exch_recv
			if (is_ftx)
				buf_offset += snprintf(buf + buf_offset, len - buf_offset,
					"<GRIDSQUARE:%d>%s ", field_len, field_value);
			else
				buf_offset += snprintf(buf + buf_offset, len - buf_offset,
					"<%s:%d>%s ", adif_names[i], field_len, field_value);
			output_done = true;
			break;
		case 14: // xota
			strncpy(sig, field_value, sizeof(sig));
			break;
		case 15: // xota_loc
			if (!strcmp("POTA", sig)) {
				buf_offset += snprintf(buf + buf_offset, len - buf_offset,
					"<MY_SIG_INFO:%d>%s ", field_len, field_value);
				output_done = true;
			} else if (!strcmp("IOTA", sig)) {
				buf_offset += snprintf(buf + buf_offset, len - buf_offset,
					"<IOTA:%d>%s ", field_len, field_value);
				output_done = true;
			}
			// SOTA (as default) is taken care of below: adif_names[15] = MY_SOTA_REF
			break;
		default:
			break;
		}
		// Special cases above can set output_done = true. Otherwise, assume
		// output for this field is not yet done, so do it the generic way:
		// the ADIF field name comes from the adif_names array.
		if (!output_done && field_len > 0) {
			//~ printf("    default output @%d\n", buf_offset);
			buf_offset += snprintf(buf + buf_offset, len - buf_offset,
				"<%s:%d>%s ", adif_names[i], field_len, field_value);
		}
	}
	buf_offset += snprintf(buf + buf_offset, len - buf_offset, "<EOR>\n");
	return buf_offset;
}

bool logbook_next(void *stmt) {
	return sqlite3_step(stmt) == SQLITE_ROW;
}

void logbook_end_query(void *stmt) {
	sqlite3_finalize(stmt);
}

int export_adif(const char *path, const char *start_date, const char *end_date, const char *source) {
	char buf[4096];
	sqlite3_stmt *stmt = prepare_query_by_date(start_date, end_date);
	if (!stmt)
		return -1;

	//add to the bottom of the logbook
	FILE *pf = fopen(path, "w");
	fwrite(buf, 1, write_adif_header(buf, sizeof(buf), source), pf);

	while (logbook_next(stmt)) {
		fwrite(buf, 1, write_adif_record(stmt, buf, sizeof(buf)), pf);
	}
	logbook_end_query(stmt);
	fclose(pf);
}

// Added the data folder for the save location  - W9JES
int get_filename(char* path)
{
	GtkWidget* dialog;
	GtkFileChooser* chooser;
	GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SAVE;
	gint res;

	// Create a file chooser dialog
	dialog = gtk_file_chooser_dialog_new("Save Logbook As..",
		NULL, action, "_Cancel", GTK_RESPONSE_CANCEL, "_Save", GTK_RESPONSE_ACCEPT,
		NULL);

	chooser = GTK_FILE_CHOOSER(dialog);

	// Set default folder, filename, and file filter
	gtk_file_chooser_set_current_folder(chooser, "/home/pi/sbitx/data");
	gtk_file_chooser_set_current_name(chooser, "Untitled.adi");
	gtk_file_chooser_set_do_overwrite_confirmation(chooser, TRUE);
	GtkFileFilter* filter = gtk_file_filter_new();
	gtk_file_filter_add_pattern(filter, "*.adi");
	gtk_file_filter_set_name(filter, "ADI files (*.adi)");
	gtk_file_chooser_add_filter(chooser, filter);

	// Run the dialog and process the user response
	res = gtk_dialog_run(GTK_DIALOG(dialog));
	if (res == GTK_RESPONSE_ACCEPT) {
		char* filename;
		filename = gtk_file_chooser_get_filename(chooser);
		// Do something with the selected filename
		strcpy(path, filename);
		g_free(filename);
		gtk_widget_destroy(dialog);
		return 0;
	}

	gtk_widget_destroy(dialog);
	return -1;
}

/* Export functions */

/*
int get_filename(char *path) {
	GtkWidget *dialog;
	GtkFileChooser *chooser;
	GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SAVE;
	gint res;

	// Create a file chooser dialog
	dialog = gtk_file_chooser_dialog_new("Save Logbook As..",
	  NULL, action, "_Cancel", GTK_RESPONSE_CANCEL, "_Save", GTK_RESPONSE_ACCEPT,
	  NULL);

	chooser = GTK_FILE_CHOOSER(dialog);

	// Set default filename and file filter
	gtk_file_chooser_set_current_name(chooser, "Untitled.adi");
	gtk_file_chooser_set_do_overwrite_confirmation(chooser, TRUE);
	GtkFileFilter *filter = gtk_file_filter_new();
	gtk_file_filter_add_pattern(filter, "*.adi");
	gtk_file_filter_set_name(filter, "ADI files (*.adi)");
	gtk_file_chooser_add_filter(chooser, filter);

	// Run the dialog and process the user response
	res = gtk_dialog_run(GTK_DIALOG(dialog));
	if (res == GTK_RESPONSE_ACCEPT) {
		char *filename;
		filename = gtk_file_chooser_get_filename(chooser);
		// Do something with the selected filename
				strcpy(path, filename);
		g_free(filename);
			gtk_widget_destroy(dialog);
				return 0;
	}

	gtk_widget_destroy(dialog);
		return -1;
}
*/

/*
void import_button_clicked(GtkWidget *window) {
	GtkWidget *dialog, *content_area, *vbox, *hbox, *frombox, *tobox;
	GtkWidget *start_label, *end_label;
	GtkWidget *start_calendar, *end_calendar;
	GtkWidget *save_button, *cancel_button;

	// Create a new dialog
	dialog = gtk_dialog_new_with_buttons("Date Selection",
	  NULL, GTK_DIALOG_MODAL,
	  "_Save", GTK_RESPONSE_OK, "_Cancel", GTK_RESPONSE_CANCEL, NULL);

	content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

	// Create vertical box container
	vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
	gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
	gtk_container_add(GTK_CONTAINER(content_area), vbox);


	// horizontal box to have the from and to dates side by side
	hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
	gtk_container_set_border_width(GTK_CONTAINER(hbox), 10);
	gtk_container_add(GTK_CONTAINER(content_area), hbox);

	// vertical box to have the from date
	frombox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
	gtk_container_set_border_width(GTK_CONTAINER(frombox), 10);
	gtk_container_add(GTK_CONTAINER(hbox), frombox);

	// Create From Date label
	start_label = gtk_label_new("From Date:");
	gtk_label_set_xalign(GTK_LABEL(start_label), 0); // Left-align the text within the label
	gtk_box_pack_start(GTK_BOX(frombox), start_label, FALSE, FALSE, 0);

	// Create start date calendar
	start_calendar = gtk_calendar_new();
	gtk_box_pack_start(GTK_BOX(frombox), start_calendar, TRUE, TRUE, 0);

	// vertical box to have the from date
	tobox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
	gtk_container_set_border_width(GTK_CONTAINER(tobox), 10);
	gtk_container_add(GTK_CONTAINER(hbox), tobox);

	// Create To Date label
	end_label = gtk_label_new("To Date:");
	gtk_label_set_xalign(GTK_LABEL(end_label), 0); // Left-align the text within the label
	gtk_box_pack_start(GTK_BOX(tobox), end_label, FALSE, FALSE, 0);

	// Create end date calendar
	end_calendar = gtk_calendar_new();
	gtk_box_pack_start(GTK_BOX(tobox), end_calendar, TRUE, TRUE, 0);

	// Show all widgets
	gtk_widget_show_all(dialog);
		gint response = gtk_dialog_run(GTK_DIALOG(dialog));
		if (response == GTK_RESPONSE_OK){
			char path[1000], start_str[20], end_str[20];
			if (get_filename(path) != -1){
				guint start_year, start_month, start_day, end_year, end_month, end_day;
				gtk_calendar_get_date((GtkCalendar *)end_calendar,
					&end_year, &end_month, &end_day);
				gtk_calendar_get_date((GtkCalendar *)start_calendar,
					&start_year, &start_month, &start_day);
				sprintf(start_str,"%04d-%02d-%02d",start_year, start_month + 1, start_day);
				sprintf(end_str, "%04d-%02d-%02d", end_year, end_month + 1, end_day);
				export_adif(path, start_str, end_str);
				printf("saved logs from %s to %s to file %s\n", start_str, end_str, path);
			}
		}
	gtk_widget_destroy(dialog);
}
*/

void export_button_clicked(GtkWidget* window)
{
	GtkWidget *dialog, *content_area, *vbox, *hbox, *frombox, *tobox;
	GtkWidget *start_label, *end_label;
	GtkWidget *start_calendar, *end_calendar;
	GtkWidget *save_button, *cancel_button;

	// Create a new dialog
	dialog = gtk_dialog_new_with_buttons("Date Selection",
		NULL, GTK_DIALOG_MODAL,
		"_Save", GTK_RESPONSE_OK, "_Cancel", GTK_RESPONSE_CANCEL, NULL);

	content_area = gtk_dialog_get_content_area(GTK_DIALOG(dialog));

	// Create vertical box container
	vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
	gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
	gtk_container_add(GTK_CONTAINER(content_area), vbox);

	// horizontal box to have the from and to dates side by side
	hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);
	gtk_container_set_border_width(GTK_CONTAINER(hbox), 10);
	gtk_container_add(GTK_CONTAINER(content_area), hbox);

	// vertical box to have the from date
	frombox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
	gtk_container_set_border_width(GTK_CONTAINER(frombox), 10);
	gtk_container_add(GTK_CONTAINER(hbox), frombox);

	// Create From Date label
	start_label = gtk_label_new("From Date:");
	gtk_label_set_xalign(GTK_LABEL(start_label), 0); // Left-align the text within the label
	gtk_box_pack_start(GTK_BOX(frombox), start_label, FALSE, FALSE, 0);

	// Create start date calendar
	start_calendar = gtk_calendar_new();
	gtk_box_pack_start(GTK_BOX(frombox), start_calendar, TRUE, TRUE, 0);

	// vertical box to have the from date
	tobox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
	gtk_container_set_border_width(GTK_CONTAINER(tobox), 10);
	gtk_container_add(GTK_CONTAINER(hbox), tobox);

	// Create To Date label
	end_label = gtk_label_new("To Date:");
	gtk_label_set_xalign(GTK_LABEL(end_label), 0); // Left-align the text within the label
	gtk_box_pack_start(GTK_BOX(tobox), end_label, FALSE, FALSE, 0);

	// Create end date calendar
	end_calendar = gtk_calendar_new();
	gtk_box_pack_start(GTK_BOX(tobox), end_calendar, TRUE, TRUE, 0);

	// Show all widgets
	gtk_widget_show_all(dialog);
	gint response = gtk_dialog_run(GTK_DIALOG(dialog));
	if (response == GTK_RESPONSE_OK) {
		char path[1000], start_str[20], end_str[20];
		if (get_filename(path) != -1) {
			guint start_year, start_month, start_day, end_year, end_month, end_day;
			gtk_calendar_get_date((GtkCalendar*)end_calendar,
				&end_year, &end_month, &end_day);
			gtk_calendar_get_date((GtkCalendar*)start_calendar,
				&start_year, &start_month, &start_day);
			snprintf(start_str, sizeof(start_str), "%04d-%02d-%02d", start_year, start_month + 1, start_day);
			snprintf(end_str, sizeof(end_str), "%04d-%02d-%02d", end_year, end_month + 1, end_day);
			export_adif(path, start_str, end_str, "sbitx GUI");
			printf("saved logs from %s to %s to file %s\n", start_str, end_str, path);
		}
	}
	gtk_widget_destroy(dialog);
}

/*!
	Validator (signal handler) for callsigns.
	Allows only uppercase letters, numbers, slashes and hyphens.
*/
static void on_callsign_changed(GtkWidget* widget, gpointer data)
{
	const gchar* text = gtk_entry_get_text(GTK_ENTRY(widget));
	gchar* result = g_strdup(text);

	for (int i = 0; i < strlen(text); ++i) {
		if (!(isalnum(text[i]) || text[i] == '/' || text[i] == '-')) {
			result[i] = '\0';
		} else {
			result[i] = toupper(text[i]);
		}
	}

	gtk_entry_set_text(GTK_ENTRY(widget), result);
	g_free(result);
}

// Function to create the dialog box
int edit_qso(char* qso_id, char* freq, char* mode, char* callsign, char* rst_sent, char* exchange_sent,
	char* rst_recv, char* exchange_recv, char* comment)
{
	GtkWidget *dialog, *grid, *label,
		*entry_freq, *entry_mode, *entry_callsign, *entry_rst_sent, *entry_exchange_sent,
		*entry_rst_recv, *entry_exchange_recv, *entry_comment;
	GtkWidget *ok_button, *cancel_button;
	char title[20];

	snprintf(title, sizeof(title), "Edit QSO %s", qso_id);

	dialog = gtk_dialog_new_with_buttons(title, NULL,
		GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
		"SAVE", GTK_RESPONSE_ACCEPT,
		"Cancel", GTK_RESPONSE_CANCEL,
		NULL);
	/*
		// Create a dialog window
		dialog = gtk_dialog_new_with_buttons("Dialog Example",
											 NULL,
											 GTK_DIALOG_MODAL,
											 "_Cancel",
											 GTK_RESPONSE_CANCEL,
											 "_OK",
											 GTK_RESPONSE_OK,
											 NULL);
	*/
	// Create a grid for organizing the dialog's contents
	grid = gtk_grid_new();
	gtk_grid_set_column_spacing(GTK_GRID(grid), 5);
	gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
	gtk_container_set_border_width(GTK_CONTAINER(grid), 30);
	gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(dialog))), grid, TRUE, TRUE, 0);

	// freq field
	label = gtk_label_new("Freq");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 0, 1, 1);
	entry_freq = gtk_entry_new();
	gtk_grid_attach(GTK_GRID(grid), entry_freq, 1, 0, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_freq), (gchar*)freq);
	gtk_entry_set_max_length(GTK_ENTRY(entry_freq), 19);
	gtk_entry_set_input_purpose(GTK_ENTRY(entry_freq), GTK_INPUT_PURPOSE_NUMBER);

	// mode field
	label = gtk_label_new("Mode");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 1, 1, 1);
	entry_mode = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_mode), 19);
	gtk_grid_attach(GTK_GRID(grid), entry_mode, 1, 1, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_mode), (gchar*)mode);

	// Callsign field
	label = gtk_label_new("Callsign");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 2, 1, 1);
	entry_callsign = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_callsign), 19);
	g_signal_connect(entry_callsign, "changed", G_CALLBACK(on_callsign_changed), NULL); // Connect signal handler
	gtk_grid_attach(GTK_GRID(grid), entry_callsign, 1, 2, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_callsign), (gchar*)callsign);

	// rst_sent field
	label = gtk_label_new("RST Sent");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 3, 1, 1);
	entry_rst_sent = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_rst_sent), 19);
	gtk_grid_attach(GTK_GRID(grid), entry_rst_sent, 1, 3, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_rst_sent), (gchar*)rst_sent);

	// exchange_sent field
	label = gtk_label_new("Exchange Sent");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 4, 1, 1);
	entry_exchange_sent = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_exchange_sent), 19);
	gtk_grid_attach(GTK_GRID(grid), entry_exchange_sent, 1, 4, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_exchange_sent), (gchar*)exchange_sent);

	// rst_recv field
	label = gtk_label_new("RST Rcvd");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 5, 1, 1);
	entry_rst_recv = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_rst_recv), 19);
	gtk_grid_attach(GTK_GRID(grid), entry_rst_recv, 1, 5, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_rst_recv), (gchar*)rst_recv);

	// exchange_recv field
	label = gtk_label_new("Exchange Rcvd");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 6, 1, 1);
	entry_exchange_recv = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_exchange_recv), 19);
	gtk_grid_attach(GTK_GRID(grid), entry_exchange_recv, 1, 6, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_exchange_recv), (gchar*)exchange_recv);

	// comment field
	label = gtk_label_new("Comment");
	gtk_grid_attach(GTK_GRID(grid), label, 0, 7, 1, 1);
	entry_comment = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_comment), 999);
	gtk_grid_attach(GTK_GRID(grid), entry_comment, 1, 7, 1, 1);
	gtk_entry_set_text(GTK_ENTRY(entry_comment), (gchar*)comment);

	/*
		// Report checkbox
		check_button = gtk_check_button_new_with_label("Report");
		gtk_grid_attach(GTK_GRID(grid), check_button, 0, 3, 2, 1);
	*/

	// Connect OK button signal to the handler function
	ok_button = gtk_dialog_get_widget_for_response(GTK_DIALOG(dialog), GTK_RESPONSE_OK);
	//	g_signal_connect(ok_button, "clicked", G_CALLBACK(ok_button_clicked), entry);

	gtk_widget_show_all(dialog);
	// gtk_dialog_run(GTK_DIALOG(dialog));
	gint response = gtk_dialog_run(GTK_DIALOG(dialog));

	if (response == GTK_RESPONSE_ACCEPT) {
		//~ const gchar* callsign_response = gtk_entry_get_text(GTK_ENTRY(entry_callsign));
		strncpy(freq, gtk_entry_get_text(GTK_ENTRY(entry_freq)), 20);
		strncpy(mode, gtk_entry_get_text(GTK_ENTRY(entry_mode)), 20);
		strncpy(callsign, gtk_entry_get_text(GTK_ENTRY(entry_callsign)), 20);
		strncpy(rst_sent, gtk_entry_get_text(GTK_ENTRY(entry_rst_sent)), 20);
		strncpy(rst_recv, gtk_entry_get_text(GTK_ENTRY(entry_rst_recv)), 20);
		strncpy(exchange_sent, gtk_entry_get_text(GTK_ENTRY(entry_exchange_sent)), 20);
		strncpy(exchange_recv, gtk_entry_get_text(GTK_ENTRY(entry_exchange_recv)), 20);
		strncpy(comment, gtk_entry_get_text(GTK_ENTRY(entry_comment)), 1000);
		gtk_widget_destroy(dialog);
		return GTK_RESPONSE_OK;
	} else {
		gtk_widget_destroy(dialog);
		return GTK_RESPONSE_CANCEL;
	}
}

void add_to_list(GtkListStore* list_store, const gchar* id, const gchar* qso_date, const gchar* freq,
	const gchar* mode, const gchar* callsign, const gchar* rst_sent, const gchar* exchange_sent,
	const gchar* rst_recv, const gchar* exchange_recv, const gchar* tx_pwr, const gchar* swr,
	const gchar* xota, const gchar* xota_loc, const gchar* comments)
{
	GtkTreeIter iter;
	gtk_list_store_append(list_store, &iter);
	gtk_list_store_set(list_store, &iter,
		0, id,
		1, qso_date,
		2, freq,
		3, mode,
		4, callsign,
		5, rst_sent,
		6, exchange_sent,
		7, rst_recv,
		8, exchange_recv,
		9, tx_pwr,
		10, swr,
		11, xota,
		12, xota_loc,
		13, comments,
		-1);
}

int logbook_fill(int from_id, int count, const char* query)
{
	sqlite3_stmt *stmt;
	char statement[200], param[2000];

	if (db == NULL)
		logbook_open();

	// add to the bottom of the logbook
	if (from_id > 0) {
		if (query)
			snprintf(statement, sizeof(statement), "select * from logbook "
							   "where (callsign_recv LIKE '%s%%' AND id < %d) ",
				query, from_id);
		else
			snprintf(statement, sizeof(statement), "select * from logbook where id < %d ", from_id);
	}
	// last 200 QSOs
	else if (from_id == 0) {
		if (query)
			snprintf(statement, sizeof(statement), "select * from logbook "
							   "where callsign_recv LIKE '%s%%' ",
				query);
		else
			strncpy(statement, "select * from logbook ", sizeof(statement));
	}
	// latest QSOs after from_id (top of the log)
	else {
		if (query)
			snprintf(statement, sizeof(statement), "select * from logbook "
							   "where (callsign_recv LIKE '%s%%' AND id > %d) ",
				query, -from_id);
		else
			snprintf(statement, sizeof(statement), "select * from logbook where id > %d ", -from_id);
	}

	char stmt_count[100];
	snprintf(stmt_count, sizeof(stmt_count), "ORDER BY id DESC LIMIT %d;", count);
	strcat(statement, stmt_count);
	// printf("[%s]\n", statement);
	sqlite3_prepare_v2(db, statement, -1, &stmt, NULL);

	int rec = 0;
	char id[10], qso_time[20], qso_date[20], freq[20], mode[20], callsign[20],
		rst_recv[20], exchange_recv[20], rst_sent[20], exchange_sent[20],
		tx_pwr[10], swr[10], xota[5], xota_loc[16], comments[1000];

	// these might be missing, so initialize them with empty strings
	memset(tx_pwr, 0, sizeof(tx_pwr));
	memset(swr, 0, sizeof(swr));

	while (sqlite3_step(stmt) == SQLITE_ROW) {
		int i;
		int num_cols = sqlite3_column_count(stmt);
		for (i = 0; i < num_cols; i++) {

			char const* col_name = sqlite3_column_name(stmt, i);
			if (!strcmp(col_name, "id"))
				strcpy(id, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "qso_date"))
				strcpy(qso_date, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "qso_time"))
				strcpy(qso_time, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "freq"))
				strcpy(freq, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "mode"))
				strcpy(mode, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "callsign_recv"))
				strcpy(callsign, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "rst_sent"))
				strcpy(rst_sent, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "rst_recv"))
				strcpy(rst_recv, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "exch_sent"))
				strcpy(exchange_sent, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "exch_recv"))
				strcpy(exchange_recv, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "tx_power"))
				strcpy(tx_pwr, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "vswr"))
				strcpy(swr, sqlite3_column_text(stmt, i));
			else if (!strcmp(col_name, "xota"))
				strncpy(xota, sqlite3_column_text(stmt, i), sizeof(xota));
			else if (!strcmp(col_name, "xota_loc"))
				strncpy(xota_loc, sqlite3_column_text(stmt, i), sizeof(xota_loc));
			else if (!strcmp(col_name, "comments"))
				strcpy(comments, sqlite3_column_text(stmt, i));
		}

		strcat(qso_date, " ");
		strcat(qso_date, qso_time);
		add_to_list(list_store, id,  qso_date, freq, mode,
			callsign, rst_sent, exchange_sent, rst_recv, exchange_recv,
			tx_pwr, swr, xota, xota_loc, comments);
	}
	sqlite3_finalize(stmt);
}

void clear_tree(GtkListStore* list_store)
{
	gtk_list_store_clear(list_store);
}

void search_button_clicked(GtkWidget* entry, gpointer search_box)
{
	const gchar* search_text = gtk_entry_get_text(GTK_ENTRY(search_box));
	logbook_refill(strlen(search_text) == 0 ? NULL : search_text);
}

void search_update(GtkWidget* entry, gpointer search_box)
{
	search_button_clicked(NULL, entry);
}

void delete_button_clicked(GtkWidget* entry, gpointer tree_view)
{
	gchar *qso_id, *mode, *freq, *callsign, *rst_sent, *rst_recv, *exchange_sent,
		*exchange_recv, *comment;
	GtkTreeIter iter;

	GtkTreeModel* model = gtk_tree_view_get_model(GTK_TREE_VIEW(tree_view));

	if (!gtk_tree_selection_get_selected(selection, &model, &iter))
		return;

	gtk_tree_model_get(model, &iter, 0, &qso_id, -1);

	GtkWidget* dialog = gtk_message_dialog_new(NULL,
		GTK_DIALOG_MODAL, GTK_MESSAGE_QUESTION, GTK_BUTTONS_YES_NO,
		"Do you want to delete #%s", qso_id);
	int response = gtk_dialog_run(GTK_DIALOG(dialog));
	if (response == GTK_RESPONSE_YES) {
		char statement[100], *err_msg;
		snprintf(statement, sizeof(statement), "DELETE FROM logbook WHERE id='%s';", qso_id);
		sqlite3_exec(db, statement, 0, 0, &err_msg);
	}
	gtk_widget_destroy(dialog);
	g_free(qso_id);
	// printf("Response %d\n", response);

	// refill the log
	logbook_refill(NULL);
}

void edit_button_clicked(GtkWidget* entry, gpointer tree_view)
{
	gchar *_qso_id, *_mode, *_freq, *_callsign, *_rst_sent, *_rst_recv, *_exchange_sent,
		*_exchange_recv, *_comment;
	char id[10], qso_time[20], qso_date[20], freq[20], mode[20], callsign[20],
		rst_recv[20], exchange_recv[20], rst_sent[20], exchange_sent[20],
		tx_pwr[10], swr[10], xota[5], xota_loc[16], comment[1000];
	GtkTreeIter iter;

	GtkTreeModel* model = gtk_tree_view_get_model(GTK_TREE_VIEW(tree_view));

	if (!gtk_tree_selection_get_selected(selection, &model, &iter))
		return;

	// from docs: "values with type G_TYPE_STRING... have to be freed"
	// i.e. gtk_tree_model_get() allocates just enough memory to hold each result
	gtk_tree_model_get(model, &iter, 0, &_qso_id,
		2, &_freq, 3, &_mode, 4, &_callsign, 5, &_rst_sent, 6, &_exchange_sent,
		7, &_rst_recv, 8, &_exchange_recv, 13, &_comment,
		-1);

	// But we allow the user to change the sizes (especially the comment field!)
	// so copy what we got into known-size strings, and free the strings from gtk_tree_model_get()
	strncpy(id, _qso_id, sizeof(id)); g_free(_qso_id);
	strncpy(mode, _mode, sizeof(mode)); g_free(_mode);
	strncpy(freq, _freq, sizeof(freq)); g_free(_freq);
	strncpy(callsign, _callsign, sizeof(callsign)); g_free(_callsign);
	strncpy(rst_sent, _rst_sent, sizeof(rst_sent)); g_free(_rst_sent);
	strncpy(exchange_sent, _exchange_sent, sizeof(exchange_sent)); g_free(_exchange_sent);
	strncpy(rst_recv, _rst_recv, sizeof(rst_recv)); g_free(_rst_recv);
	strncpy(exchange_recv, _exchange_recv, sizeof(exchange_recv)); g_free(_exchange_recv);
	strncpy(comment, _comment, sizeof(comment)); g_free(_comment);

	if (edit_qso(id, freq, mode, callsign, rst_sent, exchange_sent, rst_recv, exchange_recv, comment)) {
		char statement[2000], *err_msg;
		snprintf(statement, sizeof(statement),
			"UPDATE logbook SET mode = '%s', freq = '%s', callsign_recv = '%s', rst_sent = '%s', "
			"exch_sent = '%s',rst_recv = '%s', exch_recv = '%s', comments = '%s' WHERE id = '%s'",
			mode, freq, callsign, rst_sent, exchange_sent, rst_recv, exchange_recv,
			comment, id);

		sqlite3_exec(db, statement, 0, 0, &err_msg);
	}

	// refill the log
	logbook_refill(NULL);
}

// Function to handle row activation
void on_row_activated(GtkTreeView* treeview, GtkTreePath* path, GtkTreeViewColumn* column, gpointer user_data)
{
	GtkTreeModel* model = gtk_tree_view_get_model(treeview);
	gchar *qso_id, *mode, *freq, *callsign, *rst_sent, *rst_recv, *exchange_sent,
		*exchange_recv, *comment;
	GtkTreeIter iter;

	if (!gtk_tree_model_get_iter(model, &iter, path))
		return;
	gtk_tree_model_get(model, &iter, 0, &qso_id,
		2, &freq, 3, &mode, 4, &callsign, 5, &rst_sent, 6, &exchange_sent,
		7, &rst_recv, 8, &exchange_recv, 9, &comment,
		-1);

	if (edit_qso(qso_id, freq, mode, callsign, rst_sent, exchange_sent, rst_recv, exchange_recv, comment)) {
		char statement[1000], *err_msg;
		snprintf(statement, sizeof(statement),
			"UPDATE logbook SET mode = '%s', freq = '%s', callsign_recv = '%s', rst_sent = '%s', "
			"exch_sent = '%s',rst_recv = '%s', exch_recv = '%s', comments = '%s' WHERE id = '%s'",
			mode, freq, callsign, rst_sent, exchange_sent, rst_recv, exchange_recv,
			comment, qso_id);

		sqlite3_exec(db, statement, 0, 0, &err_msg);
	}

	g_free(qso_id);
	g_free(mode);
	g_free(freq);
	g_free(callsign);
	g_free(rst_sent);
	g_free(exchange_sent);
	g_free(rst_recv);
	g_free(exchange_recv);
	g_free(comment);

	// refill the log
	logbook_refill(NULL);
}

// Function to handle row selection
void on_selection_changed(GtkTreeSelection* selection, gpointer user_data)
{
	GtkTreeModel* model;
	GtkTreeIter iter;
	if (gtk_tree_selection_get_selected(selection, &model, &iter)) {
		gtk_tree_selection_selected_foreach(selection, (GtkTreeSelectionForeachFunc)gtk_tree_view_row_activated, NULL);
	}
}

void logbook_close(GtkWidget* widget, gpointer user_data)
{
	/*
		At the time this callback occurs, the toplevel is already being destroyed,
		so do NOT call gtk_widget_destroy().

		 We do need to drop our own reference to the list store so the
		 model can be finalized once the tree view releases its ref.
		 Finally, clear module-level pointers to avoid dangling references.
	*/
	if (list_store) {
		g_object_unref(list_store);
		list_store = NULL;
	}
	tree_view = NULL;
	selection = NULL;
	logbook_window = NULL;
}

/*!
    Handle key presses in the logbook window.
    Close the logbook when Ctrl-W (or Ctrl-w) is pressed.
    Return TRUE if the event was handled to stop further processing.
 */
gboolean logbook_key_press(GtkWidget* widget, GdkEventKey* event, gpointer user_data)
{
	if ((event->state & GDK_CONTROL_MASK) &&
	    (event->keyval == GDK_KEY_w || event->keyval == GDK_KEY_W)) {
		if (logbook_window) {
			gtk_widget_destroy(logbook_window);
			return TRUE;
		}
	}
	return FALSE;
}

void logbook_list_open()
{
	// If the logbook window is already open and the user hits the
	// main window LOG button again, just raise the logbook window on top.
	if (logbook_window != NULL) {
		gtk_window_present(GTK_WINDOW(logbook_window));
		return;
	}

	// Create a logbook_window
	GtkWidget* scrolled_window;
	logbook_window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	gtk_window_set_title(GTK_WINDOW(logbook_window), "Logbook");
	g_signal_connect(logbook_window, "destroy", G_CALLBACK(logbook_close), NULL);
	g_signal_connect(logbook_window, "key-press-event", G_CALLBACK(logbook_key_press), NULL);
	gtk_container_set_border_width(GTK_CONTAINER(logbook_window), 10);
	gtk_window_set_default_size(GTK_WINDOW(logbook_window), 780, 400); // Set initial logbook_window size

	// Create a box to hold the elements
	GtkWidget* vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
	gtk_container_add(GTK_CONTAINER(logbook_window), vbox);

	// Create a toolbar
	GtkWidget* toolbar = gtk_toolbar_new();
	gtk_box_pack_start(GTK_BOX(vbox), toolbar, FALSE, FALSE, 0);
	// gtk_widget_override_background_color(toolbar, GTK_STATE_FLAG_NORMAL, &(GdkRGBA){0, 0, 0, 1});

	// Create search entry
	GtkWidget* search_entry = gtk_entry_new();
	GtkToolItem* entry_tool_item = gtk_tool_item_new();
	gtk_container_add(GTK_CONTAINER(entry_tool_item), search_entry);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), entry_tool_item, -1);

	/*
		// Create "Search" button
		GtkWidget *search_button = gtk_button_new_with_label("Search");
		GtkToolItem *search_tool_item = gtk_tool_item_new();
		gtk_container_add(GTK_CONTAINER(search_tool_item), search_button);
		gtk_toolbar_insert(GTK_TOOLBAR(toolbar), search_tool_item, -1);
	*/
	// Create "Edit..." button
	GtkWidget* edit_button = gtk_button_new_with_label("Edit...");
	GtkToolItem* edit_tool_item = gtk_tool_item_new();
	gtk_container_add(GTK_CONTAINER(edit_tool_item), edit_button);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), edit_tool_item, -1);

	// Create "Delete" button
	GtkWidget* delete_button = gtk_button_new_with_label("Delete");
	GtkToolItem* delete_tool_item = gtk_tool_item_new();
	gtk_container_add(GTK_CONTAINER(delete_tool_item), delete_button);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), delete_tool_item, -1);

	// Create "Import" button  - W9JES
	//	GtkWidget *import_button = gtk_button_new_with_label("Import...");
	//	GtkToolItem *import_tool_item = gtk_tool_item_new();
	//	gtk_container_add(GTK_CONTAINER(import_tool_item), import_button);
	//	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), import_tool_item, -1);

	// Create "Export" button	- W9JES
	GtkWidget* export_button = gtk_button_new_with_label("Export...");
	GtkToolItem* export_tool_item = gtk_tool_item_new();
	gtk_container_add(GTK_CONTAINER(export_tool_item), export_button);
	gtk_toolbar_insert(GTK_TOOLBAR(toolbar), export_tool_item, -1);

	// Create a scrolled window
	scrolled_window = gtk_scrolled_window_new(NULL, NULL);
	gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled_window),
		GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
	gtk_box_pack_start(GTK_BOX(vbox), scrolled_window, TRUE, TRUE, 0);

	// Create a list store
	if (!list_store)
		list_store = gtk_list_store_new(14, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING,
			G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING,
			G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING, G_TYPE_STRING);
	else
		clear_tree(list_store);

	// Create a tree view and set up columns with headings aligned to the left
	tree_view = gtk_tree_view_new_with_model(GTK_TREE_MODEL(list_store));
	const char *headings[] = {"#", "Date", "Freq", "Mode", "Call", "Sent", "Exch",
			"Recv", "Exch", "Tx Pwr", "SWR", "xOTA", "xOTA Loc", "Comments"};
	for (int i = 0; i < 14; ++i) {
		GtkCellRenderer *renderer = gtk_cell_renderer_text_new();
		GtkTreeViewColumn *column = gtk_tree_view_column_new_with_attributes(headings[i], renderer,
			"text", i, NULL);
		gtk_tree_view_column_set_alignment(column, 0.0); // Set alignment to the left (0.0)
		gtk_tree_view_append_column(GTK_TREE_VIEW(tree_view), column);
	}

	// Connect buttons to handlers, passing tree_view as user_data
	g_signal_connect(edit_button, "clicked", G_CALLBACK(edit_button_clicked), tree_view);
	g_signal_connect(delete_button, "clicked", G_CALLBACK(delete_button_clicked), tree_view);
	g_signal_connect(search_entry, "changed", G_CALLBACK(search_update), tree_view);
	// These handlers take logbook_window as user_data
	//	g_signal_connect(import_button, "clicked", G_CALLBACK(import_button_clicked), logbook_window);  - W9JES
	g_signal_connect(export_button, "clicked", G_CALLBACK(export_button_clicked), logbook_window); // W9JES
	/*
		// Apply CSS for tree view
		GtkCssProvider *cssProvider = gtk_css_provider_new();
		gtk_css_provider_load_from_data(cssProvider,
		   "treeview { background-color: black; color: white; font: 10pt sans-serif; }\n"
		   "treeview row:selected { background-color: blue; color: white; border: solid 1px white;}\n",
		   -1, NULL);
		GtkStyleContext *context = gtk_widget_get_style_context(GTK_WIDGET(tree_view));
		gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(cssProvider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
		g_object_unref(cssProvider);
	*/
	// Add tree view to scrolled window
	gtk_container_add(GTK_CONTAINER(scrolled_window), tree_view);
	clear_tree(list_store);
	logbook_fill(0, 10000, NULL);

	// Connect row activation signal
	//		gtk_tree_view_set_activate_on_single_click((GtkTreeView *)tree_view, FALSE);
	//	g_signal_connect(tree_view, "row-activated", G_CALLBACK(on_row_activated), NULL);

	// Enable row selection
	selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(tree_view));
	gtk_tree_selection_set_mode(selection, GTK_SELECTION_SINGLE);

	//	g_signal_connect(selection, "changed", G_CALLBACK(on_selection_changed), tree_view);

	// Show all widgets
	gtk_widget_show_all(logbook_window);
}
