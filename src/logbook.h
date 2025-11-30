#define _XOPEN_SOURCE
#include <time.h>

void logbook_add(const char *contact_callsign, const char *rst_sent, const char *exchange_sent,
	const char *rst_recv, const char *exchange_recv, int tx_power, int tx_vswr,
	const char *xota, const char *xota_loc, const char *comments);
int logbook_query(char *query, int from_id, char *result_file);
int logbook_count_dup(const char *callsign, int last_seconds);
int logbook_prev_log(const char *callsign, char *result);
int logbook_get_grids(void (*f)(char *,int));
void logbook_list_open();
void logbook_open();
time_t logbook_grid_last_qso(const char *id, int len);
time_t logbook_last_qso(const char * callsign, int len);

// ADIF export
// start_date can be null or empty if you want all records, unrestricted
void *prepare_query_by_date(const char *start_date, const char *end_date);
bool logbook_next(void *stmt);
void logbook_end_query(void *stmt);
int write_adif_header(char *buf, int len, const char *source);
int write_adif_record(void *stmt, char *buf, int len);
int export_adif(const char *path, const char *start_date, const char *end_date, const char *source);
