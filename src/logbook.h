#define _XOPEN_SOURCE
#include <time.h>

void logbook_add(char *contact_callsign, char *rst_sent, char *exchange_sent,
	char *rst_recv, char *exchange_recv, int tx_power, int tx_vswr,
	char *xota, char *xota_loc, char *comments);
int logbook_query(char *query, int from_id, char *result_file);
int logbook_count_dup(const char *callsign, int last_seconds);
int logbook_prev_log(const char *callsign, char *result);
int logbook_get_grids(void (*f)(char *,int));
void logbook_list_open();
void logbook_open();
bool logbook_grid_exists(char *id);
bool logbook_caller_exists(char * id);

// ADIF export
// start_date can be null or empty if you want all records, unrestricted
void *prepare_query_by_date(const char *start_date, const char *end_date);
bool logbook_next(void *stmt);
void logbook_end_query(void *stmt);
int write_adif_header(char *buf, int len, const char *source);
int write_adif_record(void *stmt, char *buf, int len);
int export_adif(const char *path, const char *start_date, const char *end_date, const char *source);
