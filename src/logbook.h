void logbook_add(char *contact_callsign, char *rst_sent, char *exchange_sent,
char *rst_recv, char *exchange_recv, char *comments);
int logbook_query(char *query, int from_id, char *result_file);
int logbook_count_dup(const char *callsign, int last_seconds);
int logbook_prev_log(const char *callsign, char *result);
int logbook_get_grids(void (*f)(char *,int));
void logbook_list_open();
void logbook_open();
bool logbook_grid_exists(char *id);
bool logbook_caller_exists(char * id);
