// Common HAMLIB error codes
typedef enum
{
    HAMLIB_OK = 0,
    HAMLIB_ERR_INVALID_PARAM = -11,
    HAMLIB_ERR_PROTOCOL = -12,
    HAMLIB_ERR_NOT_IMPLEMENTED = -13
} hamlib_error_t;


// FIELD TYPES USED IN THE GTK FIELDS
#define FIELD_NUMBER 0
#define FIELD_BUTTON 1
#define FIELD_TOGGLE 2
#define FIELD_SELECTION 3
#define FIELD_TEXT 4
#define FIELD_STATIC 5
#define FIELD_CONSOLE 6
// Function prototypes for hamlib.c

// Command handling functions
static int hamlib_set_freq(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_freq(int cs, int is_extended, char *argv[], int argc);
static int hamlib_set_mode(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_mode(int cs, int is_extended, char *argv[], int argc);
static int hamlib_set_vfo(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_vfo(int cs, int is_extended, char *argv[], int argc);
static int hamlib_set_split(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_split(int cs, int is_extended, char *argv[], int argc);
static int hamlib_set_func(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_func(int cs, int is_extended, char *argv[], int argc);
static int hamlib_set_level(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_level(int cs, int is_extended, char *argv[], int argc);
static int hamlib_set_ptt(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_ptt(int cs, int is_extended, char *argv[], int argc);
static int hamlib_dump_state(int cs, int is_extended);
static int hamlib_get_rig_info(int client_socket, int is_extended, char *argv[], int argc);
static int hamlib_get_powerstat(int cs, int is_extended);
static int hamlib_send_cmd_raw(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_vfo_info(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_trn(int cs, int is_extended, char *argv[], int argc);
static int hamlib_get_clock(int cs, int is_extended, char *argv[], int argc);

// Helper functions
void send_rprt(int client_socket, int code);
void send_response(int client_socket, char *response);
int get_max_sd();
void handle_client_data(int client_socket, const char *data, ssize_t len);
void start_hamlib_listener();
void stop_hamlib_listener();
void *start_listener_thread(void *arg);
void initialize_hamlib();