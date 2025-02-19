

enum rig_errcode_e {
    RIG_OK = 0,     /*!< 0 No error, operation completed successfully */
    RIG_EINVAL,     /*!< 1 invalid parameter */
    RIG_ECONF,      /*!< 2 invalid configuration (serial,..) */
    RIG_ENOMEM,     /*!< 3 memory shortage */
    RIG_ENIMPL,     /*!< 4 function not implemented, but will be */
    RIG_ETIMEOUT,   /*!< 5 communication timed out */
    RIG_EIO,        /*!< 6 IO error, including open failed */
    RIG_EINTERNAL,  /*!< 7 Internal Hamlib error, huh! */
    RIG_EPROTO,     /*!< 8 Protocol error */
    RIG_ERJCTED,    /*!< 9 Command rejected by the rig */
    RIG_ETRUNC,     /*!< 10 Command performed, but arg truncated */
    RIG_ENAVAIL,    /*!< 11 Function not available */
    RIG_ENTARGET,   /*!< 12 VFO not targetable */
    RIG_BUSERROR,   /*!< 13 Error talking on the bus */
    RIG_BUSBUSY,    /*!< 14 Collision on the bus */
    RIG_EARG,       /*!< 15 NULL RIG handle or any invalid pointer parameter in get arg */
    RIG_EVFO,       /*!< 16 Invalid VFO */
    RIG_EDOM,       /*!< 17 Argument out of domain of func */
    RIG_EDEPRECATED,/*!< 18 Function deprecated */
    RIG_ESECURITY,  /*!< 19 Security error */
    RIG_EPOWER,     /*!< 20 Rig not powered on */
    RIG_ELIMIT,     /*!< 21 Limit exceeded */
    RIG_EACCESS,    /*!< 22 Access denied -- e.g. port already in use */
    RIG_EEND        // MUST BE LAST ITEM IN LAST
};

// Common HAMLIB error codes
typedef enum
{
    HAMLIB_OK = 0,
    HAMLIB_ERR_INVALID_PARAM = -RIG_EINVAL,
    HAMLIB_ERR_PROTOCOL = -RIG_EPROTO,
    HAMLIB_ERR_NOT_IMPLEMENTED = -RIG_ENIMPL
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
static void send_response(int client_socket,  char *response);
static int get_max_sd();
static void handle_client_data(int client_socket, const char *data, ssize_t len);
void start_hamlib_listener();
void stop_hamlib_listener();
static void *start_listener_thread(void *arg);
void initialize_hamlib();

// Reflector Audio Services
//extern int start_reflector_service(const char *ip);
/* Call this to stop the reflector service.
   This will signal the service thread to exit and wait for it to finish. */
//extern void stop_reflector_service(void);

/* Returns true if the reflector service is currently running. */
//extern bool is_reflector_service_running(void);