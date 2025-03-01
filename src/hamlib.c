/*****************************************************************************
 * hamlib.c
 *
 * A self-contained Hamlib-compatible TCP server that:
 *   - Listens on port 4532 (non-blocking + select).
 *   - Parses Single-letter (e.g. "F 14000000"), Textual (e.g. "\set_freq 14000000"),
 *     and Extended commands (e.g. "+\set_freq 14000000").
 *   - Has a PROPERTY DICTIONARY for levels (RFPOWER, MICGAIN, COMP, etc.)
 *     and for boolean functions (NB, DSP, TUNER, etc.) if you like.
 *   - Contains embedded support for "MODEL 2" standard RIGCTL functions and levels (remaps them to ours)
 *   - Merges set_func, get_func, set_level, get_level to optionally consult
 *     the property table for validation .
 *
 * HOW TO ADD NEW RIGCTL and RIGTLD COMMANDS:
 *   1) In the command_id_t enum, add an entry (e.g. CMD_SET_XIT).
 *   2) In parse_command_name(), map single-letter or textual tokens (e.g. "Z" / "set_xit").
 *   3) Write a new function (e.g. hamlib_set_xit()) containing:
 *       - rigctld doc snippet in a comment,
 *       - optional extended output if 'is_extended' is set,
 *       - the actual logic (cmd_exec(...), field_set(...), or property lookups, etc.).
 *   4) In interpret_line()'s switch, add a case for CMD_SET_XIT calling hamlib_set_xit().
 *
 * HOW TO ADD NEW PROPERTIES:
 *   1) In property_table[], add a line, e.g.
 *        { "ANR", PT_BOOLEAN, 0.0f, 1.0f, 0 },
 *      for a boolean, or
 *        { "RFPOWER", PT_FLOAT, 0.0f, 100.0f, 0 },
 *      for a numeric range.
 *   2) If you have a required func name that needs to be mapped to an internal field name, then
 *   add a line to property_mapping_table[], e.g.
 *     { "NR", "ANR" },
 *   2) If it is a "func" (boolean on/off), set PT_BOOLEAN and implement in sdr_radio_set_property()
 *      or sdr_radio_get_property() how to actually toggle the rig.
 *   3) Now set_func / get_func or set_level / get_level can reference the property table to do
 *      range checks or boolean checks automatically.
 *
 *   Currently this code does NOT support extended separators such as ";", "," and "|"
 *   however you can connect to rigctld  if you need this functionality.
 *
 *   Note that EXTENSIVE RANGE CHECKING NOT IMPLEMENTED YET,  BECAUSE SBITX fields do much of it automatically
 *
 *  NOTES on use...
 *  1. If you connect directly to this library on the IP address and port 4532, it acts like a rigctld daemon and ALL SBITX properties are available to you access
 *  2. If you run the rigctld daemon to connect to this library and connect to IT instead you'll be LIMITED to "model=2" commands and properties
 *     which are a subset of the full set of properties available here, but are standard for most rigs and many radio control apps like "CATRADIO"
 *
 *****************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <complex.h>
#include <math.h>
#include <fcntl.h>
#include <complex.h>
#include <fftw3.h>
#include <pthread.h>
#include "sdr.h"
#include "sdr_ui.h"
#include <stdbool.h>

#define RIG_H // Avoids redefining the struct field

#include "rig.h"
#include "hamlib.h"

#define PORT         4532
#define PRODUCT      "SBITX v3 Hybrid SDR"
#define VERSION      "4.3"
#define MAX_CLIENTS  10
#define MAX_DATA     4000 // Extended response size required for handling CAPS-related commands
#define DEBUG        0
#define RIG_LEVEL_SCALAR 1


// Communication buffers
static int client_sockets[MAX_CLIENTS] = {0};
char client_ips[MAX_CLIENTS][INET_ADDRSTRLEN];

static int welcome_socket = -1;
static volatile int running = 1;
static pthread_mutex_t client_mutex = PTHREAD_MUTEX_INITIALIZER;

char incoming_data[MAX_CLIENTS][MAX_DATA];
int incoming_ptr[MAX_CLIENTS] = {0};

char resp[MAX_DATA] = {0};      // Used for the large responses
bool is_debug = DEBUG;          // Setting U DEBUG 1 through the socket turns on debug displays
bool is_locked = false;         // This is used to lock the radio from changing modes.
// WSJT-X and other apps query this to ensure the mode
// doesn't change while they have control.

/*****************************************************************************
 *  EXTERNALS (mainly from sbitx.c and sbitx_gtk):
 *****************************************************************************/
extern struct field *get_field_by_label(char *label);
extern long get_freq(void);
extern int get_default_passband_bw(void);
extern int get_passband_bw(void);
extern int field_int(char *label);
extern int field_set(const char *label, const char *new_value);
extern const char *field_str(const char *label);
extern int get_field_value_by_label(char *label, char *val);

extern void cmd_exec(char *cmd);
extern void remote_execute(char *cmd);
extern void sdr_request(char *cmd, char *out);
extern void hamlib_tx(int onoff);
extern void send_error_rprt(int client_socket, int code);

extern void send_response(int client_socket, char *response);

/*****************************************************************************
 * PROPERTY DICTIONARY
 * To store info for "func" or "level" commands, or both.
 * Note that the level commands support "?" argument to show a list of FLOAT values from this dictionary
 * Note that the func commands support "?" argument to show a list of BOOLEAN and STRING values from this dictionary
 *****************************************************************************/
typedef enum
{
    PT_FLOAT,
    PT_BOOLEAN,
    PT_STRING
} property_type_t;

typedef struct
{
    const char *name;         // e.g. "RFPOWER", "NB", "COMP", "MICGAIN"
    property_type_t type;     // float, boolean, or string
    float min_val;            // for numeric
    float max_val;            // for numeric
    int max_len;              // for strings if PT_STRING
} property_definition_t;

// Duplicating this to avoid including sdr_ui.h..
struct field
{
    char *cmd;

    int (*fn)();

    int x, y, width, height;
    char label[30];
    int label_width;
    char value[128];
    char value_type; // NUMBER, SELECTION, TEXT, TOGGLE, BUTTON
    int font_index;     // refers to font_style table
    char selection[1000];
    long int min, max;
    int step;
    int section;
    char is_dirty;
    char update_remote;
    void *data;
};


// Duplicating code from sbitx_gtk (note to teeam: which SHOULD be refactored into an .h file.)
#define STACK_DEPTH 4
struct band
{
    char name[10];
    int start;
    int stop;
    // int	power;
    // int	max;
    int index;
    int freq[STACK_DEPTH];
    int mode[STACK_DEPTH];
    // Make drive and IF band specific - n1qm
    int if_gain;
    int drive;
    // add tune power to band - W9JES
    int tnpwr;
};

/// BAND
// Declare the set_field function
extern void set_field(const char *field, const char *value);
// Declare external variables
#define NUM_BANDS 9 // Required because band_stack definitions are not yet in .h files.
extern struct band band_stack[NUM_BANDS];
extern const char *mode_name[];
extern void set_field(const char *field, const char *value);
extern const char *get_current_band();
extern void change_band(char *request);
int calculate_current_band();
int current_band_index;

// Provide a function to choose the execution mode for commands
// Which can either be queued or direct

void execute_command(char *cmd)
{
    if (is_debug)
        printf("[DEBUG] Executing command: %s\n", cmd);
    cmd_exec(cmd);
    ///remote_execute(cmd); // Queued not reliable
}

// Locates the current band string based on frequency
const char *get_current_band()
{
    current_band_index =  calculate_current_band();

    if (current_band_index < 0 || current_band_index >= NUM_BANDS)
    {
        return "Unknown";
    }

    return band_stack[current_band_index].name;
}

// Calculate the current_band's index based on the frequency
// Note this should eventually be modified to support multiple VFOs
int calculate_current_band()
{
    int freq_now = field_int("FREQ");
    int band_now = 1;
    for (int i = 0; i < sizeof(band_stack) / sizeof(struct band); i++)
    {
        if (band_stack[i].start <= freq_now && freq_now <= band_stack[i].stop)
            band_now = i;
    }
    return band_now;
}


typedef struct
{
    const char *external_name; // Hamlib property name
    const char *internal_name; // Radio's property name
} property_mapping_t;


// Supported properties in HAMLIB

static property_definition_t property_table[] = {
        {"DSP", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"ANR", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"NOTCH", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"TUNE", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"REC", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"KBD", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"TXEQ", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"RXEQ", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"VFOLK", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"AUTO", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"FT8_AUTO", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"FT8_TX1ST", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"RIT", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"SPLIT", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"RS", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"TA", PT_BOOLEAN, 0.0f, 1.0f, 0},
        // {"LOCK", PT_BOOLEAN, 0.0f, 1.0f, 0},
        {"RFPOWER", PT_FLOAT, 0.0f, 1.0f, 0},
        {"RFPOWER_METER", PT_FLOAT, 0.0f, 100.0f, 0},
        {"MICGAIN", PT_FLOAT, 0.0f, 100.0f, 0},
        {"COMP", PT_FLOAT, 0.0f, 100.0f, 0},
        {"AUDIO", PT_FLOAT, 0.0f, 100.0f, 0},
        {"VOLUME", PT_FLOAT, 0.0f, 100.0f, 0},
        {"BW", PT_FLOAT, 100.0f, 6000.0f, 0},
        {"DRIVE", PT_FLOAT, 0.0f, 100.0f, 0},
        {"IF", PT_FLOAT, 0.0f, 100.0f, 0},
        {"MIC", PT_FLOAT, 0.0f, 100.0f, 0},
        {"LOWCUT", PT_FLOAT, 50.0f, 5000.0f, 0},
        {"HIGHCUT", PT_FLOAT, 50.0f, 5000.0f, 0},
        {"WFMIN", PT_FLOAT, 0.0f, 200.0f, 0},
        {"WFSPD", PT_FLOAT, 20.0f, 150.0f, 0},
        {"WFMAX", PT_FLOAT, 0.0f, 200.0f, 0},
        {"SCOPEGAIN", PT_FLOAT, 1.0f, 25.0f, 0},
        {"SCOPESIZE", PT_FLOAT, 50.0f, 150.0f, 0},
        {"INTENSITY", PT_FLOAT, 2.0f, 10.0f, 0},
        {"NFREQ", PT_FLOAT, 60.0f, 3000.0f, 0},
        {"BNDWTH", PT_FLOAT, 60.0f, 1000.0f, 0},
        {"TNDUR", PT_FLOAT, 2.0f, 30.0f, 0},
        {"TNPWR", PT_FLOAT, 0.0f, 100.0f, 0},
        {"TXMON", PT_FLOAT, 0.0f, 100.0f, 0},
        {"BFO", PT_FLOAT, -2995.0f, 3000.0f, 0},
        {"TX_PITCH", PT_FLOAT, 0.0f, 5000.0f, 0},
        {"FT8_REPEAT", PT_FLOAT, 0.0f, 10.0f, 0},
        {"PITCH", PT_FLOAT, -5000.0f, 5000.0f, 0},
        {"WPM", PT_FLOAT, 0.0f, 100.0f, 0},
        {"CW_DELAY", PT_FLOAT, 0.0f, 100.0f, 0},
        {"METER", PT_FLOAT, 0.0f, 1500.0, 0},
        {"STRENGTH", PT_FLOAT, -54.0f, 100.0, 0},
        {"SWR", PT_FLOAT, 0.0f, 100.0, 0},
        {"REF", PT_FLOAT, 0.0f, 100.0, 0},
        {"POWER", PT_FLOAT, 0.0f, 1.0, 0},
        {"RIT", PT_STRING, 0.0f, 100.0f, 0},
        {"STEP", PT_STRING, 0.0f, 100.0f, 0},
        {"MENU", PT_STRING, 0.0f, 100.0f, 0},
        {"SPLIT", PT_STRING, 0.0f, 100.0f, 0},
        {"VFO", PT_STRING, 0.0f, 100.0f, 0},
        {"SPAN", PT_STRING, 0.0f, 100.0f, 0},
        {"SPECT", PT_STRING, 0.0f, 100.0f, 0},
        {"MODE", PT_STRING, 0.0f, 100.0f, 0},
        {"CW_INPUT", PT_STRING, 0.0f, 100.0f, 0},
        {"CALL", PT_STRING, 0.0f, 100.0f, 0},
        {"SENT", PT_STRING, 0.0f, 100.0f, 0},
        {"RECV", PT_STRING, 0.0f, 100.0f, 0},
        {"EXCH", PT_STRING, 0.0f, 100.0f, 0},
        {"F1", PT_STRING, 0.0f, 100.0f, 0},
        {"F2", PT_STRING, 0.0f, 100.0f, 0},
        {"F3", PT_STRING, 0.0f, 100.0f, 0},
        {"F4", PT_STRING, 0.0f, 100.0f, 0},
        {"F5", PT_STRING, 0.0f, 100.0f, 0},
        {"F6", PT_STRING, 0.0f, 100.0f, 0},
        {"F7", PT_STRING, 0.0f, 100.0f, 0},
        {"F8", PT_STRING, 0.0f, 100.0f, 0},
        {"F9", PT_STRING, 0.0f, 100.0f, 0},
        {"F10", PT_STRING, 0.0f, 100.0f, 0},
        {NULL, 0, 0.0f, 0.0f, 0} /* sentinel */
};

static property_mapping_t property_mapping_table[] = {
        {"LOWCUT", "LOW"},
        {"HIGHCUT", "HIGH"},
        {"METER", "STRENGTH"},
        {"VOLUME", "AUDIO"},
        {"RFPOWER_METER", "POWER"},
        {"NOTCHF", "NOTCH"},
        {"NOTCHF_RAW", "NOTCH"},
        {"ANF", "NOTCH"},
        {"CWPITCH", "PITCH"},
        {"KEYSPD", "WPM"},
        {"AGC", "FAGC"},
        {"LOCK", "VFOLK"},
        {NULL, NULL} /* sentinel */
};

// Remap property names if required, you'll find this in the level and func functions...
const char *resolve_property_name(const char *external_name)
{
    for (int i = 0; property_mapping_table[i].external_name; ++i)
    {
        if (strcmp(property_mapping_table[i].external_name, external_name) == 0)
        {
            return property_mapping_table[i].internal_name; // Return internal name
        }
    }
    return external_name; // Default to external name if no mapping exists
}

/*****************************************************************************
 * These two functions use the radio's internal property dictionary to set or get values.
 * They can then be mapped to OUR property dictionary (which is geared towards rigctl).
 *****************************************************************************/

static bool sdr_radio_set_property(const char *prop_name, const char *value_str)
{
//    printf("[DEBUG] sdr_radio_set_property: prop_name=%s, value_str=%s\n", prop_name, value_str);
    // TODO: Optimize this eventually using a hash table or something.

    for (int i = 0; property_table[i].name; i++)
    {
        if (strcasecmp(property_table[i].name, prop_name) == 0)
        {
            // Handle BOOLEAN
            if (property_table[i].type == PT_BOOLEAN)
            {
                /* must be '0' or '1' => turn ON or OFF. We could do: */
                if (!strcmp(value_str, "1"))
                {

                    // turn on
                    if (is_debug)
                    {
                        printf("[DEBUG] Setting boolean %s=ON\n", prop_name);
                    }
                    field_set(prop_name, (char *) "ON");  // or "1"
                }
                else
                {
                    if (is_debug)
                        printf("[DEBUG] Setting boolean %s=OFF\n", prop_name);
                    field_set(prop_name, (char *) "OFF");
                }
                return true;
            }
                // Handle FLOAT
            else if (property_table[i].type == PT_FLOAT)
            {
                float val = (float) atof(value_str);
                if (val < property_table[i].min_val || val > property_table[i].max_val)
                {
                    if (is_debug)
                        printf("[ERROR] %s out of range.\n", prop_name);
                    return false;
                }
                // Set a flow value
                char buf[32];
                sprintf(buf, "%.2f", val);
                field_set(prop_name, buf);
                if (is_debug)
                    printf("[DEBUG] Setting float property %s=%.2f\n", prop_name, val);
                return true;
            }

                // Handle STRING
            else if (property_table[i].type == PT_BOOLEAN || property_table[i].type == PT_STRING)
            {
                /* check length, etc. */
                if ((int) strlen(value_str) > property_table[i].max_len)
                {
                    if (is_debug)
                        printf("[ERROR] String too long for %s\n", prop_name);
                    return false;
                }
                field_set(prop_name, (char *) value_str);
                return true;
            }
        }
    }
    /* if not found in property table, fallback to handlers */
    return false;
}

static bool sdr_radio_get_property(const char *prop_name, char *out_buf, size_t out_sz)
{
    // printf("[DEBUG] sdr_radio_get_property: prop_name=%s\n", prop_name);
    /* We do the reverse: read from 'field_*' or some other store. */
    for (int i = 0; property_table[i].name; i++)
    {
        if (strcasecmp(property_table[i].name, prop_name) == 0)
        {
            // Handle BOOLEAN
            if (property_table[i].type == PT_BOOLEAN)
            {
                /* read if ON or OFF from field. */
                char val[64];
                if (get_field_value_by_label((char *) prop_name, val) == -1)
                    return false;
                if (!strcmp(val, "ON") || strcmp(val, "1") == 0)
                    snprintf(out_buf, out_sz, "1");
                else
                    snprintf(out_buf, out_sz, "0");
                return true;
            }
                // Handle FLOAT
            else if (property_table[i].type == PT_FLOAT)
            {
                /* read from field, parse as float. */
                char val[64];
                if (get_field_value_by_label((char *) prop_name, val) == -1)
                    return false;
                /* e.g. "50.00" => store in out_buf. */
                snprintf(out_buf, out_sz, "%s", val);
                return true;
            }
                // Handle STRING
            else if (property_table[i].type == PT_STRING)
            {
                /* read as string. */
                char val[128];
                if (get_field_value_by_label((char *) prop_name, val) == -1)
                    return false;
                snprintf(out_buf, out_sz, "%s", val);
                return true;
            }
        }
    }
    return false; /* not found */
}

/*****************************************************************************
 * send_response + "RPRT x" helper (Only to be used BEFORE normal response functions  used
 * is used, otherwise risk of multiple packets sent for one message)
 *****************************************************************************/

void send_error_rprt(int client_socket, int code)
{
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "RPRT %d\n", code);
    send_response(client_socket, tmp);
}

/*****************************************************************************
 * LINE MODE DETECTION (Extended/Textual/Single)
 *****************************************************************************/
typedef enum
{
    LINE_MODE_INVALID = 0,
    LINE_MODE_SINGLE,
    LINE_MODE_TEXTUAL,
    LINE_MODE_EXTENDED
} line_mode_t;

static line_mode_t parse_line_mode(char *line)
{
    if (line == (char *) 0 || !line[0]) return LINE_MODE_INVALID;

    if (line[0] == '+' && line[1] == '\\')
    {
        memmove(line, line + 2, strlen(line));
        return LINE_MODE_EXTENDED;
    }
    if(line[0] == '+')
    {
        memmove(line, line + 1, strlen(line));
        return LINE_MODE_EXTENDED;
    }

    if (line[0] == '\\')
    {
        memmove(line, line + 1, strlen(line));
        return LINE_MODE_TEXTUAL;
    }
    return LINE_MODE_SINGLE;
}

/*****************************************************************************
 * COMMAND ENUM + parse_command_name
 *****************************************************************************/
typedef enum
{
    CMD_INVALID = 0,

    /* Frequencies */
    CMD_SET_FREQ,
    CMD_GET_FREQ,

    /* Mode */
    CMD_SET_MODE,
    CMD_GET_MODE,
    CMD_GET_TRN,

    /* VFO */
    CMD_SET_VFO,
    CMD_GET_VFO,

    /* Split */
    CMD_SET_SPLIT,
    CMD_GET_SPLIT,

    /* Functions (U/u) => set_func, get_func */
    CMD_SET_FUNC,
    CMD_GET_FUNC,

    /* Levels (L/l) => set_level, get_level */
    CMD_SET_LEVEL,
    CMD_GET_LEVEL,

    /* PTT (T/t) => set_ptt, get_ptt */
    CMD_SET_PTT,
    CMD_GET_PTT,

    /* Dump state (D) => \dump_state */
    CMD_DUMP_STATE,
    CMD_GET_RIG_INFO,

    /* Power stat (P) => get_powerstat */
    CMD_GET_POWERSTAT,

    /* w => send_cmd raw */
    CMD_SEND_CMD_RAW,

    /* Lock */
    CMD_SET_LOCK,
    CMD_GET_LOCK,

    /* Get VFO info */
    CMD_GET_VFO_INFO,

    /* Get the rig clock time */
    CMD_GET_CLOCK,

    /* Q => quit */
    CMD_QUIT,

    // Dump caps
    CMD_DUMP_CAPS,
    // Check vdo options

    CMD_CHK_VFO,
    // Handle vfo ops
    CMD_VFO_OP,

    // Set RIT, Get RIT
    CMD_SET_RIT,
    CMD_GET_RIT,


    // Unknown command
    CMD_UNKNOWN
} command_id_t;

// Parse a command name from a string to a token, e.g. "set_freq" => CMD_SET_FREQ
// The main reason for this is to allow a simple switch statement to process all the commands
// despite their 3 different formats.

static command_id_t parse_command_name(const char *cmd_str)
{
    if (!cmd_str || !cmd_str[0]) return CMD_INVALID;

    if (strcmp(cmd_str, "F") == 0 || strcasecmp(cmd_str, "set_freq") == 0)
        return CMD_SET_FREQ;
    if (strcmp(cmd_str, "f") == 0 || strcasecmp(cmd_str, "get_freq") == 0)
        return CMD_GET_FREQ;

    if (strcmp(cmd_str, "M") == 0 || strcasecmp(cmd_str, "set_mode") == 0)
        return CMD_SET_MODE;
    if (strcmp(cmd_str, "m") == 0 || strcasecmp(cmd_str, "get_mode") == 0)
        return CMD_GET_MODE;

    if (strcmp(cmd_str, "V") == 0 || strcasecmp(cmd_str, "set_vfo") == 0)
        return CMD_SET_VFO;
    if (strcmp(cmd_str, "v") == 0 || strcasecmp(cmd_str, "get_vfo") == 0)
        return CMD_GET_VFO;

    if (strcmp(cmd_str, "S") == 0 || strcasecmp(cmd_str, "set_split_vfo") == 0)
        return CMD_SET_SPLIT;
    if (strcmp(cmd_str, "s") == 0 || strcasecmp(cmd_str, "get_split_vfo") == 0)
        return CMD_GET_SPLIT;

    if (strcmp(cmd_str, "U") == 0 || strcasecmp(cmd_str, "set_func") == 0)
        return CMD_SET_FUNC;
    if (strcmp(cmd_str, "u") == 0 || strcasecmp(cmd_str, "get_func") == 0)
        return CMD_GET_FUNC;

    if (strcmp(cmd_str, "L") == 0 || strcasecmp(cmd_str, "set_level") == 0)
        return CMD_SET_LEVEL;
    if (strcmp(cmd_str, "l") == 0 || strcasecmp(cmd_str, "get_level") == 0)
        return CMD_GET_LEVEL;

    if (strcmp(cmd_str, "T") == 0 || strcasecmp(cmd_str, "set_ptt") == 0)
        return CMD_SET_PTT;
    if (strcmp(cmd_str, "t") == 0 || strcasecmp(cmd_str, "get_ptt") == 0)
        return CMD_GET_PTT;

    if (strcmp(cmd_str, "D") == 0 || strcasecmp(cmd_str, "dump_state") == 0)
        return CMD_DUMP_STATE;

    if (strcmp(cmd_str, "P") == 0 || strcasecmp(cmd_str, "get_powerstat") == 0)
        return CMD_GET_POWERSTAT;
    if (cmd_str[0] == (char) 0xf3 || strcasecmp(cmd_str, "get_vfo_info") == 0)
        return CMD_GET_VFO_INFO;

    if (strcmp(cmd_str, "w") == 0 || strcasecmp(cmd_str, "W") == 0 ||
        strcasecmp(cmd_str, "send_cmd") == 0)
        return CMD_SEND_CMD_RAW;

    if (strcmp(cmd_str, "Q") == 0 || strcasecmp(cmd_str, "q") == 0 ||
        strcasecmp(cmd_str, "quit") == 0)
        return CMD_QUIT;

    if (cmd_str[0] == (char) 0xf5 || strcasecmp(cmd_str, "get_rig_info") == 0)
        return CMD_GET_RIG_INFO;

    if (strcmp(cmd_str, "a") == 0 || strcasecmp(cmd_str, "get_trn") == 0)
        return CMD_GET_TRN;

    if (strcasecmp(cmd_str, "set_lock_mode") == 0)
        return CMD_SET_LOCK;

    if (strcasecmp(cmd_str, "get_lock_mode") == 0)
        return CMD_GET_LOCK;

    if (strcmp(cmd_str, "1") == 0 || strcasecmp(cmd_str, "dump_caps") == 0)
        return CMD_DUMP_CAPS;

    if (strcasecmp(cmd_str, "get_clock") == 0)
        return CMD_GET_CLOCK;

    if (strcasecmp(cmd_str, "chk_vfo") == 0)
        return CMD_CHK_VFO;

    if (strcmp(cmd_str, "G") == 0 || strcasecmp(cmd_str, "vfo_op") == 0)
        return CMD_VFO_OP;

    if (strcmp(cmd_str, "J") == 0 || strcasecmp(cmd_str, "set_rit") == 0)
        return CMD_SET_RIT;

    if (strcmp(cmd_str, "j") == 0 || strcasecmp(cmd_str, "get_rit") == 0)
        return CMD_GET_RIT;


    return CMD_UNKNOWN;
}


// One response buffer per client context.
char client_response_buffers[MAX_CLIENTS][MAX_DATA];

// Helper: Given a client socket, return its index in client_response_buffers.
static int get_client_index(int cs)
{
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (client_sockets[i] == cs)
            return i;
    }
    return -1;  // Not found.
}

/// RESPONSE BUFFER MANAGEMENT
/// These functions are used to manage the response buffer for each client in order to ensure
/// that the response is sent in a single packet. This is important for rigctl compatibility.
///

// Call this to clear the response buffer for the client and begin a new response packet
void begin_new_response(int cs)
{
    int idx = get_client_index(cs);
    if (idx != -1)
    {
        client_response_buffers[idx][0] = '\0';
    }
}

// Instead of sending immediately, add data to the client's response buffer.
void add_response(int cs, const char *data)
{
    int idx = get_client_index(cs);
    if (idx != -1)
    {
        strncat(client_response_buffers[idx],
                data,
                MAX_DATA - strlen(client_response_buffers[idx]) - 1);
    }
}

// When done building the response, send it all at once.
void flush_response(int cs)
{
    int idx = get_client_index(cs);
    if (idx != -1)
    {
        int len = strlen(client_response_buffers[idx]);
        if (len > 0)
        {
            if (is_debug)
                printf("response to client %d: %s\n", cs, client_response_buffers[idx]);
            if (send(cs, client_response_buffers[idx], len, 0) < 0)
            {
                perror("send");
                close(cs);
                client_sockets[idx] = 0;
            }
        }
        // Clear the buffer after sending.
        client_response_buffers[idx][0] = '\0';
    }
}



/*****************************************************************************
 * Now the hamlib_* functions, each with rigctld doc snippet and logic.
 * plus property dictionary and mapping calls where relevant.
 *****************************************************************************/

// Dump Caps - This is a special command that is used to dump the capabilities of the radio.
// Note that this caps is not typically used by support apps, but is useful for debugging.
// Most apps use dump_state instead.

const char dump_caps_response[] =
        "Rig command: \\dump_caps\n"
        "Caps dump for model: \t2\n"
        "Model name: \tSBITX\n"
        "Hamlib version: \t4.0\n"
        "Mfg name:\tHF Signals\n"
        "Backend version:\tv3\n"
        "Backend copyright: \tMIT\n"
        "Backend status:\tStable\n"
        "Rig type:\tTransceiver\n"
        "PTT type:\tRig capable\n"
        "DCD type:\tNot supported\n"
        "Port type:\tNetwork\n"
        "Write delay:\t0ms, timeout 5000ms, 3 retry\n"
        "Post Write delay:\t0ms\n"
        "Has targetable VFO:\tPTT FREQ MODE\n"
        "Has transceive:\tNo\n"
        "Announce:\t0x0\n"
        "Max RIT:\t-100.0kHz/+100.0kHz\n"
        "Max IF-SHIFT:\t-0.0kHz/+0.0kHz\n"
        "Preamp:\tNone\n"
        "Attenuator:\tNone\n"
        "CTCSS:\tNone\n"
        "DCS:\tNone\n"
        "Get functions:\tDSP ANR NOTCH TUNE REC KBD TXEQ RXEQ LOCK VFOLK AUTO KBD FT8_AUTO FT8_TX1ST RIT SPLIT RS TA AGC STEP MENU SPLIT VFO SPAN SPECT MODE CW_INPUT CALL SENT RECV EXCH NR MENU F1 F2 F3 F4 F5 F6 F7 F8 F9 F10 \n"
        "Set functions:\tDSP ANR NOTCH TUNE REC KBD TXEQ RXEQ LOCK VFOLK AUTO KBD FT8_AUTO FT8_TX1ST RIT SPLIT RS TA AGC STEP MENU SPLIT VFO SPAN SPECT MODE CW_INPUT CALL SENT RECV EXCH NR MENU F1 F2 F3 F4 F5 F6 F7 F8 F9 F10 \n"
        "Get level:\tIF(0..100/1.0) RF(0..1.0/0.1) MICGAIN(0..100/1.0) STRENGTH(-11..40/1) \n"
        "Set level:\tIF(0..100/1.0) RF(0..1.0/0.1) MICGAIN(0..100/1.0) STRENGTH(-11..40/1) \n"
        "Extra levels:\tRFPOWER METER MICGAIN COMP AUDIO VOLUME BW DRIVE IF MIC LOWCUT HIGHCUT WFMIN WFSPD WFMAX SCOPEGAIN SCOPESIZE INTENSITY NFREQ BNDWTH TNDUR TNPWR TXMON BFO TX_PITCH FT8_REPEAT PITCH WPM CW_DELAY METER STRENGTH SWR REF POWER\n"
        "Get parameters:\tNone\n"
        "Set parameters:\tNone\n"
        "Extra parameters:\tNone\n"
        "Mode list:\tAM CW CWB LSB USB PKTUSB DIGI DIGITAL\n"
        "VFO list:\tVFOA VFOB\n"
        "VFO Ops:\tNone\n"
        "Scan Ops:\tNone\n"
        "Number of banks:\t0\n"
        "Memory name desc size:\t8\n"
        "Memories:\n"
        "    No memory banks defined\n"
        "TX ranges status, region 1:\tOK (0)\n"
        "RX ranges status, region 1:\tOK (0)\n"
        "TX ranges status, region 2:\tOK (0)\n"
        "RX ranges status, region 2:\tOK (0)\n"
        "Tuning steps:\n"
        "    1 Hz: CW CWB\n"
        "    Any: AM CW CWB LSB USB PKTUSB\n"
        "Tuning steps status:\tOK (0)\n"
        "Filters:\tNone\n"
        "Bandwidths:\tNone\n"
        "Has priv data:\tY\n"
        "Has Init:\tY\n"
        "Has Cleanup:\tY\n"
        "Has Open:\tN\n"
        "Has Close:\tN\n"
        "Can set Conf:\tN\n"
        "Can get Conf:\tN\n"
        "Can set Frequency:\tY\n"
        "Can get Frequency:\tY\n"
        "Can set Mode:\tY\n"
        "Can get Mode:\tY\n"
        "Can set VFO:\tY\n"
        "Can get VFO:\tY\n"
        "Can set PTT:\tY\n"
        "Can get PTT:\tY\n"
        "Can get DCD:\tN\n"
        "Can set Repeater Duplex:\tN\n"
        "Can get Repeater Duplex:\tN\n"
        "Can set Repeater Offset:\tN\n"
        "Can get Repeater Offset:\tN\n"
        "Can set Split Freq:\tY\n"
        "Can get Split Freq:\tY\n"
        "Can set Split Mode:\tN\n"
        "Can get Split Mode:\tN\n"
        "Can set Split VFO:\tY\n"
        "Can get Split VFO:\tY\n"
        "Can set Tuning Step:\tY\n"
        "Can get Tuning Step:\tY\n"
        "Can set RIT:\tY\n"
        "Can get RIT:\tY\n"
        "Can set XIT:\tN\n"
        "Can get XIT:\tN\n"
        "Can set CTCSS:\tN\n"
        "Can get CTCSS:\tN\n"
        "Can set DCS:\tN\n"
        "Can get DCS:\tN\n"
        "Can set CTCSS Squelch:\tN\n"
        "Can get CTCSS Squelch:\tN\n"
        "Can set DCS Squelch:\tN\n"
        "Can get DCS Squelch:\tN\n"
        "Can set Power Stat:\tN\n"
        "Can get Power Stat:\tY\n"
        "Can Reset:\tN\n"
        "Can get Ant:\tN\n"
        "Can set Ant:\tN\n"
        "Can set Transceive:\tN\n"
        "Can get Transceive:\tY\n"
        "Can set Func:\tY\n"
        "Can get Func:\tY\n"
        "Can set Level:\tY\n"
        "Can get Level:\tY\n"
        "Can set Param:\tN\n"
        "Can get Param:\tN\n"
        "Can send DTMF:\tN\n"
        "Can recv DTMF:\tN\n"
        "Can send Morse:\tN\n"
        "Can decode Events:\tY\n"
        "Can set Bank:\tN\n"
        "Can set Mem:\tN\n"
        "Can get Mem:\tN\n"
        "Can set Channel:\tN\n"
        "Can get Channel:\tN\n"
        "Can ctl Mem/VFO:\tN\n"
        "Can Scan:\tN\n"
        "Can get Info:\tY\n"
        "Can get power2mW:\tN\n"
        "Can get mW2power:\tN\n"
        "Overall backend warnings:\t0\n";


/// Optional command to dump the capabilities of the radio.
static int hamlib_dump_caps(int cs, int is_extended, char *argv[], int argc)
{
    begin_new_response(cs);

    // Extended mode header (if needed)
    if (is_extended)
    {
        add_response(cs, (char *) "dump_caps:\n");
    }
    add_response(cs, (char *) dump_caps_response);
    add_response(cs, (char *) "RPRT 0\n");
    flush_response(cs);
    return 0;
}

/*
 * F, set_freq 'Frequency'
 * "Set 'Frequency', in Hz."ƒ
 *ƒ
 */
static int hamlib_set_freq(int cs, int is_extended, char *argv[], int argc)
{
    //   printf("[DEBUG] hamlib_set_freq: argc=%d\n", argc);
    long freq;
    char cmd[50];

    if (argc < 1) return -1;

    begin_new_response(cs);

    if (is_extended)
    {
        long freq_val = atol(argv[0]);
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_freq %ld:\nFreq: %ld\n", freq_val, freq_val);
        add_response(cs, tmp);
    }

    const char *vfo = argv[0];

    if (!strncmp(vfo, "VFO", 3)) // Odd but possible as in F VFOA 14200000
        freq = atoi(vfo + 5);
    else
        freq = atol(vfo);

    sprintf(cmd, "freq %ld", freq);
    execute_command(cmd);

    if (!is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

/*
 * f, get_freq
 * "Get 'Frequency', in Hz."
 */
static int hamlib_get_freq(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_freq: argc=%d\n", argc);
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_freq:\n");
    }
    char response[20];
    if (is_extended)
    {
        sprintf(response, "Freq: %ld\n", get_freq());
    }
    else
    {
        sprintf(response, "%ld\n", get_freq());
    }

    add_response(cs, response);
    if (is_extended)
    {
        add_response(cs, "RPRT 0\n");
    }


    flush_response(cs);

    return 0;
}

/*
 * M, set_mode 'Mode' 'Passband'
 * "Set 'Mode' (USB, LSB, etc.) and 'Passband' or '0' for default"
 */
static int hamlib_set_mode(int cs, int is_extended, char *argv[], int argc)
{
    // printf("[DEBUG] hamlib_set_mode: argc=%d\n", argc);
    if (argc < 1) return -1;

    begin_new_response(cs);
    const char *supported_modes[] = {"USB", "LSB", "CW", "CWR", "DIGI", "AM", "PKTUSB", "DIGI", "FT8"};
    char mode[32] = "";
    char passband[32] = "0";

    strcpy(mode, argv[0]);

    if (argc > 1)  // Optional
        strcpy(passband, argv[1]);

    if (is_extended)
    {
        if (strcmp(argv[0], "?") == 0)
        {

            char tmp[128];
            snprintf(tmp, sizeof(tmp), "set_mode: ?\n");
            add_response(cs, tmp);
        }
        else
        {
            char tmp[128];
            snprintf(tmp, sizeof(tmp), "set_mode: %s %s:\nMode: %s\nPassband: %s\n", mode, passband, mode, passband);
            add_response(cs, tmp);
        }
    }

    if (strcmp(argv[0], "?") == 0)
    {
        strcpy(resp, "");
        for (int i = 0; i < sizeof(supported_modes) / sizeof(supported_modes[0]); i++)
        {
            strncat(resp, supported_modes[i], sizeof(resp) - strlen(resp) - 1);
            strncat(resp, " ", sizeof(resp) - strlen(resp) - 1);
        }
        strcat(resp, "\n");
        add_response(cs, resp);
        flush_response(cs);
        return 0;
    }


    // Support for DIGI mode from programs likw WSJT-X
    if (strcmp(mode, "PKTUSB") == 0)
        strcpy(mode, "DIGI");

    int found = 0;
    for (int i = 0; i < sizeof(supported_modes) / sizeof(supported_modes[0]); i++)
    {
        if (strcmp(mode, supported_modes[i]) == 0)
        {
            found = 1;
            break;
        }
    }
    if (!found) // Don't allow
    {
        add_response(cs, (char *) "RPRT -9\n");
        flush_response(cs);
        return -9;
    }
    if (is_locked)
    {   // Add response for not allowed operation


        ///add_response(cs, (char *) "RPRT -9\n");
        flush_response(cs);
        return -9;
    }

    char cmd[50];
    sprintf(cmd, "mode %s", mode);
    execute_command(cmd);
    // Passband is in Hz as an integer, -1 for no change, or ’0’ for the radio backend default.
    if (strcmp(passband, "0") == 0) //
    {
        char bw_str[10];
        sprintf(bw_str, "%i", get_default_passband_bw());
        field_set("BW", bw_str);
    }
    else if ( strcmp(passband, "-1") == 0) // Special case for WSJT-X (-1 passband value)
    {
        // DO NOTHING TO CHANGE PASSBAND
    }
    else
    {
        field_set("BW", passband);
    }

    if(is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);

    return 0;
}


/*
 * m, get_mode
 * "Get 'Mode' and 'Passband'. e.g. 'USB 2400'."
 */
static int hamlib_get_mode(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_mode: argc=%d\n", argc);
    const char *supported_modes[] = {"USB", "LSB", "CW", "CWR", "DIGI", "AM", "PKTUSB", "FT8"};
    char mode[20], passband[20], response[50];
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_mode:\n");
    }

    if (argc > 0 && strcmp(argv[0], "?") == 0)
    {
        strcpy(resp, "");
        for (int i = 0; i < sizeof(supported_modes) / sizeof(supported_modes[0]); i++)
        {
            strcat(resp, supported_modes[i]);
            strcat(resp, " ");
        }
        strcat(resp, "\n");
        add_response(cs, resp);
        flush_response(cs);
        return 0;
    }


    get_field_value_by_label("MODE", mode);
    //  get_field_value_by_label("BW", passband);

    if (strcmp(mode, "DIGI") == 0)
    {
        strcpy(mode, "PKTUSB");
    }

    if (argc > 0 ) // User tried to sneak in a mode parameter
    {
        strcpy(mode, argv[0]); // Ignore
        snprintf(response, sizeof(response), is_extended ? "Passband: %i\n" : "%i\n",
                 get_passband_bw());

    }
    else
        snprintf(response, sizeof(response), is_extended ? "Mode: %s\nPassband: %i\n" : "%s\n%i\n", mode,
                 get_passband_bw());

    add_response(cs, response);
    if(is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);

    return 0;
}

/*
 * V, set_vfo 'VFO'
 * "Set 'VFO': VFOA, VFOB, etc."
 * "Set 'VFO': VFOA, VFOB, etc."
 */

char map_currVFO[3];
const char* map_vfo_request(const char* request) {
    //   printf("[DEBUG] map_vfo_request: request=%s\n", request);

    if (strcmp(request, "Main") == 0 || strcmp(request, "RX") == 0) {
        return "A";
    } else if (strcmp(request, "Sub") == 0 || strcmp(request, "TX") == 0) {
        return "B";
    } else if (strcmp(request, "currVFO") == 0) {
        // Assuming get_current_vfo() is a function that returns the current VFO

        get_field_value_by_label("VFO", map_currVFO);
        return map_currVFO;
    } else {
        // Default case, return the request as is
        return request+3;
    }
}
static int hamlib_set_vfo(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_set_vfo: argc=%d\n", argc);
    if (argc < 1) return -1;
    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_vfo %s:\nVFO: %s\n", argv[0], argv[0]);
        add_response(cs, tmp);
    }
    {
        char tmp[5];
        strcpy(tmp, map_vfo_request(argv[0]));
        field_set("VFO", tmp);
        if (is_extended)
        {
            add_response(cs, (char *) "RPRT 0\n");
        }
    }
    flush_response(cs);
    return 0;
}

/*
 * v, get_vfo
 * "Get current 'VFO'. e.g. 'VFOA' or 'VFOB'."
 */
static int hamlib_get_vfo(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_vfo: argc=%d\n", argc);
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_ßvfo:\n");

    }
    {
        char currVFO[2];
        get_field_value_by_label("VFO", currVFO);
        if (is_extended)
        {
            char tmp[128];
            snprintf(tmp, sizeof(tmp),
                     "VFO: %s\n", currVFO);
            add_response(cs, tmp);
            add_response(cs, (char *) "RPRT 0\n");
            flush_response(cs);
            return 0;
        }
        if (currVFO[0] == 'A')
            add_response(cs, (char *) "VFOA\n");
        else
            add_response(cs, (char *) "VFOB\n");
    }

    flush_response(cs);
    return 0;
}

/*
 * S, set_split_vfo 'Split' 'TX VFO'
 * "Set 'Split' mode, '0' or '1', plus TX VFO, e.g. 'VFOB'."
 */
static int hamlib_set_split(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_set_split: argc=%d\n", argc);
    if (argc < 1) return -1;

    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp), "set_split_vfo %s %s\n", argv[0], argc == 2 ? argv[1] : "");
        add_response(cs, tmp);
    }

    if (argc < 2 && strcmp(argv[0], "0") != 0) return -9;

    if (strcmp(argv[0], "0") == 0)
    {
        field_set("SPLIT", "OFF");
    }
    else
    {
        field_set("VFO", argv[1] + 3);
        field_set("SPLIT", "ON");
    }

    if (is_extended)
    {
        add_response(cs, "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

/*
 * s, get_split_vfo
 * "Get 'Split' mode, '0' or '1', plus TX VFO if split=1."
 */
static int hamlib_get_split(int cs, int is_extended, char *argv[], int argc)
{
    //   printf("[DEBUG] hamlib_get_split: argc=%d\n", argc);
    char curr_split[4];
    begin_new_response(cs);
    get_field_value_by_label("SPLIT", curr_split);

    if (is_extended)
    {
        add_response(cs, (char *) "get_split_vfo:\n");
    }

    if (strcmp(curr_split, "OFF") == 0)
    {
        add_response(cs, is_extended ? (char *) "Split: 0\n" : (char *) "0\n");

        char vfo[4];
        get_field_value_by_label("VFO", vfo);

        char tmp[128];
        snprintf(tmp, sizeof(tmp), is_extended ? "TX VFO: VFO%s\n" : "VFO%s\n", vfo);
        add_response(cs, tmp);
    }
    else
    {
        add_response(cs, (char *) "1\n");
        if (is_extended)
        {
            add_response(cs, (char *) "TX VFO: VFOB\n");
        }
        else
        {
            add_response(cs, (char *) "VFOB\n");
        }
    }
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}


/*****************************************************************************
 * NOW set_func / get_func that may use the property dictionary or your code
 *****************************************************************************/

/*
 * U, set_func 'Func' 'Func Status'
 * "Set 'Func' (NB, COMP, TUNER, etc.) to on/off. For example, 'NB 1' => turn NB on."
 */
static int hamlib_set_func(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_set_func: argc=%d\n", argc);
    if (argc < 2) return -1;
    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_func %s %s:\n%s: %s\n", argv[0], argv[1], argv[0], argv[1]);
        add_response(cs, tmp);
    }

    /* We can attempt to set it via the property table. If not found, fallback to your old code. */
    //  printf("About to resolve property name\n");
    const char *mapped_func_name = resolve_property_name(argv[0]);
    /* For a function named argv[0] (e.g. "NB") we pass argv[1] ("1") */
    // printf("About to set property name\n");
    if (!sdr_radio_set_property(mapped_func_name, argv[1]))
    {

        extern int set_func(const char *, const char *);
        //    printf("About to set_func property name\n");
        int ret = set_func(mapped_func_name, argv[1]);
        if (ret != 0)
        {
            add_response(cs, (char *) "RPRT -11\n");
        }
        else
        {
            add_response(cs, (char *) "RPRT 0\n");
        }
        // }
    }
    else
    {
        /* success using property table */
        if (is_extended)
            add_response(cs, (char *) "RPRT 0\n");
    }
    //  printf("About to flush response\n");
    flush_response(cs);
    return 0;
}


/*
 * u, get_func 'Func'
 * "Get 'Func' status. Return '1' if on, '0' if off, etc."
 */
static int hamlib_get_func(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_func: argc=%d\n", argc);
    if (argc < 1) return -1;

    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp), "get_func %s:\n", argv[0]);
        add_response(cs, tmp);
    }

    if (strcmp(argv[0], "?") == 0)
    {
        strcpy(resp, "");
        for (int i = 0; property_table[i].name != NULL; i++)
        {
            if (property_table[i].type == PT_STRING || property_table[i].type == PT_BOOLEAN)
            {
                strcat(resp, property_table[i].name);
                strcat(resp, " ");
            }

        }
        strcat(resp, "\n");
        add_response(cs, resp);
        flush_response(cs);
        return 0;
    }

    char out_buf[128];
    const char *mapped_func_name = resolve_property_name(argv[0]);

    if (!sdr_radio_get_property(mapped_func_name, out_buf, sizeof(out_buf)))
    {
        extern void command_get_func(int client_socket, const char *func);
        command_get_func(cs, mapped_func_name);
    }
    else
    {
        strcat(out_buf, "\n");
        if (is_extended)
        {
            char tmp[128];
            snprintf(tmp, sizeof(tmp), "%s: %s\n", argv[0], out_buf);
            add_response(cs, tmp);
        }
        else
        {
            add_response(cs, out_buf);
        }
    }
    flush_response(cs);
    return 0;
}

/*****************************************************************************
 * set_level / get_level with property table
 *****************************************************************************/

/*
 * L, set_level 'Level' 'Level Value'
 * "Set 'Level' to float/int, e.g. 'RFPOWER' => .5"
 */
static int hamlib_set_level(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_set_level: argc=%d\n", argc);
    if (argc < 2) return -1;
    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_level: %s %s\n%s: %s\n", argv[0], argv[1], argv[0], argv[1]);
        add_response(cs, tmp);
    }
    const char *mapped_level_name = resolve_property_name(argv[0]);
//    printf("Mapped level name is %s\n", mapped_level_name);

    /* Attempt property-based approach: */
    if (!sdr_radio_set_property(mapped_level_name, argv[1]) ||
        strcmp(argv[0], "RFPOWER") == 0 || // These are likely coming from rigctld
        strcmp(argv[0], "RF") == 0 ||
        strcmp(argv[0], "MICGAIN") == 0 ||
        strcmp(argv[0], "FAGC") == 0 ||
        strcmp(argv[0], "AGC") == 0 ||
        strcmp(argv[0], "MONITOR_GAIN") == 0 ||
        strcmp(argv[0], "NR") == 0 ||      // This one needs it's own handler due to difference in LEVEL/FUNC issue.
        strcmp(argv[0], "COMP") == 0        // This one needs it's own handler due to difference in LEVEL/FUNC issue.
            )
    {
        /* fallback to your older 'command_set_level(...)' code: */
        extern hamlib_error_t command_set_level(const char *, float);
        float val = (float) atof(argv[1]);
        hamlib_error_t err = command_set_level(mapped_level_name, val);
        if (err == HAMLIB_OK)
        {
            if (!is_extended) add_response(cs, (char *) "RPRT 0\n");
        }
        else
        {
            if (err == HAMLIB_ERR_INVALID_PARAM)
                add_response(cs, (char *) "RPRT -1\n");
            else if (err == HAMLIB_ERR_NOT_IMPLEMENTED)
                add_response(cs, (char *) "RPRT -12\n");
            else
                add_response(cs, (char *) "RPRT -13\n");
        }
    }
    else
    {
        /* success with property table */
        if (!is_extended)
        {
            add_response(cs, (char *) "RPRT 0\n");
        }
    }
    flush_response(cs);
    return 0;
}

/*
 * l, get_level 'Level'
 * "Get 'Level' value as float or int."
 */
static int hamlib_get_level(int cs, int is_extended, char *argv[], int argc)
{
    //   printf("[DEBUG] hamlib_get_level: argc=%d\n", argc);
    if (argc < 1) return -1;
    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "get_level: %s\n%s: ", argv[0], argv[0]);
        add_response(cs, tmp);
    }


    if (strcmp(argv[0], "?") == 0)
    {
        strcpy(resp, "");
        for (int i = 0; i < 200; i++)
        {
            if (property_table[i].name == NULL) break;
            if (property_table[i].type == PT_FLOAT)
            {
                strcat(resp, property_table[i].name);
                strcat(resp, " ");
            }
        }
        strcat(resp, "RPRT 0\n");
        add_response(cs, resp);
        flush_response(cs);
        return 0;
    }
    char out_buf[128];
    const char *mapped_level_name = resolve_property_name(argv[0]);
    if (is_debug)printf("Mapped level name is %s\n", mapped_level_name);

    if (!sdr_radio_get_property(mapped_level_name, out_buf, sizeof(out_buf)))
    {
        /* fallback to your older command_get_level(...) code: */
        extern void command_get_level(int client_socket, const char *level);
        command_get_level(cs, mapped_level_name);
    }
    else
    {
        strcat(out_buf, "\n");
        add_response(cs, out_buf);
    }

    if(is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

void command_tx_control(int client_socket, int s)
{
    begin_new_response(client_socket);
    //printf("tx_control(%d)\n", s);
    if (s >= 1)
    {
        hamlib_tx(1);
    }
    if (s == 0)
    {
        hamlib_tx(0);
    }
    if (s == -1)
    {
        char tx_status[100];
        sdr_request("stat:tx=1", (char *) tx_status);
        if (!strcmp(tx_status, "ok on"))
            add_response(client_socket, "1\n");
        else
            add_response(client_socket, "0\n");
        return;
    }
    add_response(client_socket, "RPRT 0\n");
    flush_response(client_socket);
}
/*****************************************************************************
 * T, set_ptt 'PTT' (0 or 1)
 * t, get_ptt
 *****************************************************************************/

/*
 * T, set_ptt 'PTT'
 * "Set 'PTT', 0=RX or 1=TX."
 */
static int hamlib_set_ptt(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_set_ptt: argc=%d\n", argc);
    int ptt_val = 1;
    if (argc > 0) ptt_val = atoi(argv[0]);
    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[64];
        snprintf(tmp, sizeof(tmp),
                 "set_ptt %d:\nPTT: %d\n", ptt_val, ptt_val);
        add_response(cs, tmp);
    }
    {
        extern void command_tx_control(int client_socket, int s);
        command_tx_control(cs, ptt_val);
    }
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

/*
 * t, get_ptt
 * "Get 'PTT' status, returns 0 or 1."
 */
static int hamlib_get_ptt(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_ptt: argc=%d\n", argc);
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_ptt:\n");
    }
    {
        /* your code calls command_tx_control(cs,-1) => returns '0\n' or '1\n' */
        extern void command_tx_control(int client_socket, int s);
        command_tx_control(cs, -1);
    }
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

/*****************************************************************************
 * D, dump_state => multi-line rig caps
 *****************************************************************************/
static char dump_state_response[] =
        "1\n" // Version 1
        "2\n" // Limited mode 2
        "2\n" // ITU Region

        // RX frequency range
        "100000 30000000 0x23E -1 -1 0x10000003 0x0\n"
        "0 0 0 0 0 0 0\n"

        // TX frequency ranges
        "3500000 4000000 0x23E -1 40000 0x10000003 0x0\n"
        "7000000 7300000 0x23E -1 40000 0x10000003 0x0\n"
        "10000000 10150000 0x23E -1 30000 0x10000003 0x0\n"
        "14000000 14350000 0x23E -1 30000 0x10000003 0x0\n"
        "18000000 18200000 0x23E -1 20000 0x10000003 0x0\n"
        "21000000 21450000 0x23E -1 10000 0x10000003 0x0\n"
        "24800000 25000000 0x23E -1 10000 0x10000003 0x0\n"
        "28000000 29700000 0x23E -1 6000 0x10000003 0x0\n"
        "0 0 0 0 0 0 0\n"

        // Tuning steps
        "0x23E 10\n"
        "0x23E 1\n"
        "0 0\n"

        // Filters
        "0x82 500\n" // CW Normal
        "0x82 200\n" // CW Narrow
        "0x82 2000\n"// CW Wide
        "0x0C 2700\n" // SSB Normal
        "0x0C 1400\n" // SSB Narrow
        "0x0C 3900\n" // SSB Wide
        "0x21 10000\n"
        "0x21 5000\n"
        "0x21 20000\n"
        "0x221 5000\n"  /* AM | AMS | FM narrow */
        "0 0\n"

        // Max RIT and XIT
        "25000\n"
        "-25000\n"
        "0\n"

        // Announce bitfield
        "0\n"

        // Preamp and attenuator
        "0\n"
        "0\n"

        // **Updated Get Functions**
        "0x41010105\n"  // Now includes LOCK and RIT

        // **Updated Set Functions**
        "0x41010105\n"

        // **Updated Get Levels**
        //AF RF NR CWPITCH RFPOWER MICGAIN KEYSPD NOTCHF AGC SWR STRENGTH NOTCHF_RAW MONITOR_GAIN
        "0x305002F918\n"

        // **Updated Set Levels**
        // FAGC COMP ANF LOCK RIT TUNER
        "0x305002F918\n"

        // No extra parameters
        "0\n"
        "0\n";

char extra_response[] = "vfo_opts=0xFFFFF\n"
                        "ptt_type=0x00000001\n"  // PTT=RIG
                        "targetable_vfo=0x00000003\n"
                        "has_set_vfo=1\n"
                        "has_get_vfo=1\n"
                        "has_set_freq=1\n"
                        "has_get_freq=1\n"
                        "timeout=0\n"
                        "rig_model=2\n"
                        "done\n"
                        "0\n";

static int hamlib_dump_state(int cs, int is_extended)
{
    //  printf("[DEBUG] hamlib_dump_state\n");
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "dump_state:\n");
        add_response(cs, dump_state_response);
        add_response(cs, extra_response);

    }
    else
    {
        add_response(cs, dump_state_response);
        add_response(cs, extra_response);
        add_response(cs, (char *) "RPRT 0\n");
    }
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

static int hamlib_get_rig_info(int client_socket, int is_extended, char *argv[], int argc)
{
    // printf("[DEBUG] hamlib_get_rig_info: argc=%d\n", argc);
    begin_new_response(client_socket);
    if (is_extended)
    {
        add_response(client_socket, (char *) "get_rig_info:\n");
    }

    char mode[10];
    char freq[20];
    char split[10];

    get_field_value_by_label("SPLIT", split);

    // Get the VFO A Values
    get_field_value_by_label("VFOA", freq);
    get_field_value_by_label("MODE", mode);

    snprintf(resp, sizeof(resp), "VFO=VFOA Freq=%s Mode=%s Width=%ld RX=1 TX=1\n",
             freq, mode, get_passband_bw());
    add_response(client_socket, resp);

    // Get the VFO B Values
    get_field_value_by_label("VFOB", freq);

    snprintf(resp, sizeof(resp), "VFO=VFOB Freq=%s Mode=%s Width=%ld RX=0 TX=0\n",
             freq, mode, get_passband_bw());
    add_response(client_socket, resp);

    snprintf(resp, sizeof(resp), "Split=%s\nSatMode=0\nRig=%s\nVersion=%s\nApp=Hamlib\n",
             strcmp(split, "OFF") == 0 ? "0" : "1", PRODUCT, VERSION);
    add_response(client_socket, resp);
    if (is_extended)
    {
        add_response(client_socket, (char *) "RPRT 0\n");
    }
    flush_response(client_socket);
    return 0;
}

/*****************************************************************************
 * P => get_powerstat
 *****************************************************************************/
static int hamlib_get_powerstat(int cs, int is_extended)
{
    // printf("[DEBUG] hamlib_get_powerstat\n");
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_powerstat:\nPower: 1\n");
    }
    else
    {
        add_response(cs, (char *) "1\n");
    }
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

/*****************************************************************************
 * w => send_cmd 'Cmd'
 *****************************************************************************/
static int hamlib_send_cmd_raw(int cs, int is_extended, char *argv[], int argc)
{
//printf("[DEBUG] hamlib_send_cmd_raw: argc=%d\n", argc);
    if (argc < 2) return -1;
    begin_new_response(cs);

    if (is_extended)
    {
        char tmp[256];
        if (argc > 1 && argc < 3)
            snprintf(tmp, sizeof(tmp),
                     "send_cmd: %s\nCommand: %s\n", argv[0], argv[0]);
        if (argc > 2 && argc <= 3)
            snprintf(tmp, sizeof(tmp),
                     "send_cmd: %s %s\n%s: %s\n", argv[0], argv[1], argv[0], argv[1]);
        add_response(cs, tmp);
    }

    char combined[256] = "";
    for (int i = 0; i < argc; i++)
    {
        if (i > 0) strcat(combined, " ");
        strcat(combined, argv[i]);
    }

    execute_command(combined);
    if (!is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }

    flush_response(cs);
    return 0;
}

static int hamlib_get_vfo_info(int cs, int is_extended, char *argv[], int argc)
{
    // printf("[DEBUG] hamlib_get_vfo_info: argc=%d\n", argc);


    //Freq: 21019800
    //Mode: CW
    //Width: 500
    //Split: 1
    //SatMode: 0

    begin_new_response(cs);

    char mode[10];
    char freq[20];
    char vfo[10];
    char bandwidth[10];

    char split[4];
    char tx_vfo[4];
    extern char vfo_a_mode[10];
    extern char vfo_b_mode[10];

    if (argc < 1) return -1;
    if (argc > 0)
        strcpy(vfo, argv[0]);

    get_field_value_by_label(vfo, freq);
    get_field_value_by_label("SPLIT", split);
    get_field_value_by_label("BW", bandwidth);

    /// Apparently we only have one mode... so we will just use that for both
    get_field_value_by_label("MODE", mode);
    // If mode is DIGI, change it to PKTUSB
    if (strcmp(mode, "DIGI") == 0)
        strcpy(mode, "PKTUSB");

    // Get the mode based on which VFOA or VFOB
//        if (!strcmp(vfo, "VFOA"))
//            strcpy(mode, vfo_a_mode);
//        else
//            strcpy(mode, vfo_b_mode);

    if (is_extended)
    {
        sprintf(resp, "get_vfo_info: %s\nFreq: %s\nMode: %s\nWidth: %s\nSplit: %s \nRPRT 0\n", argv[0], freq, mode,
                bandwidth,
                strcmp(split, "OFF") == 0 ? "0" : "1");
    }
    else
        sprintf(resp, "%s\n%s\n%s\n%s\n", freq, mode, bandwidth, strcmp(split, "OFF") == 0 ? "0" : "1");

    add_response(cs, resp);
    flush_response(cs);
    return 0;
}

// Handle Get 'Transceive' mode which in our caase is always off... MacLoggerDX expects this
// TODO: This should be a serious mode, would be really neat for things like sending
//        METER readings live, from the radio to the client.

static int hamlib_get_trn(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_trn: argc=%d\n", argc);
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_trn:\n");
    }
    sprintf(resp, "OFF\n");
    add_response(cs, resp);
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

//   Gets rig clock -- note that some rigs do not handle seconds or milliseconds. Format is ISO8601 YYYY-MM-DDTHH:MM:SS.sss+ZZ where +ZZ is either -/+ UTC offset
static int hamlib_get_clock(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_clock: argc=%d\n", argc);
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_clock:\n");
    }
    {
        char clock[50];
//        sdr_request("clock", clock);
        // Calculate the current clock time
        time_t now;
        struct tm *tm;
        now = time(0);
        tm = localtime(&now);
        strftime(clock, sizeof(clock), "%Y-%m-%dT%H:%M:%S%z\n", tm);
        if (is_extended)
        {
            // Write clock colon time
            char tmp[128];
            snprintf(tmp, sizeof(tmp),
                     "Clock: %s\n", clock);
            add_response(cs, tmp);
        }
        else
            add_response(cs, clock);
    }
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

static int hamlib_set_lock_mode(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_set_lock_mode: argc=%d\n", argc);
    if (argc < 1) return -1;
    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_lock_mode: %s\nLock Mode: %s\n", argv[0], argv[0]);
        add_response(cs, tmp);
    }
    is_locked = strcmp(argv[0], "0") == 0 ? false : true;
    field_set("VFOLK", argv[0]);
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }

    flush_response(cs);
    return 0;
}

static int hamlib_chk_vfo(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_chk_vfo: argc=%d\n", argc);
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "check_vfo:\n");
    }

    add_response(cs, "0\n");
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

static int hamlib_get_lock_mode(int cs, int is_extended, char *argv[], int argc)
{
// printf("[DEBUG] hamlib_get_lock_mode: argc=%d\n", argc);
    begin_new_response(cs);
    char lock[10];
//    const char *lock = field_str("VFOLK");
    sdr_radio_get_property("VFOLK", lock, sizeof(lock));

    if (is_extended)
    {
        add_response(cs, (char *) "get_lock_mode:\n");
    }

    if (strcmp(lock, "0") == 0)
    {
        add_response(cs, is_extended ? (char *) "Lock Mode: 0\n" : (char *) "0\n");
    }
    else
    {
        add_response(cs, is_extended ? (char *) "Lock Mode: 1\n" : (char *) "1\n");
    }
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}


vfo_op_t VFO_OP_string_to_enum(const char *str)
{
    if (strcmp(str, "CPY") == 0) return RIG_OP_CPY;
    if (strcmp(str, "XCHG") == 0) return RIG_OP_XCHG;
    if (strcmp(str, "FROM_VFO") == 0) return RIG_OP_FROM_VFO;
    if (strcmp(str, "TO_VFO") == 0) return RIG_OP_TO_VFO;
    if (strcmp(str, "MCL") == 0) return RIG_OP_MCL;
    if (strcmp(str, "UP") == 0) return RIG_OP_UP;
    if (strcmp(str, "DOWN") == 0) return RIG_OP_DOWN;
    if (strcmp(str, "BAND_UP") == 0) return RIG_OP_BAND_UP;
    if (strcmp(str, "BAND_DOWN") == 0) return RIG_OP_BAND_DOWN;
    if (strcmp(str, "LEFT") == 0) return RIG_OP_LEFT;
    if (strcmp(str, "RIGHT") == 0) return RIG_OP_RIGHT;
    if (strcmp(str, "TUNE") == 0) return RIG_OP_TUNE;
    if (strcmp(str, "TOGGLE") == 0) return RIG_OP_TOGGLE;
    return RIG_OP_NONE;
}

// G, vfo_op 'Mem/VFO Op'
//Perform a 'Mem/VFO Op'.
//
//Mem/VFO Operation is a token: ‘CPY’, ‘XCHG’, ‘FROM_VFO’, ‘TO_VFO’, ‘MCL’, ‘UP’, ‘DOWN’, ‘BAND_UP’, ‘BAND_DOWN’, ‘LEFT’, ‘RIGHT’, ‘TUNE’, ‘TOGGLE’.
//
//Note: Passing a ‘?’ (query) as the first argument instead of a Mem/VFO Op token
// will return a space separated list of radio backend supported Set Mem/VFO Op tokens.
// Use this to determine the supported Mem/VFO Ops of a given radio

char *current_vfo_fullName(char shortName)
{
    static char vfoName[10] = {0};
    strcpy(vfoName, "VFO");
    vfoName[3] = shortName;

}

// Correct the hamlib_vfo_op function
static int hamlib_vfo_op(int cs, int is_extended, char *argv[], int argc)
{
    //   printf("[DEBUG] hamlib_vfo_op: argc=%d\n", argc);
    char freq[20];
    char vfo[10];
    char step[10];
    long f, s;
    const int num_bands = NUM_BANDS;

    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "vfo_op:\n");
    }

    if (argc < 1)
    {
        add_response(cs, (char *) "RPRT -1\n");
        flush_response(cs);
        return -1;
    }

    if (strcmp(argv[0], "?") == 0)
    {
        add_response(cs, (char *) "CPY XCHG UP DOWN BAND_UP BAND_DOWN TOGGLE\n");
        flush_response(cs);
        return 0;
    }

    vfo_op_t op = VFO_OP_string_to_enum(argv[0]);
    if (op == RIG_OP_NONE)
    {
        add_response(cs, (char *) "RPRT -11\n");
        flush_response(cs);
        return -11;
    }

    switch (op)
    {
        case RIG_OP_CPY:
            // Get the current vfo
            get_field_value_by_label("VFO", vfo);
            get_field_value_by_label(current_vfo_fullName(vfo[0]), freq);
            field_set("VFOB", freq);
            field_set("VFOA", freq);
            field_set("VFO", vfo);  // Refresh the VFO
            break;
        case RIG_OP_XCHG:
            char freq_a[20];
            char freq_b[20];
            get_field_value_by_label("VFO", vfo);
            get_field_value_by_label("VFOA", freq_a);
            get_field_value_by_label("VFOB", freq_b);
            field_set("VFOA", freq_b);
            field_set("VFOB", freq_a);
            field_set("VFO", vfo);  // Refresh the VFO
            break;
        case RIG_OP_UP:
            get_field_value_by_label("VFO", vfo);
            get_field_value_by_label("STEP", step);
            get_field_value_by_label(current_vfo_fullName(vfo[0]), freq);
            f = atol(freq);
            s = atol(step);
            f += s;
            snprintf(freq, sizeof(freq), "%ld", f);
            field_set("FREQ", freq);
            break;
        case RIG_OP_DOWN:
            get_field_value_by_label("VFO", vfo);
            get_field_value_by_label("STEP", step);
            get_field_value_by_label(current_vfo_fullName(vfo[0]), freq);
            f = atol(freq);
            s = atol(step);
            f -= s;
            snprintf(freq, sizeof(freq), "%ld", f);
            field_set("FREQ", freq);
            break;
        case RIG_OP_BAND_DOWN:
        case RIG_OP_BAND_UP:

            const char *current_band = get_current_band();
            int current_band_index = -1;
            for (int i = 0; i < num_bands; i++)
            {
                if (strcmp(current_band, band_stack[i].name) == 0)
                {
                    current_band_index = i;
                    break;
                }
            }
            if (current_band_index == -1)
            {
                add_response(cs, (char *) "RPRT -11\n");
                flush_response(cs);
                return -11;
            }
            if (op == RIG_OP_BAND_DOWN && current_band_index > 0)
            {
                change_band(band_stack[current_band_index - 1].name);
            }
            else if (op == RIG_OP_BAND_UP && current_band_index < num_bands - 1)
            {
                change_band(band_stack[current_band_index + 1].name);
            }

            break;
        case RIG_OP_LEFT:
        case RIG_OP_RIGHT:
        case RIG_OP_TUNE:
            add_response(cs, (char *) "RPRT -11\n");
            flush_response(cs);
            return -11;
        case RIG_OP_TOGGLE:
            get_field_value_by_label("VFO", vfo);
            if (strcmp(vfo, "A") == 0)
            {
                field_set("VFO", "B");
            }
            else
            {
                field_set("VFO", "A");
            }
            break;
        default:
            add_response(cs, (char *) "RPRT -11\n");
            flush_response(cs);
    }
    if(is_extended)
        add_response(cs, (char *) "RPRT 0\n");
    flush_response(cs);
    return 0;
}

// Set RIT by executing the command RIT_DELTA followed by the parameter
static int hamlib_set_rit(int cs, int is_extended, char *argv[], int argc)
{
    //   printf("[DEBUG] hamlib_set_rit: argc=%d\n", argc);
    if (argc < 1) return -1;
    begin_new_response(cs);
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_rit %s:\nRIT: %s\n", argv[0], argv[0]);
        add_response(cs, tmp);
    }

    char rit[20];
    int rit_val = atol(argv[0]);
    snprintf(rit, sizeof(rit), "RIT_DELTA %i", rit_val);
    execute_command(rit);

    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

/// Get RIT by returning the current RIT_DELTA value
static int hamlib_get_rit(int cs, int is_extended, char *argv[], int argc)
{
    //  printf("[DEBUG] hamlib_get_rit: argc=%d\n", argc);
    begin_new_response(cs);
    if (is_extended)
    {
        add_response(cs, (char *) "get_rit:\n");
    }
    char rit[20];
    // Get the ritval from the field "RIT_DELTA"
    int rit_val = field_int("RIT_DELTA");
    snprintf(rit, sizeof(rit), "%i\n", rit_val);
    add_response(cs, rit);
    if (is_extended)
    {
        add_response(cs, (char *) "RPRT 0\n");
    }
    flush_response(cs);
    return 0;
}

/*****************************************************************************
 * Q => quit
 *****************************************************************************/

/*****************************************************************************
 * MASTER interpret_line
 *****************************************************************************/
static void interpret_line(int client_socket, const char *line_in)
{
    //  printf("[DEBUG] interpret_line: line_in=%s\n", line_in);
    char line[512];
    char command[30];
    strncpy(line, line_in, sizeof(line));
    line[sizeof(line) - 1] = '\0';

    line_mode_t mode = parse_line_mode(line);
    if (mode == LINE_MODE_INVALID)
    {
        send_error_rprt(client_socket, -11);
        return;
    }

    /* tokenize */
    char *argv[16];
    int argc = 0;
    {
        char *tok = strtok(line, " \t\r\n");
        while (tok && argc < 15)
        {
            argv[argc++] = tok;
            tok = strtok(NULL, " \t\r\n");
        }
        argv[argc] = NULL;
    }
    if (argc < 1)
    {
        send_error_rprt(client_socket, -1);
        return;
    }
    // Debug input string
    if (is_debug)
    {
        printf("Command: ");
        for (int i = 0; i < argc; i++)
            printf("%s ", argv[i]);
        printf("  --> ");
    }
    strcpy(command, argv[0]); // Hold it for later

    command_id_t cmd_id = parse_command_name(argv[0]);

    // Move args down for handlers that need it
    for (int i = 1; i < argc; i++)
    {
        argv[i - 1] = argv[i];
    }
    argc--;
    argv[argc] = NULL;

    int is_extended = (mode == LINE_MODE_EXTENDED);
    int ret = 0;

    switch (cmd_id)
    {
        case CMD_SET_FREQ:
            ret = hamlib_set_freq(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_FREQ:
            ret = hamlib_get_freq(client_socket, is_extended, argv, argc);
            break;

        case CMD_SET_MODE:
            ret = hamlib_set_mode(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_MODE:
            ret = hamlib_get_mode(client_socket, is_extended, argv, argc);
            break;

        case CMD_SET_VFO:
            ret = hamlib_set_vfo(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_VFO:
            ret = hamlib_get_vfo(client_socket, is_extended, argv, argc);
            break;

        case CMD_SET_SPLIT:
            ret = hamlib_set_split(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_SPLIT:
            ret = hamlib_get_split(client_socket, is_extended, argv, argc);
            break;

        case CMD_SET_FUNC:
            ret = hamlib_set_func(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_FUNC:
            ret = hamlib_get_func(client_socket, is_extended, argv, argc);
            break;

        case CMD_SET_LEVEL:
            ret = hamlib_set_level(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_LEVEL:
            ret = hamlib_get_level(client_socket, is_extended, argv, argc);
            break;

        case CMD_SET_PTT:
            ret = hamlib_set_ptt(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_PTT:
            ret = hamlib_get_ptt(client_socket, is_extended, argv, argc);
            break;

        case CMD_DUMP_STATE:
            ret = hamlib_dump_state(client_socket, is_extended);
            break;
        case CMD_GET_POWERSTAT:
            ret = hamlib_get_powerstat(client_socket, is_extended);
            break;

        case CMD_SEND_CMD_RAW:
            ret = hamlib_send_cmd_raw(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_VFO_INFO:
            ret = hamlib_get_vfo_info(client_socket, is_extended, argv, argc);
            break;
        case CMD_SET_LOCK:
            ret = hamlib_set_lock_mode(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_LOCK:
            ret = hamlib_get_lock_mode(client_socket, is_extended, argv, argc);
            break;

        case CMD_SET_RIT:
            ret = hamlib_set_rit(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_RIT:
            ret = hamlib_get_rit(client_socket, is_extended, argv, argc);
            break;
        case CMD_DUMP_CAPS:
            ret = hamlib_dump_caps(client_socket, is_extended, argv, argc);
            break;

        case CMD_QUIT:
            ret = 0;
            if (is_extended)
                send_response(client_socket, "quit:\n");
            //printf(" About to send RPRT 0\n");
            send_error_rprt(client_socket, ret);
            //printf("Sent RPRT 0\n");

            //printf("About to shutdown and close socket %d\n", client_socket);
            shutdown(client_socket, SHUT_RDWR);
            //printf("Close socket %d\n", client_socket);
            close(client_socket);
            printf("Shutdown and close socket %d\n", client_socket);

            // Remove the socket from the client list under lock
            // pthread_mutex_lock(&client_mutex);
            for (int i = 0; i < MAX_CLIENTS; i++)
            {
                if (client_sockets[i] == client_socket)
                {
                    printf("Hamlib client disconnected. Closing socket %d.\n", client_socket);
                    client_sockets[i] = 0;  // remove it immediately
                    break;
                }
            }
            //  pthread_mutex_unlock(&client_mutex);


            // Optionally, call FD_CLR(client_socket, &readss) if you have a global FD_SET
            // FD_CLR(client_socket, &reads)

            //shutdown(client_socket, SHUT_RDWR);


            //close(client_socket);
            return;
        case CMD_GET_RIG_INFO:
            ret = hamlib_get_rig_info(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_TRN:
            ret = hamlib_get_trn(client_socket, is_extended, argv, argc);
            break;
        case CMD_GET_CLOCK:
            ret = hamlib_get_clock(client_socket, is_extended, argv, argc);
            break;
        case CMD_CHK_VFO:
            ret = hamlib_chk_vfo(client_socket, is_extended, argv, argc);
            break;
        case CMD_VFO_OP:

            ret = hamlib_vfo_op(client_socket, is_extended, argv, argc);
            break;

        case CMD_UNKNOWN:
        case CMD_INVALID:
        default:
            // Send an error response of the command and all of the arguments
            // in the form: <command>: <arg1> <arg2> <arg3> ...
            char tmp[256];
            begin_new_response(client_socket);
            if (is_extended)
            {
                strcpy(tmp, command);
                strcat(tmp, ":");
                strcat(tmp, "\n");
                add_response(client_socket, tmp);
            }
            add_response(client_socket, "RPRT -11\n");
            printf("UNKNOWN command %s: -11\n", command);
            flush_response(client_socket); // Single packet response
            fflush(stdout);
            return;
    }

    if (ret != 0)
    {
        char tmp[100];
        begin_new_response(client_socket);
        sprintf(tmp, "RPTR %d\n", ret);
        printf("Error in command %s: %d\n", command, ret);
        // show the arguments to the user in a printf in this error
        for (int i = 0; i < argc; i++)
        {
            printf("Parameter: %d : %s",i, argv[i]);
        }


        add_response(client_socket, tmp);
        flush_response(client_socket); // Single packet response
        fflush(stdout);
    }
}

/*****************************************************************************
 * handle_client_data, plus the server loop
 *****************************************************************************/
void handle_client_data(int client_socket, const char *data, ssize_t len)
{
    //   printf("[DEBUG] handle_client_data: len=%zd\n", len);
    int idx = -1;
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (client_sockets[i] == client_socket)
        {
            idx = i;
            break;
        }
    }
    if (idx < 0) return;

    for (ssize_t i = 0; i < len; i++)
    {
        char c = data[i];
        if (c == '\n')
        {
            incoming_data[idx][incoming_ptr[idx]] = '\0';
            interpret_line(client_socket, incoming_data[idx]);
            incoming_ptr[idx] = 0;
        }
        else
        {
            if (incoming_ptr[idx] < MAX_DATA - 1)
            {
                incoming_data[idx][incoming_ptr[idx]++] = c;
            }
        }
    }
}

void send_response(int client_socket, char *response)
{
    //   printf("[DEBUG] send_response: response=%s\n", response);
    if (is_debug)
        printf("Sending response: %s", response);
    int e = send(client_socket, response, strlen(response), 0);

    if (e < 0)
    {
        printf("Error sending response\n");
        perror("send");
        if (is_debug)
        {
            printf("Hamlib client disconnected (send). Closing socket %d.\n", client_socket);
        }
        close(client_socket);
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            if (client_sockets[i] == client_socket)
            {
                client_sockets[i] = 0;
                break;
            }
        }
    }
}

int get_max_sd()
{
    int max_sd = welcome_socket;
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (client_sockets[i] > max_sd) max_sd = client_sockets[i];
    }
    return max_sd;
}

#include <fcntl.h>

// Set the socket to non-blocking mode
void set_socket_non_blocking(int socket_fd)
{
    int flags = fcntl(socket_fd, F_GETFL, 0);
    if (flags == -1)
    {
        perror("fcntl F_GETFL");
        exit(EXIT_FAILURE);
    }

    flags |= O_NONBLOCK;
    if (fcntl(socket_fd, F_SETFL, flags) == -1)
    {
        perror("fcntl F_SETFL");
        exit(EXIT_FAILURE);
    }
}

static void *server_thread(void *arg)
{
    //   printf("[DEBUG] server_thread started\n");
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    welcome_socket = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (welcome_socket < 0)
    {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }
    set_socket_non_blocking(welcome_socket);
    int opt = 1;
    struct linger ling;
    ling.l_onoff = 1;     // enable linger
    ling.l_linger = 0;    // zero-second linger time => immediate close
    if (setsockopt(welcome_socket, SOL_SOCKET, SO_LINGER, &ling, sizeof(ling)) < 0)
    {
        perror("setsockopt failed");
        close(welcome_socket);
        exit(EXIT_FAILURE);
    }

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(4532);
    server_addr.sin_addr.s_addr = INADDR_ANY;
    memset(server_addr.sin_zero, 0, sizeof(server_addr.sin_zero));

    if (bind(welcome_socket, (struct sockaddr *) &server_addr, sizeof(server_addr)) < 0)
    {
        perror("Failed to bind socket");
        close(welcome_socket);
        exit(EXIT_FAILURE);
    }
    //  printf("Socket bound successfully\n");

    if (listen(welcome_socket, 5) < 0)
    {
        perror("Failed to listen on socket");
        close(welcome_socket);
        exit(EXIT_FAILURE);
    }
    //printf("Listening on socket\n");

    printf("SBitX v3 HAMLIB Server (normal + extended RIGCTL protocol) listening on port 4532...\n");

    while (running)
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(welcome_socket, &readfds);

        pthread_mutex_lock(&client_mutex);
        int max_sd = get_max_sd();
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            int sd = client_sockets[i];
            if (sd > 0) FD_SET(sd, &readfds);
        }
        pthread_mutex_unlock(&client_mutex);

        // Wait up to 100ms for activity
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 100000;
        int activity = select(max_sd + 1, &readfds, NULL, NULL, &tv);
        if (activity < 0 && errno != EINTR)
        {
            perror("select error");
            break;
        }

        // Check for new connections
        if (FD_ISSET(welcome_socket, &readfds))
        {
            int new_sd = accept(welcome_socket, (struct sockaddr *) &client_addr, &addr_len);
            if (new_sd >= 0)
            {
                // fcntl(new_sd, F_SETFL, O_NONBLOCK);
                set_socket_non_blocking(new_sd);
                printf("New client on socket %d\n", new_sd);

                pthread_mutex_lock(&client_mutex);
                int assigned = 0;
                for (int i = 0; i < MAX_CLIENTS; i++)
                {
                    if (client_sockets[i] == 0)
                    {
                        client_sockets[i] = new_sd;
                        if (inet_ntop(AF_INET, &client_addr.sin_addr, client_ips[i], INET_ADDRSTRLEN) == NULL)
                        {
                            perror("inet_ntop");
                        }
                        else
                        {
                            printf("Client IP: %s\n", client_ips[i]);
                        }
                        assigned = 1;
                        break;
                    }
                }
                if (!assigned)
                {
                    printf("Too many clients.\n");
                    close(new_sd);
                }
                pthread_mutex_unlock(&client_mutex);
            }
            else
            {
                if (errno != EAGAIN && errno != EWOULDBLOCK)
                {
                    perror("accept error");
                }
            }
        }

        pthread_mutex_lock(&client_mutex);
        // Check for client data
        for (int i = 0; i < MAX_CLIENTS; i++)
        {
            int sd = client_sockets[i];
            if (sd > 0 && FD_ISSET(sd, &readfds))
            {
                char buffer[1024];
                int rlen = recv(sd, buffer, sizeof(buffer), 0);
                if (rlen > 0)
                {
                    buffer[rlen] = '\0';
                    handle_client_data(sd, buffer, rlen);
                }
                else if (rlen == 0 || (rlen < 0 && errno != EAGAIN && errno != EWOULDBLOCK))
                {
                    printf("Client %d disconnected.\n", sd);
                    close(sd);
                    client_sockets[i] = 0;
                    memset(client_ips[i], 0, INET_ADDRSTRLEN);

                }
            }
        }
        pthread_mutex_unlock(&client_mutex);
    }

    // Clean up
    close(welcome_socket);
    pthread_mutex_lock(&client_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++)
    {
        if (client_sockets[i] > 0)
        {
            close(client_sockets[i]);
            client_sockets[i] = 0;
        }
    }
    pthread_mutex_unlock(&client_mutex);
    return NULL;
}

// Start the HAMLIB listener thread
void start_hamlib_listener()
{
    //   printf("[DEBUG] start_hamlib_listener\n");
    pthread_t tid;
    if (pthread_create(&tid, NULL, server_thread, NULL) != 0)
    {
        perror("Failed to create server thread");
        exit(1);
    }
    pthread_detach(tid);
}

// Stop the HAMLIB listener thread
void stop_hamlib_listener()
{
    //   printf("[DEBUG] stop_hamlib_listener\n");
    running = 0;
    close(welcome_socket);
}

// Start the HAMLIB listener thread
void *start_listener_thread(void *arg)
{
    //   printf("[DEBUG] start_listener_thread\n");
    start_hamlib_listener();
    return NULL;
}

// Initialize the HAMLIB listener thread
void initialize_hamlib()
{
    //   printf("[DEBUG] initialize_hamlib\n");
    pthread_t tid;
    if (pthread_create(&tid, NULL, start_listener_thread, NULL) != 0)
    {
        perror("Failed to create listener thread");
        exit(1);
    }
    pthread_detach(tid);
}






/*****************************************************************************
 * set_func
 *
 * Fallback for: "U, set_func 'Func' 'Func Status'"
 *
 *  - "Func" can be NB, COMP, VOX, TONE, TSQL, etc.
 *  - "Func Status" is a non-null value indicating on/off (1 or 0).
 *
 * If you are also using the property dictionary for these, you can remove or
 * reduce these checks for "ANR", "DSP", etc. Instead, if you prefer the old
 * logic, keep it here.
 *****************************************************************************/
int set_func(const char *func, const char *value)
{
    //  printf("[DEBUG] set_func: func=%s, value=%s\n", func, value);
    // Validate the function name
    if (func == NULL)
    {
        fprintf(stderr, "set_func: Invalid function name (NULL)\n");
        return -1; // Error
    }

    // If user typed '?', we might want to list supported funcs in older code, etc.
    if (strcmp(func, "?") == 0)
    {
        // TOODO: hm...no socket...
        //send_response(client_socket, (char *) "NB COMP ANR TUNER RIT\n");
        return 0;
    }

    // Convert "0"/"1" to "OFF"/"ON" for some commands:
    static char onOff[4];
    if (strcmp(value, "1") == 0)
        strcpy(onOff, "ON");
    else
        strcpy(onOff, "OFF");

    if (strcmp(func, "ANF") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "NOTCH %s", onOff);
        execute_command(buffer);
        return 0;
    }
    else if (strcmp(func, "FAGC") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "AGC %s", strcmp(value, "1") == 0 ? "FAST" : "OFF");
        execute_command(buffer);
        return 0;
    }
    else if (strcmp(func, "DSP") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "DSP %s", onOff);
        execute_command(buffer);
        return 0;
    }
    else if (strcmp(func, "LOCK") == 0)
    {
        is_locked = (strcmp(value, "1") == 0);
        return 0;
    }
    else if (strcmp(func, "NOTCH") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "NOTCH %s", onOff);
        execute_command(buffer);
        return 0;
    }
    else if (strcmp(func, "TUNER") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "TUNE %s", onOff);
        execute_command(buffer);
        return 0;
    }
    else if (strcmp(func, "NR") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "ANR %s", onOff ? "ON" : "OFF");
        execute_command(buffer);
        return 0;
    }
    else if (strcmp(func, "COMP") == 0) // Function for compatability
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "COMP %s", strcmp(value, "1") == 0 ? "5" : "0");
        execute_command(buffer);
        return 0;
    }
    else if (strcmp(func, "DEBUG") == 0) // Trun debugging on and off
    {
        if (strcmp(value, "1") == 0)
            is_debug = 1;
        else
            is_debug = 0;
        return 0;
    }



    // or fallback to setting a field:
    if (field_set(func, (const char *) value) != -1)
        return 0;

    fprintf(stderr, "set_func: Unsupported function '%s'\n", func);
    return -2; // Unsupported
}

//Helper:  Read a function's value from a toggle field
char *field_toggle(const char *label)
{
    struct field *f = get_field_by_label((char *) label);  // Lookup the field by its label
    if (f == NULL || f->value_type != FIELD_TOGGLE)
    {
        return "";  // Error: field not found or not a toggle
    }
    return f->value;
}

/*****************************************************************************
 * command_get_func
 *
 * Fallback for: "u, get_func 'Func'"
 *
 * If your dictionary-based approach fails to find "Func", or you prefer the old
 * logic, we do these toggles, e.g. reading 'field_toggle()' or 'field_int()'.
 *****************************************************************************/
void command_get_func(int client_socket, const char *func)
{
    // printf("[DEBUG] command_get_func: func=%s\n", func);
    // Old logic from your snippet:
    //  - If "?" => list supported.
    //  - If we find a toggle (ON/OFF), return 1 or 0.
    //  - If "COMP", read "COMP" int, etc.
    //  - If not found, RPRT -11.


    if (strchr(func, '?'))
    {
        // Return a space-separated list of radio backend supported get function tokens
        // e.g. "NB COMP TUNER ANR ..."
        add_response(client_socket, (char *) "NB COMP ANR TUNER RIT\n");
        return;
    }

    // Attempt reading from a toggle field, for instance:
    char *field_toggle(const char *lbl);
    extern char *field_toggle(const char *lbl);

    const char *toggle_value = field_toggle(func); // returns "" if not found
    if (toggle_value && toggle_value[0])
    {
        // e.g. if toggle_value=="ON" => "1"
        sprintf(resp, "%d\n", (strcmp(toggle_value, "ON") == 0) ? 1 : 0);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(func, "COMP") == 0)
    {
        int compression = field_int("COMP");
        sprintf(resp, "%d\n", compression > 0 ? 1 : 0);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(func, "DEBUG") == 0)
    {
        sprintf(resp, "%d\n", is_debug);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(func, "LOCK") == 0)
    {
        sprintf(resp, "%d\n", is_locked);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(func, "NR") == 0)
    {
        char field_value[100];
        if (get_field_value_by_label((char *) "ANR", field_value) == -1)
        { // Not found :-(  RPRT -11
            add_response(client_socket, (char *) "RPRT -11\n");
            return;
        }
        else
        {
            sprintf(resp, "%d\n", strcmp(field_value, "ON") == 0 ? 1 : 0);
            add_response(client_socket, resp);
            return;
        }
    }
    else if (strcmp(func, "TUNER") == 0)
    {
        char field_value[100];
        if (get_field_value_by_label((char *) "TUNER", field_value) == -1)
        { // Not found :-(  RPRT -11
            add_response(client_socket, (char *) "RPRT -11\n");
            return;
        }
        else
        {
            sprintf(resp, "%d\n", strcmp(field_value, "ON") == 0 ? 1 : 0);
            add_response(client_socket, resp);
            return;
        }
    }

    /* else if (whatever) ... */

    // If still not found, attempt a direct field_value read:
    char field_value[100];
    strcpy(field_value, "");
    if (get_field_value_by_label((char *) func, field_value) == -1)
    { // Not found :-(  RPRT -11
        add_response(client_socket, (char *) "RPRT -11\n");
    }
    else
    { // Found :-)  Return the value
        sprintf(resp, "%s\n", field_value);
        add_response(client_socket, resp);
    }
}

/*****************************************************************************
 * command_set_level
 *
 * Fallback for: "L, set_level 'Level' 'Value'"
 *
 * If your property dictionary doesn't recognize the level, we do this older code.
 *****************************************************************************/

// TODO: Range checks that are unsupported as yet by SBITX Fields.
hamlib_error_t command_set_level(const char *level, float value)
{

    if (is_debug)
        printf("command_set_level %s to %f\n", level, value);
    // Copied from your snippet
    char sdr_cmd[256];

    if (strcmp(level, "RFPOWER") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "DRIVE %d", (int) ((float) value * 100.0));
    }
    else if (strcmp(level, "AF") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "AUDIO %d", (int) ((float) value * 100.0));
    }
    else if (strcmp(level, "RF") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "IF %d", (int) ((float) value * 100.0));
    }
    else if (strcmp(level, "MICGAIN") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "MIC %d", (int) ((float) value * 100.0));
    }
    else if (strcmp(level, "NR") == 0) // Feature compatibility with rigctld
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "ANR %d", value > 0 ? "ON" : "OFF"); // String!!!
    }
    else if (strcmp(level, "MONITOR_GAIN") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "TXMON %d", (int) ((float) value * 100.0));
    }
    else if (strcmp(level, "MIC") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "MIC %d", (int) value);
    }
    else if (strcmp(level, "COMP") == 0 || strcmp(level, "COMPRESS") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "COMP %d", (int) value);
    }
    else if (strcmp(level, "RXGAIN") == 0 || strcmp(level, "IF") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "IF %d", (int) value);
    }
    else if (strcmp(level, "VOLUME") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "AUDIO %d", (int) value);
    }
    else if (strcmp(level, "LOWCUT") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "LOW %d", (int) value);
    }
    else if (strcmp(level, "HIGHCUT") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "LOW %d", (int) value);  // possibly a bug from original?
    }
    else if (strcmp(level, "BRIDGE") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "bridge=%d", (int) value);
    }
    else if (strcmp(level, "SIDETONE") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "SIDETONE %d", (int) value);
    }
    else if (strcmp(level, "FAGC") == 0)
    {
        switch ((int) value)
        {
            case 0:
                snprintf(sdr_cmd, sizeof(sdr_cmd), "AGC OFF");
                break;
            case 1:
                snprintf(sdr_cmd, sizeof(sdr_cmd), "AGC FAST");

                break;
            case 2:
                snprintf(sdr_cmd, sizeof(sdr_cmd), "AGC MED");
                break;
            case 3:
                snprintf(sdr_cmd, sizeof(sdr_cmd), "AGC SLOW");
                break;
            default:
                snprintf(sdr_cmd, sizeof(sdr_cmd), "AGC FAST"); // For kenwood
        }

    }
    else
    {
        return -1; // Invalid parameter
    }

    if (is_debug)
        printf("command_set_level EXECUTE: %s\n", sdr_cmd);
    execute_command(sdr_cmd);
    return HAMLIB_OK;  // success
}

// `RIG_LEVEL_STRENGTH: \a val is an integer, representing the S Meter
//level in dB relative to S9, according to the ideal S Meter scale.
//The ideal S Meter scale is as follow: S0=-54, S1=-48, S2=-42, S3=-36,
//S4=-30, S5=-24, S6=-18, S7=-12, S8=-6, S9=0, +10=10, +20=20,
//+30=30, +40=40, +50=50 and +60=60. This is the responsibility
//of the backend to return values calibrated for this scale.
//The frontend will then apply the calibration to the actual S Meter
//scale of the radio, and display the result to the user.
//The value is a signed integer, representing the S Meter level in dB
//relative to S9, according to the ideal S Meter scale.
//The ideal S Meter scale is as follow: S0=-54, S1=-48, S2=-42, S3=-36,
//S4=-30, S5=-24, S6=-18, S7=-12, S8=-6, S9=0, +10=10, +20=20,
extern struct rx *rx_list;

extern int calculate_s_meter(struct rx *r, double rx_gain);

float get_rig_level_strength()
{
    // Calculate the S-Meter level based on the current rx state
    struct rx *r = rx_list;
    double rx_gain = (double) get_rx_gain();
    int s_meter = calculate_s_meter(r, rx_gain);

    // Convert S-meter value to dB relative to S9
    // (maybe a tad low, but damn close)
    float db_relative_to_s9 = -54.0f + 6.0f * (s_meter / 100.0f);
    return db_relative_to_s9;
}

/*****************************************************************************
 * command_get_level
 *
 * Fallback for: "l, get_level 'Level'"
 *
 * If dictionary doesn't find the property, we do this older code.
 *****************************************************************************/
void command_get_level(int client_socket, const char *level)
{
    //   printf("[DEBUG] command_get_level: level=%s\n", level);
    int value = 0;

    if (strcmp(level, "RFPOWER") == 0)
    {
        float drive = (float) field_int("DRIVE") / 100.0f;
        snprintf(resp, sizeof(resp), "%1.2f\n", drive);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "MIC") == 0)
    {
        value = field_int("MIC");
    }
    else if (strcmp(level, "MICGAIN") == 0) // Handle floating value
    {
        float value = (float) field_int("MIC");
        snprintf(resp, sizeof(resp), "%.2f\n", value / 100.);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "COMP") == 0 || strcmp(level, "COMPRESS") == 0)
    {
        value = field_int("COMP");
    }
    else if (strcmp(level, "RXGAIN") == 0)
    {
        value = field_int("IF");
    }
    else if (strcmp(level, "DSP") == 0)
    {
        value = field_int("DSP");
    }
    else if (strcmp(level, "LOCK") == 0)
    {
        value = is_locked;
    }
    else if (strcmp(level, "ANR") == 0 || strcmp(level, "NR") == 0)
    {
        char strValue[100];
        if (get_field_value_by_label((char *) "ANR", strValue) != -1)
        {
            int iValue = 0;
            // possible values for value are OFF SLOW MED and FAST
            // CHeck each one and map OFF to 0, FAST to 1, MED to 2, SLOW TO 3,
            // and return the result
            if (strcmp(strValue, "OFF") == 0)
                iValue = 0;
            else
                iValue = 1;
            snprintf(resp, sizeof(resp), "%d\n", iValue);
            add_response(client_socket, resp);
            return;
        }
    }
    else if (strcmp(level, "TUNER") == 0)
    {
        value = field_int("TUNE");
    }
    else if (strcmp(level, "DEBUG") == 0)
    {
        value = is_debug;
    }
    else if (strcmp(level, "TXMON") == 0)
    {
        value = field_int("TXMON");
    }
    else if (strcmp(level, "MONITOR_GAIN") == 0)
    {
        // Handle floating point variant of "TXMON"
        float fValue = (float) field_int("TXMON");
        snprintf(resp, sizeof(resp), "%.2f\n", fValue / 100.0);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "VOLUME") == 0)
    {
        value = field_int("AUDIO");
    }
    else if (strcmp(level, "AF") == 0)
    {
        float fValue = (float) field_int("AUDIO");
        snprintf(resp, sizeof(resp), "%.2f\n", fValue / 100.0);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "RF") == 0)
    {
        float fValue = (float) field_int("IF");
        snprintf(resp, sizeof(resp), "%.2f\n", fValue / 100.0);
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "SWR") == 0)
    {
        char meter_str[100];
        int vswr = field_int("REF");
        int power = field_int("POWER");

        // power is in 1/10th of watts and vswr is also 1/10th
        if (power < 30)
            vswr = 10;

        float swr = (float) vswr / 10.0;
        snprintf(resp, sizeof(resp), "%.2f\n", swr);
        add_response(client_socket, resp);
        return;

    }
    else if ((strcmp(level, "FAGC") == 0 || strcmp(level, "AGC") == 0)) // Compatability with standard
    {
        char strValue[100];
        if (get_field_value_by_label((char *) "AGC", strValue) == -1)
        {
            int iValue = 0;
            // possible values for value are OFF SLOW MED and FAST
            // CHeck each one and map OFF to 0, FAST to 1, MED to 2, SLOW TO 3,
            // and return the result
            if (strcmp(strValue, "OFF") == 0)
                iValue = 0;
            else if (strcmp(strValue, "FAST") == 0)
                iValue = 1;
            else if (strcmp(strValue, "MED") == 0)
                iValue = 2;
            else if (strcmp(strValue, "SLOW") == 0)
                iValue = 3;
            snprintf(resp, sizeof(resp), "%d\n", iValue);
            add_response(client_socket, resp);
            return;
        }
    }
    else if (strcmp(level, "LOWCUT") == 0)
    {
        value = field_int("LOW");
    }
    else if (strcmp(level, "HIGHCUT") == 0)
    {
        value = field_int("HIGH");
    }
    else if (strcmp(level, "BRIDGE") == 0)
    {
        value = field_int("BRIDGE");
    }
    else if (strcmp(level, "SIDETONE") == 0)
    {
        value = field_int("SIDETONE");
    }
    else if (strcmp(level, "STRENGTH") == 0)
    {

        snprintf(resp, sizeof(resp), "%i\n", (int) get_rig_level_strength());
        add_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "METER") == 0)
    {
        snprintf(resp, sizeof(resp), "%2.1f\n", get_rig_level_strength);
        add_response(client_socket, resp);
        return;
    }
    else
    {
        /* not recognized in fallback => try reading direct field. */
        char field_value[100] = "";
        if (get_field_value_by_label((char *) level, field_value) == -1)
        {
            add_response(client_socket, (char *) "RPRT -1\n"); // RIG_EINVAL
        }
        else
        {
            snprintf(resp, sizeof(resp), "%s\n", field_value);
            add_response(client_socket, resp);
        }
        return;
    }
    if (is_debug) printf("Got level: %d\n", value);
    snprintf(resp, sizeof(resp), "%d\n", value);
    add_response(client_socket, resp);
}

