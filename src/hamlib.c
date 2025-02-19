/*****************************************************************************
 * hamlib.c
 *
 * A self-contained Hamlib-compatible TCP server that:
 *   - Listens on port 4532 (non-blocking + select).
 *   - Parses Single-letter (e.g. "F 14000000"), Textual (e.g. "\set_freq 14000000"),
 *     and Extended commands (e.g. "+\set_freq 14000000").
 *   - Has a PROPERTY DICTIONARY for levels (RFPOWER, MICGAIN, COMP, etc.)
 *     and for boolean functions (NB, DSP, TUNER, etc.) if you like.
 *   - Merges set_func, get_func, set_level, get_level to optionally consult
 *     the property table for validation and range checking. (NOT IMPLEMENTED  BECAUSE SBITX.X DOES IT)
 *   - Each command block has rigctld(8) doc comments.
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
#include "hamlib.h"
#define PORT         4532
#define PRODUCT      "SBITX v3 Hybrid SDR"
#define VERSION      "4.3"
#define MAX_CLIENTS  10
#define MAX_DATA     1000
#define DEBUG        0

// TODO: Optimize the return sequence so that there's only ONE send_response() call per command.
//       Implemented this for get_vfo_info but no others, yet...

static int client_sockets[MAX_CLIENTS] = {0};
static int welcome_socket = -1;
static volatile int running = 1;
static pthread_mutex_t client_mutex = PTHREAD_MUTEX_INITIALIZER;

char incoming_data[MAX_CLIENTS][MAX_DATA];
int incoming_ptr[MAX_CLIENTS] = {0};
char resp[MAX_DATA] = {0}; // Used for the large responses
bool is_debug = DEBUG;
bool is_locked = false; // This is used to lock the radio from changing settings. However
                        // it is not implemented in the radio at the moment, just helps
                        // WSJT-X with some of it's queries.

/*****************************************************************************
 * FROM YOUR ORIGINAL CODE / EXTERNALS:
 *****************************************************************************/
extern struct field *get_field_by_label(char *label);

extern long get_freq(void);

extern int get_default_passband_bw(void);

extern int get_passband_bw(void);

extern int field_int(char *label);

extern int field_set(const char *label, const char *new_value);

extern int get_field_value_by_label(char *label, char *val);

extern void cmd_exec(char *cmd);

extern void remote_execute(char *cmd);

extern void sdr_request(char *cmd, char *out);

extern void hamlib_tx(int onoff);

extern const char *field_str(const char *label);

extern void send_rprt(int client_socket, int code);

extern void send_response(int client_socket, char *response); // (already defined above)
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
typedef struct {
    const char *external_name; // Hamlib property name
    const char *internal_name; // Radio's property name
} property_mapping_t;


/* Example table: NB, COMP => booleans; RFPOWER => float range; etc. */
static property_definition_t property_table[] = {
        {"DSP",       PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"ANR",       PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"NOTCH",     PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"TUNE",      PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"REC",       PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"KBD",       PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"TXEQ",      PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"RXEQ",      PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"LOCK",      PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"VFOLK",     PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"AUTO",      PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"KBD",       PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"FT8_AUTO",  PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"FT8_TX1ST", PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"RIT",       PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"SPLIT",     PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"RS",        PT_BOOLEAN, 0.0f,     1.0f,    0},
        {"TA",        PT_BOOLEAN, 0.0f,     1.0f,    0},


        {"RFPOWER",   PT_FLOAT,   0.0f,     1.0f,    0},
        {"RFPOWER_METER", PT_FLOAT, 0.0f, 100.0f, 0},
        {"MICGAIN",   PT_FLOAT,   0.0f,     100.0f,  0},
        {"COMP",      PT_FLOAT,   0.0f,     100.0f,  0},
        {"AUDIO",     PT_FLOAT,   0.0f,     100.0f,  0},
        {"VOLUME",    PT_FLOAT,   0.0f,     100.0f,  0},
        {"BW",        PT_FLOAT,   100.0f,   6000.0f, 0},
        {"DRIVE",     PT_FLOAT,   0.0f,     100.0f,  0},
        {"IF",        PT_FLOAT,   0.0f,     100.0f,  0},
        {"MIC",       PT_FLOAT,   0.0f,     100.0f,  0},
        {"LOWCUT",    PT_FLOAT,   50.0f,    5000.0f, 0},
        {"HIGHCUT",   PT_FLOAT,   50.0f,    5000.0f, 0},
        {"WFMIN",     PT_FLOAT,   0.0f,     200.0f,  0},
        {"WFSPD",     PT_FLOAT,   20.0f,    150.0f,  0},
        {"WFMAX",     PT_FLOAT,   0.0f,     200.0f,  0},
        {"SCOPEGAIN", PT_FLOAT,   1.0f,     25.0f,   0},
        {"SCOPESIZE", PT_FLOAT,   50.0f,    150.0f,  0},
        {"INTENSITY", PT_FLOAT,   2.0f,     10.0f,   0},
        {"NFREQ",     PT_FLOAT,   60.0f,    3000.0f, 0},
        {"BNDWTH",    PT_FLOAT,   60.0f,    1000.0f, 0},
        {"TNDUR",     PT_FLOAT,   2.0f,     30.0f,   0},
        {"TNPWR",     PT_FLOAT,   0.0f,     100.0f,  0},
        {"TXMON",     PT_FLOAT,   0.0f,     100.0f,  0},
        {"BFO",       PT_FLOAT,   -2995.0f, 3000.0f, 0},
        {"TX_PITCH",  PT_FLOAT,   0.0f,     5000.0f, 0},
        {"FT8_REPEAT", PT_FLOAT,   0.0f,     10.0f,   0},
        {"PITCH",     PT_FLOAT,   -5000.0f, 5000.0f, 0},
        {"WPM",       PT_FLOAT,   0.0f,     100.0f,  0},
        {"CW_DELAY",  PT_FLOAT,   0.0f,     100.0f,  0},
        {"METER",     PT_FLOAT,   0.0f,     1500.0,  0},
        {"STRENGTH",     PT_FLOAT,  -54.0f,  100.0,  0},
        {"SWR",     PT_FLOAT,     0.0f,      100.0,  0},
        {"REF",     PT_FLOAT,     0.0f,      100.0,  0},
        {"POWER",     PT_FLOAT,   0.0f,      100.0,  0},


        {"AGC",       PT_STRING,  0.0f,     100.0f,  0},
 //       {"RIT",       PT_STRING,  0.0f,     100.0f,  0}, // Radio does support this... don't understand XIT usage though as not on GUI.
 //       {"XIT",       PT_STRING,  0.0f,     100.0f,  0},
        {"STEP",      PT_STRING,  0.0f,     100.0f,  0},
        {"MENU",      PT_STRING,  0.0f,     100.0f,  0},
        {"SPLIT",     PT_STRING,  0.0f,     100.0f,  0},
        {"VFO",       PT_STRING,  0.0f,     100.0f,  0},
        {"SPAN",      PT_STRING,  0.0f,     100.0f,  0},
        {"SPECT",     PT_STRING,  0.0f,     100.0f,  0},
        {"MODE",      PT_STRING,  0.0f,     100.0f,  0},
        {"CW_INPUT",  PT_STRING,  0.0f,     100.0f,  0},
        {"CALL",      PT_STRING,  0.0f,     100.0f,  0},
        {"SENT",      PT_STRING,  0.0f,     100.0f,  0},
        {"RECV",      PT_STRING,  0.0f,     100.0f,  0},
        {"EXCH",      PT_STRING,  0.0f,     100.0f,  0},
        {"NR",        PT_STRING,  0.0f,     100.0f,  0},
        {"MENU",      PT_STRING,  0.0f,     100.0f,  0},
        {"F1",        PT_STRING,  0.0f,     100.0f,  0},
        {"F2",        PT_STRING,  0.0f,     100.0f,  0},
        {"F3",        PT_STRING,  0.0f,     100.0f,  0},
        {"F4",        PT_STRING,  0.0f,     100.0f,  0},
        {"F5",        PT_STRING,  0.0f,     100.0f,  0},
        {"F6",        PT_STRING,  0.0f,     100.0f,  0},
        {"F7",        PT_STRING,  0.0f,     100.0f,  0},
        {"F8",        PT_STRING,  0.0f,     100.0f,  0},
        {"F9",        PT_STRING,  0.0f,     100.0f,  0},
        {"F10",       PT_STRING,  0.0f,     100.0f,  0},
        {NULL,        0,          0.0f,     0.0f,    0} /* sentinel */
};

static property_mapping_t property_mapping_table[] = {
    {"MICGAIN", "MIC"},  // Map MICGAIN to SBITX Field MIC
    {"LOWCUT", "LOW"},
    {"HIGHCUT", "HIGH"},
    {"METER", "STRENGTH"},  // This one is a function calculating in DB function
    {"VOLUME", "AUDIO"},
    {"NR", "ANR"},
    {"RFPOWER_METER", "POWER"},
    {"VOLUME", "AUDIO"}

};

// Remap property names if required, you'll find this in the level and func functions...

const char *resolve_property_name(const char *external_name)
{
    for (int i = 0; i < sizeof(property_mapping_table) / sizeof(property_mapping_table[0]); ++i)
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
                        printf("[DEBUG] Setting boolean %s=ON\n", prop_name);
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
                if(get_field_value_by_label((char *) prop_name, val) == -1)
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
                if(get_field_value_by_label((char *) prop_name, val)==-1)
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
                if(get_field_value_by_label((char *) prop_name, val) == -1)
                  return false;
                snprintf(out_buf, out_sz, "%s", val);
                return true;
            }
        }
    }
    return false; /* not found */
}

/*****************************************************************************
 * send_response + "RPRT x" helper
 *****************************************************************************/

void send_rprt(int client_socket, int code)
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
    if (!line || !line[0]) return LINE_MODE_INVALID;
    if (line[0] == '+')
    {
        memmove(line, line + 2, strlen(line));
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
    CMD_QUIT
} command_id_t;

// Parse a command name from a string to a token, e.g. "set_freq" => CMD_SET_FREQ
// The main reason for this is to allow a simple switch statement to process all the commands despite their 3 different formats.

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
    if ( strcasecmp(cmd_str, "set_lock_mode") == 0)
        return CMD_SET_LOCK;
    if (strcasecmp(cmd_str, "get_lock_mode") == 0)
        return CMD_GET_LOCK;

    if (strcasecmp(cmd_str, "get_clock") == 0)
        return CMD_GET_CLOCK;
    return CMD_INVALID;
}

/*****************************************************************************
 * Now the hamlib_* functions, each with rigctld doc snippet and logic.
 * plus property dictionary and mapping calls where relevant.
 *****************************************************************************/

/*
 * F, set_freq 'Frequency'
 * "Set 'Frequency', in Hz."
 *
 */
static int hamlib_set_freq(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;
    if (is_extended)
    {
        long freq_val = atol(argv[0]);
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_freq %ld:\nFreq: %ld\n", freq_val, freq_val);
        send_response(cs, tmp);
    }
    {
        /* your original code: */
        long freq;
        char cmd[50];
        const char *f = argv[0];

        if (!strncmp(f, "VFO", 3)) // Odd but possible as in F VFOA 14200000
            freq = atoi(f + 5);
        else
            freq = atol(f);

        sprintf(cmd, "freq %ld", freq);
        cmd_exec(cmd);

        if (!is_extended)
        {
            send_response(cs, (char *) "RPRT 0\n");
        }
    }
    return 0;
}

/*
 * f, get_freq
 * "Get 'Frequency', in Hz."
 */
static int hamlib_get_freq(int cs, int is_extended, char *argv[], int argc)
{
    if (is_extended)
    {
        send_response(cs, (char *) "get_freq:\n");
    }
    char response[20];
    if (is_extended)
        sprintf(response, "Freq: %ld\n", get_freq());
    else
        sprintf(response, "%ld\n", get_freq());
    send_response(cs, response);

    return 0;
}

/*
 * M, set_mode 'Mode' 'Passband'
 * "Set 'Mode' (USB, LSB, etc.) and 'Passband' or '0' for default"
 */
static int hamlib_set_mode(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;

    const char *supported_modes[] = {"USB", "LSB", "CW", "CWR", "DIGI", "AM", "PKTUSB", "FT8"};
    char mode[32] = "";
    char passband[32] = "0";

    strcpy(mode, argv[0]);
    if (argc > 1) strcpy(passband, argv[1]);

    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp), "set_mode %s %s:\nMode: %s\nPassband: %s\n", mode, passband, mode, passband);
        send_response(cs, tmp);
    }

    if (strcmp(argv[0], "?") == 0)
    {
        strcpy(resp, "");
        for (int i = 0; i < sizeof(supported_modes) / sizeof(supported_modes[0]); i++)
        {
            strcat(resp, supported_modes[i]);
            strcat(resp, " ");
        }
        strcat(resp, "\n");
        send_response(cs, resp);
        return 0;
    }

    // Support for DIGI mode from programs likw WSJT-X
    if (strcmp(mode, "PKTUSB") == 0) strcpy(mode, "DIGI");

    int found = 0;
    for (int i = 0; i < sizeof(supported_modes) / sizeof(supported_modes[0]); i++)
    {
        if (strcmp(mode, supported_modes[i]) == 0)
        {
            found = 1;
            break;
        }
    }
    if (!found)
    {
        send_response(cs, (char *)"RPRT -9\n");
        return -9;
    }

    char cmd[50];
    sprintf(cmd, "mode %s", mode);
    cmd_exec(cmd);

    if (strcmp(passband, "0") == 0)
    {
        char bw_str[10];
        sprintf(bw_str, "%d", get_default_passband_bw());
        field_set("BW", bw_str);
    }
    else
    {
        field_set("BW", passband);
    }

    if (!is_extended)
    {
        send_response(cs, (char *)"RPRT 0\n");
    }

    return 0;
}


/*
 * m, get_mode
 * "Get 'Mode' and 'Passband'. e.g. 'USB 2400'."
 */
static int hamlib_get_mode(int cs, int is_extended, char *argv[], int argc)
{
    const char *supported_modes[] = {"USB", "LSB", "CW", "CWR", "DIGI", "AM", "PKTUSB", "FT8"};
    char mode[20], passband[20], response[50];

    if (is_extended)
    {
        send_response(cs, (char *)"get_mode:\n");
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
        send_response(cs, resp);
        return 0;
    }

    get_field_value_by_label("MODE", mode);
    get_field_value_by_label("BW", passband);

    if (strcmp(mode, "DIGI") == 0)
    {
        strcpy(mode, "PKTUSB");
    }

    snprintf(response, sizeof(response), is_extended ? "Mode: %s\nPassband: %s\n" : "%s\n%s\n", mode, passband);
    send_response(cs, response);

    return 0;
}

/*
 * V, set_vfo 'VFO'
 * "Set 'VFO': VFOA, VFOB, etc."
 */
static int hamlib_set_vfo(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_vfo %s:\nVFO: %s\n", argv[0], argv[0]);
        send_response(cs, tmp);
    }
    {
        char tmp[5];
        strcpy(tmp, argv[0] + 3);
        field_set("VFO", tmp);
        if (!is_extended)
        {
            send_response(cs, (char *) "RPRT 0\n");
        }
    }
    return 0;
}

/*
 * v, get_vfo
 * "Get current 'VFO'. e.g. 'VFOA' or 'VFOB'."
 */
static int hamlib_get_vfo(int cs, int is_extended, char *argv[], int argc)
{
    if (is_extended)
    {
        send_response(cs, (char *) "get_vfo:\n");
    }
    {
        char currVFO[2];
        get_field_value_by_label("VFO", currVFO);
        if (is_extended)
        {
            char tmp[128];
            snprintf(tmp, sizeof(tmp),
                     "VFO: %s\n", currVFO);
            send_response(cs, tmp);
            return 0;
        }
        if (currVFO[0] == 'A')
            send_response(cs, (char *) "VFOA\n");
        else
            send_response(cs, (char *) "VFOB\n");
    }
    return 0;
}

/*
 * S, set_split_vfo 'Split' 'TX VFO'
 * "Set 'Split' mode, '0' or '1', plus TX VFO, e.g. 'VFOB'."
 */
static int hamlib_set_split(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;

    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp), "set_split_vfo %s %s\n", argv[0], argc == 2 ? argv[1] : "");
        send_response(cs, tmp);
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

    if (!is_extended)
    {
        send_response(cs, "RPRT 0\n");
    }

    return 0;
}

/*
 * s, get_split_vfo
 * "Get 'Split' mode, '0' or '1', plus TX VFO if split=1."
 */
static int hamlib_get_split(int cs, int is_extended, char *argv[], int argc)
{
    char curr_split[4];
    get_field_value_by_label("SPLIT", curr_split);

    if (is_extended)
    {
        send_response(cs, (char *)"get_split_vfo:\n");
    }

    if (strcmp(curr_split, "OFF") == 0)
    {
        send_response(cs, is_extended ? (char *)"Split: 0\n" : (char *)"0\n");

        char vfo[4];
        get_field_value_by_label("VFO", vfo);

        char tmp[128];
        snprintf(tmp, sizeof(tmp), is_extended ? "TX VFO: VFO%s\n" : "VFO%s\n", vfo);
        send_response(cs, tmp);
    }
    else
    {
        send_response(cs, (char *)"1\n");
        if (is_extended)
        {
            send_response(cs, (char *)"TX VFO: VFOB\n");
        }
        else
        {
            send_response(cs, (char *)"VFOB\n");
        }
    }

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
    if (argc < 2) return -11;
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_func %s %s:\n%s: %s\n", argv[0], argv[1], argv[0], argv[1]);
        send_response(cs, tmp);
    }

    /* We can attempt to set it via the property table. If not found, fallback to your old code. */
    {
        const char *mapped_func_name = resolve_property_name(argv[0]);
        /* For a function named argv[0] (e.g. "NB") we pass argv[1] ("1") */
        if (!sdr_radio_set_property(mapped_func_name, argv[1]))
        {
            /* fallback to your old set_func code if you want: */
            extern int set_func(const char *, const char *);
            int ret = set_func(mapped_func_name, argv[1]);
            if (ret != 0)
            {
                send_response(cs, (char *) "RPRT -11\n");
            }
            else
            {
                if (!is_extended) send_response(cs, (char *) "RPRT 0\n");
            }
        }
        else
        {
            /* success using property table */
            if (!is_extended)
                send_response(cs, (char *) "RPRT 0\n");
        }
    }
    return 0;
}


/*
 * u, get_func 'Func'
 * "Get 'Func' status. Return '1' if on, '0' if off, etc."
 */
static int hamlib_get_func(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;

    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp), "get_func %s:\n", argv[0]);
        send_response(cs, tmp);
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
        send_response(cs, resp);
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
            send_response(cs, tmp);
        }
        else
        {
            send_response(cs, out_buf);
        }
    }

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
    if (argc < 2) return -11;
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_level %s %s:\n%s: %s\n", argv[0], argv[1], argv[0], argv[1]);
        send_response(cs, tmp);
    }
    const char *mapped_level_name = resolve_property_name(argv[0]);
//    printf("Mapped level name is %s\n", mapped_level_name);

    /* Attempt property-based approach: */
    if (!sdr_radio_set_property(mapped_level_name, argv[1]) || strcmp(argv[0], "RFPOWER") == 0)
    {
        /* fallback to your older 'command_set_level(...)' code: */
        extern hamlib_error_t command_set_level(const char *, float);
        float val = (float) atof(argv[1]);
        hamlib_error_t err = command_set_level(mapped_level_name, val);
        if (err == HAMLIB_OK)
        {
            if (!is_extended) send_response(cs, (char *) "RPRT 0\n");
        }
        else
        {
            if (err == HAMLIB_ERR_INVALID_PARAM)
                send_response(cs, (char *) "RPRT -11\n");
            else if (err == HAMLIB_ERR_NOT_IMPLEMENTED)
                send_response(cs, (char *) "RPRT -12\n");
            else
                send_response(cs, (char *) "RPRT -13\n");
        }
    }
    else
    {
        /* success with property table */
        if (!is_extended)
        {
            send_response(cs, (char *) "RPRT 0\n");
        }
    }
    return 0;
}

/*
 * l, get_level 'Level'
 * "Get 'Level' value as float or int."
 */
static int hamlib_get_level(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "get_level %s:\n", argv[0]);
        send_response(cs, tmp);
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
        strcat(resp, "\n");
        send_response(cs, resp);
        return 0;
    }
    char out_buf[128];
    const char *mapped_level_name = resolve_property_name(argv[0]);
    if(is_debug)printf("Mapped level name is %s\n", mapped_level_name);

    if (!sdr_radio_get_property(mapped_level_name, out_buf, sizeof(out_buf)))
    {
        /* fallback to your older command_get_level(...) code: */
        extern void command_get_level(int client_socket, const char *level);
        command_get_level(cs, mapped_level_name);
    }
    else
    {
       strcat(out_buf, "\n");
       send_response(cs, out_buf);
     }

    return 0;
}

void command_tx_control(int client_socket, int s)
{
    //printf("tx_control(%d)\n", s);
    if (s == 1)
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
            send_response(client_socket, "1\n");
        else
            send_response(client_socket, "0\n");
        return;
    }
    send_response(client_socket, "RPRT 0\n");

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
    int ptt_val = 1;
    if (argc > 0) ptt_val = atoi(argv[0]);
    if (is_extended)
    {
        char tmp[64];
        snprintf(tmp, sizeof(tmp),
                 "set_ptt %d:\nPTT: %d\n", ptt_val, ptt_val);
        send_response(cs, tmp);
    }
    {
        extern void command_tx_control(int client_socket, int s);
        command_tx_control(cs, ptt_val);
    }
    return 0;
}

/*
 * t, get_ptt
 * "Get 'PTT' status, returns 0 or 1."
 */
static int hamlib_get_ptt(int cs, int is_extended, char *argv[], int argc)
{
    if (is_extended)
    {
        send_response(cs, (char *) "get_ptt:\n");
    }
    {
        /* your code calls command_tx_control(cs,-1) => returns '0\n' or '1\n' */
        extern void command_tx_control(int client_socket, int s);
        command_tx_control(cs, -1);
    }
    return 0;
}

/*****************************************************************************
 * D, dump_state => multi-line rig caps
 *****************************************************************************/
static char dump_state_response[] =
        "0\n"
        "2\n"
        "2\n"
        "100000 30000000 0x2ef -1 -1 0x1 0x0\n"
        "0 0 0 0 0 0 0\n"
        "0 0 0 0 0 0 0\n"
        "0xef 1\n"
        "0xef 0\n"
        "0 0\n"
        "0x82 500\n"
        "0x82 200\n"
        "0x82 2000\n"
        "0x221 5000\n"
        "0x0c 2700\n"
        "0 0\n"
        "0\n"
        "0\n"
        "0\n"
        "0\n"
        "0\n"
        "0\n"
        "0\n"
        "0x40000020\n"
        "0x20\n"
        "0\n"
        "0\n";

static int hamlib_dump_state(int cs, int is_extended)
{
    if (is_extended)
    {
        send_response(cs, (char *) "dump_state:\n");
        send_response(cs, dump_state_response);
    }
    else
    {
        send_response(cs, dump_state_response);
        send_response(cs, (char *) "RPRT 0\n");
    }
    return 0;
}

static int hamlib_get_rig_info(int client_socket, int is_extended, char *argv[], int argc)
{
    if (is_extended)
    {
        send_response(client_socket, (char *)"get_rig_info\n\n");
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
    send_response(client_socket, resp);

    // Get the VFO B Values
    get_field_value_by_label("VFOB", freq);

    snprintf(resp, sizeof(resp), "VFO=VFOB Freq=%s Mode=%s Width=%ld RX=0 TX=0\n",
             freq, mode, get_passband_bw());
    send_response(client_socket, resp);

    snprintf(resp, sizeof(resp), "Split=%s\nSatMode=0\nRig=%s\nVersion=%s\nApp=Hamlib\n",
             strcmp(split, "OFF") == 0 ? "0" : "1", PRODUCT, VERSION);
    send_response(client_socket, resp);

    return 0;
}

/*****************************************************************************
 * P => get_powerstat
 *****************************************************************************/
static int hamlib_get_powerstat(int cs, int is_extended)
{
    if (is_extended)
    {
        send_response(cs, (char *) "get_powerstat:\nPower: 1\n");
    }
    else
    {
        send_response(cs, (char *) "1\n");
    }
    return 0;
}

/*****************************************************************************
 * w => send_cmd 'Cmd'
 *****************************************************************************/
static int hamlib_send_cmd_raw(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;
    if (is_extended)
    {
        char tmp[256];
        if (argc > 1 && argc < 3)
            snprintf(tmp, sizeof(tmp),
                     "send_cmd %s:\nCommand: %s\n", argv[0], argv[0]);
        if (argc > 2 && argc <= 3)
            snprintf(tmp, sizeof(tmp),
                     "send_cmd %s %s:\n%s: %s\n", argv[0], argv[1], argv[0], argv[1]);
        send_response(cs, tmp);
    }
    {
        char combined[256] = "";
        for (int i = 0; i < argc; i++)
        {
            if (i > 0) strcat(combined, " ");
            strcat(combined, argv[i]);
        }
        remote_execute(combined);
        if (!is_extended)
        {
            send_response(cs, (char *) "RPRT 0\n");
        }
    }
    return 0;
}

static int hamlib_get_vfo_info(int cs, int is_extended, char *argv[], int argc)
{
    if (false ) // turn this off
    {
        if (argc > 0)
        {
            sprintf(resp, "get_vfo_info: %s\n\n", argv[0]);
            send_response(cs, resp);
        }
    }
    {
        char mode[10];
        char freq[20];
        char vfo[10];

        char split[4];
        char tx_vfo[4];
        extern char vfo_a_mode[10];
        extern char vfo_b_mode[10];

        if (argc < 1) return -11;
        if (argc > 0)
            strcpy(vfo, argv[0]);

        get_field_value_by_label(vfo, freq);
        get_field_value_by_label("SPLIT", split);

        /// Apparently we only have one mode... so we will just use that for both
        get_field_value_by_label("MODE", mode);
        // Get the mode based on which VFOA or VFOB
//        if (!strcmp(vfo, "VFOA"))
//            strcpy(mode, vfo_a_mode);
//        else
//            strcpy(mode, vfo_b_mode);

        if (is_extended)
        {
            sprintf(resp, "get_vfo_info: %s\n\nFreq: %s\nMode: %s\nVFO: %s\nSplit: %s \n\n", argv[0],freq, mode, vfo,
                    strcmp(split, "OFF") == 0 ? "0" : "1");
        }
        else
            sprintf(resp, "%s\n%s\n%s\n%s\n", freq, mode, vfo, strcmp(split, "OFF") == 0 ? "0" : "1");

        send_response(cs, resp);
    }
    return 0;
}

// Handle Get 'Transceive' mode which in our caase is always off... MacLoggerDX expects this
// TODO: This should be a serious mode, would be really neat for things like sending
//        METER readings live, from the radio to the client.

static int hamlib_get_trn(int cs, int is_extended, char *argv[], int argc)
{
    if (is_extended)
    {
        send_response(cs, (char *) "get_trn:\n");
    }
    sprintf(resp, "OFF\n");
    send_response(cs, resp);
    return 0;
}

//   Gets rig clock -- note that some rigs do not handle seconds or milliseconds. Format is ISO8601 YYYY-MM-DDTHH:MM:SS.sss+ZZ where +ZZ is either -/+ UTC offset
static int hamlib_get_clock(int cs, int is_extended, char *argv[], int argc)
{
    if (is_extended)
    {
        send_response(cs, (char *) "get_clock:\n");
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
            send_response(cs, tmp);
        }
        else
            send_response(cs, clock);
    }
    return 0;
}

static int hamlib_set_lock_mode(int cs, int is_extended, char *argv[], int argc)
{
    if (argc < 1) return -11;
    if (is_extended)
    {
        char tmp[128];
        snprintf(tmp, sizeof(tmp),
                 "set_lock_mode %s:\nLock Mode: %s\n", argv[0], argv[0]);
        send_response(cs, tmp);
    }
    {
        if (strcmp(argv[0], "0") == 0)
        {
            is_locked = false;

        }
        else
        {
            is_locked = true;
        }
        if (!is_extended)
        {
            send_response(cs, (char *) "RPRT 0\n");
        }
    }
    return 0;
}

static int hamlib_get_lock_mode(int cs, int is_extended, char *argv[], int argc)
{
    char lock[4];
   if(is_locked)
        strcpy(lock, "1");
    else
        strcpy(lock, "0");

    if (is_extended)
    {
        send_response(cs, (char *)"get_lock_mode:\n");
    }

    if (strcmp(lock, "0") == 0)
    {
        send_response(cs, is_extended ? (char *)"Lock Mode: 0\n" : (char *)"0\n");
    }
    else
    {
        send_response(cs, is_extended ? (char *)"Lock Mode: 1\n" : (char *)"1\n");
    }

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
    char line[512];
    strncpy(line, line_in, sizeof(line));
    line[sizeof(line) - 1] = '\0';

    line_mode_t mode = parse_line_mode(line);
    if (mode == LINE_MODE_INVALID)
    {
        send_rprt(client_socket, -11);
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
        send_rprt(client_socket, -11);
        return;
    }
    // Debug input string
    if (is_debug)
    {
        for (int i = 0; i < argc; i++)
            printf("%s ", argv[i]);
        printf("  --> ");
    }
    command_id_t cmd_id = parse_command_name(argv[0]);
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
        case CMD_QUIT:
            ret = 0;
            if (is_extended)
                send_response(client_socket, "quit:\n");
            //printf(" About to send RPRT 0\n");
            send_rprt(client_socket, ret);
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
        default:
            ret = -11;
            break;
    }

    if (is_extended)
    {
        send_rprt(client_socket, ret);
    }
    else if (ret != 0)
    {
        send_rprt(client_socket, ret);
    }
}

/*****************************************************************************
 * handle_client_data, plus the server loop
 *****************************************************************************/
void handle_client_data(int client_socket, const char *data, ssize_t len)
{
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

static void *server_thread(void *arg)
{
    struct sockaddr_in server_addr, client_addr;
    socklen_t addr_len = sizeof(client_addr);

    welcome_socket = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
    if (welcome_socket < 0)
    {
        perror("Failed to create socket");
        exit(EXIT_FAILURE);
    }
//    printf("Socket created successfully\n");

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
//    if (setsockopt(welcome_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
//    {
//        perror("setsockopt failed");
//        close(welcome_socket);
//        exit(EXIT_FAILURE);
//    }
//    printf("Socket options set successfully\n");

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
                fcntl(new_sd, F_SETFL, O_NONBLOCK);
                printf("New client on socket %d\n", new_sd);

                pthread_mutex_lock(&client_mutex);
                int assigned = 0;
                for (int i = 0; i < MAX_CLIENTS; i++)
                {
                    if (client_sockets[i] == 0)
                    {
                        client_sockets[i] = new_sd;
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
                perror("accept error");
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
    running = 0;
    close(welcome_socket);
}

// Start the HAMLIB listener thread
void *start_listener_thread(void *arg)
{
    start_hamlib_listener();
    return NULL;
}

// Initialize the HAMLIB listener thread
void initialize_hamlib()
{
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

    // Example older code from your snippet checking various function names:
    if (strcmp(func, "ANR") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "ANR %s", onOff);
        remote_execute(buffer);
        return 0;
    }
    else if (strcmp(func, "DSP") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "DSP %s", onOff);
        remote_execute(buffer);
        return 0;
    }
    else if (strcmp(func, "NOTCH") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "NOTCH %s", onOff);
        remote_execute(buffer);
        return 0;
    }
    else if (strcmp(func, "TUNER") == 0)
    {
        char buffer[40];
        snprintf(buffer, sizeof(buffer), "TUNE %s", onOff);
        remote_execute(buffer);
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
    /* Repeat for "COMP", "RIT", etc. if you want. */

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
    // Old logic from your snippet:
    //  - If "?" => list supported.
    //  - If we find a toggle (ON/OFF), return 1 or 0.
    //  - If "COMP", read "COMP" int, etc.
    //  - If not found, RPRT -11.


    if (strchr(func, '?'))
    {
        // Return a space-separated list of radio backend supported get function tokens
        // e.g. "NB COMP TUNER ANR ..."
        send_response(client_socket, (char *) "NB COMP ANR TUNER RIT\n");
        return;
    }

    // Attempt reading from a toggle field, for instance:
    char *field_toggle(const char *lbl); // maybe in your code
    extern char *field_toggle(const char *lbl);

    const char *toggle_value = field_toggle(func); // returns "" if not found
    if (toggle_value && toggle_value[0])
    {
        // e.g. if toggle_value=="ON" => "1"
        sprintf(resp, "%d\n", (strcmp(toggle_value, "ON") == 0) ? 1 : 0);
        send_response(client_socket, resp);
        return;
    }
    else if (strcmp(func, "COMP") == 0)
    {
        int compression = field_int("COMP");
        sprintf(resp, "%d\n", compression > 0 ? 1 : 0);
        send_response(client_socket, resp);
        return;
    }
    else if (strcmp(func, "DEBUG") == 0)
    {
        sprintf(resp, "%d\n", is_debug);
        send_response(client_socket, resp);
        return;
    }
    /* else if (whatever) ... */

    // If still not found, attempt a direct field_value read:
    char field_value[100];
    strcpy(field_value, "");
    if (get_field_value_by_label((char *) func, field_value) == -1)
    { // Not found :-(  RPRT -11
        send_response(client_socket, (char *) "RPRT -11\n");
    }
    else
    { // Found :-)  Return the value
        sprintf(resp, "%s\n", field_value);
        send_response(client_socket, resp);
    }
}

/*****************************************************************************
 * command_set_level
 *
 * Fallback for: "L, set_level 'Level' 'Value'"
 *
 * If your property dictionary doesn't recognize the level, we do this older code.
 *****************************************************************************/
hamlib_error_t command_set_level(const char *level, float value)
{
    // Copied from your snippet
    char sdr_cmd[256];

    if (strcmp(level, "RFPOWER") == 0)
    {
        snprintf(sdr_cmd, sizeof(sdr_cmd), "DRIVE %d", (int) ((float)value * 100.0));
    }
    else if (strcmp(level, "MICGAIN") == 0 || strcmp(level, "MIC") == 0)
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
    else
    {
        return HAMLIB_ERR_INVALID_PARAM; // e.g. -11
    }

    remote_execute(sdr_cmd);
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
    int value = 0;

    if (strcmp(level, "RFPOWER") == 0)
    {
        float drive = (float) field_int("DRIVE") / 100.0f;
        snprintf(resp, sizeof(resp), "%.2f\n", drive);
        send_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "MICGAIN") == 0 || strcmp(level, "MIC") == 0)
    {
        value = field_int("MIC");
    }
    else if (strcmp(level, "COMP") == 0 || strcmp(level, "COMPRESS") == 0)
    {
        value = field_int("COMP");
    }
    else if (strcmp(level, "RXGAIN") == 0)
    {
        value = field_int("IF");
    }
    else if (strcmp(level, "TXMON") == 0)
    {
        value = field_int("TXMON");
    }
    else if (strcmp(level, "VOLUME") == 0)
    {
        value = field_int("AUDIO");
    }
    else if (strcmp(level, "SWR") == 0)
    {
            char meter_str[100];
            int vswr = field_int("REF");
            int power = field_int("POWER");

            // power is in 1/10th of watts and vswr is also 1/10th
            if (power < 30)
                vswr = 10;

        float swr= (float)vswr/10.0;
        snprintf(resp, sizeof(resp), "%.2f\n", swr);
        send_response(client_socket, resp);
        return;

    }
    else if (strcmp(level, "AGC") == 0)
    {
        value = field_int("AGC");
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

        snprintf(resp, sizeof(resp), "%2.1f dB\n", get_rig_level_strength());
        send_response(client_socket, resp);
        return;
    }
    else if (strcmp(level, "METER") == 0)
    {
        snprintf(resp, sizeof(resp), "%2.1f dB\n", get_rig_level_strength);
        send_response(client_socket, resp);
        return;
    }
    else
    {
        /* not recognized in fallback => try reading direct field. */
        char field_value[100] = "";
        if (get_field_value_by_label((char *) level, field_value) == -1)
        {
            send_response(client_socket, (char *) "RPRT -11\n");
        }
        else
        {
            snprintf(resp, sizeof(resp), "%s\n", field_value);
            send_response(client_socket, resp);
        }
        return;
    }
    printf("Got level: %d\n", value);
  snprintf(resp, sizeof(resp), "%d\n", value);
  send_response(client_socket, resp);
}

