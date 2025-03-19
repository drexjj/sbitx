#include <stdint.h>

void setup();
void loop();
void display();
void redraw();
void key_pressed(char c);
int field_set(const char *label, const char *new_value);
int get_field_value(char *id, char *value);
int get_field_value_by_label(char *label, char *value);
extern int spectrum_plot[];
void remote_execute(char *command);
int remote_update_field(int i, char *text);
void web_get_spectrum(char *buff);
void save_user_settings(int forced);
int web_get_console(char *buff, int max);
int remote_audio_output(int16_t *samples);
const char *field_str(const char *label);
int field_int(char *label);
int is_in_tx();
void abort_tx();
void enter_qso();
extern int display_freq;

// A mixed bag of named styles used in various places in various UIs.
typedef enum {
	// semantic styles (only for the console so far):
	// STYLE_LOG must come first, because it's 0, the default,
	// and we use memset to initalize the console
	STYLE_LOG = 0,
	STYLE_MYCALL,
	STYLE_CALLER,
	STYLE_GRID,
	STYLE_RST,
	// mode-specific semantics
	STYLE_FT8_RX,
	STYLE_FT8_TX,
	STYLE_FT8_QUEUED,
	STYLE_FT8_REPLY,
	STYLE_CW_RX,
	STYLE_CW_TX,
	STYLE_FLDIGI_RX,
	STYLE_FLDIGI_TX,
	STYLE_TELNET,

	// non-semantic styles, for other fields and UI elements
	STYLE_FIELD_LABEL,
	STYLE_FIELD_VALUE,
	STYLE_LARGE_FIELD,
	STYLE_LARGE_VALUE,
	STYLE_SMALL,
	STYLE_SMALL_FIELD_VALUE,
	STYLE_BLACK
}  sbitx_style;

/*	At first glance this may look silly: not the simplest way to style the "console".
	But this is an experiment in reusable UI design. Each instance of this struct can be applied
	to a span within the _entire_ body of text, even if the text is editable (to an extent),
	even if lines can be much longer than what we have in our "console".
	It's possible to save all text to a file, along with a vector of these structs in another file,
	and "replay" it with styling later: no need for some ad-hoc markup language.
	So a saved console session could just about make the logbook redundant: you could
	reconstruct the logbook from the session log, if you needed to. Time will tell whether
	that's useful, or just a waste of space to keep console logs around for too long.

	Text is meaningful to humans; metadata is kept separate. This is a better way to
	tag text for use by remote UIs that may present the information in a different way.

	The struct is 64 bits on purpose: it packs well in memory (most computers are 64-bit),
	and a memory image of a vector of these structs is meant to be portable to all
	little-endian machines.

	Too bad `semantic` is so short, but it's hard to imagine shortening any of the
	other fields (for the general use case outside this UI).
*/
typedef struct {
	uint32_t start_row : 32;
	uint16_t start_column : 16;
	uint8_t length : 8;
	uint8_t semantic : 8; // used directly as style in this UI
} text_span_semantic;

// maximum sem_count in write_console_semantic()
#define MAX_CONSOLE_LINE_STYLES 8

#define EXT_PTT 26 //ADDED BY KF7YDU, solder lead wire to J17, which ties to pin 32.
extern int ext_ptt_enable;
void enter_qso();
void call_wipe();
void write_console(sbitx_style style, const char *text);
// write plain text, with semantically-tagged spans that imply styling
void write_console_semantic(const char *text, const text_span_semantic *sem, int sem_count);
int macro_load(char *filename, char *output);
int macro_exec(int key, char *dest);
void macro_label(int fn_key, char *label);
void macro_list(char *output);
void macro_get_keys(char *output);
void update_log_ed();
void write_call_log();
time_t time_sbitx();

#define VER_STR "sbitx v5.01" // Brought to you by the sBitx 64 Bit Development Team
