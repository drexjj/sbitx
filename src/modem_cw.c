// standard library includes
#include <assert.h>
#include <complex.h>
#include <ctype.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

// third-party library includes
#include <fftw3.h>
#include <wiringPi.h>

// project-specific includes
#include "sdr.h"
#include "sdr_ui.h"
#include "modem_cw.h"
#include "sound.h"

// defines and constants
#define MAX_SYMBOLS 100
#define CW_MAX_SYMBOLS 12
#define FLOAT_SCALE (1073741824.0)
#define HIGH_DECAY 100  // used in updating noise floor
#define NOISE_DECAY 100 

// structs and typedefs
struct morse_tx {
	char c;
	char *code;
};

struct morse_rx {
	char *c;
	char *code;
};

struct bin {
	float coeff;
	float sine;
	float cosine;
	float omega;
	int k;
	double scalingFactor;
	int freq;
	int n;
};

struct symbol {
	char is_mark;
	int magnitude;
	int ticks;
};

struct cw_decoder {
	int n_samples_per_block;
	int dash_len;
	int mark;
	int prev_mark;
	int n_bins;
	int ticker;
	int high_level;
	int noise_floor;
	int sig_state;
	int magnitude;
	int symbol_magnitude;
	int wpm;

	struct bin signal_center;
	struct bin signal_plus;
	struct bin signal_minus;

	int32_t history_sig;
	struct symbol symbol_str[MAX_SYMBOLS];
	int next_symbol;
};

struct cw_decoder decoder;

// Morse code tables
struct morse_tx morse_tx_table[] = {
	{'~', " "}, //dummy, a null character
	{' ', " "}, {'a', ".-"}, {'b', "-..."},	{'c', "-.-."}, {'d', "-.."},
	{'e', "."}, {'f', "..-."}, {'g', "--."}, {'h', "...."}, {'i', ".."},
	{'j', ".---"}, {'k', "-.-"}, {'l', ".-.."}, {'m', "--"}, {'n', "-."},
	{'o', "---"}, {'p', ".--."}, {'q', "--.-"}, {'r', ".-."}, {'s', "..."},
	{'t', "-"}, {'u', "..-"}, {'v', "...-"}, {'w', ".--"}, {'x', "-..-"},
	{'y', "-.--"}, {'z', "--.."}, {'1', ".----"}, {'2', "..---"}, {'3', "...--"},
	{'4', "....-"}, {'5', "....."}, {'6', "-...."}, {'7', "--..."}, {'8', "---.."},
	{'9', "----."}, {'0', "-----"}, {'.', ".-.-.-"}, {',', "--..--"}, {'?', "..--.."},
  {'/', "-..-."}, {'\'', "--..--"}, {'&', "-...-"},
	{'=', "-...-"},   // BT
	{'<', ".-.-."},   // AR
	{'>', "...-.-"},  // SK
	{'(', "-.--."},   // KN
	{':', ".-..."},   // AS
};

struct morse_rx morse_rx_table[] = {
	{"~", " "}, //dummy, a null character
	{" ", " "}, {"A", ".-"}, {"B", "-..."}, {"C", "-.-."}, {"D", "-.."},
	{"E", "."}, {"F", "..-."}, {"G", "--."}, {"H", "...."}, {"I", ".."},
	{"J", ".---"}, {"K", "-.-"}, {"L", ".-.."}, {"M", "--"}, {"N", "-."},
	{"O", "---"}, {"P", ".--."}, {"Q", "--.-"}, {"R", ".-."}, {"S", "..."},
	{"T", "-"}, {"U", "..-"}, {"V", "...-"}, {"W", ".--"}, {"X", "-..-"},
	{"Y", "-.--"}, {"Z", "--.."}, {"1", ".----"}, {"2", "..---"}, {"3", "...--"},
	{"4", "....-"}, {"5", "....."}, {"6", "-...."}, {"7", "--..."}, {"8", "---.."},
	{"9", "----."}, {"0", "-----"}, {"<STOP>", ".-.-.-"}, {"<COMMA>", "--..--"},
	{"?", "..--.."}, {"/", "-..-."}, { "'", ".----."}, {"!", "-.-.--"}, {":", "---..."},
	{"-", "-....-"}, {"_", "..--.-"},
	{"@", ".--.-."}, {"<AR>", ".-.-."},
	{"<AS>", ".-..."},
	{"<STOP>", ".-.-."},
	{"<BT>", "-...-"},
	{"5nn", ".....-.-."},
	{"ur", "..-.-."},
};


// global variables
static unsigned long millis_now = 0;
static int cw_key_state = 0;
static int cw_period;
static struct vfo cw_tone, cw_env;
static int keydown_count = 0;
static int keyup_count = 0;
static float cw_envelope = 1;
static int cw_tx_until = 0;
static int data_tx_until = 0;

static char *symbol_next = NULL;
char iambic_symbol[4];
char cw_symbol_prev = ' ';

static uint8_t cw_current_symbol = CW_IDLE;
static uint8_t cw_next_symbol = CW_IDLE;
static uint8_t cw_last_symbol = CW_IDLE;
static uint8_t cw_mode = CW_STRAIGHT;
static int cw_bytes_available = 0;
char cw_key_letter[CW_MAX_SYMBOLS];

static FILE *pfout = NULL; //this is debugging out, not used normally

//////////////////////////////////////////
// CW transmit and keyer functions
//////////////////////////////////////////

static uint8_t cw_get_next_symbol(){  //symbol to translate into CW_DOT, CW_DASH, etc
	// note this is part of transmitting cw, not rx

	if (!symbol_next)
		return CW_IDLE;
	
	uint8_t s = *symbol_next++;

	switch(s){
		case '.': 
			return CW_DOT;
		case '-': 
			return  CW_DASH;
		case 0:
			symbol_next = NULL; //we are at the end of the string
			return CW_DASH_DELAY;
		case '/': 
			return CW_DASH_DELAY;
		case ' ': 
			return  CW_WORD_DELAY;
	}
	return CW_IDLE;
}


//cw_read_key() is called 96000 times a second
//it should not poll gpio lines or text input, those are done in modem_poll()
//and we only read the status from the variable updated by modem_poll()

static int cw_read_key(){
	char c;

	//process cw key before other cw inputs (macros, keyboard)
	if (cw_key_state != CW_IDLE) {
		return cw_key_state;
	}

	if (cw_current_symbol != CW_IDLE)
		return CW_IDLE;

	//we are still sending the previously typed character..
	if (symbol_next){
		uint8_t s = cw_get_next_symbol();
		return s;
	}

	//return if a symbol is being transmitted
	if (cw_bytes_available == 0)
		return CW_IDLE;

	get_tx_data_byte(&c);
	symbol_next = morse_tx_table->code; // point to the first symbol, by default

	for (int i = 0; i < sizeof(morse_tx_table)/sizeof(struct morse_tx); i++)
		if (morse_tx_table[i].c == tolower(c)){
			symbol_next = morse_tx_table[i].code;
			char buff[5];
			buff[0] = toupper(c);
			buff[1] = 0;
			write_console(FONT_CW_TX, buff);
		}
	if (symbol_next)
		return cw_get_next_symbol(); 
	else
		return CW_IDLE;
}

// Function prototype for the state machine handler
void handle_cw_state_machine(uint8_t, uint8_t);

// use input from macro playback, keyboard or key/paddle to key the transmitter
// keydown and keyup times
float cw_tx_get_sample() {
  float sample = 0;
  uint8_t state_machine_mode;
  uint8_t symbol_now;
  
  if ((keydown_count == 0) && (keyup_count == 0)) {
    // note current time to use with UI value of CW_DELAY to control break-in
    millis_now = millis();
    // set CW pitch if needed
    if (cw_tone.freq_hz != get_pitch())
      vfo_start( &cw_tone, get_pitch(), 0);
  }
  
  // check to see if input available from macro or keyboard
  if ((cw_bytes_available > 0) || (symbol_next != NULL)) {
    state_machine_mode = CW_KBD;
    cw_current_symbol = CW_IDLE;
  } else
    state_machine_mode = cw_mode;
  
  // iambic modes require polling key during keydown/keyup
  // other modes only check when idle
  if (((state_machine_mode == CW_STRAIGHT || 
        state_machine_mode == CW_BUG ||
        state_machine_mode == CW_ULTIMATIC || 
        state_machine_mode == CW_KBD) && 
        (keydown_count == 0 && keyup_count == 0))
        ||
        (state_machine_mode == CW_IAMBIC || 
        state_machine_mode == CW_IAMBICB)) {
    symbol_now = cw_read_key();
    handle_cw_state_machine(state_machine_mode, symbol_now);
  }

  // key the transmitter with some shaping
  // at 20 wpm  a CW_DOT starts with keydown_count = 5760
  if (keydown_count > 0) {
    if(cw_envelope < 0.999)
      cw_envelope = ((vfo_read(&cw_env)/FLOAT_SCALE) + 1)/2;
    keydown_count--;
  } else {  // countdown all the keydown_count before doing keyup_count
    if(cw_envelope > 0.001)
      cw_envelope = ((vfo_read(&cw_env)/FLOAT_SCALE) + 1)/2;
    if (keyup_count > 0)
      keyup_count--;
  }
  sample = (vfo_read(&cw_tone) / FLOAT_SCALE) * cw_envelope;
  
  // keep extending 'cw_tx_until' while we're sending
  if ((symbol_now == CW_DOWN) || (symbol_now == CW_DOT) ||
      (symbol_now == CW_DASH) || (symbol_now == CW_SQUEEZE) ||
      (keydown_count > 0))
    cw_tx_until = millis_now + get_cw_delay();
  // if macro or keyboard characters remain in the buffer
  // prevent switching from xmit to rcv and cutting off macro
  if (cw_bytes_available != 0)
    cw_tx_until = millis_now + 1000;

  return sample / 8;
}


// This function implements the KB2ML sBitx keyer state machine for each CW mode
// State machine uses mode, current state and input to determine keydown_count
// and keyup_count needed to key transmitter.
void handle_cw_state_machine(uint8_t state_machine_mode, uint8_t symbol_now) {
  static uint8_t cw_next_symbol_flag = 0;  // used in iambic modes
  // printf("state_machine_mode %d\n", state_machine_mode);
  // printf("cw_current_symbol %d\n", cw_current_symbol);
  // printf("symbol_now %d\n", symbol_now);
  switch (state_machine_mode) {
  case CW_STRAIGHT:
      if (symbol_now == CW_IDLE)
        cw_current_symbol = CW_IDLE;
      if (symbol_now == CW_DOWN) {
        keydown_count = 1; // this is very short, much less than a dit
        keyup_count = 0;
        cw_current_symbol = CW_DOWN;
      }
    break; // done with CW_STRAIGHT mode

  case CW_BUG:
    // Vibroplex 'bug' emulation mode.  The 'dit' contact produces
    // a string of dits at the chosen WPM, the "dash" contact is
    // completely manual and usually used just for dashes
    switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_IDLE)
        cw_current_symbol = CW_IDLE;
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = 1;
        keyup_count = 0;
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) {
        cw_current_symbol = CW_IDLE;
      }
      break; // exit CW_IDLE case
    case CW_DOT:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = 1; // works like straight key
        keyup_count = 0;
        cw_current_symbol = CW_DASH;
      }
      break; // exit CW_DOT case
    case CW_DASH:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = 1;
        keyup_count = 0;
        cw_current_symbol = CW_DASH;
      }
      break; // exit CW_DASH case
    }
    break; // done with CW_BUG mode

  case CW_ULTIMATIC:
    // when both paddles are squeezed, whichever one was squeezed last gets repeated
    switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_IDLE) {
        // do nothing, stay in same state
      }
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = cw_period * 3;
        keyup_count = cw_period;
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_last_symbol = CW_DASH;
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_IDLE case
    case CW_DOT:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_SQUEEZE) {
        keydown_count = cw_period * 3;
        keyup_count = cw_period;
        cw_last_symbol = CW_DASH;
        cw_current_symbol = CW_SQUEEZE;
      }
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = cw_period * 3;
        keyup_count = cw_period;
        cw_current_symbol = CW_DASH;
      }
      break; // exit CW_DOT case
    case CW_DASH:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_SQUEEZE) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_last_symbol = CW_DOT;
        cw_current_symbol = CW_SQUEEZE;
      }
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = cw_period * 3;
        keyup_count = cw_period;
        cw_current_symbol = CW_DASH;
      }
      break; // exit CW_DASH case
    case CW_SQUEEZE:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_SQUEEZE) {
        if (cw_last_symbol == CW_DOT) {
          keydown_count = cw_period;
          cw_last_symbol = CW_DOT;
        } else {
          keydown_count = cw_period * 3;
          cw_last_symbol = CW_DASH;
        }
        keyup_count = cw_period;
        cw_current_symbol = CW_SQUEEZE;
      }
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = cw_period * 3;
        keyup_count = cw_period;
        cw_current_symbol = CW_DASH;
      }
      break; // exit CW_SQUEEZE case
    }
    break; // done with CW_ULTIMATIC mode

  case CW_IAMBIC:
    // aka iambic A
    // the keyer stops sending the current bit when you release the paddles
    
    // before checking new paddle input, look for a symbol ready to go
    // don't act on anything else until it is cleared
    if (cw_next_symbol_flag == 1) {
      if (keyup_count == 0) {
        if (cw_next_symbol == CW_DOT) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        } else if (cw_next_symbol == CW_DASH) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        }
        cw_next_symbol_flag = 0;
      }
    goto end_of_iambic;
    }
    
    switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT) {
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        }
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        }
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) {
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
          cw_next_symbol = CW_DASH;
          cw_next_symbol_flag = 1;
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_IDLE case
    case CW_DOT:
      if (symbol_now == CW_IDLE) {
        // do nothing, stay in same state
      }
      if (symbol_now == CW_DOT) {
        // this is a dot following a previous dot
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        }
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        } else if (keyup_count > 0) {
          // early paddle input for next dash
          cw_next_symbol = CW_DASH;
          cw_next_symbol_flag = 1;
        }
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_SQUEEZE) {
        if (keydown_count > 0) {
          if (cw_last_symbol == CW_DOT) {
            cw_next_symbol = CW_DASH;
            cw_next_symbol_flag = 1;
          } else if (cw_last_symbol == CW_DASH) {
            cw_next_symbol = CW_DOT;
            cw_next_symbol_flag = 1;
          }
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_DOT case
    case CW_DASH:
      if (symbol_now == CW_IDLE) {
        // do nothing, stay in same state
      }
      if (symbol_now == CW_DOT) {
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        } else if (keyup_count > 0) {
          // early paddle input for next dot
          cw_next_symbol = CW_DOT;
          cw_next_symbol_flag = 1;
        }
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DASH) {
        // this is a dash following a previous dash
        if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        }
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) {
        if (keydown_count > 0) {
          if (cw_last_symbol == CW_DOT) {
            cw_next_symbol = CW_DASH;
            cw_next_symbol_flag = 1;
          } else if (cw_last_symbol == CW_DASH) {
            cw_next_symbol = CW_DOT;
            cw_next_symbol_flag = 1;
          }
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_DASH case
    case CW_SQUEEZE:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT) {
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
        }
        cw_last_symbol = CW_DOT;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
        }
        cw_last_symbol = CW_DASH;
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) { // alternate dot and dash
        if (keyup_count == 0) {
          if (cw_last_symbol == CW_DOT) {
            keydown_count = cw_period * 3;
            keyup_count = cw_period;
            cw_last_symbol = CW_DASH;
          } else if (cw_last_symbol == CW_DASH) {
            keydown_count = cw_period;
            keyup_count = cw_period;
            cw_last_symbol = CW_DOT;
          }
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_SQUEEZE case
    }
    end_of_iambic:
    break; // done with CW_IAMBIC mode

  case CW_IAMBICB:
    // when both paddles are squeezed, whichever one was squeezed last gets repeated
    // when both paddles are released the keyer will finish the dit or dah and add
    // an additional opposite element
    
    // before checking new paddle input, look for a symbol ready to go
    if (cw_next_symbol_flag == 1) {
      if (keyup_count == 0) {
        if (cw_next_symbol == CW_DOT) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        } else if (cw_next_symbol == CW_DASH) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        }
        cw_next_symbol_flag = 0;
      }
    goto end_of_iambicB;
    }
    
    switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT) {
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        } 
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        }
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) {
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
          cw_next_symbol = CW_DASH;
          cw_next_symbol_flag = 1;
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_IDLE case
    case CW_DOT:
      if (symbol_now == CW_IDLE) {
        // do nothing, stay in same state
      }
      if (symbol_now == CW_DOT) {
        // this is a dot following a previous dot
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        }
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        if (keyup_count > 0) {
          // early paddle input for next dash
          cw_next_symbol = CW_DASH;
          cw_next_symbol_flag = 1;
        } else if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        }
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_SQUEEZE) {
        if (keydown_count > 0) {
          if (cw_last_symbol == CW_DOT) {
            cw_next_symbol = CW_DASH;
            cw_next_symbol_flag = 1;
          } else if (cw_last_symbol == CW_DASH) {
            cw_next_symbol = CW_DOT;
            cw_next_symbol_flag = 1;
          }
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_DOT case
    case CW_DASH:
      if (symbol_now == CW_IDLE) {
        // do nothing, stay in same state
      }
      if (symbol_now == CW_DOT) {
        if (keyup_count > 0) {
          // early paddle input for next dot
          cw_next_symbol = CW_DOT;
          cw_next_symbol_flag = 1;
        } else if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
          cw_last_symbol = CW_DOT;
        } 
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DASH) {
        // this is a dash following a previous dash
        if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
          cw_last_symbol = CW_DASH;
        }
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) {
        if (keydown_count > 0) {
          if (cw_last_symbol == CW_DOT) {
            cw_next_symbol = CW_DASH;
            cw_next_symbol_flag = 1;
          } else if (cw_last_symbol == CW_DASH) {
            cw_next_symbol = CW_DOT;
            cw_next_symbol_flag = 1;
           }
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_DASH case
    case CW_SQUEEZE:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT) {
        if (keyup_count == 0) {
          keydown_count = cw_period;
          keyup_count = cw_period;
        }
        cw_last_symbol = CW_DOT;
        cw_current_symbol = CW_DOT;
      }
      if (symbol_now == CW_DASH) {
        if (keyup_count == 0) {
          keydown_count = cw_period * 3;
          keyup_count = cw_period;
        }
        cw_last_symbol = CW_DASH;
        cw_current_symbol = CW_DASH;
      }
      if (symbol_now == CW_SQUEEZE) { // alternate dot and dash
        if (keyup_count == 0) {
          if (cw_last_symbol == CW_DOT) {
            keydown_count = cw_period * 3;
            keyup_count = cw_period;
            cw_last_symbol = CW_DASH;
          } else if (cw_last_symbol == CW_DASH) {
            keydown_count = cw_period;
            keyup_count = cw_period;
            cw_last_symbol = CW_DOT;
          }
        }
        if (cw_last_symbol == CW_DOT) {
          cw_next_symbol = CW_DASH;
          cw_next_symbol_flag = 1;
        } else {
          cw_next_symbol = CW_DOT;
          cw_next_symbol_flag = 1;
        }
        cw_current_symbol = CW_SQUEEZE;
      }
      break; // exit CW_SQUEEZE case
    }
    end_of_iambicB:
    break; // done with CW_IAMBICB mode

  case CW_KBD:
    // this mode handles symbols coming from keyboard or macros
    switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_IDLE) {
        cw_last_symbol = CW_IDLE;
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT) {
        keydown_count = cw_period;
        keyup_count = cw_period;
        cw_last_symbol = CW_DOT;
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DASH) {
        keydown_count = cw_period * 3;
        keyup_count = cw_period;
        cw_last_symbol = CW_DASH;
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DOT_DELAY) {  // this is never used?!
        keyup_count = cw_period * 1;
        cw_last_symbol = CW_DOT_DELAY;
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_DASH_DELAY) {  // a NULL
        if (cw_last_symbol == CW_WORD_DELAY) {
          // no more delay extension needed
        } else {
          // extend single delay after dot or dash to total 3 dit lengths
          // betweeen characters
          keyup_count = cw_period * 2;
        }
        cw_last_symbol = CW_DASH_DELAY; 
        cw_current_symbol = CW_IDLE;
      }
      if (symbol_now == CW_WORD_DELAY) {  // a space
        if (cw_last_symbol == CW_DASH_DELAY) {
          // extend delay after end of character delay four more
          // dit-lengths to total seven dit lengths
          keyup_count = cw_period * 4;
        } else {
          keyup_count = cw_period * 7;
        }
        cw_last_symbol = CW_WORD_DELAY;
        cw_current_symbol = CW_IDLE;
      }
      break;
    }
    break; // done with CW_KBD mode
  } // end of the state machine switch case
} // end of handle_cw_state_machine function


////////////////////////////////////////////////////////////////
// CW DECODER FUNCTIONS
// processing flow
// cw_rx(int32_t *samples, int count)  // called from modem_rx()
// │                                      in modems.c
// ├── cw_rx_bin(&decoder, s)
// │   ├── cw_rx_bin_detect(&p->signal_center, samples)
// │   ├── cw_rx_bin_detect(&p->signal_plus, samples)
// │   ├── cw_rx_bin_detect(&p->signal_minus, samples)
// │
// ├── cw_rx_update_levels(&decoder)
// │
// └── cw_rx_process_symbol(&decoder)
//     ├── cw_rx_add_symbol(&decoder, char symbol)  [on mark/space transitions]
//     ├── cw_rx_match_letter(&decoder)             [on symbol/letter boundary]
////////////////////////////////////////////////////////////////

// CW decoder function prototypes
void cw_rx(int32_t *samples, int count);
static void cw_rx_bin(struct cw_decoder *p, int32_t *samples);
static int cw_rx_bin_detect(struct bin *p, int32_t *data);
static void cw_rx_update_levels(struct cw_decoder *p);
void cw_rx_process_symbol(struct cw_decoder *p);
static void cw_rx_add_symbol(struct cw_decoder *p, char symbol);
static void cw_rx_match_letter(struct cw_decoder *p);

// CW decoder initialization and polling prototypes
void cw_init(void);
static void cw_rx_bin_init(struct bin *p, float freq, int n, float sampling_freq);
void cw_poll(int bytes_available, int tx_is_on);

// downsample a block of samples and call decoding functions
void cw_rx(int32_t *samples, int count) {
  int decimation_factor = 8;  // 96 kHz to 12 kHz
  // not going to check this for every block, this was development code ...
  // if (count % (decimation_factor * decoder.n_bins)) {
  //   printf("cw_decoder bins don't align up with sample block %d vs %d\n",
  //	 count, decoder.n_bins);
  //   assert(0);
  // }
  // use decimation_factor to downsample
  int32_t s[128];
  for (int i = 0; i < decoder.n_bins; i++) {	
	  s[i] = samples[i * decimation_factor] >> 8;			
  }
  cw_rx_bin(&decoder, s);          // look for signal
  cw_rx_update_levels(&decoder);   // update high and low noise levels
  cw_rx_process_symbol(&decoder);  // handle the signal
}

// look for presence of signal in this block of samples
static void cw_rx_bin(struct cw_decoder *p, int32_t *samples){
  // check all three frequency bins and pick the largest
  int mag_center = cw_rx_bin_detect(&p->signal_center, samples);
  int mag_plus   = cw_rx_bin_detect(&p->signal_plus, samples);
  int mag_minus  = cw_rx_bin_detect(&p->signal_minus, samples);
  int sig_now = mag_center;
  if (mag_plus > sig_now)  sig_now = mag_plus;
  if (mag_minus > sig_now) sig_now = mag_minus;
	
	// compare to recent magnitude levels to see if signal present
  p->magnitude = sig_now;
	if (p->magnitude > (p->high_level * 6)/10){
			p->sig_state = 30000;
	}
	else if (p->magnitude <  (p->high_level * 4)/10 ){ 
		p->sig_state = 0;
	}
	p->ticker++;

	// PROVIDE DEBUGGING SUPPORT
	if (pfout){
		int sym_mag = p->symbol_str[p->next_symbol].magnitude;
		int mag = p->magnitude;
		int snr1 = 1;
		if (p->noise_floor > 100)
			snr1 = (p->magnitude * 10)/p->noise_floor;
		int snr = 0;
		if (snr1 > 20)
			snr = 10000;
		for (int i = 0; i < p->n_bins; i++){
			fwrite(&mag,2,1,pfout);	
		//fwrite(&mark,2,1,pfout);	
			fwrite(&p->mark, 2, 1, pfout);
			fwrite(&sym_mag, 2, 1, pfout);
			fwrite(&snr, 2, 1, pfout);
		}
	} // END DEBUGGING SUPPORT
}

// use Goertzel algorithm to detect the magnitude of a specific frequency bin
static int cw_rx_bin_detect(struct bin *p, int32_t *data){
    // Q1 and Q2 are the previous two states in the Goertzel recurrence
    float Q2 = 0;
    float Q1 = 0;
    // iterate over each sample in the block
    for (int index = 0; index < p->n; index++){
        float Q0;
        // Goertzel recurrence relation:
        Q0 = p->coeff * Q1 - Q2 + (float) (*data);
        // shift variables for next iteration
        Q2 = Q1;
        Q1 = Q0;	
        data++;
    }
    // compute in-phase (cosine) and quadrature (sine) components at the target frequency
    double real = (Q1 * p->cosine - Q2) / p->scalingFactor;
    double imag = (Q1 * p->sine) / p->scalingFactor;
    int magnitude = sqrt(real*real + imag*imag); 
    return magnitude;
}

// update signal level tracking for recent high_level and noise_floor
static void cw_rx_update_levels(struct cw_decoder *p) {
  // treat current magnitude as a candidate for the new high level
  int new_high = p->magnitude;
  // if the current signal is higher than the tracked peak, update high_level instantly.
  if (p->high_level < p->magnitude)
    p->high_level = new_high;
  else
    // decay high_level smoothly toward the new value.
    p->high_level = (p->magnitude + ((HIGH_DECAY - 1) * p->high_level)) / HIGH_DECAY;
  // if current magnitude is much lower (less than 40% of high_level)
  // it might be background noise or a space between morse marks
  if (p->magnitude < (p->high_level * 4) / 10) {
    // Clamp the minimum magnitude to 100
    if (p->magnitude < 100) p->magnitude = 100;
    // Update the noise floor with a similar decay mechanism.
    p->noise_floor = (p->magnitude + ((NOISE_DECAY - 1) * p->noise_floor)) / NOISE_DECAY;
    // accumulate the magnitude for the current symbol (dot/dash/space)
    // to indicate average strength of the symbol
    p->symbol_magnitude += p->magnitude;
  }
}

// new function replaces cw_rx_denoise and cw_rx_detect_symbol
// smooths input 'sig_state' using a sliding window, then analyzes mark/space transitions
// to drive Morse code symbol detection
void cw_rx_process_symbol(struct cw_decoder *p) {
  // use sliding window to smooth sig_state over time
  p->history_sig <<= 1;   // Shift register: oldest bit out, make room for new sample
  if (p->sig_state)       // If current input is a 'mark'
    p->history_sig |= 1;  // Set least significant bit

  p->prev_mark = p->mark; // store mark as prev_mark before updating it
  uint16_t sig = p->history_sig & 0b11111; // copy the window's last five bits
  // use Kernighan's algorithm to count number of set bits
  int count = 0;
  while (sig > 0) {
    sig &= (sig - 1); 
    count++;     
  }
  // use majority voting to decide
  if (count >= 3 ) p->mark = 30000;
  else p->mark = 0;

  // detect mark/space transitions and symbol boundaries
  if (p->mark == 0 && p->prev_mark > 0) {  // End of mark
    cw_rx_add_symbol(p, 'm');
    p->ticker = 0;
  } else if (p->mark > 1 && p->prev_mark == 0) {   // Start of mark
    cw_rx_add_symbol(p, ' ');
    p->ticker = 0;                                 // Start timing the new mark
  } else if (p->mark == 0 && p->prev_mark == 0) {  // Continuing space
    if (p->next_symbol == 0) {
      if (p->ticker > (p->dash_len * 3) / 2) {     // Long space = word gap
        write_console(FONT_CW_RX, " ");
        p->ticker = 0;
      }
    } else if (p->ticker > p->dash_len / 2) {      // End of a symbol (dot/dash)
      cw_rx_add_symbol(p, ' ');
      cw_rx_match_letter(p);
      if (p->ticker > (p->dash_len * 3) / 2) {
        write_console(FONT_CW_RX, " ");
      }
      p->ticker = 0;
    }
  } else if (p->mark > 0 && p->prev_mark > 0) {  // Still in a mark (possible long dash)
    if (p->ticker > p->dash_len * 3) p->ticker = p->dash_len;  // Clamp overly long dashes
  }
}

// add a mark or space to the symbol buffer, store its duration (ticks),
// and update the symbol's average magnitude
static void cw_rx_add_symbol(struct cw_decoder *p, char symbol) {
    // if it's full clear it
    if (p->next_symbol == MAX_SYMBOLS)
        p->next_symbol = 0;
    // Only ' ' (space) is treated as a space; all other symbols are marks.
    if (symbol == ' ') {
        p->symbol_str[p->next_symbol].is_mark = 0;
    } else {
        p->symbol_str[p->next_symbol].is_mark = 1;
    }
    // Store the duration of the symbol (number of ticks since last transition).
    p->symbol_str[p->next_symbol].ticks = p->ticker;
    // update the average magnitude for this symbol using a weighted average
    p->symbol_str[p->next_symbol].magnitude =
        ((p->symbol_str[p->next_symbol].magnitude * 10) + p->magnitude) / 11;
    // Move to the next position in the symbol buffer.
    p->next_symbol++;
}

// take string of marks and spaces with their durations in "ticks" and
// translate them into a Morse code character
static void cw_rx_match_letter(struct cw_decoder *decoder) {
  char morse_code_string[MAX_SYMBOLS];
  // if no symbols have been received, there's nothing to decode.
  if (decoder->next_symbol == 0) {
    return;
  }
  // initialize state variables for processing symbols
  int is_currently_in_mark = 0;
  int current_segment_ticks = 0;  
  morse_code_string[0] = '\0';  // Ensure the string starts empty
  // calculate the minimum duration for a valid dot
  int min_valid_symbol_duration = (decoder->dash_len / 6);
  // iterate through all received symbols (marks and spaces)
  for (int i = 0; i < decoder->next_symbol; i++) {
    struct symbol_info current_symbol = decoder->symbol_str[i];
    if (current_symbol.is_mark) {  // if the current symbol is a 'mark' (signal present)
      if (!is_currently_in_mark && current_symbol.ticks > min_valid_symbol_duration) {
        is_currently_in_mark = 1;
        current_segment_ticks = 0;  // reset tick counter for the new mark segment
      }
    } else {  // If the current symbol is a 'space' (silence)
      if (is_currently_in_mark && current_symbol.ticks > min_valid_symbol_duration) {
        is_currently_in_mark = 0;  // We are now in a space

        // classify the preceding mark based on its duration
        if (current_segment_ticks > decoder->dash_len / 2) {
          // this was a dash
          strcat(morse_code_string, "-");
          // now make adaptive adjustment of dash_len
          // refine the expected dash length based on observed dashes
          // new dash length is an average, weighted towards the observed length
          int observed_dash_length =
              ((decoder->dash_len * 3) + current_segment_ticks) / 4;
          // validate the new dash length before updating to prevent extreme swings
          // It must be within a reasonable range (half to double the initial
          // expected length)
          int initial_theoretical_dash_len =
              (18 * SAMPLING_FREQ) / (5 * N_BINS * decoder->wpm);
          if (initial_theoretical_dash_len / 2 < observed_dash_length &&
              observed_dash_length < initial_theoretical_dash_len * 2) {
            decoder->dash_len = observed_dash_length;
          }
          // debugging the dash_len adaptation.
          // printf("Updated dash_len: %d\n", decoder->dash_len);
        } else if (current_segment_ticks >= min_valid_symbol_duration &&
                   current_segment_ticks <= decoder->dash_len / 2) {
          // this was a dot
          strcat(morse_code_string, ".");
        }
      }
    }
    current_segment_ticks +=
        current_symbol.ticks;  // Accumulate ticks for the current segment (mark or space)
  }

  // reset the symbol buffer for the next letter/sequence
  decoder->next_symbol = 0;
  // attempt to match the generated Morse code string to a character in the lookup table
  for (int i = 0; i < sizeof(morse_rx_table) / sizeof(struct morse_rx); i++) {
    if (!strcmp(morse_code_string, morse_rx_table[i].code)) {
      // Match found, write the decoded character to the console
      write_console(FONT_CW_RX, morse_rx_table[i].c);
      return;  // successfully decoded a character
    }
  }
  // if no match was found in the table, output the raw dot/dash sequence.
  write_console(FONT_CW_RX, morse_code_string);
}

void cw_init(){	
	//cw rx initialization
	decoder.ticker = 0;
	decoder.n_bins = N_BINS;
	decoder.next_symbol = 0;
	decoder.sig_state = 0;
	decoder.magnitude= 0;
	decoder.prev_mark = 0;
	decoder.history_sig = 0;
	decoder.symbol_magnitude = 0;
	decoder.wpm = 12;

	// dot len (in msec)) = 1200/wpm; dash len = 3600/wpm
	// each block of nbins = n_bins/sampling seconds; 
	// dash len is (3600 / wpm)/ ((nbins * 1000)/samping_freq) 
	decoder.dash_len = (18 * SAMPLING_FREQ) / (5 * N_BINS* INIT_WPM); 

	// initialize all three signal bins
  cw_rx_bin_init(&decoder.signal_center, INIT_TONE, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_plus,   INIT_TONE + 30, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_minus,  INIT_TONE - 30, N_BINS, SAMPLING_FREQ);
	
	//init cw tx with some reasonable values
  //cw_env shapes the envelope of the cw waveform
  //frequency was at 50 (20 ms rise time), changed it to 200 (4 ms rise time)
  //to improve cw performance at higher speeds
  //NOTE: cw_env is not used with "data driven waveform"
	vfo_start(&cw_env, 200, 49044); //start in the third quardrant, 270 degree
	vfo_start(&cw_tone, 700, 0);
	cw_period = 9600; 		// At 96ksps, 0.1sec = 1 dot at 12wpm
	cw_key_letter[0] = 0;
	keydown_count = 0;
	keyup_count = 0;
	cw_envelope = 0;
}

// initialize a struct bin for use with Goertzel algorithm
static void cw_rx_bin_init(struct bin *p, float freq, int n, 
	float sampling_freq){

  p->k = (int) (0.5 + ((n * freq) / sampling_freq));
  p->omega = (2.0 * M_PI * p->k) / n;
  p->sine = sin(p->omega);
  p->cosine = cos(p->omega);
  p->coeff = 2.0 * p->cosine;
	p->n = n;
	p->freq = freq;
	p->scalingFactor = n / 2.0;
}

void cw_poll(int bytes_available, int tx_is_on){
	cw_bytes_available = bytes_available;
	cw_key_state = key_poll();
	int wpm  = field_int("WPM");
	cw_period = (12 * 9600)/wpm;

	//retune the rx pitch if needed
	int cw_rx_pitch = field_int("PITCH");
	if (cw_rx_pitch != decoder.signal_center.freq) {
        cw_rx_bin_init(&decoder.signal_center, cw_rx_pitch, N_BINS, SAMPLING_FREQ);
        cw_rx_bin_init(&decoder.signal_plus,   cw_rx_pitch + 30, N_BINS, SAMPLING_FREQ);
        cw_rx_bin_init(&decoder.signal_minus,  cw_rx_pitch - 30, N_BINS, SAMPLING_FREQ);
  }
	// check if the wpm has changed
	if (wpm != decoder.wpm){
		decoder.wpm = wpm;
		decoder.dash_len = (18 * SAMPLING_FREQ) / (5 * N_BINS* wpm); 
	}	

	// TX ON if bytes are avaiable (from macro/keyboard) or key is pressed
	// of we are in the middle of symbol (dah/dit) transmission 
	
	if (!tx_is_on && (cw_bytes_available || cw_key_state || (symbol_next && *symbol_next)) > 0){
		tx_on(TX_SOFT);
		millis_now = millis();
		cw_tx_until = get_cw_delay() + millis_now;
		cw_mode = get_cw_input_method();
	}
	else if (tx_is_on && cw_tx_until < millis_now){
			tx_off();
	}
}

void cw_abort(){
	//flush all the tx text buffer
  //actually does nothing
}
