/*
	We use two different tables for sending and receiving morse code
	The receiving table also contains dot and dash sequences that result
	in abbreviations like <BT> or those phrases thata are usually
	run together or are frequently used words.

	TXing:
	The keyer is adapted from KC4IFB's description in QEX of Sept/Oct 2009

	The routine cw_get_sample() is called for each audio sample being being
	transmitted. It has to be quick, without loops and with no device I/O.
	The cw_get_sample() is also called from the thread that does DSP, so stalling
	it is dangerous.

	The modem_poll is called about 10 to 20 times a second from 
	the 'user interface' thread.

	The key is physically read from the GPIO by calling key_poll() through
	modem_poll and the value is stored as cw_key_state.

	The cw_get_sample just reads this cw_key_state instead of polling the GPIO.

	the cw_read_key() routine returns the next dash/dot/space/, etc to be sent
	the word 'symbol' is used to denote a dot, dash, a gaps that are dot, dash or
	word long. These are defined in sdr.h (they shouldn't be) 
	
	Rxing:
	The CW decoder is entirely written from the scratch after a preliminary
	read of a few Arduino decoders.

	All the state variables are stored in the struct cw_decoder. 
	You could run multiple instances of the cw_decoder to simultaneously
	decoder a band of signals. In the current implementation, we only use
	one cw_decoder.

	cw_rx() is called to process the audio samples

	1. Each cw_decoder has a struct bin that is initialized to a particular
	central frequency.

	2. the n_bins field of cw_decoder takes that many samples at a time 
	and tried to calculate the magnitude of the signal at that freq.

	3. We maintained a running average of the highs and the lows (corresponding
	to the signal peak and the noise floor). These are updated in a moving
	average as high_level and threshold elements in cw_rx_update_levels() 
	function.
	
	4. In cw_rx_process (), we threshold the signal magnitude to generate
	'mark' and 'space'. Each of them in placed into a string of struct symbol.
	We maintain a track of the magnitude, time (in terms of ticks).

	5. the cw_rx_denoise() skips small bumps of less than 4 ticks in a mark
	or space and improves the readability to a great degree. denoiser
	essentialy produces a bit queue of the marks and spaces in a 32-bit 
	integer used as a bit filed. it watches for a continuous 4 bits of 
	zeros or ones before flipping between mark and space.

	6. cw_rx_detect_symbol(), produces a stream of mark/space symbols stored 
	in cw_decoder's sybmol_str array. Whenever an inter letter space
	is detected, the string of symbols is submitted to cw_rx_match_letter().

	7. The match_letter uses the symbols in terms of their magnitude, duration
	to first-fit a pattern from the morse_rx_table. This table should ideally
	be read from a text file. It could, in the future also contain callsign
	database. 
	This function needs to be worked on to work probabilitisically with
	best match where the magnitude of the signal is marginally across the
	threshold between the signal and the noisefloor. 
	
*/
#include <stdio.h>
#include <sys/time.h>   //added to support debug timing code
#include <stdlib.h>
#include <assert.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <time.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include <pthread.h>
#include <unistd.h>
#include <ctype.h>
#include <wiringPi.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "modem_cw.h"
#include "sound.h"

struct morse_tx {
	char c;
	char *code;
};

struct morse_tx morse_tx_table[] = {
	{'~', " "}, //dummy, a null character
	{' ', " "},
	{'a', ".-"},
	{'b', "-..."},
	{'c', "-.-."},
	{'d', "-.."},
	{'e', "."},
	{'f', "..-."},
	{'g', "--."},
	{'h', "...."},
	{'i', ".."},
	{'j', ".---"},
	{'k', "-.-"},
	{'l', ".-.."},
	{'m', "--"},
	{'n', "-."},
	{'o', "---"},
	{'p', ".--."},
	{'q', "--.-"},
	{'r', ".-."},
	{'s', "..."},
	{'t', "-"},
	{'u', "..-"},
	{'v', "...-"},
	{'w', ".--"},
	{'x', "-..-"},
	{'y', "-.--"},
	{'z', "--.."},
	{'1', ".----"},
	{'2', "..---"},
	{'3', "...--"},
	{'4', "....-"},
	{'5', "....."},
	{'6', "-...."},
	{'7', "--..."},
	{'8', "---.."},
	{'9', "----."},
	{'0', "-----"},
	{'.', ".-.-.-"},
	{',', "--..--"},
	{'?', "..--.."},
	{'/', "-..-."},
	{' ', " "},
	{'=', "-...-"},   //BT (prosigns based upon k40/k42 keyers) W0ANM
	{'<', ".-.-."},   //AR W0ANM
	{'>', "...-.-"},  //SK W0ANM
	//{'+', ".-.-."},
	{'+', "--.- .-. .-.. ..--.."}, //qrl ?  W0ANM
	{'(', "-.--."},   //KN
	//{'[', ".-.-."},
	{'[', "--.- .-. --.."},   //qrz  W0ANM
	//{']', ".-..."},
	{']', "--.- ... .-.."},   //qsl  W0ANM
	{':', ".-..."},   //AS  W0ANM
	{'\'', "--..--"},
	{'&', "-...-"},
};

struct morse_rx {
	char *c;
	char *code;
};

struct morse_rx morse_rx_table[] = {
	{"~", " "}, //dummy, a null character
	{" ", " "},
	{"A", ".-"},
	{"B", "-..."},
	{"C", "-.-."},
	{"D", "-.."},
	{"E", "."},
	{"F", "..-."},
	{"G", "--."},
	{"H", "...."},
	{"I", ".."},
	{"J", ".---"},
	{"K", "-.-"},
	{"L", ".-.."},
	{"M", "--"},
	{"N", "-."},
	{"O", "---"},
	{"P", ".--."},
	{"Q", "--.-"},
	{"R", ".-."},
	{"S", "..."},
	{"T", "-"},
	{"U", "..-"},
	{"V", "...-"},
	{"W", ".--"},
	{"X", "-..-"},
	{"Y", "-.--"},
	{"Z", "--.."},
	{"1", ".----"},
	{"2", "..---"},
	{"3", "...--"},
	{"4", "....-"},
	{"5", "....."},
	{"6", "-...."},
	{"7", "--..."},
	{"8", "---.."},
	{"9", "----."},
	{"0", "-----"},
	{"<STOP>", ".-.-.-"},
	{"<COMMA>", "--..--"},
	{"?", "..--.."},
	{"/", "-..-."},
	{ "'", ".----."},
	{"!", "-.-.--"},
	{":", "---..."},
	{"-", "-....-"},
	{"_", "..--.-"},
	{"@", ".--.-."},
	{"<AR>", ".-.-."},
	{"<AS>", ".-..."},
	{"<STOP>", ".-.-."},
	{"<BT>", "-...-"},
	//{"vu2", "...-..-..---"}, erroneous  W9JES
	//{"vu3", "...-..-...--"}, erroneous  W9JES
	{"5nn", ".....-.-."},
	{"ur", "..-.-."},
};

struct bin {
	float coeff;
	float sine;
	float cosine; 
	float omega;
	int k;
	double scalingFactor;
	int	freq;
	int n;
};

#define MAX_SYMBOLS 100 

struct symbol {
	char is_mark;
	int	magnitude;
	int	ticks;
};

struct cw_decoder{
	int n_samples_per_block;
	int dash_len;
	int mark;
	int prev_mark;
	int	n_bins;
	int ticker;
	int high_level;
	int noise_floor;
	int sig_state;
	int magnitude;
	int symbol_magnitude; // track the magnitude of the current symbol
	int wpm; // as set by the user
	
	struct bin signal;

	// this is a shift register of the states encountered
	int32_t history_sig;
	struct symbol symbol_str[MAX_SYMBOLS];
	int next_symbol;
};

struct cw_decoder decoder;
#define FLOAT_SCALE (1073741824.0)

/* cw tx state variables */
static unsigned long millis_now = 0;

static int cw_key_state = 0;
static int cw_period;
static struct vfo cw_tone, cw_env;
static int keydown_count=0;
static int keyup_count = 0;
static float cw_envelope = 1;		//used to shape the envelope
static int cw_tx_until = 0;			//delay switching to rx, expect more txing
static int data_tx_until = 0;

static char *symbol_next = NULL;
pthread_t iambic_thread;
char iambic_symbol[4];
char cw_symbol_prev = ' ';

static uint8_t cw_current_symbol = CW_IDLE;
static uint8_t cw_next_symbol = CW_IDLE;
static uint8_t cw_last_symbol = CW_IDLE;
static uint8_t cw_mode = CW_STRAIGHT;
static int cw_bytes_available = 0; //chars available in the tx queue
#define CW_MAX_SYMBOLS 12
char cw_key_letter[CW_MAX_SYMBOLS];

static uint8_t cw_get_next_symbol(){  //symbol to translate into CW_DOT, CW_DASH, etc

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
			write_console(STYLE_CW_TX, buff);
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

static FILE *pfout = NULL; //this is debugging out, not used normally

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

static int cw_rx_bin_detect(struct bin *p, int32_t *data){
	float Q2 = 0;
	float Q1 = 0;
	for (int index = 0; index < p->n; index++){
	  float Q0;
  	Q0 = p->coeff * Q1 - Q2 + (float) (*data);
  	Q2 = Q1;
  	Q1 = Q0;	
		data++;
 	}
	double real = (Q1 * p->cosine - Q2) / p->scalingFactor;
  double imag = (Q1 * p->sine) / p->scalingFactor;

 	int  magnitude = sqrt(real*real + imag*imag); 
	return magnitude;
} 

static void cw_rx_match_letter(struct cw_decoder *p){
	char code[MAX_SYMBOLS];

	if (p->next_symbol == 0){
		return;
	}

	int len = p->next_symbol;
	int in_mark = 0;
	int total_ticks = 0;
	int min_dot = (p->dash_len / 6); 
	code[0] = 0;
	int i = 0;

	while(i < p->next_symbol){
		if (p->symbol_str[i].is_mark){
			if(!in_mark && p->symbol_str[i].ticks > min_dot){
				in_mark = 1;
				total_ticks = 0;
			}
		}
		else {
			if(in_mark && p->symbol_str[i].ticks > min_dot){
				in_mark = 0;
				if (total_ticks > p->dash_len / 2){
					strcat(code, "-");
					//track the dashes
					int new_dash = ((p->dash_len * 3) + total_ticks)/4;
					int init_dash_len = (18 * SAMPLING_FREQ) / (5 * N_BINS* p->wpm); 
					if (init_dash_len/2 <  new_dash && new_dash < init_dash_len * 2)
						p->dash_len = new_dash;
					//printf("%d\n", p->dash_len);
				}
				else if (min_dot <= total_ticks && total_ticks <= p->dash_len/2)
					strcat(code, ".");
			}
		}
		total_ticks += p->symbol_str[i].ticks;
		i++;
	}	

	p->next_symbol = 0;
	for (int i = 0; i < sizeof(morse_rx_table)/sizeof(struct morse_rx); i++)
		if (!strcmp(code, morse_rx_table[i].code)){
			write_console(STYLE_CW_RX, morse_rx_table[i].c);
			return;
		}
	//un-decoded phrases
	write_console(STYLE_CW_RX, code);

}

static void cw_rx_add_symbol(struct cw_decoder *p, char symbol){
	if (p->next_symbol == MAX_SYMBOLS)
		p->next_symbol = 0;
	p->symbol_str[p->next_symbol].is_mark = symbol == ' '? 0: 1;
	p->symbol_str[p->next_symbol].ticks = p->ticker;
	p->symbol_str[p->next_symbol].magnitude = 
		((p->symbol_str[p->next_symbol].magnitude *10) + p->magnitude)/11;
	p->next_symbol++;
}

/*
The highs maybe due to noise (that usually lasts very short durations,
Using large n_bins usually does away with that.

*/

#define HIGH_DECAY 100 
#define NOISE_DECAY 100 

static void cw_rx_update_levels(struct cw_decoder *p){
	int new_high = p->magnitude;

	if (p->high_level < p->magnitude)
		p->high_level = new_high;
	else
		p->high_level = (p->magnitude + ((HIGH_DECAY -1) 
			* p->high_level))/HIGH_DECAY;

	if (p->magnitude <  (p->high_level * 4)/10 ){ 
		// clamp the lows to prevent inf
		if (p->magnitude < 100)
			p->magnitude = 100;
		p->noise_floor = (p->magnitude + ((NOISE_DECAY -1) 
			* p->noise_floor))/NOISE_DECAY;
		p->symbol_magnitude += p->magnitude;
	}
}

//we skip the smaller glitches
void cw_rx_denoise(struct cw_decoder *p){

	p->history_sig <<= 1;
	if (p->sig_state)
		p->history_sig |= 1;

	p->prev_mark = p->mark;
	switch(p->history_sig & 0xf){
	case 0:
	case 1:
	case 2:
	case 3:
	case 4:
	case 8:
			p->mark = 0;
			break;
	default:
		p->mark = 30000;
		break;	
	}	
}

static void cw_rx_detect_symbol(struct cw_decoder *p){

	if (p->mark == 0 && p->prev_mark > 0){ //end of mark
		cw_rx_add_symbol(p, 'm');
		p->ticker = 0;
	}
	else if (p->mark > 1 && p->prev_mark == 0){ //start of mark
		cw_rx_add_symbol(p, ' ');
		p->ticker = 0;//reset the timer to measure the length of the mark
	}
	else if (p->mark == 0 && p->prev_mark == 0){ //continuing space
		if (p->next_symbol == 0){
	 		if(p->ticker > (p->dash_len * 3)/2){
				write_console(STYLE_CW_RX, " ");
				p->ticker = 0;
			}
		}
		else if (p->ticker >  p->dash_len/2){
			cw_rx_add_symbol(p, ' ');
			cw_rx_match_letter(p);
			if (p->ticker > (p->dash_len * 3)/2){
				write_console(STYLE_CW_RX, " ");
			}
			p->ticker = 0;
		}
	}
	else if (p->mark > 0  && p->prev_mark > 0){	// skip unusually long dashes
		if (p->ticker > p->dash_len * 3)
			p->ticker = p->dash_len;
	}
}

static void cw_rx_bin(struct cw_decoder *p, int32_t *samples){

	int sig_now = cw_rx_bin_detect(&p->signal, samples);
	
	p->magnitude = sig_now;

	if (p->magnitude > (p->high_level * 6)/10){
			p->sig_state = 30000;
	}
	else if (p->magnitude <  (p->high_level * 4)/10 ){ 
		p->sig_state = 0;
	}

	cw_rx_update_levels(p);
	cw_rx_denoise(p); //this also updates the mark member of struct cw_decode
	cw_rx_detect_symbol(p);
	p->ticker++;

	//only in case of debugging
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
	}
}

void cw_rx(int32_t *samples, int count){
	//the samples better be an integral multiple of n_bins
	int decimation_factor = 96000/SAMPLING_FREQ;
	if (count % (decimation_factor * decoder.n_bins)){
		printf("cw_decoder bins don't align up with sample block %d vs %d\n",
			count, decoder.n_bins);
		assert(0);
	}

	//we decimate the samples from 96000 to 12000
	//this hard coded here 
	int32_t s[128];
	for (int i = 0; i < decoder.n_bins; i++){	
		s[i] = samples[i * 8] >> 8;			
	}
	cw_rx_bin(&decoder, s);
}

/* For now, we will init the dash_len
	 to be 20 wpm initially and track it from there on.
	 This may cause a few inital missed letters until the
	 dash_len converges the senders speed. But it is 
	 better than a manual way to set it.
	 At 20 wpm, it will scale from 10 wpm to 40 wpm. 
	 Below, 10 wpm you don't really need a decoder.
	 For those transmitting at higher than 40 wpm, .. some other day
*/

void cw_init(){	
	//cw rx initializeation
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

	cw_rx_bin_init(&decoder.signal, INIT_TONE, N_BINS, SAMPLING_FREQ);
	
	//init cw tx with some reasonable values
  //cw_env shapes the envelope of the cw waveform
  //frequency was at 50 (20 ms rise time), changed it to 200 (4 ms rise time)
  //to improve cw performance at higher speeds
	vfo_start(&cw_env, 200, 49044); //start in the third quardrant, 270 degree
	vfo_start(&cw_tone, 700, 0);
	cw_period = 9600; 		// At 96ksps, 0.1sec = 1 dot at 12wpm
	cw_key_letter[0] = 0;
	keydown_count = 0;
	keyup_count = 0;
	cw_envelope = 0;
}

void cw_poll(int bytes_available, int tx_is_on){
	cw_bytes_available = bytes_available;
	cw_key_state = key_poll();
	int wpm  = field_int("WPM");
	cw_period = (12 * 9600)/wpm;

	//retune the rx pitch if needed
	int cw_rx_pitch = field_int("PITCH");
	if (cw_rx_pitch != decoder.signal.freq)
		cw_rx_bin_init(&decoder.signal, cw_rx_pitch, N_BINS, SAMPLING_FREQ);

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
