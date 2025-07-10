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
#include <math.h>
#include <stdbool.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <time.h>
#include <complex.h>
#include <fftw3.h>
#include <pthread.h>
#include <unistd.h>
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

static int cw_envelope_pos = 0; // position within the raised cosine envelope
static int cw_envelope_len = 480; // length of the envelope

// Blackman-Harris cw envelope
// data values were calculated in external spreadsheet
const float cw_envelope_data[480] = {
    0.0, 0.000001822646818, 0.000004862928747, 0.000009124651631, 0.00001461314364,
    0.00002133525526, 0.00002929935926, 0.00003851535071, 0.00004899464693, 0.00006075018742,
    0.00007379643391, 0.00008814937022, 0.0001038265022, 0.0001208468579, 0.000139230987,
    0.0001590009611, 0.0001801803736, 0.0002027943394, 0.0002268694947, 0.0002524339972,
    0.0002795175253, 0.0003081512785, 0.0003383679766, 0.0003702018599, 0.0004036886883,
    0.0004388657414, 0.0004757718181, 0.0005144472358, 0.00055493383, 0.000597274954,
    0.000641515478, 0.0006877017885, 0.0007358817877, 0.0007861048926, 0.000838422034,
    0.0008928856557, 0.0009495497136, 0.001008469674, 0.001069702514, 0.001133306717,
    0.001199342276, 0.001267870687, 0.001338954951, 0.001412659571, 0.001489050552,
    0.001568195394, 0.001650163095, 0.00173502415, 0.00182285054, 0.001913715741,
    0.002007694712, 0.002104863897, 0.002205301222, 0.002309086089, 0.002416299377,
    0.002527023435, 0.002641342079, 0.002759340589, 0.002881105707, 0.003006725627,
    0.003136289998, 0.003269889912, 0.003407617907, 0.003549567953, 0.003695835457,
    0.003846517247, 0.004001711576, 0.004161518108, 0.004326037916, 0.004495373476,
    0.00466962866, 0.004848908724, 0.005033320309, 0.005222971427, 0.005417971458,
    0.005618431137, 0.005824462549, 0.006036179119, 0.006253695604, 0.006477128085,
    0.006706593951, 0.006942211898, 0.007184101912, 0.007432385261, 0.007687184486,
    0.007948623384, 0.008216827003, 0.008491921626, 0.008774034761, 0.009063295124,
    0.009359832633, 0.009663778389, 0.009975264662, 0.01029442488, 0.01062139362,
    0.01095630658, 0.01129930056, 0.01165051347, 0.01201008431, 0.01237815311,
    0.01275486099, 0.01314035006, 0.01353476346, 0.01393824534, 0.01435094079,
    0.01477299587, 0.0152045576, 0.01564577389, 0.01609679355, 0.01655776627,
    0.0170288426, 0.01751017391, 0.01800191239, 0.01850421103, 0.01901722357,
    0.01954110449, 0.020076009, 0.02062209299, 0.02117951305, 0.02174842637,
    0.0223289908, 0.02292136477, 0.02352570726, 0.02414217781, 0.02477093647,
    0.02541214377, 0.02606596068, 0.02673254864, 0.02741206944, 0.02810468528,
    0.02881055867, 0.02952985244, 0.0302627297, 0.03100935379, 0.0317698883,
    0.03254449696, 0.03333334368, 0.03413659246, 0.03495440742, 0.03578695268,
    0.03663439242, 0.03749689077, 0.03837461181, 0.03926771955, 0.04017637784,
    0.04110075039, 0.04204100071, 0.04299729205, 0.04396978741, 0.04495864945,
    0.04596404052, 0.04698612253, 0.048025057, 0.04908100497, 0.05015412695,
    0.05124458294, 0.05235253231, 0.05347813384, 0.0546215456, 0.05578292498,
    0.0569624286, 0.05816021229, 0.05937643102, 0.06061123891, 0.06186478915,
    0.06313723393, 0.06442872447, 0.06573941092, 0.06706944232, 0.06841896659,
    0.06978813043, 0.07117707935, 0.07258595755, 0.07401490791, 0.07546407198,
    0.07693358984, 0.07842360016, 0.07993424009, 0.08146564522, 0.08301794957,
    0.08459128548, 0.08618578363, 0.08780157298, 0.08943878066, 0.09109753203,
    0.09277795053, 0.0944801577, 0.09620427313, 0.09795041436, 0.09971869689,
    0.1015092341, 0.1033221373, 0.1051575154, 0.1070154753, 0.1088961214,
    0.110799556, 0.1127258787, 0.114675187, 0.1166475756, 0.118643137,
    0.1206619608, 0.1227041342, 0.1247697418, 0.1268588652, 0.1289715835,
    0.1311079729, 0.1332681068, 0.1354520557, 0.1376598871, 0.1398916657,
    0.1421474529, 0.1444273072, 0.1467312842, 0.1490594358, 0.1514118113,
    0.1537884565, 0.1561894137, 0.1586147223, 0.1610644181, 0.1635385335,
    0.1660370975, 0.1685601357, 0.17110767, 0.1736797188, 0.176276297,
    0.1788974158, 0.1815430826, 0.1842133014, 0.186908072, 0.1896273909,
    0.1923712504, 0.1951396391, 0.1979325417, 0.200749939, 0.2035918078,
    0.2064581209, 0.2093488471, 0.2122639512, 0.2152033937, 0.2181671313,
    0.2211551164, 0.2241672971, 0.2272036175, 0.2302640174, 0.2333484323,
    0.2364567935, 0.239589028, 0.2427450584, 0.2459248029, 0.2491281755,
    0.2523550857, 0.2556054386, 0.2588791349, 0.2621760707, 0.2654961377,
    0.2688392233, 0.2722052101, 0.2755939764, 0.2790053957, 0.2824393374,
    0.2858956658, 0.2893742409, 0.2928749183, 0.2963975485, 0.2999419779,
    0.3035080481, 0.3070955958, 0.3107044536, 0.314334449, 0.3179854052,
    0.3216571405, 0.3253494686, 0.3290621989, 0.3327951356, 0.3365480787,
    0.3403208234, 0.3441131603, 0.3479248752, 0.3517557495, 0.3556055598,
    0.3594740783, 0.3633610725, 0.3672663051, 0.3711895345, 0.3751305144,
    0.379088994, 0.3830647179, 0.3870574261, 0.3910668542, 0.3950927334,
    0.3991347901, 0.4031927466, 0.4072663205, 0.4113552251, 0.4154591693,
    0.4195778576, 0.4237109902, 0.4278582629, 0.4320193673, 0.4361939907,
    0.4403818162, 0.4445825227, 0.4487957847, 0.453021273, 0.4572586539,
    0.4615075898, 0.4657677391, 0.4700387562, 0.4743202914, 0.4786119913,
    0.4829134985, 0.4872244516, 0.4915444859, 0.4958732324, 0.5002103187,
    0.5045553687, 0.5089080026, 0.5132678373, 0.5176344858, 0.522007558,
    0.52638666, 0.530771395, 0.5351613626, 0.5395561592, 0.5439553779,
    0.548358609, 0.5527654393, 0.5571754528, 0.5615882306, 0.5660033508,
    0.5704203885, 0.5748389163, 0.5792585039, 0.5836787184, 0.5880991243,
    0.5925192836, 0.5969387559, 0.6013570981, 0.6057738652, 0.6101886097,
    0.6146008819, 0.6190102301, 0.6234162004, 0.6278183372, 0.6322161827,
    0.6366092774, 0.64099716, 0.6453793677, 0.6497554359, 0.6541248985,
    0.6584872881, 0.6628421357, 0.6671889711, 0.6715273231, 0.675856719,
    0.6801766853, 0.6844867473, 0.6887864298, 0.6930752564, 0.69735275,
    0.7016184332, 0.7058718276, 0.7101124546, 0.714339835, 0.7185534896,
    0.7227529386, 0.7269377023, 0.7311073008, 0.7352612544, 0.7393990833,
    0.7435203081, 0.7476244495, 0.7517110287, 0.7557795673, 0.7598295875,
    0.763860612, 0.7678721645, 0.7718637692, 0.7758349513, 0.7797852371,
    0.7837141538, 0.7876212299, 0.7915059951, 0.7953679803, 0.799206718,
    0.8030217422, 0.8068125885, 0.810578794, 0.8143198978, 0.8180354408,
    0.8217249658, 0.8253880176, 0.8290241434, 0.8326328922, 0.8362138155,
    0.8397664674, 0.8432904041, 0.8467851845, 0.8502503702, 0.8536855255,
    0.8570902175, 0.8604640161, 0.8638064945, 0.8671172285, 0.8703957974,
    0.8736417837, 0.8768547729, 0.8800343544, 0.8831801207, 0.8862916679,
    0.8893685958, 0.892410508, 0.8954170119, 0.8983877184, 0.9013222429,
    0.9042202045, 0.9070812264, 0.9099049361, 0.9126909653, 0.9154389501,
    0.9181485309, 0.9208193525, 0.9234510646, 0.9260433211, 0.9285957808,
    0.9311081073, 0.9335799689, 0.9360110389, 0.9384009954, 0.9407495217,
    0.9430563062, 0.9453210422, 0.9475434285, 0.9497231691, 0.9518599732,
    0.9539535556, 0.9560036365, 0.9580099415, 0.9599722018, 0.9618901544,
    0.9637635418, 0.9655921122, 0.9673756199, 0.9691138246, 0.9708064922,
    0.9724533943, 0.9740543088, 0.9756090194, 0.9771173158, 0.978578994,
    0.9799938561, 0.9813617103, 0.9826823713, 0.9839556597, 0.9851814028,
    0.986359434, 0.9874895931, 0.9885717264, 0.9896056867, 0.9905913331,
    0.9915285314, 0.9924171538, 0.9932570792, 0.994048193, 0.9947903872,
    0.9954835604, 0.9961276181, 0.9967224722, 0.9972680415, 0.9977642513,
    0.9982110337, 0.9986083278, 0.9989560792, 0.9992542402, 0.9995027701,
    0.9997016348, 0.9998508072, 0.9999502668, 1.0
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

  // data driven envelope shaping
  // key transmitter with envelope contained in cw_envelope_data[]
  if (keydown_count > 0) {
    if (cw_envelope_pos < cw_envelope_len)
      cw_envelope = cw_envelope_data[cw_envelope_pos++];
    else
      cw_envelope = 1.0f;
    keydown_count--;
  } 
  else if (keyup_count > 0) {
    if (cw_envelope_len > cw_envelope_pos)
      cw_envelope = cw_envelope_data[cw_envelope_pos--];
    else
      cw_envelope = 0.0f;
    keyup_count--;
  }
  sample = ((vfo_read(&cw_tone) / FLOAT_SCALE) * cw_envelope) / 8;
  
  // keep extending 'cw_tx_until' while we're sending
  if ((symbol_now == CW_DOWN) || (symbol_now == CW_DOT) ||
      (symbol_now == CW_DASH) || (symbol_now == CW_SQUEEZE) ||
      (keydown_count > 0))
    cw_tx_until = millis_now + get_cw_delay();
  // if macro or keyboard characters remain in the buffer
  // prevent switching from xmit to rcv and cutting off macro
  if (cw_bytes_available != 0)
    cw_tx_until = millis_now + 1000;

  return sample;
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
      if (symbol_now == CW_IDLE) {
        keydown_count = 0;
        keyup_count = 1;
        cw_current_symbol = CW_IDLE;
      }
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
     if (symbol_now == CW_IDLE) {
        keydown_count = 0;
        keyup_count = 1;
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
			write_console(FONT_CW_RX, morse_rx_table[i].c);
			return;
		}
	//un-decoded phrases
	write_console(FONT_CW_RX, code);

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
				write_console(FONT_CW_RX, " ");
				p->ticker = 0;
			}
		}
		else if (p->ticker >  p->dash_len/2){
			cw_rx_add_symbol(p, ' ');
			cw_rx_match_letter(p);
			if (p->ticker > (p->dash_len * 3)/2){
				write_console(FONT_CW_RX, " ");
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
