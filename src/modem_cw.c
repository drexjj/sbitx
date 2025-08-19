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
#define FLOAT_SCALE (1073741824.0)  // 2^30
#define HIGH_DECAY 50    // controls max high_level adjustment
#define NOISE_DECAY 100  // controls max noise_level adjustment

// structs and typedefs
struct morse_tx {
	char c;
	const char *code;
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
  // denoise/detect timing and state
  bool mark;
  bool prev_mark;
  bool sig_state;
  uint32_t history_sig;
  int ticker;
  int dot_len;   // use dot as timing unit
  // levels and SNR state
  int magnitude;
  int high_level;
  int noise_floor;
  // frequency-bin tracking
  int max_bin_idx;
  int max_bin_streak;
  // Goertzel bins
  struct bin signal_minus2;
  struct bin signal_minus1;
  struct bin signal_center;
  struct bin signal_plus1;
  struct bin signal_plus2;
  // configuration/state
  int n_bins;
  int wpm;
  int next_symbol;
  int last_char_was_space;
  struct symbol symbol_str[MAX_SYMBOLS];
};

static struct cw_decoder decoder;

// Morse code tables
static const struct morse_tx morse_tx_table[] = {
	{'~', " "}, //dummy, a null character
	{' ', " "}, {'a', ".-"}, {'b', "-..."},	{'c', "-.-."}, {'d', "-.."},
	{'e', "."}, {'f', "..-."}, {'g', "--."}, {'h', "...."}, {'i', ".."},
	{'j', ".---"}, {'k', "-.-"}, {'l', ".-.."}, {'m', "--"}, {'n', "-."},
	{'o', "---"}, {'p', ".--."}, {'q', "--.-"}, {'r', ".-."}, {'s', "..."},
	{'t', "-"}, {'u', "..-"}, {'v', "...-"}, {'w', ".--"}, {'x', "-..-"},
	{'y', "-.--"}, {'z', "--.."}, 
  {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"}, {'5', "....."},
  {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."}, {'0', "-----"},
  {'.', ".-.-.-"}, {',', "--..--"}, {'?', "..--.."},
  {'/', "-..-."}, {'"', ".-..-."}, {'&', "-...-"},
	{'=', "-...-"},   // BT
	{'<', ".-.-."},   // AR
	{'>', "...-.-"},  // SK
	{'(', "-.--."},   // KN
	{':', ".-..."}    // AS
};

// 256-entry look-up table gets filled from morse tx table above
static const char *morse_lut[256];

static const struct morse_rx morse_rx_table[] = {
	{"~", " "}, //dummy, a null character
	{" ", " "}, {"A", ".-"}, {"B", "-..."}, {"C", "-.-."}, {"D", "-.."},
	{"E", "."}, {"F", "..-."}, {"G", "--."}, {"H", "...."}, {"I", ".."},
	{"J", ".---"}, {"K", "-.-"}, {"L", ".-.."}, {"M", "--"}, {"N", "-."},
	{"O", "---"}, {"P", ".--."}, {"Q", "--.-"}, {"R", ".-."}, {"S", "..."},
	{"T", "-"}, {"U", "..-"}, {"V", "...-"}, {"W", ".--"}, {"X", "-..-"},
	{"Y", "-.--"}, {"Z", "--.."}, 
  {"1", ".----"}, {"2", "..---"}, {"3", "...--"},	{"4", "....-"}, {"5", "....."},
  {"6", "-...."}, {"7", "--..."}, {"8", "---.."}, {"9", "----."}, {"0", "-----"}, 
	{"?", "..--.."}, {"/", "-..-."}, { "'", ".----."}, {"!", "-.-.--"}, {":", "---..."},
	{"-", "-....-"}, {"_", "..--.-"},
	{".", ".-.-.-"}, {",", "--..--"}, 
  {"@", ".--.-."}, 
  {"<BK>", "-...-.-"},
  {"<BT>", "-...-"},
  {"<AR>", ".-.-."},
  {"<SK>", "...-.-"},
  {"<KN>", "-.--."},
	{"<AS>", ".-..."},
  // frequently run-together characters that we want to decode right
	{"FB", "..-.-..."}, {"UR", "..-.-."}, {"RST", "._...._"}, {"5NN", ".....-.-."},	
  {"CQ", "-.-.--.-"},	{"73", "--......--"}, {"TNX", "--.-..-"}, {"HW", ".....--"}
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

static const char *symbol_next = NULL;

bool cw_reverse = false;    // cw paddles reverse if true
static uint8_t cw_current_symbol = CW_IDLE;
static uint8_t cw_next_symbol = CW_IDLE;
static uint8_t cw_last_symbol = CW_IDLE;
static uint8_t cw_mode = CW_STRAIGHT;
static int cw_bytes_available = 0;

static int cw_envelope_pos = 0; // position within the envelope
static int cw_envelope_len = 480; // length of the envelope

// Blackman-Harris cw envelope
// data values were calculated in external spreadsheet
static const float cw_envelope_data[480] = {
  0.0f, 0.000001822646818f, 0.000004862928747f, 0.000009124651631f, 0.00001461314364f,
  0.00002133525526f, 0.00002929935926f, 0.00003851535071f, 0.00004899464693f, 0.00006075018742f,
  0.00007379643391f, 0.00008814937022f, 0.0001038265022f, 0.0001208468579f, 0.000139230987f,
  0.0001590009611f, 0.0001801803736f, 0.0002027943394f, 0.0002268694947f, 0.0002524339972f,
  0.0002795175253f, 0.0003081512785f, 0.0003383679766f, 0.0003702018599f, 0.0004036886883f,
  0.0004388657414f, 0.0004757718181f, 0.0005144472358f, 0.00055493383f, 0.000597274954f,
  0.000641515478f, 0.0006877017885f, 0.0007358817877f, 0.0007861048926f, 0.000838422034f,
  0.0008928856557f, 0.0009495497136f, 0.001008469674f, 0.001069702514f, 0.001133306717f,
  0.001199342276f, 0.001267870687f, 0.001338954951f, 0.001412659571f, 0.001489050552f,
  0.001568195394f, 0.001650163095f, 0.00173502415f, 0.00182285054f, 0.001913715741f,
  0.002007694712f, 0.002104863897f, 0.002205301222f, 0.002309086089f, 0.002416299377f,
  0.002527023435f, 0.002641342079f, 0.002759340589f, 0.002881105707f, 0.003006725627f,
  0.003136289998f, 0.003269889912f, 0.003407617907f, 0.003549567953f, 0.003695835457f,
  0.003846517247f, 0.004001711576f, 0.004161518108f, 0.004326037916f, 0.004495373476f,
  0.00466962866f, 0.004848908724f, 0.005033320309f, 0.005222971427f, 0.005417971458f,
  0.005618431137f, 0.005824462549f, 0.006036179119f, 0.006253695604f, 0.006477128085f,
  0.006706593951f, 0.006942211898f, 0.007184101912f, 0.007432385261f, 0.007687184486f,
  0.007948623384f, 0.008216827003f, 0.008491921626f, 0.008774034761f, 0.009063295124f,
  0.009359832633f, 0.009663778389f, 0.009975264662f, 0.01029442488f, 0.01062139362f,
  0.01095630658f, 0.01129930056f, 0.01165051347f, 0.01201008431f, 0.01237815311f,
  0.01275486099f, 0.01314035006f, 0.01353476346f, 0.01393824534f, 0.01435094079f,
  0.01477299587f, 0.0152045576f, 0.01564577389f, 0.01609679355f, 0.01655776627f,
  0.0170288426f, 0.01751017391f, 0.01800191239f, 0.01850421103f, 0.01901722357f,
  0.01954110449f, 0.020076009f, 0.02062209299f, 0.02117951305f, 0.02174842637f,
  0.0223289908f, 0.02292136477f, 0.02352570726f, 0.02414217781f, 0.02477093647f,
  0.02541214377f, 0.02606596068f, 0.02673254864f, 0.02741206944f, 0.02810468528f,
  0.02881055867f, 0.02952985244f, 0.0302627297f, 0.03100935379f, 0.0317698883f,
  0.03254449696f, 0.03333334368f, 0.03413659246f, 0.03495440742f, 0.03578695268f,
  0.03663439242f, 0.03749689077f, 0.03837461181f, 0.03926771955f, 0.04017637784f,
  0.04110075039f, 0.04204100071f, 0.04299729205f, 0.04396978741f, 0.04495864945f,
  0.04596404052f, 0.04698612253f, 0.048025057f, 0.04908100497f, 0.05015412695f,
  0.05124458294f, 0.05235253231f, 0.05347813384f, 0.0546215456f, 0.05578292498f,
  0.0569624286f, 0.05816021229f, 0.05937643102f, 0.06061123891f, 0.06186478915f,
  0.06313723393f, 0.06442872447f, 0.06573941092f, 0.06706944232f, 0.06841896659f,
  0.06978813043f, 0.07117707935f, 0.07258595755f, 0.07401490791f, 0.07546407198f,
  0.07693358984f, 0.07842360016f, 0.07993424009f, 0.08146564522f, 0.08301794957f,
  0.08459128548f, 0.08618578363f, 0.08780157298f, 0.08943878066f, 0.09109753203f,
  0.09277795053f, 0.0944801577f, 0.09620427313f, 0.09795041436f, 0.09971869689f,
  0.1015092341f, 0.1033221373f, 0.1051575154f, 0.1070154753f, 0.1088961214f,
  0.110799556f, 0.1127258787f, 0.114675187f, 0.1166475756f, 0.118643137f,
  0.1206619608f, 0.1227041342f, 0.1247697418f, 0.1268588652f, 0.1289715835f,
  0.1311079729f, 0.1332681068f, 0.1354520557f, 0.1376598871f, 0.1398916657f,
  0.1421474529f, 0.1444273072f, 0.1467312842f, 0.1490594358f, 0.1514118113f,
  0.1537884565f, 0.1561894137f, 0.1586147223f, 0.1610644181f, 0.1635385335f,
  0.1660370975f, 0.1685601357f, 0.17110767f, 0.1736797188f, 0.176276297f,
  0.1788974158f, 0.1815430826f, 0.1842133014f, 0.186908072f, 0.1896273909f,
  0.1923712504f, 0.1951396391f, 0.1979325417f, 0.200749939f, 0.2035918078f,
  0.2064581209f, 0.2093488471f, 0.2122639512f, 0.2152033937f, 0.2181671313f,
  0.2211551164f, 0.2241672971f, 0.2272036175f, 0.2302640174f, 0.2333484323f,
  0.2364567935f, 0.239589028f, 0.2427450584f, 0.2459248029f, 0.2491281755f,
  0.2523550857f, 0.2556054386f, 0.2588791349f, 0.2621760707f, 0.2654961377f,
  0.2688392233f, 0.2722052101f, 0.2755939764f, 0.2790053957f, 0.2824393374f,
  0.2858956658f, 0.2893742409f, 0.2928749183f, 0.2963975485f, 0.2999419779f,
  0.3035080481f, 0.3070955958f, 0.3107044536f, 0.314334449f, 0.3179854052f,
  0.3216571405f, 0.3253494686f, 0.3290621989f, 0.3327951356f, 0.3365480787f,
  0.3403208234f, 0.3441131603f, 0.3479248752f, 0.3517557495f, 0.3556055598f,
  0.3594740783f, 0.3633610725f, 0.3672663051f, 0.3711895345f, 0.3751305144f,
  0.379088994f, 0.3830647179f, 0.3870574261f, 0.3910668542f, 0.3950927334f,
  0.3991347901f, 0.4031927466f, 0.4072663205f, 0.4113552251f, 0.4154591693f,
  0.4195778576f, 0.4237109902f, 0.4278582629f, 0.4320193673f, 0.4361939907f,
  0.4403818162f, 0.4445825227f, 0.4487957847f, 0.453021273f, 0.4572586539f,
  0.4615075898f, 0.4657677391f, 0.4700387562f, 0.4743202914f, 0.4786119913f,
  0.4829134985f, 0.4872244516f, 0.4915444859f, 0.4958732324f, 0.5002103187f,
  0.5045553687f, 0.5089080026f, 0.5132678373f, 0.5176344858f, 0.522007558f,
  0.52638666f, 0.530771395f, 0.5351613626f, 0.5395561592f, 0.5439553779f,
  0.548358609f, 0.5527654393f, 0.5571754528f, 0.5615882306f, 0.5660033508f,
  0.5704203885f, 0.5748389163f, 0.5792585039f, 0.5836787184f, 0.5880991243f,
  0.5925192836f, 0.5969387559f, 0.6013570981f, 0.6057738652f, 0.6101886097f,
  0.6146008819f, 0.6190102301f, 0.6234162004f, 0.6278183372f, 0.6322161827f,
  0.6366092774f, 0.64099716f, 0.6453793677f, 0.6497554359f, 0.6541248985f,
  0.6584872881f, 0.6628421357f, 0.6671889711f, 0.6715273231f, 0.675856719f,
  0.6801766853f, 0.6844867473f, 0.6887864298f, 0.6930752564f, 0.69735275f,
  0.7016184332f, 0.7058718276f, 0.7101124546f, 0.714339835f, 0.7185534896f,
  0.7227529386f, 0.7269377023f, 0.7311073008f, 0.7352612544f, 0.7393990833f,
  0.7435203081f, 0.7476244495f, 0.7517110287f, 0.7557795673f, 0.7598295875f,
  0.763860612f, 0.7678721645f, 0.7718637692f, 0.7758349513f, 0.7797852371f,
  0.7837141538f, 0.7876212299f, 0.7915059951f, 0.7953679803f, 0.799206718f,
  0.8030217422f, 0.8068125885f, 0.810578794f, 0.8143198978f, 0.8180354408f,
  0.8217249658f, 0.8253880176f, 0.8290241434f, 0.8326328922f, 0.8362138155f,
  0.8397664674f, 0.8432904041f, 0.8467851845f, 0.8502503702f, 0.8536855255f,
  0.8570902175f, 0.8604640161f, 0.8638064945f, 0.8671172285f, 0.8703957974f,
  0.8736417837f, 0.8768547729f, 0.8800343544f, 0.8831801207f, 0.8862916679f,
  0.8893685958f, 0.892410508f, 0.8954170119f, 0.8983877184f, 0.9013222429f,
  0.9042202045f, 0.9070812264f, 0.9099049361f, 0.9126909653f, 0.9154389501f,
  0.9181485309f, 0.9208193525f, 0.9234510646f, 0.9260433211f, 0.9285957808f,
  0.9311081073f, 0.9335799689f, 0.9360110389f, 0.9384009954f, 0.9407495217f,
  0.9430563062f, 0.9453210422f, 0.9475434285f, 0.9497231691f, 0.9518599732f,
  0.9539535556f, 0.9560036365f, 0.9580099415f, 0.9599722018f, 0.9618901544f,
  0.9637635418f, 0.9655921122f, 0.9673756199f, 0.9691138246f, 0.9708064922f,
  0.9724533943f, 0.9740543088f, 0.9756090194f, 0.9771173158f, 0.978578994f,
  0.9799938561f, 0.9813617103f, 0.9826823713f, 0.9839556597f, 0.9851814028f,
  0.986359434f, 0.9874895931f, 0.9885717264f, 0.9896056867f, 0.9905913331f,
  0.9915285314f, 0.9924171538f, 0.9932570792f, 0.994048193f, 0.9947903872f,
  0.9954835604f, 0.9961276181f, 0.9967224722f, 0.9972680415f, 0.9977642513f,
  0.9982110337f, 0.9986083278f, 0.9989560792f, 0.9992542402f, 0.9995027701f,
  0.9997016348f, 0.9998508072f, 0.9999502668f, 1.0f
};

//////////////////////////////////////////
// CW transmit and keyer functions
//////////////////////////////////////////

// build the look-up table from morse_tx_table; called once during initialization
static void cw_init_morse_lut(void)
{
    for (int i = 0; i < 256; ++i)
        morse_lut[i] = NULL;

    const size_t n = sizeof(morse_tx_table) / sizeof(morse_tx_table[0]);
    for (size_t i = 0; i < n; ++i) {
        unsigned char ch = (unsigned char)morse_tx_table[i].c;
        morse_lut[ch] = morse_tx_table[i].code;
        // also map uppercase letters if entry is lowercase a–z
        if (ch >= 'a' && ch <= 'z') {
            morse_lut[(unsigned char)(ch - 'a' + 'A')] = morse_tx_table[i].code;
        }
    }
    // just make sure space is mapped
    morse_lut[(unsigned char)' '] = " ";
}

static uint8_t cw_get_next_symbol(){  //symbol to translate into CW_DOT, CW_DASH, etc
	// we are transmitting cw, not rx
  
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
    //process cw key before other cw inputs (macros, keyboard)
    if (cw_key_state != CW_IDLE)
        return cw_key_state;
    if (cw_current_symbol != CW_IDLE)
        return CW_IDLE;
    // we are still sending the previously typed character..
    if (symbol_next)
        return cw_get_next_symbol();
    // no queued input
    if (cw_bytes_available == 0)
        return CW_IDLE;

    char c;
    get_tx_data_byte(&c);

    const unsigned char uc = (unsigned char)c;
    // fast lookup; table contains lowercase entries, and we also mapped uppercase
    symbol_next = morse_lut[uc];

    if (symbol_next) {
        char buff[2] = { (char)toupper(uc), 0 };  // safe ctype: uc is unsigned char
        write_console(FONT_CW_TX, buff);
        return cw_get_next_symbol();
    } else {
        // unknown character: ignore
        return CW_IDLE;
    }
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

  // data driven cw envelope shaping
  // key transmitter with envelope contained in cw_envelope_data[]
  if (keydown_count > 0) {
    if (cw_envelope_pos < cw_envelope_len)
      cw_envelope = cw_envelope_data[cw_envelope_pos++];
    else
      cw_envelope = 1.0f;
    keydown_count--;
  } 
  else if (keyup_count > 0) {
    if (cw_envelope_pos > 0) {
      cw_envelope_pos--;
      cw_envelope = cw_envelope_data[cw_envelope_pos];
    } else 
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

// This code implements the KB2ML sBitx keyer state machine for each CW mode
// State machine uses mode, current state and input to determine keydown_count
// and keyup_count needed to key transmitter.  The large switch case 
// statement has been replaced with dedicated functions for each cw mode

/* used in iambic modes to queue the next element */
static uint8_t cw_next_symbol_flag = 0;

// inline functions replace repeated code
static inline void key_off_short(void)         { keydown_count = 0;           keyup_count = 1;           }
static inline void key_on_short(void)          { keydown_count = 1;           keyup_count = 0;           }
static inline void send_dot(void)              { keydown_count = cw_period;   keyup_count = cw_period;   }
static inline void send_dash(void)             { keydown_count = cw_period*3; keyup_count = cw_period;   }
static inline void schedule(uint8_t sym)       { cw_next_symbol = sym;        cw_next_symbol_flag = 1;   }

static inline void send_symbol_now(uint8_t sym) {
  if (sym == CW_DOT) { send_dot();  cw_last_symbol = CW_DOT; }
  else               { send_dash(); cw_last_symbol = CW_DASH; }
}

static inline void schedule_opposite_of_last(void) {
  schedule(cw_last_symbol == CW_DOT ? CW_DASH : CW_DOT);
}

// functions for each cw mode start here
// straight key
static void handle_mode_straight(uint8_t symbol_now) {
  if (symbol_now == CW_IDLE) { key_off_short(); cw_current_symbol = CW_IDLE; }
  if (symbol_now == CW_DOWN) { key_on_short();  cw_current_symbol = CW_DOWN; }
}

// Vibroplex 'bug' emulation mode.  The 'dit' contact produces
// a string of dits at the chosen WPM, the "dash" contact is
// completely manual and usually used just for dashes
static void handle_mode_bug(uint8_t symbol_now) {
  switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_IDLE)     { key_off_short(); cw_current_symbol = CW_IDLE; }
      if (symbol_now == CW_DOT)      { send_dot();      cw_current_symbol = CW_DOT;  }
      if (symbol_now == CW_DASH)     { key_on_short();  cw_current_symbol = CW_DASH; }
      if (symbol_now == CW_SQUEEZE)  { cw_current_symbol = CW_IDLE; }
      break;
    case CW_DOT:
      if (symbol_now == CW_IDLE)     { cw_current_symbol = CW_IDLE; }
      if (symbol_now == CW_DOT)      { send_dot();      cw_current_symbol = CW_DOT;  }
      if (symbol_now == CW_DASH)     { key_on_short();  cw_current_symbol = CW_DASH; }
      break;
    case CW_DASH:
      if (symbol_now == CW_IDLE)     { cw_current_symbol = CW_IDLE; }
      if (symbol_now == CW_DOT)      { send_dot();      cw_current_symbol = CW_DOT;  }
      if (symbol_now == CW_DASH)     { key_on_short();  cw_current_symbol = CW_DASH; }
      break;
  }
}

// Ultimatic mode - when both paddles are squeezed, whichever one was squeezed last gets repeated
static void handle_mode_ultimatic(uint8_t symbol_now) {
  switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_DOT)         { send_dot();  cw_current_symbol = CW_DOT;  }
      else if (symbol_now == CW_DASH)   { send_dash(); cw_current_symbol = CW_DASH; }
      else if (symbol_now == CW_SQUEEZE){ send_dot(); cw_last_symbol = CW_DASH; cw_current_symbol = CW_SQUEEZE; }
      break;
    case CW_DOT:
      if (symbol_now == CW_IDLE)        { cw_current_symbol = CW_IDLE; }
      else if (symbol_now == CW_SQUEEZE){ send_dash(); cw_last_symbol = CW_DASH; cw_current_symbol = CW_SQUEEZE; }
      else if (symbol_now == CW_DOT)    { send_dot();  cw_current_symbol = CW_DOT;  }
      else if (symbol_now == CW_DASH)   { send_dash(); cw_current_symbol = CW_DASH; }
      break;
    case CW_DASH:
      if (symbol_now == CW_IDLE)        { cw_current_symbol = CW_IDLE; }
      else if (symbol_now == CW_SQUEEZE){ send_dot(); cw_last_symbol = CW_DOT;  cw_current_symbol = CW_SQUEEZE; }
      else if (symbol_now == CW_DOT)    { send_dot();  cw_current_symbol = CW_DOT;  }
      else if (symbol_now == CW_DASH)   { send_dash(); cw_current_symbol = CW_DASH; }
      break;
    case CW_SQUEEZE:
      if (symbol_now == CW_IDLE)        { cw_current_symbol = CW_IDLE; }
      else if (symbol_now == CW_SQUEEZE){
        if (cw_last_symbol == CW_DOT) { send_dot();  cw_last_symbol = CW_DOT;  }
        else                          { send_dash(); cw_last_symbol = CW_DASH; }
        cw_current_symbol = CW_SQUEEZE;
      } else if (symbol_now == CW_DOT)  { send_dot();  cw_current_symbol = CW_DOT;  }
        else if (symbol_now == CW_DASH) { send_dash(); cw_current_symbol = CW_DASH; }
      break;
  }
}

// iambic modes alternate dots and dashes when paddles are squeezed
static void handle_mode_iambic_common(uint8_t symbol_now, bool modeB) {
  /* emit queued symbol as soon as gap is available */
  if (cw_next_symbol_flag && (keyup_count == 0)) {
    send_symbol_now(cw_next_symbol);
    cw_next_symbol_flag = 0;
    return;
  }

  switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_DOT) {
        if (keyup_count == 0) send_symbol_now(CW_DOT);
        cw_current_symbol = CW_DOT;
      } else if (symbol_now == CW_DASH) {
        if (keyup_count == 0) send_symbol_now(CW_DASH);
        cw_current_symbol = CW_DASH;
      } else if (symbol_now == CW_SQUEEZE) {
        if (keyup_count == 0) { send_symbol_now(CW_DOT); schedule(CW_DASH); }
        cw_current_symbol = CW_SQUEEZE;
      }
      break;

    case CW_DOT:
      if (symbol_now == CW_DOT) {
        if (keyup_count == 0) send_symbol_now(CW_DOT);
      } else if (symbol_now == CW_DASH) {
        if (keyup_count == 0) send_symbol_now(CW_DASH);
        else                  schedule(CW_DASH);
        cw_current_symbol = CW_IDLE;
      } else if (symbol_now == CW_SQUEEZE) {
        if (keydown_count > 0) schedule_opposite_of_last();
        cw_current_symbol = CW_SQUEEZE;
      }
      break;

    case CW_DASH:
      if (symbol_now == CW_DASH) {
        if (keyup_count == 0) send_symbol_now(CW_DASH);
      } else if (symbol_now == CW_DOT) {
        if (keyup_count == 0) send_symbol_now(CW_DOT);
        else                  schedule(CW_DOT);
        cw_current_symbol = CW_IDLE;
      } else if (symbol_now == CW_SQUEEZE) {
        if (keydown_count > 0) schedule_opposite_of_last();
        cw_current_symbol = CW_SQUEEZE;
      }
      break;

    case CW_SQUEEZE:
      if (symbol_now == CW_IDLE) {
        cw_current_symbol = CW_IDLE;
      } else if (symbol_now == CW_DOT) {
        if (keyup_count == 0) send_dot();
        cw_last_symbol = CW_DOT;
        cw_current_symbol = CW_DOT;
      } else if (symbol_now == CW_DASH) {
        if (keyup_count == 0) send_dash();
        cw_last_symbol = CW_DASH;
        cw_current_symbol = CW_DASH;
      } else if (symbol_now == CW_SQUEEZE) {
        /* alternate immediate, and in mode B also queue opposite */
        if (keyup_count == 0) {
          if (cw_last_symbol == CW_DOT) { send_dash(); cw_last_symbol = CW_DASH; }
          else                          { send_dot();  cw_last_symbol = CW_DOT;  }
        }
        if (modeB) schedule_opposite_of_last();
        cw_current_symbol = CW_SQUEEZE;
      }
      break;
  }
}

// keyboard mode handles symbols coming from keyboard or macros
static void handle_mode_kbd(uint8_t symbol_now) {
  /* single-state behavior */
  if (symbol_now == CW_IDLE) {
    cw_last_symbol = CW_IDLE; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DOT) {
    send_dot();  cw_last_symbol = CW_DOT;  cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DASH) {
    send_dash(); cw_last_symbol = CW_DASH; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DOT_DELAY) {
    keyup_count = cw_period * 1; cw_last_symbol = CW_DOT_DELAY; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DASH_DELAY) { /* inter-char extension */
    if (cw_last_symbol != CW_WORD_DELAY) keyup_count = cw_period * 2;
    cw_last_symbol = CW_DASH_DELAY; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_WORD_DELAY) { /* inter-word spacing */
    keyup_count = (cw_last_symbol == CW_DASH_DELAY) ? cw_period * 4 : cw_period * 7;
    cw_last_symbol = CW_WORD_DELAY; cw_current_symbol = CW_IDLE;
  }
}

// just call the function based on current mode
void handle_cw_state_machine(uint8_t state_machine_mode, uint8_t symbol_now) {
  // reverse paddle input (EXPERIMENTAL)
  if (cw_reverse && state_machine_mode != CW_KBD) {
    if (symbol_now == CW_DOT) symbol_now = CW_DASH;
    else if (symbol_now == CW_DASH) symbol_now = CW_DOT;
  }
    
  switch (state_machine_mode) {
    case CW_STRAIGHT:  handle_mode_straight(symbol_now);                  break;
    case CW_BUG:       handle_mode_bug(symbol_now);                       break;
    case CW_ULTIMATIC: handle_mode_ultimatic(symbol_now);                 break;
    case CW_IAMBIC:    handle_mode_iambic_common(symbol_now, false);      break;
    case CW_IAMBICB:   handle_mode_iambic_common(symbol_now, true);       break;
    case CW_KBD:       handle_mode_kbd(symbol_now);                       break;
    default: break;
  }
}

//////////////////////////////////////////////////////////////////////////
// CW DECODER FUNCTIONS
// processing flow
// cw_rx(int32_t *samples, int count)  // called from modem_rx()
// │                                      in modems.c
// ├── apply_fir_filter(...) 
// │
// ├── cw_rx_bin(&decoder, s)
// │   ├── cw_rx_bin_detect(&p->signal_center, samples)
// │   ├── cw_rx_bin_detect(&p->signal_plus, samples)
// │   ├── cw_rx_bin_detect(&p->signal_minus, samples)
// │
// ├── cw_rx_update_levels(&decoder)
// │
// ├── cw_rx_denoise(&decoder)
// └── cw_rx_detect_symbol(&decoder) 
//     ├── cw_rx_add_symbol(&decoder, char symbol)  [mark/space transitions]
//     └── cw_rx_match_letter(&decoder)             [symbol/letter boundary]
//////////////////////////////////////////////////////////////////////////

// CW decoder function prototypes
void cw_rx(int32_t *samples, int count);
static const float fir_coeffs[64];
void apply_fir_filter(int32_t *input, int32_t *output, const float *coeffs, int input_count, int order);
static void cw_rx_bin(struct cw_decoder *p, int32_t *samples);
static int  cw_rx_bin_detect(struct bin *p, int32_t *data);
static void cw_rx_update_levels(struct cw_decoder *p);
static void cw_rx_denoise(struct cw_decoder *p);
static void cw_rx_detect_symbol(struct cw_decoder *p);
static void cw_rx_add_symbol(struct cw_decoder *p, char symbol);
static void cw_rx_match_letter(struct cw_decoder *p);

// CW decoder initialization and polling function prototypes
void cw_init(void);
void cw_poll(int bytes_available, int tx_is_on);
static void cw_rx_bin_init(struct bin *p, float freq, int n, float sampling_freq);


// take block of audio samples and call cw decoding functions
void cw_rx(int32_t *samples, int count) {
  int decimation_factor = 8;  // 96 kHz to 12 kHz
  int32_t filtered_samples[count];
  int32_t s[128]; 
  // apply anti-aliasing low pass filter
  apply_fir_filter(samples, filtered_samples, fir_coeffs, count, 64);
  // use decimation_factor to downsample
  // and eliminate eight LSB (and reduce magnitude)
  for (int i = 0; i < decoder.n_bins; i++){	
    s[i] = filtered_samples[i * decimation_factor] >> 8;			
  }
  cw_rx_bin(&decoder, s);         // look for signal in this block
  cw_rx_update_levels(&decoder);  // update high and low noise levels
  cw_rx_denoise(&decoder);        // denoise the signal state
  cw_rx_detect_symbol(&decoder);  // detect Morse symbols
}

// static constant array of low-pass FIR filter coefficients
// generated for a 5000 Hz cutoff at a 96000 Hz sample rate using a Blackman window
static const float fir_coeffs[64] = {
    1.08424165e-19f, -4.95217686e-06f, -8.90038960e-06f, 9.10186219e-06f, 7.22463826e-05f,
    1.99508746e-04f, 3.97578446e-04f, 6.52204412e-04f, 9.20791878e-04f, 1.12876449e-03f,
    1.17243269e-03f, 9.30617527e-04f, 2.85948198e-04f, -8.45319093e-04f, -2.47858182e-03f,
    -4.52722324e-03f, -6.77751473e-03f, -8.87984399e-03f, -1.03625797e-02f, -1.06717427e-02f,
    -9.23519903e-03f, -5.54506182e-03f, 7.52629663e-04f, 9.77507452e-03f, 2.13417483e-02f,
    3.49485179e-02f, 4.97855288e-02f, 6.48009930e-02f, 7.88054510e-02f, 9.06039038e-02f,
    9.91377783e-02f, 1.03616099e-01f, 1.03616099e-01f, 9.91377783e-02f, 9.06039038e-02f,
    7.88054510e-02f, 6.48009930e-02f, 4.97855288e-02f, 3.49485179e-02f, 2.13417483e-02f,
    9.77507452e-03f, 7.52629663e-04f, -5.54506182e-03f, -9.23519903e-03f, -1.06717427e-02f,
    -1.03625797e-02f, -8.87984399e-03f, -6.77751473e-03f, -4.52722324e-03f, -2.47858182e-03f,
    -8.45319093e-04f, 2.85948198e-04f, 9.30617527e-04f, 1.17243269e-03f, 1.12876449e-03f,
    9.20791878e-04f, 6.52204412e-04f, 3.97578446e-04f, 1.99508746e-04f, 7.22463826e-05f,
    9.10186219e-06f, -8.90038960e-06f, -4.95217686e-06f, 1.08424165e-19f
};

// apply the FIR filter using convolution
void apply_fir_filter(int32_t *input, int32_t *output, const float *coeffs, int input_count, int order) {
    for (int i = 0; i < input_count; i++) {
        float sum = 0.0f;
        for (int j = 0; j < order; j++) {
            int k = i - j;
            if (k >= 0) {
                sum += (float)input[k] * coeffs[j];
            }
        }
        output[i] = (int32_t)sum;
    }
}

// detect signal in this block of samples
static void cw_rx_bin(struct cw_decoder *p, int32_t *samples){
  // get magnitude in each of five frequency bins
  // bins are each 93.75 Hz wide
  int mag_minus2 = cw_rx_bin_detect(&p->signal_minus2, samples);
  int mag_minus1 = cw_rx_bin_detect(&p->signal_minus1, samples);
  int mag_center = cw_rx_bin_detect(&p->signal_center, samples);
  int mag_plus1  = cw_rx_bin_detect(&p->signal_plus1,  samples);
  int mag_plus2  = cw_rx_bin_detect(&p->signal_plus2,  samples);

  // find the bin with largest magnitude and its index
  int sig_now = mag_center;  // I think of center bin as bin number 2
  int max_idx = 2;  // bin index with the largest magnitude so far
  if (mag_minus2 > sig_now) {
    sig_now = mag_minus2;
    max_idx = 0;
  }
  if (mag_minus1 > sig_now) {
    sig_now = mag_minus1;
    max_idx = 1;
  }
  if (mag_plus1  > sig_now) {
    sig_now = mag_plus1;
    max_idx = 3;
  }
  if (mag_plus2  > sig_now) {
    sig_now = mag_plus2;
    max_idx = 4;
  }
  p->magnitude = sig_now;
  
  // track winning streak count for max_bin_idx
  if (p->max_bin_idx == max_idx) {
      p->max_bin_streak++;
  } else {
      p->max_bin_streak = 1;
      p->max_bin_idx = max_idx;
  }
    
  // Compare to recent magnitude levels and consider 
  // max_bin_streak length to determine if signal present
  // I set SNR threshold higher when there is no streak going, 
  // and lower when we have a streak
  // when just starting streak we need strong signal in the center bin
  if ((p->max_bin_streak == 1) && (max_idx == 2) &&
        (p->magnitude >= p->noise_floor + 0.6f * (p->high_level - p->noise_floor)))
    p->sig_state = true;
  // with a streak of 2 or more we accept much lower SNR
  else if ((p->max_bin_streak >= 2) &&
        (p->magnitude >= p->noise_floor + 0.15f * (p->high_level - p->noise_floor)))
    p->sig_state = true;
  else
    p->sig_state = false;
  p->ticker++;
//printf("CW bins mag: -2=%d -1=%d 0=%d +1=%d +2=%d (max=%d streak=%d)\n", 
//         mag_minus2, mag_minus1, mag_center, mag_plus1, mag_plus2, p->max_bin_idx, p->max_bin_streak); 
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
  // if the current signal is higher than the tracked peak, update high_level instantly
  if (p->high_level < p->magnitude)
    p->high_level = new_high;
  else
    // decay high_level smoothly toward the new value
    p->high_level = (p->magnitude + ((HIGH_DECAY - 1) * p->high_level)) / HIGH_DECAY;
  // if current magnitude is much lower (less than half way between high and low)
  // it might be background noise or a space between morse marks
  if (p->magnitude < (p->noise_floor + 0.5f * (p->high_level - p->noise_floor))) {
    // limit the minimum magnitude to 100
    if (p->magnitude < 100) p->magnitude = 100;
    // update the noise floor with a similar decay mechanism.
    p->noise_floor = (p->magnitude + ((NOISE_DECAY - 1) * p->noise_floor)) / NOISE_DECAY;
  }
}

// updates the 'mark' state (p->mark) based on a smoothed version of 
// the raw input signal (p->sig_state)
static void cw_rx_denoise(struct cw_decoder *p) {
  p->prev_mark = p->mark; // Store mark as prev_mark BEFORE updating
  // use sliding window to smooth sig_state over time
  p->history_sig <<= 1;   // Shift register: oldest bit out, make room for new sample
  if (p->sig_state) {     // If current input is a 'mark'
    p->history_sig |= 1;  // Set least significant bit
  }
  uint16_t sig = p->history_sig & 0b11111;
  // use Kernighan's algorithm to count number of set bits (1s)
  int count = 0;
  while (sig > 0) {
    sig &= (sig - 1);
    count++;
  }
  // hysteresis enabled to replace majority voting
  if (!p->prev_mark) {
    // we are in a space, set count required to transition to mark
    p->mark = (count >= 3);
  } else {
    // we are in a mark, set count required to stay as mark
    p->mark = (count >= 3);
  }
}

// detect transitions between mark and space and if a dot, dash, character space
// or word space has occurred
static void cw_rx_detect_symbol(struct cw_decoder *p) {
  // detect mark/space transitions and symbol boundaries based on current and previous 'mark' states
  // case where we are at end of a mark (transition from mark to space)
  if (!p->mark && p->prev_mark) {
    cw_rx_add_symbol(p, 'm'); // add a 'mark' (or 'm' for measurement) symbol to the buffer
    p->ticker = 0;            // reset the ticker as a new space period begins
  }
  // case where we are at start of a mark (transition from space to mark)
  else if (p->mark && !p->prev_mark) {
    cw_rx_add_symbol(p, ' '); // add a 'space' symbol (representing the gap before the mark)
    p->ticker = 0;            // reset the ticker to start timing the new mark
  }
  // case where we are continuing space (both current and previous are space)
  else if (!p->mark && !p->prev_mark) {
    // gaps set in relation to dot length
    const int element_gap = p->dot_len;       // 1 dot
    const int char_gap    = 3 * p->dot_len;   // 3 dots
    const int word_gap    = 7 * p->dot_len;   // 7 dots

    if (p->next_symbol == 0) {
      // no symbol being built, check for word gap (long space)
      if (p->ticker >= word_gap) {
        if (!p->last_char_was_space) {
          write_console(FONT_CW_RX, " ");      // output a space to the console (word separator)
          p->last_char_was_space = 1;
        }
        p->ticker = 0;                       // reset ticker after outputting space
      }
    } else {
      // there is an ongoing symbol sequence being built (marks and possibly a trailing space)
      // close the current element when we have at least an element gap (~1 dot) and the last added entry was a mark.
      if (p->ticker >= element_gap) {
        if (p->symbol_str[p->next_symbol - 1].is_mark) {
          // Add a single element-terminating space only once per gap
          cw_rx_add_symbol(p, ' ');
          // do NOT reset ticker here; we want to keep measuring the space to detect char/word gap
        }
      }
      // finalize the character only when we reach a character gap (~3 dots)
      if (p->ticker >= char_gap) {
        cw_rx_match_letter(p);
        // if this also looks like a word gap, print a space
        if (p->ticker >= word_gap) {
          if (!p->last_char_was_space) {
            write_console(FONT_CW_RX, " ");
            p->last_char_was_space = 1;
          }
        }
        // done processing this gap
        p->ticker = 0;
      }
    }
  }
  // case where we are still in a mark (both current and previous are mark)
  else if (p->mark > 0 && p->prev_mark > 0) {
    // clamp overly long dashes to prevent ticker overflow or misinterpretation.
    if (p->ticker >= p->dot_len * 9) {
    p->ticker = p->dot_len * 3;  // cap the ticker at dash length
    }
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
  int min_valid_symbol_duration = (decoder->dot_len * 3) / 5;  // 0.6 dot
  // iterate through all received symbols (marks and spaces)
  for (int i = 0; i < decoder->next_symbol; i++) {
    if (decoder->symbol_str[i].is_mark) {  // if the current symbol is a 'mark' (signal present)
      if (!is_currently_in_mark && decoder->symbol_str[i].ticks >= min_valid_symbol_duration) {
        is_currently_in_mark = 1;
        current_segment_ticks = 0;  // reset tick counter for the new mark segment
      }
    } else {  // If the current symbol is a 'space' (silence)
      if (is_currently_in_mark && decoder->symbol_str[i].ticks >= min_valid_symbol_duration) {
        is_currently_in_mark = 0;  // We are now in a space

        // classify the preceding mark based on its duration
        // dash if > 2.5 dots
        if (current_segment_ticks >= (decoder->dot_len * 7) / 3) {  // 2.5 dots
          // this was a dash
          strcat(morse_code_string, "-");
          // now make adaptive adjustment using observed dash length
          int dash_prev = decoder->dot_len * 3;
          int new_dash  = (dash_prev * 3 + current_segment_ticks) / 4; // 75/25 weighted avg
          int theoretical_dash = (18 * SAMPLING_FREQ) / (5 * N_BINS * decoder->wpm);
          // new_dash should be between half and double expected length
          if (theoretical_dash / 2 < new_dash && new_dash < theoretical_dash * 2) {
            decoder->dot_len = new_dash / 3;
          }
        } else if (current_segment_ticks >= min_valid_symbol_duration) {
          // this was a dot
          strcat(morse_code_string, ".");
        }
      }
    }
    current_segment_ticks +=
        decoder->symbol_str[i].ticks;  // Accumulate ticks for the current segment (mark or space)
  }
  // reset the symbol buffer for the next letter/sequence
  decoder->next_symbol = 0;
	
  // attempt to match the generated Morse code string to a character in the lookup table
  for (int i = 0; i < sizeof(morse_rx_table) / sizeof(struct morse_rx); i++) {
    if (!strcmp(morse_code_string, morse_rx_table[i].code)) {
      // match found, write the decoded character to the console
      write_console(FONT_CW_RX, morse_rx_table[i].c);
      decoder->last_char_was_space = 0;
      return;  // successfully decoded a character
    }
  }
  // if no match was found in the table, output the raw dot/dash sequence.
  write_console(FONT_CW_RX, morse_code_string);
  decoder->last_char_was_space = 0;
}

void cw_init(){	
	//cw rx initialization
	decoder.ticker = 0;
	decoder.n_bins = N_BINS;
	decoder.next_symbol = 0;
	decoder.sig_state = false;
  decoder.prev_mark = false;
	decoder.magnitude= 0;
	decoder.history_sig = 0;
	decoder.wpm = 20;
  decoder.last_char_was_space = 0;
  decoder.max_bin_idx = -1;      // no previous winning bin
  decoder.max_bin_streak = 0;    // no streak yet
	decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * INIT_WPM); 

	// initialize five signal bins
  int cw_rx_pitch = field_int("PITCH");
  cw_rx_bin_init(&decoder.signal_minus2, cw_rx_pitch - 187.5, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_minus1, cw_rx_pitch - 93.75, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_center, cw_rx_pitch, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_plus1,  cw_rx_pitch + 93.75, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_plus2,  cw_rx_pitch + 187.5, N_BINS, SAMPLING_FREQ);
  
	//cw tx initialization
  cw_init_morse_lut();    // build TX Morse code look-up table
  vfo_start(&cw_tone, 700, 0);
  //NOTE: cw_env is not used to shape envelope with "data driven waveform"
	vfo_start(&cw_env, 200, 49044);
	cw_period = 9600; 		// At 96ksps, 0.1sec = 1 dot at 12wpm
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
    cw_rx_bin_init(&decoder.signal_minus2, cw_rx_pitch - 187.5, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_minus1, cw_rx_pitch - 93.75, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_center, cw_rx_pitch, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_plus1,   cw_rx_pitch + 93.75, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_plus2,   cw_rx_pitch + 187.5, N_BINS, SAMPLING_FREQ);
  }
	// check if the wpm has changed
	if (wpm != decoder.wpm){
		decoder.wpm = wpm;
		decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * wpm); 
	}	

	// TX ON if bytes are avaiable (from macro/keyboard) or key is pressed
	// of we are in the middle of symbol (dah/dit) transmission 
	
	if (!tx_is_on && (cw_bytes_available || cw_key_state || (symbol_next && *symbol_next))) {
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
