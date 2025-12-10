// standard library includes
#include <assert.h>
#include <complex.h>
#include <ctype.h>  
#include <math.h>   
#include <stdint.h> 
#include <stdbool.h>
#include <stdio.h>
#include <stddef.h>
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

// structs
struct morse_tx {
	char c;
	const char *code;
};

struct morse_rx {
	char *c;
	const char *code;
};

struct bin {
	float coeff;
	float sine;
	float cosine;
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
  // configuration
  int n_bins;
  int wpm;
  int console_font;  // decoder output STYLE_CW_RX or STYLE_CW_TX

  // bins (Goertzel filters at offsets)
  struct bin signal_minus2;
  struct bin signal_minus1;
  struct bin signal_center;
  struct bin signal_plus1;
  struct bin signal_plus2;

  // detect (instantaneous detection / denoise history)
  bool mark;
  bool prev_mark;
  bool sig_state;
  uint32_t history_sig;
  int ticker;

  // levels (SNR/high/low tracking + winning bin)
  int magnitude;
  int high_level;
  int noise_floor;
  int max_bin_idx;
  int max_bin_streak;

  // timing (dot length, gaps, symbol assembly state)
  int dot_len;              // use dot as timing unit
  int next_symbol;
  int last_char_was_space;
  float space_ema;          // EMA of inter-character space (ticks)
  float char_gap_ema;
  float word_gap_ema;

  // kmeans 2-D clustering state
  float k_centroid_noise[2];   // centroid for noise cluster
  float k_centroid_signal[2];  // centroid for signal cluster
  int   k_count_noise;         // decaying counts (informational)
  int   k_count_signal;
  float k_alpha;               // learning rate for centroid updates (0..1)
  int   k_warmup;              // warmup/seed counter
  int   k_signal_streak;       // consecutive signal assignments
  int   k_noise_streak;        // consecutive noise assignments
  bool  k_initialized;         // true when centroids have been seeded

  // mark emission parameters in log-domain
  float mark_mu_dot;
  float mark_var_dot;
  float mark_mu_dash;
  float mark_var_dash;
  bool  mark_emission_ready;

  // symbol buffer
  struct symbol symbol_str[MAX_SYMBOLS];
};

static struct cw_decoder decoder;
static struct cw_decoder tx_decoder;

// Morse code tables
static const struct morse_tx morse_tx_table[] = {
    {'~', " "}, // dummy, a null character or sentinel
    {' ', " "},

    // Letters
    {'a', ".-"},   {'b', "-..."}, {'c', "-.-."}, {'d', "-.."},  {'e', "."},
    {'f', "..-."}, {'g', "--."},  {'h', "...."}, {'i', ".."},   {'j', ".---"},
    {'k', "-.-"},  {'l', ".-.."}, {'m', "--"},   {'n', "-."},   {'o', "---"},
    {'p', ".--."}, {'q', "--.-"}, {'r', ".-."},  {'s', "..."},  {'t', "-"},
    {'u', "..-"},  {'v', "...-"}, {'w', ".--"},  {'x', "-..-"}, {'y', "-.--"},
    {'z', "--.."},

    // Digits
    {'1', ".----"}, {'2', "..---"}, {'3', "...--"}, {'4', "....-"}, {'5', "....."},
    {'6', "-...."}, {'7', "--..."}, {'8', "---.."}, {'9', "----."}, {'0', "-----"},

    // Punctuation
    {'.', ".-.-.-"},
    {',', "--..--"},
    {'?', "..--.."},
    {'\'', ".----."},   // apostrophe
    {'!', "-.-.--"},
    {'/', "-..-."},
    {')', "-.--.-"},
    {':', "---..."},
    {';', "-.-.-."},
    {'-', "-....-"},    // hyphen/minus
    {'_', "..--.-"},    // underscore
    {'"', ".-..-."},    // quote
    {'@', ".--.-."},
    {'$', "...-..-"},
    {'+', ".-.-."},     // plus (same code as AR prosign)
    {'=', "-...-"},     // equals (same code as BT prosign)
    {'&', ".-..."},     // ampersand (same code as AS prosign)
    {'(', "-.--."},     // left paren (same code as KN prosign)
    {'>', "...-.-"},    // <SK> prosign
    {'<', "-...-.-"}    // <BK> prosign
};

// 256-entry look-up table gets filled from morse tx table above
// and will add uppercase letters as well
static const char *morse_lut[256];

static const struct morse_rx morse_rx_table[] = {
    {"~", " "}, // dummy, a null character
    {" ", " "},

    {"A", ".-"},   {"B", "-..."}, {"C", "-.-."}, {"D", "-.."},  {"E", "."},
    {"F", "..-."}, {"G", "--."},  {"H", "...."}, {"I", ".."},   {"J", ".---"},
    {"K", "-.-"},  {"L", ".-.."}, {"M", "--"},   {"N", "-."},   {"O", "---"},
    {"P", ".--."}, {"Q", "--.-"}, {"R", ".-."},  {"S", "..."},  {"T", "-"},
    {"U", "..-"},  {"V", "...-"}, {"W", ".--"},  {"X", "-..-"}, {"Y", "-.--"},
    {"Z", "--.."},

    {"1", ".----"}, {"2", "..---"}, {"3", "...--"}, {"4", "....-"}, {"5", "....."},
    {"6", "-...."}, {"7", "--..."}, {"8", "---.."}, {"9", "----."}, {"0", "-----"},

    {"?", "..--.."}, {"/", "-..-."}, {"'", ".----."}, {"!", "-.-.--"}, {":", "---..."},
    {"-", "-....-"}, {"_", "..--.-"}, {".", ".-.-.-"}, {",", "--..--"}, {"@", ".--.-."},
    {")", "-.--.-"}, {"$", "...-..-"}, {";", "-.-.-."},
    // note: omitted "+", "=", "&", "(" to avoid collisions with <AR>, <BT>, <AS>, <KN>

    // Prosigns
    {"<AR>", ".-.-."},
    {"<BT>", "-...-"},
    {"<AS>", ".-..."},
    {"<KN>", "-.--."},
    {"<BK>", "-...-.-"},
    {"<SK>", "...-.-"},
    {"<KA>", "-.-.-"},

    // frequently run-together characters that we want to decode right (concatenated codes)
    {"FB",  "..-.-..."},   // F (..-.) + B (-...)
    {"UR",  "..-.-."},     // U (..-) + R (.-.)
    {"RST", ".-....-"},    // R (.-.) + S (...) + T (-)
    {"5NN", ".....-.-."},  // 5 (.....) + N (-.) + N (-.)
    {"CQ",  "-.-.--.-"},   // C (-.-.) + Q (--.-)
    {"73",  "--......--"}, // 7 (--...) + 3 (...--)
    {"TNX", "--.-..-"},    // T (-) + N (-.) + X (-..-)
    {"HW",  ".....--"},    // H (....) + W (.--)
    {"QRZ", "--.-.-.--.."} // Q (--.-) + R (.-.) + Z (--..)
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

extern int text_ready;  // flag that TEXT buffer in gui is ready to send
extern int cw_decode_enabled;  // flag that UI has enabled cw decoding

static int cw_envelope_pos = 0; // position within the envelope
static int cw_envelope_len = 480; // length of the envelope

static int32_t tx_sample_buffer[1024];  // TX decoder buffer
static int tx_buffer_pos = 0;
static bool tx_session_active = false;  // Track if we're in a TX session

// these values are to suppor a degug log only and can be deleted in operational code
static FILE *cw_rx_log_fp = NULL;
static int cw_rx_log_count = 0;
static bool cw_rx_log_done = false;

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

// low-pass FIR filter coefficients
// generated for a 5000 Hz cutoff at a 96000 Hz sample rate using a Blackman window
static const float fir_coeffs[64] = {
    1.08424165e-19f,  -4.95217686e-06f, -8.90038960e-06f, 9.10186219e-06f,  7.22463826e-05f,
    1.99508746e-04f,  3.97578446e-04f,  6.52204412e-04f,  9.20791878e-04f,  1.12876449e-03f,
    1.17243269e-03f,  9.30617527e-04f,  2.85948198e-04f,  -8.45319093e-04f, -2.47858182e-03f,
    -4.52722324e-03f, -6.77751473e-03f, -8.87984399e-03f, -1.03625797e-02f, -1.06717427e-02f,
    -9.23519903e-03f, -5.54506182e-03f, 7.52629663e-04f,  9.77507452e-03f,  2.13417483e-02f,
    3.49485179e-02f,  4.97855288e-02f,  6.48009930e-02f,  7.88054510e-02f,  9.06039038e-02f,
    9.91377783e-02f,  1.03616099e-01f,  1.03616099e-01f,  9.91377783e-02f,  9.06039038e-02f,
    7.88054510e-02f,  6.48009930e-02f,  4.97855288e-02f,  3.49485179e-02f,  2.13417483e-02f,
    9.77507452e-03f,  7.52629663e-04f,  -5.54506182e-03f, -9.23519903e-03f, -1.06717427e-02f,
    -1.03625797e-02f, -8.87984399e-03f, -6.77751473e-03f, -4.52722324e-03f, -2.47858182e-03f,
    -8.45319093e-04f, 2.85948198e-04f,  9.30617527e-04f,  1.17243269e-03f,  1.12876449e-03f,
    9.20791878e-04f,  6.52204412e-04f,  3.97578446e-04f,  1.99508746e-04f,  7.22463826e-05f,
    9.10186219e-06f,  -8.90038960e-06f, -4.95217686e-06f, 1.08424165e-19f};


//////////////////////////////////////////
//  Function prototypes
//  ordered to match definitions in code
//////////////////////////////////////////

// CW TX/keyer prototypes (ordered to match definitions in code)
static void     cw_init_morse_lut(void);
static uint8_t  cw_get_next_symbol(void);
static int      cw_read_key(void);
float           cw_tx_get_sample(void);
static void     handle_mode_straight(uint8_t symbol_now);
static void     handle_mode_bug(uint8_t symbol_now);
static void     handle_mode_ultimatic(uint8_t symbol_now);
static void     handle_mode_iambic_common(uint8_t symbol_now, bool modeB);
static void     handle_mode_kbd(uint8_t symbol_now);
void            handle_cw_state_machine(uint8_t state_machine_mode, uint8_t symbol_now);

// CW decoder entry points and helpers
void            cw_rx(int32_t *samples, int count);
static void     cw_tx_decode_samples(void);
void            apply_fir_filter(int32_t *input, int32_t *output, const float *coeffs, int input_count, int order);
static void     cw_rx_bin(struct cw_decoder *p, int32_t *samples);
int             cw_get_max_bin_highlight_index(void);
static int      cw_rx_bin_detect(struct bin *p, int32_t *data);
static int      cw_rx_kmeans_update_2d(struct cw_decoder *p, int magnitude, int max_idx,
                                       float *out_d_noise, float *out_d_signal, float *out_centroid_sep);
static int      kmeans_1d(float *values, int n, int k, float *clusters, int *assignments);
static bool     estimate_mark_emissions(struct cw_decoder *p, float *log_mark_durs, int n,
                                        float *dot_mu, float *dot_var, float *dash_mu, float *dash_var);
static void     viterbi_decode_marks(float *log_mark_durs, int n, float dot_mu, float dot_var,
                                     float dash_mu, float dash_var, int *out_assignments);
static void     cw_rx_update_levels(struct cw_decoder *p);
static void     cw_rx_denoise(struct cw_decoder *p);
static bool     cw_rx_detect_symbol(struct cw_decoder *p);
static void     cw_rx_add_symbol(struct cw_decoder *p, char symbol);
static void     cw_rx_match_letter(struct cw_decoder *decoder);
static void     cw_rx_bin_init(struct bin *p, float freq, int n, float sampling_freq);

// CW init/poll/stats API
void            cw_init(void);
char           *cw_get_stats(char *buf, size_t len);
void            cw_poll(int bytes_available, int tx_is_on);
void            cw_abort(void);

//////////////////////////////////////////
//      CW transmit and keyer functions
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
        // REMOVED WITH DECODING CW ON TX MOD
        //char buff[2] = { (char)toupper(uc), 0 };  // safe ctype: uc is unsigned char
        //write_console(STYLE_CW_TX, buff);
        return cw_get_next_symbol();
    } else {
        // unknown character: ignore
        return CW_IDLE;
    }
}

// use input from macro playback, keyboard or key/paddle to key the transmitter
// keydown and keyup times
float cw_tx_get_sample() {
  float sample = 0;
  uint8_t state_machine_mode;
  static uint8_t symbol_now = CW_IDLE;
  
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
  
  // generate cw_tone sample for transmission
  float tone = vfo_read(&cw_tone) / FLOAT_SCALE;
  // apply envelope for actual transmitted audio
  sample = (tone * cw_envelope) / 8;

  // for TX decoding, use a hard-gated tone to provide decoder
  float decode_sample = 0.0f;
  if (cw_envelope > 0.001f) decode_sample = tone / 8.0f;  //reduce level of sampled TX signal
  tx_sample_buffer[tx_buffer_pos++] = (int32_t)(decode_sample * 32768.0f);
  // when buffer is full send it to TX decoder
  if (tx_buffer_pos >= 1024) {
      cw_tx_decode_samples();
      tx_buffer_pos = 0;
  }
  
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

// used in iambic modes to queue the next element
static uint8_t cw_next_symbol_flag = 0;

// inline functions replace repeated code
static inline void key_off_short(void)   { keydown_count = 0;           keyup_count = 1;         }
static inline void key_on_short(void)    { keydown_count = 1;           keyup_count = 0;         }
static inline void send_dot(void)        { keydown_count = cw_period;   keyup_count = cw_period; }
static inline void send_dash(void)       { keydown_count = cw_period*3; keyup_count = cw_period; }
static inline void schedule(uint8_t sym) { cw_next_symbol = sym;        cw_next_symbol_flag = 1; }

static inline void send_symbol_now(uint8_t sym) {
  if (sym == CW_DOT) { send_dot();  cw_last_symbol = CW_DOT; }
  else               { send_dash(); cw_last_symbol = CW_DASH; }
}

static inline void schedule_opposite_of_last(void) {
  if (cw_last_symbol == CW_DOT) { schedule(CW_DASH); }
  else                          { schedule(CW_DOT); }
}

// functions for each cw mode start here
// straight key
static void handle_mode_straight(uint8_t symbol_now) {
  if (symbol_now == CW_IDLE) { key_off_short(); cw_current_symbol = CW_IDLE; }
  else if (symbol_now == CW_DOWN) { key_on_short();  cw_current_symbol = CW_DOWN; }
}

// Vibroplex 'bug' emulation mode.  The 'dit' contact produces
// a string of dits at the chosen WPM, the "dash" contact is
// completely manual and usually used just for dashes
static void handle_mode_bug(uint8_t symbol_now) {
  switch (cw_current_symbol) {
    case CW_IDLE:
      if (symbol_now == CW_IDLE)       { key_off_short(); cw_current_symbol = CW_IDLE; }
      else if (symbol_now == CW_DOT)   { send_dot();      cw_current_symbol = CW_DOT;  }
      else if (symbol_now == CW_DASH)  { key_on_short();  cw_current_symbol = CW_DASH; }
      else if (symbol_now == CW_SQUEEZE) { cw_current_symbol = CW_IDLE; }
      break;
    case CW_DOT:
      if (symbol_now == CW_IDLE)       { cw_current_symbol = CW_IDLE; }
      else if (symbol_now == CW_DOT)   { send_dot();      cw_current_symbol = CW_DOT;  }
      else if (symbol_now == CW_DASH)  { key_on_short();  cw_current_symbol = CW_DASH; }
      break;
    case CW_DASH:
      if (symbol_now == CW_IDLE)       { cw_current_symbol = CW_IDLE; }
      else if (symbol_now == CW_DOT)   { send_dot();      cw_current_symbol = CW_DOT;  }
      else if (symbol_now == CW_DASH)  { key_on_short();  cw_current_symbol = CW_DASH; }
      break;
    default:
      cw_current_symbol = CW_IDLE;
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
  // emit queued symbol as soon as gap is available
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
        // alternate immediate, and in IAMBICB also queue opposite
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
  // single-state behavior
  if (symbol_now == CW_IDLE) {
    cw_last_symbol = CW_IDLE; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DOT) {
    send_dot();  cw_last_symbol = CW_DOT;  cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DASH) {
    send_dash(); cw_last_symbol = CW_DASH; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DOT_DELAY) {
    keyup_count = cw_period * 1; cw_last_symbol = CW_DOT_DELAY; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_DASH_DELAY) { // inter-char extension
    if (cw_last_symbol != CW_WORD_DELAY) keyup_count = cw_period * 2;
    cw_last_symbol = CW_DASH_DELAY; cw_current_symbol = CW_IDLE;
  } else if (symbol_now == CW_WORD_DELAY) { // inter-word spacing
    keyup_count = (cw_last_symbol == CW_DASH_DELAY) ? cw_period * 4 : cw_period * 7;
    cw_last_symbol = CW_WORD_DELAY; cw_current_symbol = CW_IDLE;
  }
}

// call the keyer function for current mode
void handle_cw_state_machine(uint8_t state_machine_mode, uint8_t symbol_now) {
  static uint8_t prev_mode = 0xFF;
  // initialize when changing modes
  if (state_machine_mode != prev_mode) {
    cw_current_symbol     = CW_IDLE;
    cw_last_symbol        = CW_IDLE;
    cw_next_symbol        = CW_IDLE;
    cw_next_symbol_flag   = 0;
  }
  prev_mode = state_machine_mode;
  
  // reverse paddle input
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
// CW Decoder Functions
//
// cw_rx (entry point)   NOTE: cw_tx_decode_samples follow same flow
// └─ apply_fir_filter(input, filtered_samples, fir_coeffs, count, 64)
//    └─ decimation / build s[]
//       └─ cw_rx_bin(&decoder, s)
//          ├─ cw_rx_bin_detect(&p->signal_minus2, samples)  (Goertzel)
//          ├─ cw_rx_bin_detect(&p->signal_minus1, samples)  (Goertzel)
//          ├─ cw_rx_bin_detect(&p->signal_center, samples)  (Goertzel)
//          ├─ cw_rx_bin_detect(&p->signal_plus1,  samples)  (Goertzel)
//          └─ cw_rx_bin_detect(&p->signal_plus2,  samples)  (Goertzel)
//          └─ cw_rx_kmeans_update_2d(p, magnitude, max_idx)
//          └─ (clustering decision & set p->sig_state)
//       └─ cw_rx_update_levels(&decoder)
//       └─ cw_rx_denoise(&decoder)
//       └─ cw_rx_detect_symbol(&decoder) return true when character gap detected
//          ├─ if transition mark->space:
//          │   └─ cw_rx_add_symbol(p, 'm')
//          ├─ if transition space->mark:
//          │   └─ cw_rx_add_symbol(p, ' ')
//          ├─ when measuring gaps (space continuing):
//              └─ (maybe) write_console(STYLE_CW_RX, " ")
//       └─ cw_rx_match_letter(&decoder) when character gap detected:
//          ├─ build mark_durs[] / mark_mags[]]
//          ├─ compute log_mark_durs[]
//          ├─ estimate_mark_emissions(decoder, log_mark_durs, n_marks, ...)
//          │   └─ kmeans_1d(log_mark_durs, n_marks, 2, clusters, assignments)
//          │      └─ (iterative 1-D kmeans, returns clusters + assignments)
//          ├─ if est_ok:
//          │   └─ viterbi_decode_marks(log_mark_durs, n_marks, dot_mu, 
//          │          dot_var, dash_mu, dash_var, assignments)
//          │      └─ (Viterbi DP, returns assignments)
//          └─ else (fallback):
//              └─ greedy threshold classifier
//          ├─ build morse_code_string by iterating symbol_str[] + assignments
//          ├─ (map morse string to table)
//          │   └─ write_console(STYLE_CW_RX, morse or char)
//          └─ update decoder->dot_len (adaptation)
//
//////////////////////////////////////////////////////////////////////////

// this is the entry point for cw decoder functions
// take block of audio samples and call cw decoding functions
// inputs:  int32_t *samples, int count
// returns: void
void cw_rx(int32_t *samples, int count) {
  int decimation_factor = 8;  // 96 kHz -> 12 kHz
  int32_t filtered_samples[count];  // use local copies and don't modify original data
  int32_t s[128];
  // apply anti-aliasing low pass filter then down-sample
  apply_fir_filter(samples, filtered_samples, fir_coeffs, count, 64);
  // downsample and eliminate eight LSB (A/D was 24 bits, not 32)
  for (int i = 0; i < decoder.n_bins; i++){	
    s[i] = filtered_samples[i * decimation_factor] >> 8;			
  }
  cw_rx_bin(&decoder, s);              // look for signal in this block
  cw_rx_update_levels(&decoder);       // update high and low noise levels
  cw_rx_denoise(&decoder);             // denoise the signal state
  if (cw_rx_detect_symbol(&decoder)) { // returns true when character gap found 
    cw_rx_match_letter(&decoder);
  }
}

// entry point for decoding transmitted CW
// operates much like cw_rx()
static void cw_tx_decode_samples(void) {
  int decimation_factor = 8;  // 96 kHz -> 12 kHz
  int32_t s[N_BINS];

  // decimate directly (no FIR) and drop eight LSBs
  for (int i = 0; i < N_BINS; i++) {
    s[i] = tx_sample_buffer[i * decimation_factor] >> 8;
  }
	
  // process through decoder just like on RX
  cw_rx_bin(&tx_decoder, s);
  cw_rx_update_levels(&tx_decoder);
  cw_rx_denoise(&tx_decoder);
  if (cw_rx_detect_symbol(&tx_decoder))
    cw_rx_match_letter(&tx_decoder);
}

// apply the FIR low-pass filter using convolution
// inputs:  int32_t *input, int32_t *output, const float *coeffs, int input_count, int order
// returns: void
// notes:  added to eliminate aliasing with down-sampling (LPF possibly not needed?)
void apply_fir_filter(int32_t *input, int32_t *output, const float *coeffs, int input_count,
                      int order) {
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
// inputs:  struct cw_decoder *p, int32_t *samples
// returns: void
// notes:
static void cw_rx_bin(struct cw_decoder *p, int32_t *samples) {
  // get magnitude in each of five frequency bins
  // Bins are at -80, -35, 0, +35, +80 Hz relative to center pitch
  int mag_minus2 = cw_rx_bin_detect(&p->signal_minus2, samples);
  int mag_minus1 = cw_rx_bin_detect(&p->signal_minus1, samples);
  int mag_center = cw_rx_bin_detect(&p->signal_center, samples);
  int mag_plus1  = cw_rx_bin_detect(&p->signal_plus1,  samples);
  int mag_plus2  = cw_rx_bin_detect(&p->signal_plus2,  samples);

  // find the bin with largest magnitude and its index
  int sig_now = mag_center;
  int max_idx = 2;
  if (mag_minus2 > sig_now) { sig_now = mag_minus2; max_idx = 0; }
  if (mag_minus1 > sig_now) { sig_now = mag_minus1; max_idx = 1; }
  if (mag_plus1 > sig_now)  { sig_now = mag_plus1;  max_idx = 3; }
  if (mag_plus2 > sig_now)  { sig_now = mag_plus2;  max_idx = 4; }
  p->magnitude = sig_now;

  // track winning streak count for max_bin_idx
  if (p->max_bin_idx == max_idx) {
    p->max_bin_streak++;
  } else {
    p->max_bin_streak = 1;
    p->max_bin_idx = max_idx;
  }

  // replaced old simple SNR comparison with 2-D clustering decision
  // features: x0 = log(1+mag), x1 = distance-from-center (abs(max_idx-2))
  // we still fallback to legacy method while clustering is not yet reliable,
  // but use a hybrid policy: quick magnitude thresholds for clear cases,
  // otherwise consult kmeans when confident
  const float Z_HIGH = 0.50f;   // strong-signal threshold (normalized)
  const float Z_LOW  = 0.15f;   // clear-noise threshold (normalized)
  const float min_centroid_sep = 0.25f;  // min separation to trust clustering
  const int MIN_KCOUNT = 8;              // require some samples in each cluster
  const int min_signal_streak = 2;       // streaks for hysteresis
  const int min_noise_streak = 2;

  // always update centroids and get back distances for decision-making
  float d_noise = 0.0f, d_signal = 0.0f, centroid_sep = 0.0f;
  int assigned_signal = cw_rx_kmeans_update_2d(p, p->magnitude, max_idx, &d_noise, &d_signal, &centroid_sep);
  bool kmeans_reliable = (p->k_initialized && centroid_sep >= min_centroid_sep &&
                          p->k_count_noise >= MIN_KCOUNT && p->k_count_signal >= MIN_KCOUNT);

  // compute normalized magnitude z = (mag - noise) / (high - noise)
  float denom = (float)(p->high_level - p->noise_floor);
  if (denom <= 0.0f) denom = 1.0f;
  float z = ((float)p->magnitude - (float)p->noise_floor) / denom;

  if (z >= Z_HIGH) {
    // clear signal by magnitude
    p->sig_state = true;
  } else if (z <= Z_LOW) {
    // clear noise by magnitude
    p->sig_state = false;
  } else {
    // if signal not clearly present, consult kmeans if it's reliable
    if (kmeans_reliable) {
      // use assignment + streak hysteresis (kmeans updated streaks internally)
      if (assigned_signal) {
        if (p->k_signal_streak >= min_signal_streak) p->sig_state = true;
        // otherwise keep previous p->sig_state until streak threshold reached
      } else {
        if (p->k_noise_streak >= min_noise_streak) p->sig_state = false;
      }
    } else { 
      // clustering not reliable yet: fallback to legacy SNR heuristic
      if ((p->max_bin_streak == 1) &&
          (p->magnitude >= p->noise_floor + 0.4f * (p->high_level - p->noise_floor)))
        p->sig_state = true;
      else if ((p->max_bin_streak >= 2) &&
               (p->magnitude >= p->noise_floor + 0.15f * (p->high_level - p->noise_floor)))
        p->sig_state = true;
      else
        p->sig_state = false;
    }
  }

  p->ticker++;
}
// gets the bin index that had the strongest magnitude signal
// inputs:  
// returns: int decoder.max_bin_idx
// notes: this function is called from sbitx_gtk.c to find which bin
// to highlight in zerobeat indicator
int cw_get_max_bin_highlight_index(void) {
  // return -1 if no signal present or if no streak
  // change this if we want more 'action' in the zerobeat indicator
  if (!decoder.sig_state) return -1;
  if (decoder.max_bin_streak < 2) return -1;
  return decoder.max_bin_idx;  // 0..4, where 2 is the center bin
}

// fractional Goertzel algorithm to detect the magnitude of a specific frequency bin
// inputs:  struct bin *p, int32_t *data
// returns: int magnitude
// notes: 
static int cw_rx_bin_detect(struct bin *p, int32_t *data) {
  // Q1 and Q2 are the previous two states in the Goertzel recurrence
  float Q2 = 0;
  float Q1 = 0;
  for (int index = 0; index < p->n; index++) {
    float Q0 = p->coeff * Q1 - Q2 + (float)(*data);
    Q2 = Q1;
    Q1 = Q0;
    data++;
  }
  // compute in-phase (cosine) and quadrature (sine) components at the target frequency
  double real = (Q1 * p->cosine - Q2) / p->scalingFactor;
  double imag = (Q1 * p->sine) / p->scalingFactor;
  int magnitude = (int)(sqrt(real * real + imag * imag));
  return magnitude;
}

// 2-D clustering of bin magnitude and distance from center bin,
// i.e. features are log(1+mag), distance-from-center
// Updates centroids (always), returns assignment (1=signal,0=noise)
// and outputs distances and centroid separation so caller can decide
// whether to trust clustering or fall back to magnitude heuristic.
static int cw_rx_kmeans_update_2d(struct cw_decoder *p, int magnitude, int max_idx,
                                  float *out_d_noise, float *out_d_signal,
                                  float *out_centroid_sep) {
  const int center_idx = 2;
  float x0 = logf(1.0f + (float)magnitude);              // magnitude, log-compressed
  float x1 = fabsf((float)max_idx - (float)center_idx);  // distance from center bin

  // startup seeding: use first two samples to initialize centroids
  if (!p->k_initialized) {
    if (p->k_warmup == 0) {
      p->k_centroid_noise[0] = x0;
      p->k_centroid_noise[1] = x1;
      p->k_warmup++;
      /* Output diagnostics if requested */
      if (out_d_noise) *out_d_noise = 0.0f;
      if (out_d_signal) *out_d_signal = 0.0f;
      if (out_centroid_sep) *out_centroid_sep = 0.0f;
      return 0;  // assigned to noise for now
    } else if (p->k_warmup == 1) {
      p->k_centroid_signal[0] = x0;
      p->k_centroid_signal[1] = x1;
      p->k_warmup++;
      // ensure ordering: signal centroid should have larger x0 than noise centroid
      if (p->k_centroid_noise[0] > p->k_centroid_signal[0]) {
        float t0 = p->k_centroid_noise[0], t1 = p->k_centroid_noise[1];
        p->k_centroid_noise[0] = p->k_centroid_signal[0];
        p->k_centroid_noise[1] = p->k_centroid_signal[1];
        p->k_centroid_signal[0] = t0;
        p->k_centroid_signal[1] = t1;
      }
      p->k_initialized = true;
      if (out_d_noise) *out_d_noise = 0.0f;
      if (out_d_signal) *out_d_signal = 0.0f;
      if (out_centroid_sep) *out_centroid_sep = 0.0f;
      return 1;  // assigned to signal for this seed frame
    } else {
      // Unexpected state: seed from legacy tracked levels so centroids are reasonable
      p->k_centroid_noise[0] = logf(1.0f + (float)p->noise_floor);
      p->k_centroid_noise[1] = 2.0f;
      p->k_centroid_signal[0] = logf(1.0f + (float)p->high_level);
      p->k_centroid_signal[1] = 0.0f;
      p->k_initialized = true;
      /* fall through to normal update below */
    }
  }

  // compute Euclidean distances to both centroids in (x0,x1) space
  float dn0 = x0 - p->k_centroid_noise[0];
  float dn1 = x1 - p->k_centroid_noise[1];
  float ds0 = x0 - p->k_centroid_signal[0];
  float ds1 = x1 - p->k_centroid_signal[1];
  float d_noise = sqrtf(dn0 * dn0 + dn1 * dn1);
  float d_signal = sqrtf(ds0 * ds0 + ds1 * ds1);

  // learning rate (alpha) with safe default
  float alpha = p->k_alpha;
  if (alpha <= 0.0f) alpha = 0.04f;

  // nearest-centroid assignment and EMA update of centroids
  int assigned_signal = (d_signal < d_noise) ? 1 : 0;
  if (!assigned_signal) {
    p->k_count_noise++;
    p->k_centroid_noise[0] = (1.0f - alpha) * p->k_centroid_noise[0] + alpha * x0;
    p->k_centroid_noise[1] = (1.0f - alpha) * p->k_centroid_noise[1] + alpha * x1;
    p->k_noise_streak++;
    p->k_signal_streak = 0;
  } else {
    p->k_count_signal++;
    p->k_centroid_signal[0] = (1.0f - alpha) * p->k_centroid_signal[0] + alpha * x0;
    p->k_centroid_signal[1] = (1.0f - alpha) * p->k_centroid_signal[1] + alpha * x1;
    p->k_signal_streak++;
    p->k_noise_streak = 0;
  }

  // keep ordering sensible ... signal centroid typically has the larger magnitude
  if (p->k_centroid_signal[0] < p->k_centroid_noise[0]) {
    float t0 = p->k_centroid_noise[0];
    float t1 = p->k_centroid_noise[1];
    p->k_centroid_noise[0] = p->k_centroid_signal[0] * 0.98f;
    p->k_centroid_noise[1] = p->k_centroid_signal[1] * 1.02f;
    p->k_centroid_signal[0] = t0 * 1.02f;
    p->k_centroid_signal[1] = t1 * 0.98f;
  }

  // recompute centroid separation for diagnostics
  float dx0 = p->k_centroid_signal[0] - p->k_centroid_noise[0];
  float dx1 = p->k_centroid_signal[1] - p->k_centroid_noise[1];
  float centroid_sep = sqrtf(dx0 * dx0 + dx1 * dx1);

  if (out_centroid_sep) *out_centroid_sep = centroid_sep;
  if (out_d_noise) *out_d_noise = d_noise;
  if (out_d_signal) *out_d_signal = d_signal;

  return assigned_signal;
}

// simple 1-D kmeans (small, iterative). Operates on positive float values.
// This function takes marks with lengths and builds two centroids with dot and dash
// assignments 
//  - k: number of clusters (2 for dot/dash)
//  - clusters: on input may contain seeds (length k); on output contains centroids
//  - assignments: output array length n with cluster indices 0..k-1
// inputs:  float *values, int n, int k, float *clusters, int *assignments
// returns: int KM_MAX_ITERS (the number of iterations performed)
// notes: One centroid corresponds to the short marks (dots), the other to the long marks (dashes)
// When converged, the computed centroids (mu) and variances (var) in the log domain are 
// meaningful and can be used reliably by the Viterbi decoder to label each observed mark as dot or dash
static int kmeans_1d(float *values, int n, int k, float *clusters, int *assignments) {
    if (n <= 0 || k <= 0) return 0;
    const int KM_MAX_ITERS = 10;
    // seed clusters if zeros: min & max
    float vmin = values[0], vmax = values[0];
    for (int i = 1; i < n; ++i) {
        if (values[i] < vmin) vmin = values[i];
        if (values[i] > vmax) vmax = values[i];
    }
    if (k >= 1 && clusters[0] == 0.0f) clusters[0] = vmin;
    if (k >= 2 && clusters[1] == 0.0f) clusters[1] = vmax;

    for (int iter = 0; iter < KM_MAX_ITERS; ++iter) {
        for (int i = 0; i < n; ++i) {
            float bestd = fabsf(values[i] - clusters[0]);
            int best = 0;
            for (int c = 1; c < k; ++c) {
                float d = fabsf(values[i] - clusters[c]);
                if (d < bestd) { bestd = d; best = c; }
            }
            assignments[i] = best;
        }
        // recompute centroids
        float sum[3] = {0.0f, 0.0f, 0.0f};
        int count[3] = {0,0,0};
        for (int i = 0; i < n; ++i) {
            int a = assignments[i];
            sum[a] += values[i];
            count[a] += 1;
        }
        int converged = 1;
        for (int c = 0; c < k; ++c) {
            if (count[c] > 0) {
                float newc = sum[c] / (float)count[c];
                if (fabsf(newc - clusters[c]) > 1e-4f) converged = 0;
                clusters[c] = newc;
            }
        }
        if (converged) return iter + 1;
    }
    return KM_MAX_ITERS;
}

// Estimate mark emission parameters (dot/dash) using 1-D kmeans on
// log(mark duration).  
// inputs:   struct cw_decoder *p, float *log_mark_durs, int n,
//           float *dot_mu, float *dot_var, float *dash_mu, float *dash_var
// returns: bool TRUE (if the estimation is reliable)
// notes: computes dot_mu, dot_var, dash_mu, dash_var (all in log-domain)
static bool estimate_mark_emissions(struct cw_decoder *p, float *log_mark_durs, int n,
                                    float *dot_mu, float *dot_var, float *dash_mu, float *dash_var) {
    if (n <= 0) return false;
    if (n == 1) {
        // not enough samples; fall back to decoder->dot_len
        *dot_mu = logf(1.0f + (float)p->dot_len);
        *dash_mu = logf(1.0f + (float)p->dot_len * 3.0f);
        *dot_var = *dash_var = 0.5f;
        return false;
    }

    int assignments[MAX_SYMBOLS];
    float clusters[2] = {0.0f, 0.0f};
    // seed min/max
    float vmin = log_mark_durs[0], vmax = log_mark_durs[0];
    for (int i = 1; i < n; ++i) {
      if (log_mark_durs[i] < vmin) vmin = log_mark_durs[i];
      if (log_mark_durs[i] > vmax) vmax = log_mark_durs[i];
    }
    clusters[0] = vmin; clusters[1] = vmax;

    kmeans_1d(log_mark_durs, n, 2, clusters, assignments);

    // compute cluster stats
    float sum0 = 0.0f, sum1 = 0.0f;
    float ss0 = 0.0f, ss1 = 0.0f;
    int c0 = 0, c1 = 0;
    for (int i = 0; i < n; ++i) {
        if (assignments[i] == 0) {
          sum0 += log_mark_durs[i];
          ss0 += log_mark_durs[i]*log_mark_durs[i]; c0++;
        }
        else {
          sum1 += log_mark_durs[i];
          ss1 += log_mark_durs[i]*log_mark_durs[i];
          c1++;
        }
    }
    if (c0 == 0 || c1 == 0) {
        // degenerate clustering
        return false;
    }
    float mu0 = sum0 / (float)c0;
    float mu1 = sum1 / (float)c1;
    float var0 = (ss0 / (float)c0) - (mu0 * mu0);
    float var1 = (ss1 / (float)c1) - (mu1 * mu1);
    if (var0 <= 1e-4f) var0 = 1e-4f;
    if (var1 <= 1e-4f) var1 = 1e-4f;

    // identify dot = smaller centroid
    if (mu0 <= mu1) {
        *dot_mu = mu0; *dot_var = var0;
        *dash_mu = mu1; *dash_var = var1;
    } else {
        *dot_mu = mu1; *dot_var = var1;
        *dash_mu = mu0; *dash_var = var0;
    }

    // require minimal separation in log-domain to be confident
    const float MIN_SEP_LOG = 0.23f;  // WAS .25
    if ( (*dash_mu - *dot_mu) < MIN_SEP_LOG )
      return false;
    else
      return true;
}

// Viterbi decoder for sequence of mark log-durations using two states,
// 0=DOT, 1=DASH. Uses Gaussian emission models (log-domain).
// inputs: float *log_mark_durs, int n, float dot_mu, float dot_var, 
//         float dash_mu, float dash_var, int *out_assignments
// returns: void
// notes: outputs assignments array of length n with 0/1
static void viterbi_decode_marks(float *log_mark_durs, int n, float dot_mu, float dot_var, float dash_mu, float dash_var, int *out_assignments) {
    if (n <= 0) return;
    // range check
    if (dot_var <= 0.0f) dot_var = 1e-3f;
    if (dash_var <= 0.0f) dash_var = 1e-3f;

    // precompute emission log-likelihoods
    float emit[MAX_SYMBOLS][2];
    const float LOG2PI = logf(2.0f * (float)M_PI);
    for (int t = 0; t < n; ++t) {
        float x = log_mark_durs[t];
        float ldot = -0.5f * ( (x - dot_mu)*(x - dot_mu) / dot_var ) - 0.5f * (LOG2PI + logf(dot_var));
        float ldash = -0.5f * ( (x - dash_mu)*(x - dash_mu) / dash_var ) - 0.5f * (LOG2PI + logf(dash_var));
        emit[t][0] = ldot;
        emit[t][1] = ldash;
    }

    // transition log-probabilities (favor staying same state slightly)
    const float P_STAY = 0.55f;   // >>>>> try tweaking this <<<<<
    const float P_SWITCH = 1.0f - P_STAY;
    const float log_stay = logf(P_STAY);
    const float log_switch = logf(P_SWITCH);

    // dp arrays
    float dp[MAX_SYMBOLS][2];
    int back[MAX_SYMBOLS][2];

    // priors (equal)
    dp[0][0] = logf(0.5f) + emit[0][0];
    dp[0][1] = logf(0.5f) + emit[0][1];
    back[0][0] = back[0][1] = -1;

    for (int t = 1; t < n; ++t) {
        for (int s = 0; s < 2; ++s) {
            // compute best previous
            float bestv = dp[t-1][0] + ( (s==0) ? log_stay : log_switch );
            int bestp = 0;
            float v1 = dp[t-1][1] + ( (s==1) ? log_stay : log_switch );
            if (v1 > bestv) { bestv = v1; bestp = 1; }
            dp[t][s] = bestv + emit[t][s];
            back[t][s] = bestp;
        }
    }

    // termination: pick best final state
    int bests = (dp[n-1][1] > dp[n-1][0]) ? 1 : 0;
    int cur = bests;
    for (int t = n-1; t >= 0; --t) {
        out_assignments[t] = cur;
        cur = back[t][cur];
        if (cur < 0) break;
    }
}

// update signal level tracking
// inputs:  struct cw_decoder *p
// returns: void
// notes:
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
    // update the noise floor with a similar decay mechanism
    p->noise_floor = (p->magnitude + ((NOISE_DECAY - 1) * p->noise_floor)) / NOISE_DECAY;
  }
}

// updates the 'mark' state (p->mark) based on a smoothed version of
// the raw input signal (p->sig_state)
// inputs:  struct cw_decoder *p
// returns: void
// notes:
static void cw_rx_denoise(struct cw_decoder *p) {
  p->prev_mark = p->mark;
  // use sliding window to smooth sig_state over time
  p->history_sig <<= 1;   // Shift register: oldest bit out, make room for new sample
  if (p->sig_state)       // If current input is a 'mark'
    p->history_sig |= 1;  // then set least significant bit
  uint16_t sig = p->history_sig & 0b1111;
  // use Kernighan's algorithm to count number of set bits (1s)
  int count = 0;
  while (sig > 0) {
    sig &= (sig - 1);
    count++;
  }
  // hysteresis enabled to replace majority voting
  if (!p->prev_mark)
    // we are in a space, set count required to transition to mark
    p->mark = (count >= 3);
  else
    // we are in a mark, set count required to stay as mark
    p->mark = (count >= 3);
}

// detect transitions between mark and space and if a dot, dash, character space
// or word space has occurred
// detect Farnsworth spacing and adjust word-gap threshold when found
// returns: bool - true if a character gap was detected (caller should call matcher)
static bool cw_rx_detect_symbol(struct cw_decoder *p) {
  // Tunables
  const float ALPHA = 0.14f;    // EMA smoothing factor
  const float WORD_THR = 1.7f;  // min word gap/char gap ratio for word boundary
  const int WORD_COUNT = 3;     // number of consecutive chars before allowing word boundary
  const int MAX_MARK = 12;      // max mark duration, dots
  const float GAP_ALPHA = 0.20f; // EMA for measured gaps (char/word)

  // Nominal gaps if EMAs not ready
  int dot = p->dot_len;
  int char_gap_nom = 3 * dot;
  int word_gap_nom = 7 * dot;

  // Compute current thresholds (use EMAs if available, otherwise nominal)
  int char_gap = (p->char_gap_ema > 0.0f) ? (int)p->char_gap_ema : char_gap_nom;
  int word_gap = (p->word_gap_ema > 0.0f) ? (int)p->word_gap_ema : word_gap_nom;

  // Enforce ordering
  if (word_gap <= char_gap + dot) 
    word_gap = char_gap + 2 * dot;

  // -- Transition (MARK -> SPACE): End of element
  if (!p->mark && p->prev_mark) {
    cw_rx_add_symbol(p, 'm');
    p->ticker = 0;
    return false;
  }

  // -- Transition (SPACE -> MARK): Start of new mark, gap just finished
  if (p->mark && !p->prev_mark) {
    int gap_ticks = p->ticker;
    cw_rx_add_symbol(p, ' ');
    p->last_char_was_space = 0;

    // --- Update gap EMAs based on what we just measured ---
    if (gap_ticks >= word_gap) {
        // treat as word gap
        if (p->word_gap_ema <= 0.0f) p->word_gap_ema = (float)gap_ticks;
        else p->word_gap_ema = (1.0f - GAP_ALPHA) * p->word_gap_ema + GAP_ALPHA * (float)gap_ticks;
    } else {
        // treat as inter-character gap
        if (p->char_gap_ema <= 0.0f) p->char_gap_ema = (float)gap_ticks;
        else p->char_gap_ema = (1.0f - GAP_ALPHA) * p->char_gap_ema + GAP_ALPHA * (float)gap_ticks;
    }
    
    p->ticker = 0;
    return false;
  }

  // -- Remain in SPACE: Check for symbol/char/word gaps
  if (!p->mark && !p->prev_mark) {
    int t = p->ticker;

    // If we’re in the middle of building a character, handle element + char gaps
    if (p->next_symbol > 0) {
      // Add a single inter-element space after a mark
      if (t >= dot && p->symbol_str[p->next_symbol - 1].is_mark) {
        cw_rx_add_symbol(p, ' ');
        // do NOT reset ticker; we still want to measure the full gap
      }

      // Close the character after the char gap
      if (t >= char_gap) {
        cw_rx_match_letter(p);
        p->last_char_was_space = 0;
        // ticker is left running to continue measuring for a possible word gap
      }
    }

    // Word gap check is independent of whether we’re currently building a char
    // TEMP: ignore EMAs, use nominal values only
    int dot = p->dot_len;
    int char_gap = 3 * dot;
    int word_gap = 7 * dot;
	  if (t >= word_gap) {
      // Suppress consecutive spaces across both RX and TX decoders
      if (cw_decode_enabled && !decoder.last_char_was_space && !tx_decoder.last_char_was_space) {
        write_console(STYLE_CW_RX, " ");
      }
      decoder.last_char_was_space = 1;
      tx_decoder.last_char_was_space = 1;
      // Do NOT reset ticker; let the next space->mark transition measure the full gap
    }
  }
  // -- Remain in MARK: Avoid runaway marks (long pressed carrier)
  else if (p->mark && p->prev_mark) {
    if (p->ticker > MAX_MARK * dot) p->ticker = 3 * dot;
  }
  return false;
}

// add a mark or space to the symbol buffer, store its duration (ticks),
// and update the symbol's average magnitude
// inputs:  struct cw_decoder *p, char symbol
// returns: void
// notes:
static void cw_rx_add_symbol(struct cw_decoder *p, char symbol) {
  // if it's full clear it
  if (p->next_symbol == MAX_SYMBOLS) p->next_symbol = 0;
  // only ' ' (space) is treated as a space; all other symbols are marks
  p->symbol_str[p->next_symbol].is_mark = (symbol != ' ');  // 0 for space, 1 for  mark
  // store the duration of the symbol (number of ticks since last transition)
  p->symbol_str[p->next_symbol].ticks = p->ticker;
  // update the average magnitude for this symbol using a weighted average
  p->symbol_str[p->next_symbol].magnitude =
      ((p->symbol_str[p->next_symbol].magnitude * 10) + p->magnitude) / 11;
  // move to the next position in the symbol buffer
  p->next_symbol++;
}

// take string of marks and spaces with their durations in "ticks" and
// translate them into a Morse code character
// inputs:  struct cw_decoder *decoder
// returns: void
// notes:
// gate output by Viterbi confidence
static void cw_rx_match_letter(struct cw_decoder *decoder) {
  // if no symbols have been received, there's nothing to decode
  if (decoder->next_symbol == 0) return;

  // build arrays of mark durations and magnitudes from collected symbols
  float mark_durs[MAX_SYMBOLS];
  float mark_mags[MAX_SYMBOLS];
  int n_marks = 0;
  for (int i = 0; i < decoder->next_symbol; ++i) {
    if (decoder->symbol_str[i].is_mark) {
      mark_durs[n_marks] = (float)decoder->symbol_str[i].ticks;
      mark_mags[n_marks] = (float)decoder->symbol_str[i].magnitude;
      n_marks++;
    }
  }

  // If no marks, clear buffer and return
  if (n_marks == 0) {
    decoder->next_symbol = 0;
    return;
  }

  // convert to log-domain for clustering / emission estimation
  float log_mark_durs[MAX_SYMBOLS];
  for (int i = 0; i < n_marks; ++i) log_mark_durs[i] = logf(1.0f + mark_durs[i]);

  // estimate emissions via kmeans (dot/dash mu and var in log-domain)
  float dot_mu, dot_var, dash_mu, dash_var;
  bool est_ok = estimate_mark_emissions(decoder, log_mark_durs, n_marks, &dot_mu, &dot_var,
                                        &dash_mu, &dash_var);
  if (est_ok) {
    // store estimates in decoder for diagnostics and future use
    decoder->mark_mu_dot = dot_mu;
    decoder->mark_var_dot = dot_var;
    decoder->mark_mu_dash = dash_mu;
    decoder->mark_var_dash = dash_var;
    decoder->mark_emission_ready = true;
  } else {
    // fallback: use previous estimates if available, otherwise derive from dot_len
    if (decoder->mark_emission_ready) {
      dot_mu = decoder->mark_mu_dot;
      dot_var = decoder->mark_var_dot;
      dash_mu = decoder->mark_mu_dash;
      dash_var = decoder->mark_var_dash;
      est_ok = true;  // use cached estimates
    } else {
      dot_mu = logf(1.0f + (float)decoder->dot_len);
      dash_mu = logf(1.0f + (float)decoder->dot_len * 3.0f);
      dot_var = dash_var = 1.0f;
    }
  }

  // if we have good emission models, run Viterbi to decode sequence of marks
  int assignments[MAX_SYMBOLS];
  int used_viterbi = 0;
  if (est_ok && n_marks >= 1) {
    viterbi_decode_marks(log_mark_durs, n_marks, dot_mu, dot_var, dash_mu, dash_var, assignments);
    used_viterbi = 1;   // this supports DEBUG code
  } else {
    // fallback greedy classification by threshold: dash if >= 2.5*dot_len (approx)
    for (int i = 0; i < n_marks; ++i) {
      float ticks = mark_durs[i];
      if (ticks >= (decoder->dot_len * 7) / 3)
        assignments[i] = 1;
      else
        assignments[i] = 0;
    }
  }
    
  // build morse string from assignments in original symbol order
  char morse_code_string[MAX_SYMBOLS + 1];
  int mpos = 0;
  morse_code_string[0] = '\0';
  int mark_idx = 0;
  for (int i = 0; i < decoder->next_symbol && mpos < MAX_SYMBOLS; ++i) {
    if (decoder->symbol_str[i].is_mark) {
      morse_code_string[mpos++] = (assignments[mark_idx] == 0) ? '.' : '-';
      mark_idx++;
    } else {
      // spaces between elements are handled by cw_rx_detect_symbol and char/word gaps
    }
  }
  morse_code_string[mpos] = '\0';

  // adapt decoder->dot_len from estimated dot centroid if reliable
  if (est_ok) {
    float dot_ticks_est = expf(dot_mu) - 1.0f;
    float dash_ticks_est = expf(dash_mu) - 1.0f;
    if (dot_ticks_est > 0.0f && dash_ticks_est / dot_ticks_est > 1.5f && dash_ticks_est / dot_ticks_est < 4.5f) {
      // avoid large single-step jumps caused by noise
      const float DOT_ADAPT_ALPHA = 0.20f;   // existing alpha (can be lowered if desired)
      const float MIN_ADAPT_RATIO = 0.70f;   // do not shrink dot_len by more than 30% in one step
      const float MAX_ADAPT_RATIO = 1.50f;   // do not grow dot_len by more than 50% in one step

      float proposed = (1.0f - DOT_ADAPT_ALPHA) * (float)decoder->dot_len + DOT_ADAPT_ALPHA * dot_ticks_est;
      float min_allowed = (float)decoder->dot_len * MIN_ADAPT_RATIO;
      float max_allowed = (float)decoder->dot_len * MAX_ADAPT_RATIO;
      if (proposed < min_allowed) proposed = min_allowed;
      if (proposed > max_allowed) proposed = max_allowed;

      int new_dot_len = (int)(proposed + 0.5f);
      if (new_dot_len < 1) new_dot_len = 1;
      decoder->dot_len = new_dot_len;
    }
  }

  // in noisy conditions slowly recover dot_len back towards UI WPM
  // this prevents the decoder from remaining stuck after transient noise
  if (!decoder->mark_emission_ready) {
    int expected_dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * decoder->wpm);
    if (expected_dot_len < 1) expected_dot_len = 1;
    const float DOT_RECOVER_ALPHA = 0.25f;  // EMA step 25% toward UI WPM per character
    decoder->dot_len = (int)((1.0f - DOT_RECOVER_ALPHA) * decoder->dot_len + DOT_RECOVER_ALPHA * expected_dot_len + 0.5f);
    if (decoder->dot_len < 1) decoder->dot_len = 1;
  }
  
  // reset buffer before mapping/printing so next symbols start fresh
  decoder->next_symbol = 0;

  // If nothing decoded, nothing to do
  if (morse_code_string[0] == '\0') {
    return;
  }

  // compute average mark magnitude and normalized z
  float avg_mag = 0.0f;
  for (int i = 0; i < n_marks; ++i) avg_mag += mark_mags[i];
  avg_mag /= (float)n_marks;
  float denom = (float)(decoder->high_level - decoder->noise_floor);
  if (denom <= 0.0f) denom = 1.0f;
  float avg_z = (avg_mag - (float)decoder->noise_floor) / denom;

  // centroid separation (diagnostic)
  float dx0 = decoder->k_centroid_signal[0] - decoder->k_centroid_noise[0];
  float dx1 = decoder->k_centroid_signal[1] - decoder->k_centroid_noise[1];
  float centroid_sep = sqrtf(dx0 * dx0 + dx1 * dx1);

  // emission margin (how much each mark prefers one label over the other)
  // avg_margin = mean(|logP_dot - logP_dash|) across marks
  float sum_margin = 0.0f;
  const float LOG2PI = logf(2.0f * (float)M_PI);
  // guard small variances
  if (dot_var <= 0.0f) dot_var = 1e-3f;
  if (dash_var <= 0.0f) dash_var = 1e-3f;
  for (int t = 0; t < n_marks; ++t) {
    float x = log_mark_durs[t];
    float ldot = -0.5f * ((x - dot_mu) * (x - dot_mu) / dot_var) - 0.5f * (LOG2PI + logf(dot_var));
    float ldash =
        -0.5f * ((x - dash_mu) * (x - dash_mu) / dash_var) - 0.5f * (LOG2PI + logf(dash_var));
    sum_margin += fabsf(ldot - ldash);
  }
  float avg_margin = sum_margin / (float)n_marks;

  // sanity checks: dot/dash ratio
  float dot_ticks_est = expf(dot_mu) - 1.0f;
  float dash_ticks_est = expf(dash_mu) - 1.0f;
  float dd_ratio = (dot_ticks_est > 0.0f) ? (dash_ticks_est / dot_ticks_est) : 0.0f;
  int ratio_ok = (dd_ratio > 1.3f && dd_ratio < 5.0f) ? 1 : 0;  // I have tried to relax timing

  // cluster counts
  const int MIN_KCOUNT = 12;
  int kcount_ok =
      (decoder->k_count_signal >= MIN_KCOUNT && decoder->k_count_noise >= MIN_KCOUNT) ? 1 : 0;

  // build a blended confidence score (0..1)
  // weights chosen to favor emission margin and magnitude while still using kmeans info
  const float W_Z = 0.35f;
  const float W_SEP = 0.20f;
  const float W_MARGIN = 0.35f;
  const float W_KCNT = 0.05f;
  const float W_RATIO = 0.05f;

  // normalize components into 0..1 ranges with conservative normalization constants
  float comp_z = (avg_z - 0.05f) / (0.6f - 0.05f);  // expected useful range [0.05..0.6]
  if (comp_z < 0.0f) comp_z = 0.0f;
  if (comp_z > 1.0f) comp_z = 1.0f;
  float comp_sep = centroid_sep / 0.6f;  // treat 0.6 as a "large" sep
  if (comp_sep < 0.0f) comp_sep = 0.0f;
  if (comp_sep > 1.0f) comp_sep = 1.0f;
  float comp_margin = avg_margin / 1.2f;  // margin ~1.2 is quite confident
  if (comp_margin < 0.0f) comp_margin = 0.0f;
  if (comp_margin > 1.0f) comp_margin = 1.0f;
  float comp_kcnt = (kcount_ok) ? 1.0f : 0.0f;
  float comp_ratio = (ratio_ok) ? 1.0f : 0.0f;

  float confidence = W_Z * comp_z + W_SEP * comp_sep + W_MARGIN * comp_margin +
                     W_KCNT * comp_kcnt + W_RATIO * comp_ratio;

  const float CONF_THRESHOLD = 0.55f;   // base confidence gate

  // Base confidence gate
  if (confidence < CONF_THRESHOLD) {
    return;
  }

  // Emit decoded character if in table
   for (int i = 0; i < (int)(sizeof(morse_rx_table) / sizeof(struct morse_rx)); i++) {
    if (!strcmp(morse_code_string, morse_rx_table[i].code)) {
      if (cw_decode_enabled) {
        write_console(STYLE_CW_RX, morse_rx_table[i].c);
      }
      decoder->last_char_was_space = 0;
      tx_decoder.last_char_was_space = 0;
      return;
    }
  }
  // Fallback: show raw Morse
  // update - users not crazy about seeing the raw undecodable string so we won't show it
  //write_console(decoder->console_font, morse_code_string);
  decoder->last_char_was_space = 0;
  tx_decoder.last_char_was_space = 0;
  return;
}

// initialize a struct with values for use with Goertzel algorithm
// inputs:  struct bin *p, float freq, int n, float sampling_freq
// returns: void
// notes: filter will be centered on the specifed frequency
static void cw_rx_bin_init(struct bin *p, float freq, int n, float sampling_freq) {
  if (n <= 0) n = 1; // safeguard: n must be positive
  float omega = (2.0f * (float)M_PI * freq) / sampling_freq;  // exact fractional
  p->sine = sinf(omega);
  p->cosine = cosf(omega);
  p->coeff = 2.0f * p->cosine;
  p->n = n;
  p->freq = (int)freq;
  p->scalingFactor = ((double)n) / 2.0;
  if (p->scalingFactor == 0.0) p->scalingFactor = 1.0; // extra safety
}

// initialize struct values for cw tx and decoder 
// inputs:  none
// returns: void
void cw_init(void) {
  // RX decoder: cfg 
  decoder.n_bins = N_BINS;
  decoder.wpm = 20;
  decoder.console_font = STYLE_CW_RX;

  // RX decoder: bins 
  int cw_rx_pitch = field_int("PITCH");
  cw_rx_bin_init(&decoder.signal_minus2, cw_rx_pitch - 80.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_minus1, cw_rx_pitch - 35.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_center, cw_rx_pitch + 0.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_plus1,  cw_rx_pitch + 35.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_plus2,  cw_rx_pitch + 80.0f, N_BINS, SAMPLING_FREQ);

  // RX decoder: detect
  decoder.mark = false;
  decoder.prev_mark = false;
  decoder.sig_state = false;
  decoder.history_sig = 0;
  decoder.ticker = 0;

  // RX decoder: levels
  decoder.magnitude = 0;
  decoder.high_level = 0;
  decoder.noise_floor = 0;
  decoder.max_bin_idx = -1;
  decoder.max_bin_streak = 0;

  // RX decoder: timing
  decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * INIT_WPM);
  decoder.next_symbol = 0;
  decoder.last_char_was_space = 0;
  decoder.space_ema = 0.0f;
  decoder.char_gap_ema = 0.0f;
  decoder.word_gap_ema = 0.0f;

  // RX decoder: km (2-D clustering)
  decoder.k_alpha = 0.04f;
  decoder.k_warmup = 0;
  decoder.k_count_noise = 0;
  decoder.k_count_signal = 0;
  decoder.k_centroid_noise[0] = 0.0f;
  decoder.k_centroid_noise[1] = 0.0f;
  decoder.k_centroid_signal[0] = 0.0f;
  decoder.k_centroid_signal[1] = 0.0f;
  decoder.k_signal_streak = 0;
  decoder.k_noise_streak = 0;
  decoder.k_initialized = false;

  // RX decoder: em (mark emission params)
  decoder.mark_mu_dot  = 0.0f;
  decoder.mark_var_dot = 1.0f;
  decoder.mark_mu_dash = 0.0f;
  decoder.mark_var_dash= 1.0f;
  decoder.mark_emission_ready = false;

  // RX decoder: sym buffer
  for (int i = 0; i < MAX_SYMBOLS; ++i) {
    decoder.symbol_str[i].is_mark = 0;
    decoder.symbol_str[i].magnitude = 0;
    decoder.symbol_str[i].ticks = 0;
  }

  // TX decoder mirrors RX but uses TX pitch/font
  tx_decoder.n_bins = N_BINS;
  tx_decoder.wpm = 20;
  tx_decoder.console_font = 11;  // TX decoder uses amber/yellow font

  int cw_tx_pitch = get_pitch();
  cw_rx_bin_init(&tx_decoder.signal_minus2, cw_tx_pitch - 80.0f,  N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&tx_decoder.signal_minus1, cw_tx_pitch - 35.0f,  N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&tx_decoder.signal_center, cw_tx_pitch + 0.0f,   N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&tx_decoder.signal_plus1,  cw_tx_pitch + 35.0f,  N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&tx_decoder.signal_plus2,  cw_tx_pitch + 80.0f,  N_BINS, SAMPLING_FREQ);

  tx_decoder.mark = false;
  tx_decoder.prev_mark = false;
  tx_decoder.sig_state = false;
  tx_decoder.history_sig = 0;
  tx_decoder.ticker = 0;

  tx_decoder.magnitude = 0;
  tx_decoder.high_level = 0;
  tx_decoder.noise_floor = 0;
  tx_decoder.max_bin_idx = -1;
  tx_decoder.max_bin_streak = 0;

  tx_decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * INIT_WPM);
  tx_decoder.next_symbol = 0;
  tx_decoder.last_char_was_space = 0;
  tx_decoder.space_ema = 0.0f;
  tx_decoder.char_gap_ema = 0.0f;
  tx_decoder.word_gap_ema = 0.0f;

  tx_decoder.k_alpha = 0.04f;
  tx_decoder.k_warmup = 0;
  tx_decoder.k_count_noise = 0;
  tx_decoder.k_count_signal = 0;
  tx_decoder.k_centroid_noise[0] = 0.0f;
  tx_decoder.k_centroid_noise[1] = 0.0f;
  tx_decoder.k_centroid_signal[0] = 0.0f;
  tx_decoder.k_centroid_signal[1] = 0.0f;
  tx_decoder.k_signal_streak = 0;
  tx_decoder.k_noise_streak = 0;
  tx_decoder.k_initialized = false;

  tx_decoder.mark_mu_dot  = 0.0f;
  tx_decoder.mark_var_dot = 1.0f;
  tx_decoder.mark_mu_dash = 0.0f;
  tx_decoder.mark_var_dash= 1.0f;
  tx_decoder.mark_emission_ready = false;

  for (int i = 0; i < MAX_SYMBOLS; ++i) {
    tx_decoder.symbol_str[i].is_mark = 0;
    tx_decoder.symbol_str[i].magnitude = 0;
    tx_decoder.symbol_str[i].ticks = 0;
  }

  // CW TX side (keyer, envelope, LUT)
  cw_init_morse_lut();
  vfo_start(&cw_tone, 700, 0);
  vfo_start(&cw_env, 200, 49044);  // not used with data-driven waveform
  cw_period = 9600;                // at 96ksps, 0.1 sec = 1 dot at 12 WPM
  keydown_count = 0;
  keyup_count = 0;
  cw_envelope = 0;
}

// called from sbitx_gtk.c to display cw stats under zerobeat indicator
// return buffer with one line of cw stats with
// format: "<WPM> 1:<ratio>"  with dash/dot ratio is shown to one decimal point
// inputs:  
// returns: buf, or NULL if it fails
// notes: Uses decoder.mark_mu_* if mark_emission_ready is true, otherwise falls back
// to decoder.dot_len heuristic (dot_len and 3*dot_len for dash)
char *cw_get_stats(char *buf, size_t len)
{
  if (!buf || len == 0) return NULL;

  float dot_ticks = (float)decoder.dot_len;
  float dash_ticks = (float)decoder.dot_len * 3.0f;

  if (decoder.mark_emission_ready) {
    // mark_mu_* are in log(1 + ticks) domain so convert back to linear
    dot_ticks  = expf(decoder.mark_mu_dot)  - 1.0f;
    dash_ticks = expf(decoder.mark_mu_dash) - 1.0f;
    if (dot_ticks < 1.0f) dot_ticks = 1.0f;
    if (dash_ticks < dot_ticks) dash_ticks = dot_ticks * 3.0f;
  }

  float inst_ratio = dash_ticks / dot_ticks;

  // instantaneous WPM from current dot estimate
  float inst_wpm_f = 0.0f;
  if (dot_ticks > 0.0f) {
    inst_wpm_f = (6.0f * (float)SAMPLING_FREQ) /
                 (5.0f * (float)decoder.n_bins * dot_ticks);
  }
  if (inst_wpm_f < 1.0f) inst_wpm_f = 1.0f;

  // build smoothed values of wpm and dot-dash ratio
  static int   have_ema = 0;
  static float wpm_ema = 0.0f;
  static float ratio_ema = 0.0f;
  // Asymmetric adaptation: faster toward lower speeds, slower toward higher speeds (to resist noise spikes)
  const float WPM_ALPHA_UP   = 0.18f;  // when instant WPM is higher than the current EMA
  const float WPM_ALPHA_DOWN = 0.65f;  // when instant WPM is lower than the current EMA (adapt quickly downward)
  const float WPM_MAX_UP_STEP = 1.5f;  // cap how much the displayed WPM can increase per update
  const float RATIO_ALPHA = 0.45f;  

  if (!have_ema) {
    wpm_ema   = inst_wpm_f;
    ratio_ema = inst_ratio;
    have_ema  = 1;
  } else {
    if (inst_wpm_f < wpm_ema) {
      // Move quickly toward lower detected speeds
      wpm_ema = (1.0f - WPM_ALPHA_DOWN) * wpm_ema + WPM_ALPHA_DOWN * inst_wpm_f;
    } else {
      // Move cautiously toward higher (possibly noisy) speeds
      float candidate = (1.0f - WPM_ALPHA_UP) * wpm_ema + WPM_ALPHA_UP * inst_wpm_f;
      if (candidate - wpm_ema > WPM_MAX_UP_STEP) {
        candidate = wpm_ema + WPM_MAX_UP_STEP;
      }
      wpm_ema = candidate;
    }
    ratio_ema = (1.0f - RATIO_ALPHA) * ratio_ema + RATIO_ALPHA * inst_ratio;
  }

  int est_wpm = (int)roundf(wpm_ema);
  if (est_wpm < 1) est_wpm = 1;
  float disp_ratio = ratio_ema;

  // format the string to display under zerobeat
  snprintf(buf, len, "%dwpm 1:%.1f", est_wpm, disp_ratio);
  return buf;
}

// manage TX start/stop and keep keyer and decoder parameters in sync with
// UI fields
// inputs:  int bytes_available, int tx_is_on
// returns: void
// notes: updates global state (e.g. cw_bytes_available, cw_key_state, cw_period,
// decoder.wpm and decoder.dot_len, cw_tx_until, cw_mode) and may call tx_on(TX_SOFT)
// or tx_off() to start/stop transmission
void cw_poll(int bytes_available, int tx_is_on) {
  cw_bytes_available = bytes_available;
  cw_key_state = key_poll();
  int wpm = field_int("WPM");
  cw_period = (12 * 9600) / wpm;

  // retune the rx pitch if needed
  int cw_rx_pitch = field_int("PITCH");
  if (cw_rx_pitch != decoder.signal_center.freq) {
    cw_rx_bin_init(&decoder.signal_minus2, cw_rx_pitch - 80.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_minus1, cw_rx_pitch - 35.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_center, cw_rx_pitch + 0.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_plus1, cw_rx_pitch + 35.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_plus2, cw_rx_pitch + 80.0f, N_BINS, SAMPLING_FREQ);
  }
  // check if the wpm has changed
  if (wpm != decoder.wpm) {
    decoder.wpm = wpm;
    decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * wpm);
  }
  
  // retune the tx decoder pitch if needed
  int cw_tx_pitch = get_pitch();
	if (cw_tx_pitch != tx_decoder.signal_center.freq) {
    cw_rx_bin_init(&tx_decoder.signal_minus2, cw_tx_pitch - 80.0f,  N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&tx_decoder.signal_minus1, cw_tx_pitch - 35.0f,  N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&tx_decoder.signal_center, cw_tx_pitch + 0.0f,   N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&tx_decoder.signal_plus1,  cw_tx_pitch + 35.0f,  N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&tx_decoder.signal_plus2,  cw_tx_pitch + 80.0f,  N_BINS, SAMPLING_FREQ);
  }
	// check if the wpm has changed for tx decoder
  if (wpm != tx_decoder.wpm){
		tx_decoder.wpm = wpm;
		tx_decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * wpm);
	}
  
  // TX ON if bytes are available (from macro/keyboard) or key is pressed
  // or we are in the middle of symbol (dah/dit) transmission
  millis_now = millis();
  if (!tx_is_on && ((cw_bytes_available > 0 && text_ready == 1) ||
        cw_key_state || (symbol_next && *symbol_next))) {
    tx_on(TX_SOFT);
    cw_tx_until = get_cw_delay() + millis_now;
    cw_mode = get_cw_input_method();
  } else if (tx_is_on && cw_tx_until < millis_now) {
    // If we were in a TX session, write newline to end it
		if (tx_session_active) {
			write_console(STYLE_CW_TX, "\n");
			tx_session_active = false;
		}
    tx_off();
  }
}

// called by modem_abort() in modems.c
// inputs: none
// returns: void
void cw_abort() {
  // If we were in a TX session, write newline to end it
	if (tx_session_active) {
		write_console(STYLE_CW_TX, "\n");
		tx_session_active = false;
	}

	// Reset TX state
	keydown_count = 0;
	keyup_count = 0;
	cw_tx_until = 0;
	symbol_next = NULL;
}
