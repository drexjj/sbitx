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
#define HIGH_DECAY 100   // larger means change sig level slower
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

// result struct passed from section 7 (build morse) to section 8 (classify/output)
struct cw_morse_result {
  char morse_code[MAX_SYMBOLS + 1];  // dot-dash string, e.g. ".-"
  float confidence;                   // 0..1 blended confidence score
  float dot_mu;                       // estimated dot centroid (ticks)
  float dash_mu;                      // estimated dash centroid (ticks)
  bool  est_ok;                       // true if centroid estimation succeeded
  int   n_marks;                      // how many marks were in the symbol buffer
  bool  valid;                        // true if a non-empty string was produced
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

  // per-bin median filter history (3-tick window)
  int bin_hist[5][3];   // [bin_index][tick], circular
  int bin_hist_pos;     // shared write position (all bins advance together)
  int bin_hist_count;   // how many ticks have been stored (0..3)

  // detect (instantaneous detection / denoise history)
  bool mark;
  bool prev_mark;
  bool sig_state;
  int ticker;

  // levels (SNR/high/low tracking + winning bin)
  int magnitude;
  float smoothed_magnitude;  // EMA-filtered magnitude (adaptive alpha)
  int high_level;
  int noise_floor;
  int max_bin_idx;
  int max_bin_streak;

  // timing (dot length, gaps, symbol assembly state)
  int dot_len;              // use dot as timing unit
  int next_symbol;
  int last_char_was_space;
  int decoded_char_seen;  // set after a successful non-space decode
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

  // mark emission parameters (linear-domain tick counts)
  float mark_mu_dot;
  float mark_mu_dash;
  bool  mark_emission_ready;

  // cw_rx_denoise will track signal confidence
  float sig_confidence;

  // early dot_len adaptation (breaks stuck-low-WPM cycle)
  float  recent_marks[8];       // circular buffer of recent mark durations (ticks)
  int    recent_marks_pos;
  int    recent_marks_count;
  float  shortest_recent_mark;  // running minimum of recent marks

  // symbol buffer
  struct symbol symbol_str[MAX_SYMBOLS];
};

// minimal struct for TX self-decode (only needs Goertzel bin + WPM)
struct cw_tx_decoder {
  int wpm;
  struct bin signal_center;
};

static struct cw_tx_decoder tx_decoder;
static struct cw_decoder decoder;

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

// CW TX/keyer prototypes
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

// CW RX decoder pipeline
void            cw_rx(int32_t *samples, int count);                     // Section 1: entry point
static void     cw_rx_condition(int32_t *samples, int count,            // Section 2: signal conditioning
                                int32_t *out, int out_len);
void            apply_fir_filter(int32_t *input, int32_t *output,       //   (FIR helper)
                                 const float *coeffs, int input_count, int order);
static void     cw_rx_bin(struct cw_decoder *p, int32_t *samples);      // Section 3: frequency analysis
static int      cw_rx_bin_detect(struct bin *p, int32_t *data);         //   (Goertzel helper)
static int      cw_rx_kmeans_update_2d(struct cw_decoder *p,            //   (k-means helper)
                                       int magnitude, int max_idx,
                                       float *out_d_noise, float *out_d_signal,
                                       float *out_centroid_sep);
int             cw_get_max_bin_highlight_index(void);                   //   (UI query)
static void     cw_rx_update_levels(struct cw_decoder *p);              // Section 4: level tracking
static void     cw_rx_denoise(struct cw_decoder *p);                    // Section 5: denoising
static bool     cw_rx_detect_symbol(struct cw_decoder *p);              // Section 6: symbol detection
static void     cw_rx_add_symbol(struct cw_decoder *p, char symbol);    //   (symbol buffer helper)
static void     cw_rx_early_dot_adapt(struct cw_decoder *p,             //   (early adapt helper)
                                      int mark_ticks);
static bool     estimate_dot_dash_centroids(struct cw_decoder *p,       // Section 7: build morse
                                            float *mark_durs, int n,
                                            float *out_dot_mu, float *out_dash_mu);
static void     cw_rx_build_morse(struct cw_decoder *p,
                                  struct cw_morse_result *result);
static void     cw_rx_classify_and_output(struct cw_decoder *p,         // Section 8: classify & output
                                          struct cw_morse_result *result);
static void     cw_write_console(int style, const char *text);
static void     cw_rx_bin_init(struct bin *p, float freq, int n,        //   (Goertzel init helper)
                               float sampling_freq);

// CW TX decoder
static void     cw_tx_decode_samples(void);

// CW init/poll/stats API
void            cw_init(void);
char           *cw_get_stats(char *buf, size_t len);
void            cw_poll(int bytes_available, int tx_is_on);
void            cw_abort(void);

int is_in_tx(void);  // from modems.c

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
//  KB2ML CW Decoder
//////////////////////////////////////////////////////////////////////////
// Stage 1: Main entry point (cw_rx)
// Stage 2: Signal conditioning  (anti-alias filter + decimation)
// Stage 3: Frequency analysis   (5-bin Goertzel + smoothing + clustering)
// Stage 4: Level tracking       (high_level / noise_floor update)
// Stage 5: Denoising            (EMA + hysteresis → mark/space decision)
// Stage 6: Symbol detection     (transitions → symbol buffer, gap EMAs)
// Stage 7: Build Morse char     (mark durations → dot/dash classification)
// Stage 8: Character output     (table lookup → console, dot_len adaptation)

// Main entry point - take a block of 96 kHz audio samples and run the full
// CW decode pipeline.  cw_rx() itself does no real work — it dispatches
// to the pipeline stages below.
// inputs:  int32_t *samples (96 kHz), int count
// returns: void
void cw_rx(int32_t *samples, int count) {
  int32_t conditioned[N_BINS];
  // Stage 2: Signal conditioning (AGC hook + anti-alias + decimate)
  cw_rx_condition(samples, count, conditioned, decoder.n_bins);
  // Stage 3: Frequency analysis (Goertzel bins + clustering → sig_state)
  cw_rx_bin(&decoder, conditioned);
  // Stage 4: Signal level tracking (high_level / noise_floor)
  cw_rx_update_levels(&decoder);
  // Stage 5: Denoising / mark decision (EMA + hysteresis → mark)
  cw_rx_denoise(&decoder);
  // Stage 6: Symbol detection (transitions → symbol buffer)
  bool char_ready = cw_rx_detect_symbol(&decoder);
  // Stage 7 & 8 (only if char is ready for output)
  if (char_ready) {
    // Stage 7: Build morse string
    struct cw_morse_result result;
    // Build Morse character from symbol buffer
    cw_rx_build_morse(&decoder, &result);
    // Reset symbol buffer before output so next symbols start fresh
    decoder.next_symbol = 0;
    // Stage 8: Classify and output
    cw_rx_classify_and_output(&decoder, &result);
  }
}

//////////////////////////////////////////////////////////////////////////
// Section 2: Signal conditioning (anti-alias filter + decimate)
//////////////////////////////////////////////////////////////////////////

// Condition raw 96 kHz samples down to 12 kHz for Goertzel analysis.
//
//   FIR low-pass filter (5 kHz cutoff at 96 kHz)
//   Decimate by 8 → 12 kHz
//   Drop 8 LSBs (A/D is 24-bit, not 32)
//
// inputs:  raw 96 kHz samples, sample count,
//          output buffer (must hold at least out_len samples)
// returns: void
static void cw_rx_condition(int32_t *samples, int count,
                            int32_t *out, int out_len) {
  const int decimation_factor = 8;  // 96 kHz -> 12 kHz
  int32_t filtered[count];  // local; don't modify caller's buffer

  // anti-alias low-pass filter
  apply_fir_filter(samples, filtered, fir_coeffs, count, 64);

  // downsample and strip 8 LSBs (A/D was 24 bits, not 32)
  for (int i = 0; i < out_len; i++) {
    out[i] = filtered[i * decimation_factor] >> 8;
  }
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

//////////////////////////////////////////////////////////////////////////
// Section 3: Frequency analysis (5-bin Goertzel + median + EMA + k-means)
//////////////////////////////////////////////////////////////////////////

// Run Goertzel on 5 frequency bins around the CW pitch, apply per-bin
// 3-tick median filtering, pick the strongest, apply adaptive EMA
// smoothing, and use 2-D k-means clustering to decide whether a signal
// is present (sets p->sig_state).
//
// Bins are at -150, -75, 0, +75, +150 Hz relative to center pitch.
// inputs:  struct cw_decoder *p, int32_t *samples (12 kHz decimated)
// returns: void (updates p->magnitude, p->sig_state, p->max_bin_idx, etc.)
static void cw_rx_bin(struct cw_decoder *p, int32_t *samples) {
  // Raw Goertzel magnitudes
  int raw[5];
  raw[0] = cw_rx_bin_detect(&p->signal_minus2, samples);
  raw[1] = cw_rx_bin_detect(&p->signal_minus1, samples);
  raw[2] = cw_rx_bin_detect(&p->signal_center, samples);
  raw[3] = cw_rx_bin_detect(&p->signal_plus1,  samples);
  raw[4] = cw_rx_bin_detect(&p->signal_plus2,  samples);

  // Per-bin 3-tick median filter
  // Store raw magnitudes into circular history buffer
  int pos = p->bin_hist_pos;
  for (int b = 0; b < 5; b++) {
    p->bin_hist[b][pos] = raw[b];
  }
  p->bin_hist_pos = (pos + 1) % 3;
  if (p->bin_hist_count < 3) p->bin_hist_count++;

  // Compute filtered magnitudes: median of last 3 ticks per bin
  // (pass through raw values until we have 3 ticks of history)
  int filtered[5];
  if (p->bin_hist_count < 3) {
    for (int b = 0; b < 5; b++) filtered[b] = raw[b];
  } else {
    for (int b = 0; b < 5; b++) {
      int a = p->bin_hist[b][0];
      int b1 = p->bin_hist[b][1];
      int c = p->bin_hist[b][2];
      // sort into order and take the middle value
      if (a > b1) { int t = a; a = b1; b1 = t; }  // now a <= b1
      if (b1 > c) { b1 = c; }                     // now b1 = min(old_b1, c)
      if (a > b1) { b1 = a; }                     // now b1 = median
      filtered[b] = b1;
    }
  }

  // Pick strongest bin from filtered magnitudes
  int sig_now = filtered[2];  // start with center
  int max_idx = 2;
  if (filtered[0] > sig_now) { sig_now = filtered[0]; max_idx = 0; }
  if (filtered[1] > sig_now) { sig_now = filtered[1]; max_idx = 1; }
  if (filtered[3] > sig_now) { sig_now = filtered[3]; max_idx = 3; }
  if (filtered[4] > sig_now) { sig_now = filtered[4]; max_idx = 4; }

  // Adaptive EMA smoothing on winning magnitude
  // alpha scales with speed: at low WPM we smooth more, at high WPM nearly transparent
  // alpha = clamp(2.0 / dot_len, 0.3, 0.9) keeps the effective window under half a dot
  float ema_alpha = 2.0f / (float)p->dot_len;
  if (ema_alpha < 0.3f) ema_alpha = 0.3f;
  if (ema_alpha > 0.9f) ema_alpha = 0.9f;

  if (p->smoothed_magnitude <= 0.0f) {
    // first sample: seed the EMA
    p->smoothed_magnitude = (float)sig_now;
  } else {
    p->smoothed_magnitude = (1.0f - ema_alpha) * p->smoothed_magnitude + ema_alpha * (float)sig_now;
  }
  p->magnitude = (int)(p->smoothed_magnitude + 0.5f);

  // Track winning bin streak
  if (p->max_bin_idx == max_idx) {
    p->max_bin_streak++;
  } else {
    p->max_bin_streak = 1;
    p->max_bin_idx = max_idx;
  }

  // Signal / noise decision (hybrid magnitude + k-means)
  //
  // Features: x0 = log(1+mag), x1 = distance-from-center (abs(max_idx-2))
  // Quick magnitude thresholds for clear cases; otherwise consult k-means
  // when confident.
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

  // SNR gate: if the spread between high_level and noise_floor is too small
  // there is no usable signal — suppress detections to avoid garbage output
  // 3 dB ≈ linear ratio of ~1.41
  // 2 db = 1.25
  const float MIN_SNR_LINEAR = 1.25f;  // ~2 dB
  float snr_ratio = (p->noise_floor > 0) 
      ? (float)p->high_level / (float)p->noise_floor 
      : 0.0f;

  if (snr_ratio < MIN_SNR_LINEAR) {
    p->sig_state = false;
    p->ticker++;
    return;
  }

  if (z >= Z_HIGH) {
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
      // fall through to normal update below
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

// initialize a struct with values for use with Goertzel algorithm
// inputs:  struct bin *p, float freq, int n, float sampling_freq
// returns: void
// notes: filter will be centered on the specified frequency
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

//////////////////////////////////////////////////////////////////////////
// Section 4: Signal level tracking (high_level / noise_floor)
//////////////////////////////////////////////////////////////////////////

// update signal level tracking
// inputs:  struct cw_decoder *p
// returns: void
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

//////////////////////////////////////////////////////////////////////////
// Section 5: Denoising / mark decision (EMA + hysteresis)
//////////////////////////////////////////////////////////////////////////

// Converts sig_state (boolean, from Section 3) into a filtered mark/space
// decision using an EMA-based confidence tracker with SNR-adaptive
// hysteresis.
//
// Design principle: at low SNR, the Section 3 median filter has already
// removed noise spikes.  The denoiser should therefore be MORE responsive
// at low SNR (to catch weak dots) rather than more cautious.  Caution
// comes from the median filter; responsiveness comes from here.
//
// Produces: p->mark (boolean) and p->sig_confidence (0..1).
// inputs:  struct cw_decoder *p
// returns: void
static void cw_rx_denoise(struct cw_decoder *p) {
  // preserve previous mark state
  p->prev_mark = p->mark;

  // numeric input (1.0 for sig_state true, 0.0 for false)
  float x = p->sig_state ? 1.0f : 0.0f;

  // guard: clamp dot_len to a sane maximum so the denoiser
  // never becomes so sluggish that it can't detect real dots.
  // max_dot_len corresponds to ~5 WPM (slowest reasonable CW).
  int max_dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * 5);
  if (max_dot_len < 1) max_dot_len = 1;
  if (p->dot_len > max_dot_len) p->dot_len = max_dot_len;

  // choose tau (time constant) as a fraction of dot_len (in ticks)
  // controls responsiveness — smaller is faster
  float factor = 0.5f;  // respond over roughly half a dot by default
  float tau = (float)p->dot_len * factor;
  if (tau < 1.0f) tau = 1.0f;

  // discrete-time alpha from tau: alpha = 1 - exp(-1/tau)
  float alpha = 1.0f - expf(-1.0f / tau);

  // clamp alpha to safe bounds
  if (alpha < 0.02f) alpha = 0.02f;
  if (alpha > 0.85f) alpha = 0.85f;

  // SNR-adaptive alpha: FASTER at low SNR to catch weak dots,
  // slightly slower at high SNR where we have margin to smooth.
  // This reverses the previous policy — the median filter in
  // Section 3 now handles noise rejection, so we can afford to
  // be responsive here.
  float snr_linear = (p->noise_floor > 0) ? (float)p->high_level / (float)p->noise_floor : 1.0f;
  float snr_norm = (snr_linear - 1.0f) / 5.0f; // normalize 1..6 → 0..1
  if (snr_norm < 0.0f) snr_norm = 0.0f;
  if (snr_norm > 1.0f) snr_norm = 1.0f;

  float alpha_fast = alpha * 1.3f;   // used at low SNR (be responsive)
  float alpha_slow = alpha * 1.1f;   // used at high SNR (smooth a bit)
  alpha = alpha_fast + (alpha_slow - alpha_fast) * snr_norm;
  if (alpha < 0.01f) alpha = 0.01f;
  if (alpha > 0.95f) alpha = 0.95f;

  // update smoothed confidence
  p->sig_confidence = (1.0f - alpha) * p->sig_confidence + alpha * x;

  // SNR-adaptive hysteresis: NARROWER gap at low SNR so weak dots
  // can cross the threshold; wider gap at high SNR where we have
  // margin and want to reject occasional noise that survived the
  // median filter.
  //
  // At low  SNR (snr_norm→0): gap = 0.20 → thresholds 0.40 / 0.60
  // At high SNR (snr_norm→1): gap = 0.40 → thresholds 0.30 / 0.70
  float gap = 0.20f + 0.20f * snr_norm;
  if (gap < 0.15f) gap = 0.15f;
  if (gap > 0.50f) gap = 0.50f;
  float th_high = 0.5f + gap * 0.5f;
  float th_low  = 0.5f - gap * 0.5f;

  if (p->sig_confidence >= th_high) {
    p->mark = true;
  } else if (p->sig_confidence <= th_low) {
    p->mark = false;
  }
  // else leave p->mark unchanged (hysteresis)
}

//////////////////////////////////////////////////////////////////////////
// Section 6: Symbol detection and segmentation
//////////////////////////////////////////////////////////////////////////

// Adapt dot_len directly from observed mark durations at each mark→space
// transition.  This breaks the vicious cycle where letter-matching failures
// (caused by an inflated dot_len / char_gap) prevent the normal adaptation
// path in cw_rx_match_letter from ever running.
//
// Key insight: the shortest recent mark is almost certainly a dot.
// Noise and QSB lengthen marks; they rarely shorten them.
// If the shortest mark is much shorter than dot_len, dot_len is stuck high.
//
// inputs:  struct cw_decoder *p, int mark_ticks
// returns: void
static void cw_rx_early_dot_adapt(struct cw_decoder *p, int mark_ticks) {
  if (mark_ticks < 1) return;

  // Record this mark duration in a small circular buffer
  p->recent_marks[p->recent_marks_pos] = (float)mark_ticks;
  p->recent_marks_pos = (p->recent_marks_pos + 1) % 8;
  if (p->recent_marks_count < 8) p->recent_marks_count++;

  // need at least 3 marks to have a reliable shortest
  if (p->recent_marks_count < 3) return;

  // Find the shortest recent mark (likely a dot)
  float shortest = 1e9f;
  for (int i = 0; i < p->recent_marks_count; i++) {
    if (p->recent_marks[i] > 0.0f && p->recent_marks[i] < shortest)
      shortest = p->recent_marks[i];
  }
  p->shortest_recent_mark = shortest;

  // Check if current dot_len is significantly too large
  float ratio = (float)p->dot_len / shortest;
  if (ratio < 1.2f) return;  // dot_len is reasonable (was 1.5)

  // Pull dot_len toward the shortest mark
  float urgency = (ratio - 1.2f) / 1.3f;  // 0 at ratio=1.2, 1 at ratio=2.5
  if (urgency > 1.0f) urgency = 1.0f;

  float alpha = 0.15f + 0.35f * urgency;   // range [0.15 .. 0.50]

  float proposed = (1.0f - alpha) * (float)p->dot_len + alpha * shortest;

  // floor: don't let dot_len go below ~60 WPM equivalent
  int min_dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * 60);
  if (min_dot_len < 1) min_dot_len = 1;
  if (proposed < (float)min_dot_len) proposed = (float)min_dot_len;

  p->dot_len = (int)(proposed + 0.5f);
}

// add a mark or space to the symbol buffer, store its duration (ticks),
// and update the symbol's average magnitude
// inputs:  struct cw_decoder *p, char symbol
// returns: void
static void cw_rx_add_symbol(struct cw_decoder *p, char symbol) {
  // wrap around when full
  if (p->next_symbol == MAX_SYMBOLS) p->next_symbol = 0;
  // only ' ' (space) is treated as a space; all other symbols are marks
  p->symbol_str[p->next_symbol].is_mark = (symbol != ' ');  // 0 for space, 1 for mark
  // store the duration of the symbol (number of ticks since last transition)
  p->symbol_str[p->next_symbol].ticks = p->ticker;
  // update the average magnitude for this symbol using a weighted average
  p->symbol_str[p->next_symbol].magnitude =
      ((p->symbol_str[p->next_symbol].magnitude * 10) + p->magnitude) / 11;
  // move to the next position in the symbol buffer
  p->next_symbol++;
}

// Detect transitions between mark and space and determine if a dot, dash,
// character space or word space has occurred.
// Detect Farnsworth spacing and adjust word-gap threshold when found.
// returns: bool - true if a character gap was detected (caller should call matcher)
static bool cw_rx_detect_symbol(struct cw_decoder *p) {
  // Tunables
  const float ALPHA = 0.14f;    // EMA smoothing factor
  const float WORD_THR = 1.7f;  // min word gap/char gap ratio for word boundary
  const int WORD_COUNT = 3;     // number of consecutive chars before allowing word boundary
  const int MAX_MARK = 12;      // max mark duration, dots
  const float GAP_ALPHA = 0.20f; // EMA for measured gaps (char/word)

  // Blend dot_len with expected (UI WPM) to prevent char_gap inflation.
  // When dot_len drifts high, using it raw for gap thresholds creates a
  // positive feedback loop.  Blending with the UI-expected value caps the
  // gap growth and keeps characters closing at a reasonable rate.
  int expected_dot = (6 * SAMPLING_FREQ) / (5 * N_BINS * p->wpm);
  if (expected_dot < 1) expected_dot = 1;
  const float GAP_DOT_BLEND = 0.5f;  // 50/50 blend
  int dot = (int)(GAP_DOT_BLEND * (float)p->dot_len
                + (1.0f - GAP_DOT_BLEND) * (float)expected_dot + 0.5f);
  if (dot < 1) dot = 1;

  int char_gap_nom = 3 * dot;
  int word_gap_nom = 7 * dot;

  // Compute current thresholds (use EMAs if available, otherwise nominal)
  // Use the LONGER of EMA and a minimum floor (2 * dot) so we never
  // cut below the intra-character gap
  int char_gap = char_gap_nom;
  if (p->char_gap_ema > 0.0f) {
    int ema_gap = (int)p->char_gap_ema;
    // blend toward EMA but never go below 2*dot (the element-gap zone)
    int floor = 2 * dot;
    if (ema_gap < floor) ema_gap = floor;
    char_gap = (ema_gap < char_gap_nom) ? ema_gap : char_gap_nom;
  }

  int word_gap = word_gap_nom;
  if (p->word_gap_ema > 0.0f) {
    int ema_gap = (int)p->word_gap_ema;
    word_gap = (ema_gap < word_gap_nom) ? ema_gap : word_gap_nom;
  }
	
  // Enforce minimum separation between char and word gaps
  if (word_gap <= char_gap + 3 * dot)
    word_gap = char_gap + 4 * dot;

  // -- Transition (MARK -> SPACE): End of element
  if (!p->mark && p->prev_mark) {
    cw_rx_add_symbol(p, 'm');
    cw_rx_early_dot_adapt(p, p->ticker);
    p->ticker = 0;
    return false;
  }

  // -- Transition (SPACE -> MARK): Start of new mark, gap just finished
  if (p->mark && !p->prev_mark) {
    int gap_ticks = p->ticker;
    cw_rx_add_symbol(p, ' ');

    // Only clear the space flag if SNR suggests a real signal is present.
    // This prevents noise glitches from re-enabling space emission.
    float snr_ratio = (p->noise_floor > 0)
        ? (float)p->high_level / (float)p->noise_floor
        : 0.0f;
    if (snr_ratio >= 1.6f)
      p->last_char_was_space = 0;

    // --- Update gap EMAs based on what we just measured ---
    if (gap_ticks >= word_gap) {
        if (p->word_gap_ema <= 0.0f) p->word_gap_ema = (float)gap_ticks;
        else p->word_gap_ema = (1.0f - GAP_ALPHA) * p->word_gap_ema + GAP_ALPHA * (float)gap_ticks;
    } else {
        if (p->char_gap_ema <= 0.0f) p->char_gap_ema = (float)gap_ticks;
        else p->char_gap_ema = (1.0f - GAP_ALPHA) * p->char_gap_ema + GAP_ALPHA * (float)gap_ticks;
    }
    
    p->ticker = 0;
    return false;
  }

  // -- Remain in SPACE: Check for symbol/char/word gaps
  if (!p->mark && !p->prev_mark) {
    int t = p->ticker;

    // If we're in the middle of building a character, handle element + char gaps
    if (p->next_symbol > 0) {
      // Add a single inter-element space after a mark
      if (t >= dot && p->symbol_str[p->next_symbol - 1].is_mark) {
        cw_rx_add_symbol(p, ' ');
        // do NOT reset ticker; we still want to measure the full gap
      }

      // Close the character after the char gap
      if (t >= char_gap) {
        // Don't clear last_char_was_space here — let cw_rx_match_letter
        // clear it only if a character actually passes the confidence gate.
        // This prevents rejected noise from re-enabling space emission.
        return true;
      }
    }

    // Word gap check — only meaningful after actual decoded content
    if (t >= word_gap && p->next_symbol == 0) {
      // We already matched whatever was in the buffer (or there was nothing).
      // Only emit a space if we actually decoded a character before this gap,
      // indicated by last_char_was_space == 0.
      float snr_ratio = (p->noise_floor > 0)
          ? (float)p->high_level / (float)p->noise_floor
          : 0.0f;
      const float MIN_SNR_FOR_SPACE = 1.6f;

       if (cw_decode_enabled && p->decoded_char_seen && !p->last_char_was_space &&
            snr_ratio >= MIN_SNR_FOR_SPACE) {
        cw_write_console(p->console_font, " ");
        p->last_char_was_space = 1;
        p->decoded_char_seen = 0;
      }
    }
  }
  // -- Remain in MARK: Avoid runaway marks (long pressed carrier)
  else if (p->mark && p->prev_mark) {
    if (p->ticker > MAX_MARK * dot) p->ticker = 3 * dot;
  }
  return false;
}

//////////////////////////////////////////////////////////////////////////
// Section 7: Build Morse character
//   From decoder.symbol_str → arrays of mark durations and magnitudes
//   estimate_dot_dash_centroids() → dot_mu, dash_mu
//   Threshold at sqrt(dot_mu * dash_mu) → dot vs dash
//   Builds Morse string + computes confidence
//////////////////////////////////////////////////////////////////////////

// Estimate dot and dash centroids from mark durations using a simple
// iterative threshold split.  No k-means or variance computation needed —
// the caller only needs the two centroids to set an adaptive threshold.
//
// inputs:  struct cw_decoder *p, float *mark_durs (linear ticks), int n,
//          float *out_dot_mu, float *out_dash_mu (linear ticks)
// returns: true if a reliable dot/dash separation was found
static bool estimate_dot_dash_centroids(struct cw_decoder *p, float *mark_durs, int n,
                                    float *out_dot_mu, float *out_dash_mu) {
    if (n <= 1) {
        // not enough marks to separate dot from dash
        *out_dot_mu  = (float)p->dot_len;
        *out_dash_mu = (float)p->dot_len * 3.0f;
        return false;
    }

    // iterative threshold split: start with the mean, then refine
    // by splitting into short/long groups and recomputing centroids
    float sum_all = 0.0f;
    for (int i = 0; i < n; ++i) sum_all += mark_durs[i];
    float threshold = sum_all / (float)n;

    const int MAX_ITERS = 8;
    for (int iter = 0; iter < MAX_ITERS; ++iter) {
        float sum_short = 0.0f, sum_long = 0.0f;
        int n_short = 0, n_long = 0;

        for (int i = 0; i < n; ++i) {
            if (mark_durs[i] < threshold) {
                sum_short += mark_durs[i]; n_short++;
            } else {
                sum_long += mark_durs[i]; n_long++;
            }
        }

        // if everything fell into one group, we can't separate
        if (n_short == 0 || n_long == 0) {
            *out_dot_mu  = (float)p->dot_len;
            *out_dash_mu = (float)p->dot_len * 3.0f;
            return false;
        }

        float mu_short = sum_short / (float)n_short;
        float mu_long  = sum_long  / (float)n_long;
        float new_threshold = (mu_short + mu_long) * 0.5f;

        // converged?
        if (fabsf(new_threshold - threshold) < 0.5f) {
            threshold = new_threshold;
            *out_dot_mu  = mu_short;
            *out_dash_mu = mu_long;
            break;
        }
        threshold = new_threshold;
        *out_dot_mu  = mu_short;
        *out_dash_mu = mu_long;
    }

    // require minimal separation: dash centroid must be at least 1.5× dot centroid
    if (*out_dot_mu <= 0.0f || *out_dash_mu / *out_dot_mu < 1.5f)
        return false;

    return true;
}

// Build a Morse code string from the collected symbol buffer.
//
// Extracts mark durations and magnitudes, estimates dot/dash centroids,
// classifies each mark via adaptive geometric-mean threshold, assembles
// the dot-dash string, and computes a blended confidence score.
//
// All results are returned via the cw_morse_result struct so that the
// classification / output stage (Section 8) is fully decoupled.
//
// inputs:  struct cw_decoder *p, struct cw_morse_result *result
// returns: void (populates *result)
static void cw_rx_build_morse(struct cw_decoder *p, struct cw_morse_result *result) {
  // initialize result to "nothing decoded"
  result->morse_code[0] = '\0';
  result->confidence = 0.0f;
  result->dot_mu = 0.0f;
  result->dash_mu = 0.0f;
  result->est_ok = false;
  result->n_marks = 0;
  result->valid = false;

  // if no symbols have been received, there's nothing to decode
  if (p->next_symbol == 0) return;

  // build arrays of mark durations and magnitudes from collected symbols
  float mark_durs[MAX_SYMBOLS];
  float mark_mags[MAX_SYMBOLS];
  int n_marks = 0;
  for (int i = 0; i < p->next_symbol; ++i) {
    if (p->symbol_str[i].is_mark) {
      mark_durs[n_marks] = (float)p->symbol_str[i].ticks;
      mark_mags[n_marks] = (float)p->symbol_str[i].magnitude;
      n_marks++;
    }
  }
  result->n_marks = n_marks;

  // If no marks, nothing to build
  if (n_marks == 0) return;

  // estimate dot and dash centroids from the marks in this character
  float dot_mu, dash_mu;
  bool est_ok = estimate_dot_dash_centroids(p, mark_durs, n_marks, &dot_mu, &dash_mu);

  if (est_ok) {
    // store estimates in decoder for diagnostics and dot_len adaptation
    p->mark_mu_dot  = dot_mu;
    p->mark_mu_dash = dash_mu;
    p->mark_emission_ready = true;
  } else if (p->mark_emission_ready) {
    // use cached estimates from a previous successful decode
    dot_mu  = p->mark_mu_dot;
    dash_mu = p->mark_mu_dash;
    est_ok  = true;
  } else {
    // no prior estimates available; derive from dot_len
    dot_mu  = (float)p->dot_len;
    dash_mu = (float)p->dot_len * 3.0f;
  }

  result->dot_mu  = dot_mu;
  result->dash_mu = dash_mu;
  result->est_ok  = est_ok;

  // classify each mark using adaptive threshold at geometric mean of centroids
  // geometric mean naturally sits at the right point between 1× and 3× ratios
  int assignments[MAX_SYMBOLS];
  float threshold = sqrtf(dot_mu * dash_mu);

  // safety: if threshold is nonsensical, fall back to 2× dot_len
  if (threshold <= 0.0f)
    threshold = (float)p->dot_len * 2.0f;

  for (int i = 0; i < n_marks; ++i) {
    assignments[i] = (mark_durs[i] >= threshold) ? 1 : 0;
  }

  // build morse string from assignments in original symbol order
  int mpos = 0;
  int mark_idx = 0;
  for (int i = 0; i < p->next_symbol && mpos < MAX_SYMBOLS; ++i) {
    if (p->symbol_str[i].is_mark) {
      result->morse_code[mpos++] = (assignments[mark_idx] == 0) ? '.' : '-';
      mark_idx++;
    }
  }
  result->morse_code[mpos] = '\0';

  // --- Compute blended confidence score ---

  // compute average mark magnitude and normalized z
  float avg_mag = 0.0f;
  for (int i = 0; i < n_marks; ++i) avg_mag += mark_mags[i];
  avg_mag /= (float)n_marks;
  float denom = (float)(p->high_level - p->noise_floor);
  if (denom <= 0.0f) denom = 1.0f;
  float avg_z = (avg_mag - (float)p->noise_floor) / denom;

  // centroid separation (diagnostic)
  float dx0 = p->k_centroid_signal[0] - p->k_centroid_noise[0];
  float dx1 = p->k_centroid_signal[1] - p->k_centroid_noise[1];
  float centroid_sep = sqrtf(dx0 * dx0 + dx1 * dx1);

  // dot/dash ratio sanity check
  float dd_ratio = (dot_mu > 0.0f) ? (dash_mu / dot_mu) : 0.0f;
  int ratio_ok = (dd_ratio > 1.3f && dd_ratio < 5.0f) ? 1 : 0;

  // cluster counts
  const int MIN_KCOUNT = 12;
  int kcount_ok =
      (p->k_count_signal >= MIN_KCOUNT && p->k_count_noise >= MIN_KCOUNT) ? 1 : 0;

  // build a blended confidence score (0..1)
  const float W_Z     = 0.30f;
  const float W_SEP   = 0.15f;
  const float W_KCNT  = 0.05f;
  const float W_RATIO = 0.10f;
  const float W_SNR   = 0.40f;

  // normalize components into 0..1 ranges
  float comp_z = (avg_z - 0.05f) / (0.6f - 0.05f);
  if (comp_z < 0.0f) comp_z = 0.0f;
  if (comp_z > 1.0f) comp_z = 1.0f;
  float comp_sep = centroid_sep / 0.6f;
  if (comp_sep < 0.0f) comp_sep = 0.0f;
  if (comp_sep > 1.0f) comp_sep = 1.0f;
  float comp_kcnt = (kcount_ok) ? 1.0f : 0.0f;
  float comp_ratio = (ratio_ok) ? 1.0f : 0.0f;

  // SNR component
  float snr_now = (p->noise_floor > 0)
      ? (float)p->high_level / (float)p->noise_floor
      : 1.0f;
  float comp_snr = (snr_now - 1.0f) / 3.0f;
  if (comp_snr < 0.0f) comp_snr = 0.0f;
  if (comp_snr > 1.0f) comp_snr = 1.0f;

  result->confidence = W_Z * comp_z + W_SEP * comp_sep +
                       W_KCNT * comp_kcnt + W_RATIO * comp_ratio + W_SNR * comp_snr;

  result->valid = (result->morse_code[0] != '\0');
}

//////////////////////////////////////////////////////////////////////////
// Section 8: Character classification and output
//   Table lookup → console output
//   dot_len adaptation and recovery
//   Word-gap space emission
//////////////////////////////////////////////////////////////////////////

// shared console style tracker so both RX and TX decode paths
// insert a newline when the style changes, preventing color bleed
static int last_console_style = -1;
static bool last_console_was_newline = false;

// write decoded text to the console, inserting a newline
// on the previous style's line whenever the style changes
static void cw_write_console(int style, const char *text) {
  if (last_console_style != -1 && last_console_style != style) {
    // only insert a newline if we didn't just write one,
    // to avoid blank lines at TX→RX transitions
    if (!last_console_was_newline) {
      write_console(last_console_style, "\n");
    }
  }
  write_console(style, text);
  last_console_style = style;
  last_console_was_newline = (strcmp(text, "\n") == 0);
}

// Classify the Morse string built by Section 7, adapt dot_len, and
// output the decoded character to the console.
//
// inputs:  struct cw_decoder *p, struct cw_morse_result *result (from cw_rx_build_morse)
// returns: void
static void cw_rx_classify_and_output(struct cw_decoder *p, struct cw_morse_result *result) {
  const float CONF_THRESHOLD = 0.55f;

  // Track whether any adaptation path ran this cycle.
  // Declared early so both the multi-mark and single-mark paths can set it.
  bool adapted_this_cycle = false;

  // adapt p->dot_len only with enough marks and confidence
  if (result->est_ok && result->n_marks >= 2 && result->confidence >= CONF_THRESHOLD) {
    float dot_ticks_est  = result->dot_mu;
    float dash_ticks_est = result->dash_mu;
    if (dot_ticks_est > 0.0f && dash_ticks_est / dot_ticks_est > 1.5f
                              && dash_ticks_est / dot_ticks_est < 4.5f) {
      const float DOT_ADAPT_ALPHA = 0.12f;
      const float MIN_ADAPT_RATIO = 0.70f;
      const float MAX_ADAPT_RATIO = 1.30f;

      float proposed = (1.0f - DOT_ADAPT_ALPHA) * (float)p->dot_len
                       + DOT_ADAPT_ALPHA * dot_ticks_est;
      float min_allowed = (float)p->dot_len * MIN_ADAPT_RATIO;
      float max_allowed = (float)p->dot_len * MAX_ADAPT_RATIO;
      if (proposed < min_allowed) proposed = min_allowed;
      if (proposed > max_allowed) proposed = max_allowed;

      int new_dot_len = (int)(proposed + 0.5f);
      if (new_dot_len < 1) new_dot_len = 1;
      p->dot_len = new_dot_len;
      adapted_this_cycle = true;
    }
  }

  // Single-mark adaptation: if the character had exactly one mark,
  // we can't classify dot vs dash, but if that mark is much shorter
  // than dot_len, dot_len is probably inflated.  Pull it down gently.
  if (!adapted_this_cycle && result->n_marks == 1 && result->confidence >= CONF_THRESHOLD) {
    float single_dur = 0.0f;
    for (int i = 0; i < p->next_symbol; ++i) {
      if (p->symbol_str[i].is_mark) { single_dur = (float)p->symbol_str[i].ticks; break; }
    }
    if (single_dur > 0.0f && single_dur < (float)p->dot_len * 0.75f) {
      const float SINGLE_ALPHA = 0.08f;
      float proposed = (1.0f - SINGLE_ALPHA) * (float)p->dot_len + SINGLE_ALPHA * single_dur;
      int min_dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * 60);
      if (min_dot_len < 1) min_dot_len = 1;
      if (proposed < (float)min_dot_len) proposed = (float)min_dot_len;
      p->dot_len = (int)(proposed + 0.5f);
      adapted_this_cycle = true;
    }
  }

  // recover dot_len toward UI WPM whenever adaptation didn't run this cycle.
  if (!adapted_this_cycle) {
    int expected_dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * p->wpm);
    if (expected_dot_len < 1) expected_dot_len = 1;

    float err_ratio = (float)p->dot_len / (float)expected_dot_len;
    if (err_ratio < 1.0f) err_ratio = 1.0f / err_ratio;

    float recover_alpha = 0.40f;
    if (err_ratio > 1.5f) recover_alpha = 0.55f;
    if (err_ratio > 2.0f) recover_alpha = 0.65f;
    if (err_ratio > 4.0f) recover_alpha = 0.80f;

    p->dot_len = (int)((1.0f - recover_alpha) * p->dot_len
                             + recover_alpha * expected_dot_len + 0.5f);
    if (p->dot_len < 1) p->dot_len = 1;
  }
	
  // If nothing was built, or below confidence threshold, nothing to output
  if (!result->valid) return;
  if (result->confidence < CONF_THRESHOLD) return;

  // emit decoded character if in table
  for (int i = 0; i < (int)(sizeof(morse_rx_table) / sizeof(struct morse_rx)); i++) {
    if (!strcmp(result->morse_code, morse_rx_table[i].code)) {
      const char *decoded = morse_rx_table[i].c;
      if (decoded[0] == ' ' && decoded[1] == '\0') {
        return;
      }
      if (cw_decode_enabled) {
        cw_write_console(p->console_font, decoded);
      }
      p->last_char_was_space = 0;
      p->decoded_char_seen = 1;
      return;
    }
  }
}

//////////////////////////////////////////////////////////////////////////
// CW TX Decoder (separate from the RX decoder pipeline)
//////////////////////////////////////////////////////////////////////////

// Minimal TX self-decoder state (outside the RX decoder pipeline entirely)
static char  tx_morse_buf[CW_MAX_SYMBOLS + 1];  // dots and dashes for current char
static int   tx_morse_pos = 0;
static int   tx_mark_ticks = 0;       // how long current mark has lasted
static int   tx_space_ticks = 0;      // how long current space has lasted
static bool  tx_prev_mark = false;    // previous detection state
static int   tx_high = 0;             // peak magnitude tracker
static int   tx_noise = 0;            // noise floor tracker
static bool  tx_char_emitted = false; // did we emit a char since last space?

static void cw_tx_emit_char(void) {
  if (tx_morse_pos == 0) return;
  tx_morse_buf[tx_morse_pos] = '\0';

  // look up in the rx table
  for (int i = 0; i < (int)(sizeof(morse_rx_table) / sizeof(struct morse_rx)); i++) {
    if (!strcmp(tx_morse_buf, morse_rx_table[i].code)) {
      const char *decoded = morse_rx_table[i].c;
      if (decoded[0] != ' ' && cw_decode_enabled) {
        cw_write_console(STYLE_CW_TX, decoded);
        tx_char_emitted = true;
      }
      break;
    }
  }
  tx_morse_pos = 0;
}

// entry point for decoding transmitted CW
static void cw_tx_decode_samples(void) {
  int decimation_factor = 8;
  int32_t s[N_BINS];

  for (int i = 0; i < N_BINS; i++) {
    s[i] = tx_sample_buffer[i * decimation_factor];
  }

  tx_session_active = true;

  // known WPM → fixed dot length in ticks (1 tick = 1 Goertzel block = N_BINS/SAMPLING_FREQ sec)
  int wpm = tx_decoder.wpm;
  if (wpm < 5) wpm = 5;
  int dot_ticks = (6 * SAMPLING_FREQ) / (5 * N_BINS * wpm);
  if (dot_ticks < 1) dot_ticks = 1;

  // thresholds derived from known timing (in ticks)
  int dash_threshold  = 2 * dot_ticks;    // marks longer than this are dashes
  int char_gap        = 2 * dot_ticks;     // space longer than this ends a character
  int word_gap        = 5 * dot_ticks;     // space longer than this is a word boundary

  // run Goertzel on center bin
  int mag = cw_rx_bin_detect(&tx_decoder.signal_center, s);

  // track high and noise levels
  if (mag > tx_high) tx_high = mag;
  else tx_high = (mag + 49 * tx_high) / 50;

  if (tx_high < 1) tx_high = 1;

  // determine mark/space with simple threshold
  bool is_mark = (mag > tx_high / 3);

  // -- transition: mark → space (element just ended)
  if (!is_mark && tx_prev_mark) {
    // classify the mark that just ended
    if (tx_morse_pos < CW_MAX_SYMBOLS) {
      tx_morse_buf[tx_morse_pos++] = (tx_mark_ticks >= dash_threshold) ? '-' : '.';
    }
    tx_space_ticks = 0;
  }
  // -- transition: space → mark (new element starting)
  else if (is_mark && !tx_prev_mark) {
    // check if the space was long enough for char or word gap
    if (tx_space_ticks >= char_gap && tx_morse_pos > 0) {
      cw_tx_emit_char();
    }
     if (tx_space_ticks >= word_gap && tx_char_emitted && cw_decode_enabled) {
      cw_write_console(STYLE_CW_TX, " ");
      tx_char_emitted = false;
    }
    tx_mark_ticks = 0;
  }

  // count durations
  if (is_mark) tx_mark_ticks++;
  else         tx_space_ticks++;

  tx_prev_mark = is_mark;
}

//////////////////////////////////////////////////////////////////////////
// CW init / poll / stats / abort API
//////////////////////////////////////////////////////////////////////////

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
  cw_rx_bin_init(&decoder.signal_minus2, cw_rx_pitch - 150.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_minus1, cw_rx_pitch - 75.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_center, cw_rx_pitch + 0.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_plus1,  cw_rx_pitch + 75.0f, N_BINS, SAMPLING_FREQ);
  cw_rx_bin_init(&decoder.signal_plus2,  cw_rx_pitch + 150.0f, N_BINS, SAMPLING_FREQ);

  // RX decoder: detect
  decoder.mark = false;
  decoder.prev_mark = false;
  decoder.sig_state = false;
  decoder.ticker = 0;

  // RX decoder: levels
  decoder.magnitude = 0;
  decoder.smoothed_magnitude = 0.0f;
  decoder.high_level = 500;  // start with non-zero values for SNR
  decoder.noise_floor = 200;
  decoder.max_bin_idx = -1;
  decoder.max_bin_streak = 0;

  // RX decoder: per-bin median filter history
  for (int b = 0; b < 5; b++)
    for (int t = 0; t < 3; t++)
      decoder.bin_hist[b][t] = 0;
  decoder.bin_hist_pos = 0;
  decoder.bin_hist_count = 0;

  // RX decoder: timing
  decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * INIT_WPM);
  decoder.next_symbol = 0;
  decoder.last_char_was_space = 0;
  decoder.decoded_char_seen = 0;
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
  decoder.mark_mu_dash = 0.0f;
  decoder.mark_emission_ready = false;
  decoder.sig_confidence = 0.0f;
  decoder.recent_marks_pos = 0;
  decoder.recent_marks_count = 0;
  decoder.shortest_recent_mark = 0.0f;
  for (int i = 0; i < 8; i++) decoder.recent_marks[i] = 0.0f;

  // RX decoder: sym buffer
  for (int i = 0; i < MAX_SYMBOLS; ++i) {
    decoder.symbol_str[i].is_mark = 0;
    decoder.symbol_str[i].magnitude = 0;
    decoder.symbol_str[i].ticks = 0;
  }

  // TX decoder: only needs WPM and center bin
  tx_decoder.wpm = 20;
  int cw_tx_pitch = get_pitch();
  cw_rx_bin_init(&tx_decoder.signal_center, cw_tx_pitch + 0.0f, N_BINS, SAMPLING_FREQ);

  // TX decode state
  tx_morse_pos = 0;
  tx_mark_ticks = 0;
  tx_space_ticks = 0;
  tx_prev_mark = false;
  tx_high = 0;
  tx_noise = 0;
  tx_char_emitted = false;
  memset(tx_morse_buf, 0, sizeof(tx_morse_buf));

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

  // Hold the stats display for a period after last decode,
  // rather than blanking immediately on each poll.
  static unsigned long last_decode_time = 0;
  const unsigned long DISPLAY_HOLD_MS = 3000;  // keep stats visible for 3 seconds

  if (decoder.decoded_char_seen) {
    last_decode_time = millis();
    // Do NOT clear decoded_char_seen here — let cw_rx_detect_symbol
    // manage it for word-space gating purposes.
  }

  unsigned long now = millis();
  if (now - last_decode_time > DISPLAY_HOLD_MS) {
    buf[0] = '\0';
    return buf;
  }

  float dot_ticks = (float)decoder.dot_len;
  float dash_ticks = (float)decoder.dot_len * 3.0f;

  if (decoder.mark_emission_ready) {
    // mark_mu_* are now in linear ticks (no log transform needed)
    dot_ticks  = decoder.mark_mu_dot;
    dash_ticks = decoder.mark_mu_dash;
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
  const float WPM_ALPHA_UP   = 0.18f;
  const float WPM_ALPHA_DOWN = 0.65f;
  const float WPM_MAX_UP_STEP = 1.5f;
  const float RATIO_ALPHA = 0.45f;

  if (!have_ema) {
    wpm_ema   = inst_wpm_f;
    ratio_ema = inst_ratio;
    have_ema  = 1;
  } else {
    if (inst_wpm_f < wpm_ema) {
      wpm_ema = (1.0f - WPM_ALPHA_DOWN) * wpm_ema + WPM_ALPHA_DOWN * inst_wpm_f;
    } else {
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
    cw_rx_bin_init(&decoder.signal_minus2, cw_rx_pitch - 150.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_minus1, cw_rx_pitch - 75.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_center, cw_rx_pitch + 0.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_plus1, cw_rx_pitch + 75.0f, N_BINS, SAMPLING_FREQ);
    cw_rx_bin_init(&decoder.signal_plus2, cw_rx_pitch + 150.0f, N_BINS, SAMPLING_FREQ);
  }
  // check if the wpm has changed
  if (wpm != decoder.wpm) {
    decoder.wpm = wpm;
    decoder.dot_len = (6 * SAMPLING_FREQ) / (5 * N_BINS * wpm);
  }
  
  // retune the tx decoder pitch if needed
  int cw_tx_pitch = get_pitch();
  if (cw_tx_pitch != tx_decoder.signal_center.freq) {
    cw_rx_bin_init(&tx_decoder.signal_center, cw_tx_pitch + 0.0f, N_BINS, SAMPLING_FREQ);
  }
  // update tx decoder WPM
  if (wpm != tx_decoder.wpm) {
    tx_decoder.wpm = wpm;
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
    // Flush any pending TX decode character
    cw_tx_emit_char();
    // If we were in a TX session, write newline to end it
    if (tx_session_active) {
      cw_write_console(STYLE_CW_TX, "\n");
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
		cw_write_console(STYLE_CW_TX, "\n");
		tx_session_active = false;
	}

	// Reset TX state
	keydown_count = 0;
	keyup_count = 0;
	cw_tx_until = 0;
	symbol_next = NULL;
}
