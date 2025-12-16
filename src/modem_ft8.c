#include <assert.h>
#include <stdlib.h>
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
#include "sdr.h"
#include "sdr_ui.h"
#include "modem_ft8.h"
#include "logbook.h"

#define LOG_LEVEL LOG_INFO

#include "ft8_lib/common/common.h"
#include "ft8_lib/common/wave.h"
#include "ft8_lib/ft8/debug.h"
#include "ft8_lib/ft8/decode.h"
#include "ft8_lib/ft8/encode.h"
#include "ft8_lib/ft8/constants.h"
#include "ft8_lib/fft/kiss_fftr.h"

// We try to avoid calling automatically the same stations again and again, at least in this session
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define FTX_CALLED_SIZE 64
static char ftx_already_called[FTX_CALLED_SIZE][32];
static int ftx_already_called_n = 0;
static int recent_qso_age = 24; // hours

static float ftx_rx_buffer[FT8_MAX_BUFF];
static float ftx_tx_buffer[FT8_MAX_BUFF];
static char ftx_tx_text[128];
static char ftx_xota_text[14];
ftx_message_t ftx_tx_msg;
ftx_message_t ftx_xota_msg;
static int ftx_rx_buff_index = 0;
static int ftx_tx_buff_index = 0;
static int ftx_tx_nsamples = 0;
static int ftx_do_decode = 0;
static int ftx_do_tx = 0;
static int ftx_pitch = 0;
static pthread_t ftx_thread;
// number of repetitions left for the current message, counting down from the user setting
static int ftx_repeat = 5;
static bool is_cq = false; // is ftx_tx_text a CQ?
static bool ftx_tx1st = true;
static bool ftx_cq_alt = false;
static bool ftx_xota = false;

static const int kMin_score = 10; // Minimum sync score threshold for candidates
static const int kMax_candidates = 120;
static const int kLDPC_iterations = 20;

static const int kMax_decoded_messages = 50;

static const int kFreq_osr = 2; // Frequency oversampling rate (bin subdivision)
static const int kTime_osr = 2; // Time oversampling rate (symbol subdivision)

// styles to use for each enum value in ftx_field_t
static const int kFieldType_style_map[] = {
	STYLE_LOG,		// FTX_FIELD_UNKNOWN
	STYLE_LOG,		// FTX_FIELD_NONE
	STYLE_FT8_RX,	// FTX_FIELD_TOKEN
	STYLE_FT8_RX,	// FTX_FIELD_TOKEN_WITH_ARG
	STYLE_CALLER,	// FTX_FIELD_CALL
	STYLE_GRID,		// FTX_FIELD_GRID
	STYLE_RST		// FTX_FIELD_RST
};

#define SECS_IN_DAY (24 * 60 * 60)
static int wallclock_day_ms = 0; // starts from 0 each day

static void ftx_update_clock()
{
	struct timespec  ts;

	if (clock_gettime(CLOCK_REALTIME, &ts) == -1) {
	   perror("clock_gettime");
	   exit(EXIT_FAILURE);
	}

	time_t ms = ts.tv_nsec / 1000000;
	wallclock_day_ms = (ts.tv_sec % SECS_IN_DAY) * 1000 + ms;
	//~ printf("time %lld.%lld: %d min ms %d\n", ts.tv_sec, ms, wallclock_day_ms, wallclock_day_ms % 60000);
}

/*!
	Format the day-time-in-millseconds \a day_ms
	(or wallclock time instead if \a day_ms is -1)
	as HHMMSS.half-second to the given buffer, and return
	a pointer to the character after what has been printed
	(e.g. to continue printing something else with sprintf).
	In practice, it always prints 8 characters and returns buf + 8.
*/
// TODO move this and ftx_update_clock to sbitx.h or so: use a global clock
static char *hmst_time_sprint(char *buf, int day_ms)
{
	const int dms = day_ms >= 0 ? day_ms : wallclock_day_ms;
	int wallclock_day_s = dms / 1000;
	// simple arithmetic instead of using modulus (%): assuming the latter is much more expensive
	// it could be that this is less efficient though; it could be checked with callgrind or so
	int h = wallclock_day_s / (3600);
	int m = (wallclock_day_s - (h * 3600)) / 60;
	int s = wallclock_day_s - (h * 3600 + m * 60);
	int tenth_sec = (dms - (wallclock_day_s * 1000)) / 100;
	const int len = tenth_sec ?
		sprintf(buf, "%02d%02d%02d.%01d", h, m, s, tenth_sec) :
		sprintf(buf, "%02d%02d%02d  ", h, m, s);
	return buf + len;
}

static char *hmst_wallclock_time_sprint(char *buf)
{
	return hmst_time_sprint(buf, -1);
}

#define FT8_SYMBOL_BT 2.0f ///< symbol smoothing filter bandwidth factor (BT)
#define FT4_SYMBOL_BT 1.0f ///< symbol smoothing filter bandwidth factor (BT)

#define GFSK_CONST_K 5.336446f ///< == pi * sqrt(2 / log(2))

#define CALLSIGN_HASHTABLE_SIZE 256

static struct
{
    char callsign[12]; ///> Up to 11 symbols of callsign + trailing zeros (always filled)
    uint32_t hash;     ///> 8 MSBs contain the age of callsign; 22 LSBs contain hash value
} callsign_hashtable[CALLSIGN_HASHTABLE_SIZE];

static int callsign_hashtable_size;

void hashtable_init(void)
{
    callsign_hashtable_size = 0;
    memset(callsign_hashtable, 0, sizeof(callsign_hashtable));
}

void hashtable_cleanup(uint8_t max_age)
{
    for (int idx_hash = 0; idx_hash < CALLSIGN_HASHTABLE_SIZE; ++idx_hash)
    {
        if (callsign_hashtable[idx_hash].callsign[0] != '\0')
        {
            uint8_t age = (uint8_t)(callsign_hashtable[idx_hash].hash >> 24);
            if (age > max_age)
            {
                LOG(LOG_INFO, "Removing [%s] from hash table, age = %d\n", callsign_hashtable[idx_hash].callsign, age);
                // free the hash entry
                callsign_hashtable[idx_hash].callsign[0] = '\0';
                callsign_hashtable[idx_hash].hash = 0;
                callsign_hashtable_size--;
            }
            else
            {
                // increase callsign age
                callsign_hashtable[idx_hash].hash = (((uint32_t)age + 1u) << 24) | (callsign_hashtable[idx_hash].hash & 0x3FFFFFu);
            }
        }
    }
}

void hashtable_add(const char* callsign, uint32_t hash)
{
    uint16_t hash10 = (hash >> 12) & 0x3FFu;
    int idx_hash = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    while (callsign_hashtable[idx_hash].callsign[0] != '\0')
    {
        if (((callsign_hashtable[idx_hash].hash & 0x3FFFFFu) == hash) && (0 == strcmp(callsign_hashtable[idx_hash].callsign, callsign)))
        {
            // reset age
            callsign_hashtable[idx_hash].hash &= 0x3FFFFFu;
            LOG(LOG_DEBUG, "Found a duplicate [%s]\n", callsign);
            return;
        }
        else
        {
            LOG(LOG_DEBUG, "Hash table clash!\n");
            // Move on to check the next entry in hash table
            idx_hash = (idx_hash + 1) % CALLSIGN_HASHTABLE_SIZE;
        }
    }
    callsign_hashtable_size++;
    strncpy(callsign_hashtable[idx_hash].callsign, callsign, 11);
    callsign_hashtable[idx_hash].callsign[11] = '\0';
    callsign_hashtable[idx_hash].hash = hash;
}

bool hashtable_lookup(ftx_callsign_hash_type_t hash_type, uint32_t hash, char* callsign)
{
    uint8_t hash_shift = (hash_type == FTX_CALLSIGN_HASH_10_BITS) ? 12 : (hash_type == FTX_CALLSIGN_HASH_12_BITS ? 10 : 0);
    uint16_t hash10 = (hash >> (12 - hash_shift)) & 0x3FFu;
    int idx_hash = (hash10 * 23) % CALLSIGN_HASHTABLE_SIZE;
    while (callsign_hashtable[idx_hash].callsign[0] != '\0')
    {
        if (((callsign_hashtable[idx_hash].hash & 0x3FFFFFu) >> hash_shift) == hash)
        {
            strcpy(callsign, callsign_hashtable[idx_hash].callsign);
            return true;
        }
        // Move on to check the next entry in hash table
        idx_hash = (idx_hash + 1) % CALLSIGN_HASHTABLE_SIZE;
    }
    callsign[0] = '\0';
    return false;
}

ftx_callsign_hash_interface_t hash_if = {
    .lookup_hash = hashtable_lookup,
    .save_hash = hashtable_add
};

bool is_token_char(char ch) {
	switch(ch) {
		case 0: // quick check for terminator: faster than isalnum(), perhaps
			return false;
		case '+':
		case '-':
		case '/':
			return true;
		default:
			return isalnum(ch);
	}
}

// like strncpy, but skips <> brackets (as found in hashed callsigns),
// stops at the end of alphanumeric characters plus -+/, and returns count copied
int tokncpy(char *dst, const char *src, size_t dsize){
	if (*src == '<')
		++src;
	int c = 0;
	for (; c < dsize - 1 && is_token_char(*src); ++c)
		*dst++ = *src++;
	*dst = 0;
	return c;
}

/// Computes a GFSK smoothing pulse.
/// The pulse is theoretically infinitely long, however, here it's truncated at 3 times the symbol length.
/// This means the pulse array has to have space for 3*n_spsym elements.
/// @param[in] n_spsym Number of samples per symbol
/// @param[in] b Shape parameter (values defined for FT8/FT4)
/// @param[out] pulse Output array of pulse samples
///
static void gfsk_pulse(int n_spsym, float symbol_bt, float* pulse)
{
    for (int i = 0; i < 3 * n_spsym; ++i)
    {
        float t = i / (float)n_spsym - 1.5f;
        float arg1 = GFSK_CONST_K * symbol_bt * (t + 0.5f);
        float arg2 = GFSK_CONST_K * symbol_bt * (t - 0.5f);
        pulse[i] = (erff(arg1) - erff(arg2)) / 2;
    }
}

/// Synthesize waveform data using GFSK phase shaping.
/// The output waveform will contain n_sym symbols.
/// @param[in] symbols Array of symbols (tones) (0-7 for FT8)
/// @param[in] n_sym Number of symbols in the symbol array
/// @param[in] f0 Audio frequency in Hertz for the symbol 0 (base frequency)
/// @param[in] symbol_bt Symbol smoothing filter bandwidth (2 for FT8, 1 for FT4)
/// @param[in] symbol_period Symbol period (duration), seconds
/// @param[in] signal_rate Sample rate of synthesized signal, Hertz
/// @param[out] signal Output array of signal waveform samples (should have space for n_sym*n_spsym samples)
///
static void synth_gfsk(const uint8_t* symbols, int n_sym, float f0, float symbol_bt, float symbol_period, int signal_rate, float* signal)
{
    int n_spsym = (int)(0.5f + signal_rate * symbol_period); // Samples per symbol
    int n_wave = n_sym * n_spsym;                            // Number of output samples
    float hmod = 1.0f;

    LOG(LOG_DEBUG, "n_spsym = %d\n", n_spsym);
    // Compute the smoothed frequency waveform.
    // Length = (nsym+2)*n_spsym samples, first and last symbols extended
    float dphi_peak = 2 * M_PI * hmod / n_spsym;
    float dphi[n_wave + 2 * n_spsym];

    // Shift frequency up by f0
    for (int i = 0; i < n_wave + 2 * n_spsym; ++i)
    {
        dphi[i] = 2 * M_PI * f0 / signal_rate;
    }

    float pulse[3 * n_spsym];
    gfsk_pulse(n_spsym, symbol_bt, pulse);

    for (int i = 0; i < n_sym; ++i)
    {
        int ib = i * n_spsym;
        for (int j = 0; j < 3 * n_spsym; ++j)
        {
            dphi[j + ib] += dphi_peak * symbols[i] * pulse[j];
        }
    }

    // Add dummy symbols at beginning and end with tone values equal to 1st and last symbol, respectively
    for (int j = 0; j < 2 * n_spsym; ++j)
    {
        dphi[j] += dphi_peak * pulse[j + n_spsym] * symbols[0];
        dphi[j + n_sym * n_spsym] += dphi_peak * pulse[j] * symbols[n_sym - 1];
    }

    // Calculate and insert the audio waveform
    float phi = 0;
    for (int k = 0; k < n_wave; ++k)
    { // Don't include dummy symbols
        signal[k] = sinf(phi);
        phi = fmodf(phi + dphi[k + n_spsym], 2 * M_PI);
    }

    // Apply envelope shaping to the first and last symbols
    int n_ramp = n_spsym / 8;
    for (int i = 0; i < n_ramp; ++i)
    {
        float env = (1 - cosf(2 * M_PI * i / (2 * n_ramp))) / 2;
        signal[i] *= env;
        signal[n_wave - 1 - i] *= env;
    }
}

/*!
	Encode ftx_tx_msg or ftx_xota_msg payload onto audio carrier \a freq and output to \a signal.
	@return the number of audio samples
*/
int sbitx_ftx_msg_audio(int32_t freq, float *signal)
{
	if (!freq)
		freq = field_int("TX_PITCH");
    float frequency = 1.0 * freq;

	bool is_ft4 = !strcmp(field_str("MODE"), "FT4");

	int num_tones = is_ft4 ? FT4_NN : FT8_NN;
	float symbol_period = is_ft4 ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
	float symbol_bt = is_ft4 ? FT4_SYMBOL_BT : FT8_SYMBOL_BT;
	float slot_time = is_ft4 ? FT4_SLOT_TIME : FT8_SLOT_TIME;

    // Second, encode the binary message as a sequence of FSK tones
    uint8_t tones[num_tones]; // Array of 79 tones (symbols)
    if (is_ft4)
        ft4_encode(ftx_xota ? ftx_xota_msg.payload : ftx_tx_msg.payload, tones);
    else
        ft8_encode(ftx_xota ? ftx_xota_msg.payload : ftx_tx_msg.payload, tones);

    // Third, convert the FSK tones into an audio signal
    int sample_rate = 12000;
    int num_samples = (int)(0.5f + num_tones * symbol_period * sample_rate); // samples in the data signal
    int num_silence = (slot_time * sample_rate - num_samples) / 2;           // Silence  to make 15 seconds
    int num_total_samples = num_silence + num_samples + num_silence;         // total Number samples

    for (int i = 0; i < num_silence; i++) {
        signal[i] = 0;
        signal[i + num_samples + num_silence] = 0;
    }

    // Synthesize waveform data (signal) and save it as WAV file
	LOG(LOG_DEBUG, "%05d %s '%s' synth_gfsk %d %f %f %f %d samples %d silence %d\n",
		wallclock_day_ms % 60000, (is_ft4 ? "FT4" : "FT8"), ftx_xota ? ftx_xota_text : ftx_tx_text, num_tones,
		frequency, symbol_bt, symbol_period, sample_rate, num_samples, num_silence);
    synth_gfsk(tones, num_tones, frequency, symbol_bt, symbol_period, sample_rate, signal + num_silence);
    return num_total_samples;
}

/*!
	Encode \a message into ftx_tx_msg.
	This should only be used when the user has typed \a message;
	for programmatic cases, prefer sbitx_ft8_encode_3f'()
	@return the return value from ftx_message_encode (see enum ftx_message_rc_t in ft8_lib/message.h)
*/
int sbitx_ft8_encode(char *message)
{
    ftx_message_rc_t rc = ftx_message_encode(&ftx_tx_msg, &hash_if, message);
    if (rc != FTX_MESSAGE_RC_OK)
        printf("Cannot encode FTx message! RC = %d\n", (int)rc);

	return rc;
}

/*!
	Compose a message from the 3 fields \a call_to, \a call_de and \a extra into ftx_tx_msg.
	@return the return value from ftx_message_encode_std/nonstd/free
	(see enum ftx_message_rc_t in ft8_lib/message.h)
*/
int sbitx_ft8_encode_3f(const char* call_to, const char* call_de, const char* extra)
{
	ftx_message_rc_t rc = ftx_message_encode_std(&ftx_tx_msg, &hash_if, call_to, call_de, extra);
	if (rc != FTX_MESSAGE_RC_OK) {
		LOG(LOG_DEBUG, "   ftx_message_encode_std failed: %d\n", rc);
		rc = ftx_message_encode_nonstd(&ftx_tx_msg, &hash_if, call_to, call_de, extra);
		if (rc != FTX_MESSAGE_RC_OK) {
			LOG(LOG_DEBUG, "   ftx_message_encode_nonstd failed: %d\n", rc);
			rc = ftx_message_encode_free(&ftx_tx_msg, ftx_tx_text);
		}
	}

    if (rc != FTX_MESSAGE_RC_OK)
        printf("Cannot encode FTx 3-field message! RC = %d\n", (int)rc);

	return rc;
}

static float hann_i(int i, int N)
{
    float x = sinf((float)M_PI * i / N);
    return x * x;
}

static float hamming_i(int i, int N)
{
    const float a0 = (float)25 / 46;
    const float a1 = 1 - a0;

    float x1 = cosf(2 * (float)M_PI * i / N);
    return a0 - a1 * x1;
}

static float blackman_i(int i, int N)
{
    const float alpha = 0.16f; // or 2860/18608
    const float a0 = (1 - alpha) / 2;
    const float a1 = 1.0f / 2;
    const float a2 = alpha / 2;

    float x1 = cosf(2 * (float)M_PI * i / N);
    float x2 = 2 * x1 * x1 - 1; // Use double angle formula

    return a0 - a1 * x1 + a2 * x2;
}

void waterfall_init(ftx_waterfall_t* me, int max_blocks, int num_bins, int time_osr, int freq_osr)
{
    size_t mag_size = max_blocks * time_osr * freq_osr * num_bins * sizeof(me->mag[0]);
    me->max_blocks = max_blocks;
    me->num_blocks = 0;
    me->num_bins = num_bins;
    me->time_osr = time_osr;
    me->freq_osr = freq_osr;
    me->block_stride = (time_osr * freq_osr * num_bins);
    me->mag = (uint8_t  *)malloc(mag_size);
    LOG(LOG_DEBUG, "Waterfall size = %zu\n", mag_size);
}

void waterfall_free(ftx_waterfall_t* me)
{
    free(me->mag);
}

/// Configuration options for FT4/FT8 monitor
typedef struct
{
    float f_min;             ///< Lower frequency bound for analysis
    float f_max;             ///< Upper frequency bound for analysis
    int sample_rate;         ///< Sample rate in Hertz
    int time_osr;            ///< Number of time subdivisions
    int freq_osr;            ///< Number of frequency subdivisions
    ftx_protocol_t protocol; ///< Protocol: FT4 or FT8
} monitor_config_t;

/// FT4/FT8 monitor object that manages DSP processing of incoming audio data
/// and prepares a waterfall object
typedef struct
{
    float symbol_period; ///< FT4/FT8 symbol period in seconds
    int block_size;      ///< Number of samples per symbol (block)
    int subblock_size;   ///< Analysis shift size (number of samples)
    int nfft;            ///< FFT size
    float fft_norm;      ///< FFT normalization factor
    float* window;       ///< Window function for STFT analysis (nfft samples)
    float* last_frame;   ///< Current STFT analysis frame (nfft samples)
    ftx_waterfall_t wf;      ///< Waterfall object
    float max_mag;       ///< Maximum detected magnitude (debug stats)

    // KISS FFT housekeeping variables
    void* fft_work;        ///< Work area required by Kiss FFT
    kiss_fftr_cfg fft_cfg; ///< Kiss FFT housekeeping object
} monitor_t;

static void monitor_init(monitor_t* me, const monitor_config_t* cfg)
{
    float slot_time = (cfg->protocol == FTX_PROTOCOL_FT4) ? FT4_SLOT_TIME : FT8_SLOT_TIME;
    float symbol_period = (cfg->protocol == FTX_PROTOCOL_FT4) ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
    // Compute DSP parameters that depend on the sample rate
    me->block_size = (int)(cfg->sample_rate * symbol_period); // samples corresponding to one FSK symbol
    me->subblock_size = me->block_size / cfg->time_osr;
    me->nfft = me->block_size * cfg->freq_osr;
    me->fft_norm = 2.0f / me->nfft;
    // const int len_window = 1.8f * me->block_size; // hand-picked and optimized

    me->window = (float *)malloc(me->nfft * sizeof(me->window[0]));
    for (int i = 0; i < me->nfft; ++i)
    {
        // window[i] = 1;
        me->window[i] = hann_i(i, me->nfft);
        // me->window[i] = blackman_i(i, me->nfft);
        // me->window[i] = hamming_i(i, me->nfft);
        // me->window[i] = (i < len_window) ? hann_i(i, len_window) : 0;
    }
    me->last_frame = (float *)malloc(me->nfft * sizeof(me->last_frame[0]));

    size_t fft_work_size;
    kiss_fftr_alloc(me->nfft, 0, 0, &fft_work_size);

    //LOG(LOG_INFO, "Block size = %d\n", me->block_size);
    //LOG(LOG_INFO, "Subblock size = %d\n", me->subblock_size);
    //LOG(LOG_INFO, "N_FFT = %d\n", me->nfft);
    LOG(LOG_DEBUG, "FFT work area = %zu\n", fft_work_size);

    me->fft_work = malloc(fft_work_size);
    me->fft_cfg = kiss_fftr_alloc(me->nfft, 0, me->fft_work, &fft_work_size);

    const int max_blocks = (int)(slot_time / symbol_period);
    const int num_bins = (int)(cfg->sample_rate * symbol_period / 2);
    waterfall_init(&me->wf, max_blocks, num_bins, cfg->time_osr, cfg->freq_osr);
    me->wf.protocol = cfg->protocol;
    me->symbol_period = symbol_period;

    me->max_mag = -120.0f;
}

static void monitor_free(monitor_t* me)
{
    waterfall_free(&me->wf);
    free(me->fft_work);
    free(me->last_frame);
    free(me->window);
}

// Compute FFT magnitudes (log wf) for a frame in the signal and update waterfall data
static void monitor_process(monitor_t* me, const float* frame)
{
    // Check if we can still store more waterfall data
    if (me->wf.num_blocks >= me->wf.max_blocks)
        return;

    int offset = me->wf.num_blocks * me->wf.block_stride;
    int frame_pos = 0;

    // Loop over block subdivisions
    for (int time_sub = 0; time_sub < me->wf.time_osr; ++time_sub)
    {
        kiss_fft_scalar timedata[me->nfft];
        kiss_fft_cpx freqdata[me->nfft / 2 + 1];

        // Shift the new data into analysis frame
        for (int pos = 0; pos < me->nfft - me->subblock_size; ++pos)
        {
            me->last_frame[pos] = me->last_frame[pos + me->subblock_size];
        }
        for (int pos = me->nfft - me->subblock_size; pos < me->nfft; ++pos)
        {
            me->last_frame[pos] = frame[frame_pos];
            ++frame_pos;
        }

        // Compute windowed analysis frame
        for (int pos = 0; pos < me->nfft; ++pos)
        {
            timedata[pos] = me->fft_norm * me->window[pos] * me->last_frame[pos];
        }

        kiss_fftr(me->fft_cfg, timedata, freqdata);

        // Loop over two possible frequency bin offsets (for averaging)
        for (int freq_sub = 0; freq_sub < me->wf.freq_osr; ++freq_sub)
        {
            for (int bin = 0; bin < me->wf.num_bins; ++bin)
            {
                int src_bin = (bin * me->wf.freq_osr) + freq_sub;
                float mag2 = (freqdata[src_bin].i * freqdata[src_bin].i) + (freqdata[src_bin].r * freqdata[src_bin].r);
                float db = 10.0f * log10f(1E-12f + mag2);
                // Scale decibels to unsigned 8-bit range and clamp the value
                // Range 0-240 covers -120..0 dB in 0.5 dB steps
                int scaled = (int)(2 * db + 240);

                me->wf.mag[offset] = (scaled < 0) ? 0 : ((scaled > 255) ? 255 : scaled);
                ++offset;

                if (db > me->max_mag)
                    me->max_mag = db;
            }
        }
    }

    ++me->wf.num_blocks;
}

static int message_callsign_count(const ftx_message_offsets_t *spans)
{
	int ret = 0;
	for (int i = 0; i < FTX_MAX_MESSAGE_FIELDS; ++i)
		if (spans->types[i] == FTX_FIELD_CALL)
			++ret;
	return ret;
}

static int message_last_span_offset(const ftx_message_offsets_t *spans, ftx_field_t type)
{
	int ret = -1;
	for (int i = 0; i < FTX_MAX_MESSAGE_FIELDS; ++i)
		if (spans->types[i] == type)
			ret = spans->offsets[i];
	return ret;
}

static int sbitx_ft8_decode(float *signal, int num_samples)
{
    int sample_rate = 12000;
	bool is_ft8 = !strcmp(field_str("MODE"), "FT8");
	recent_qso_age = field_int("RCT_QSO_AGE");

    LOG(LOG_DEBUG, "sbitx_ftx_decode: %s sample rate %d Hz, %d samples, %.3f seconds\n",
		(is_ft8 ? "FT8" : "FT4"), sample_rate, num_samples, (double)num_samples / sample_rate);

    // Compute FFT over the whole signal and store it
    monitor_t mon;
    monitor_config_t mon_cfg = {
        .f_min = 100,
        .f_max = 3000,
        .sample_rate = sample_rate,
        .time_osr = kTime_osr,
        .freq_osr = kFreq_osr,
        .protocol = is_ft8 ? FTX_PROTOCOL_FT8 : FTX_PROTOCOL_FT4
    };

	// timestamp the packets
	// the time is shifted back by the time it took to capture these samples
	const int packet_time_ms = is_ft8 ? 15000 : 7500;
	const int raw_ms = (wallclock_day_ms / packet_time_ms) * packet_time_ms;
	time_t now = time(NULL);

	int i;
	char mycallsign_upper[20];
	char mycallsign[20];
	get_field_value("#mycallsign", mycallsign);
	for (i = 0; i < strlen(mycallsign); i++)
		mycallsign_upper[i] = toupper(mycallsign[i]);
	mycallsign_upper[i] = 0;

    monitor_init(&mon, &mon_cfg);

    // Process the waveform data frame by frame - you could have a live loop here with data from an audio device
    for (int frame_pos = 0; frame_pos + mon.block_size <= num_samples; frame_pos += mon.block_size)
        monitor_process(&mon, signal + frame_pos);

//    LOG(LOG_DEBUG, "Waterfall accumulated %d symbols\n", mon.wf.num_blocks);
//    LOG(LOG_INFO, "Max magnitude: %.1f dB\n", mon.max_mag);

    // Find top candidates by Costas sync score and localize them in time and frequency
    ftx_candidate_t candidate_list[kMax_candidates];
    int num_candidates = ftx_find_candidates(&mon.wf, kMax_candidates, candidate_list, kMin_score);

    // Hash table for decoded messages (to check for duplicates)
	typedef struct
	{
		int time_sec; // second within the minute
		int snr;
		int pitch;
		int last_qso_age;
		int grid_last_qso_age;
		char text[FTX_MAX_MESSAGE_LENGTH]; // message text as decoded
		ftx_message_offsets_t spans; // locations/lengths of fields in text
		ftx_message_t message; // encoded form
	} decoded_message_t;

	int num_decoded = 0;
    decoded_message_t decoded[kMax_decoded_messages];
    decoded_message_t* decoded_hashtable[kMax_decoded_messages];

    // Initialize hash table pointers
    for (int i = 0; i < kMax_decoded_messages; ++i)
        decoded_hashtable[i] = NULL;

	int n_decodes = 0;
	int crc_mismatches = 0;

    // Go over candidates and attempt to decode messages
    for (int idx = 0; idx < num_candidates; ++idx)
    {
        const ftx_candidate_t* cand = &candidate_list[idx];
        if (cand->score < kMin_score)
            continue;

        int freq_hz = lroundf((cand->freq_offset + (float)cand->freq_sub / mon.wf.freq_osr) / mon.symbol_period);
		//~ printf("freq_hz: (%d + %d / %d) / %f = %d\n", cand->freq_offset, cand->freq_sub, mon.wf.freq_osr, mon.symbol_period, freq_hz);
        float time_sec = (cand->time_offset + (float)cand->time_sub / mon.wf.time_osr) * mon.symbol_period;

        ftx_message_t message;
        ftx_decode_status_t status;
        if (!ftx_decode_candidate(&mon.wf, cand, kLDPC_iterations, &message, &status)){
            // printf("000000 %3d %+4.2f %4.0f ~  ---\n", cand->score, time_sec, freq_hz);
			if (status.crc_calculated != status.crc_extracted)
				++crc_mismatches;
            //~ else if (status.ldpc_errors > 0)
                //~ LOG(LOG_DEBUG, "LDPC decode: %d errors\n", status.ldpc_errors);
            continue;
        }

        LOG(LOG_DEBUG, "Checking hash table for %4.1fs / %dHz [%d]...\n", time_sec, freq_hz, cand->score);
        int idx_hash = message.hash % kMax_decoded_messages;
        bool found_empty_slot = false;
        bool found_duplicate = false;
        do {
            if (decoded_hashtable[idx_hash] == NULL) {
                LOG(LOG_DEBUG, "Found an empty slot\n");
                found_empty_slot = true;
            }
            else if ((decoded_hashtable[idx_hash]->message.hash == message.hash) &&
			         (0 == memcmp(decoded_hashtable[idx_hash]->message.payload, message.payload, FTX_PAYLOAD_LENGTH_BYTES))) {
				//~ ftx_message_print(&message);
                LOG(LOG_DEBUG, "Found a duplicate\n");
                found_duplicate = true;
            }
            else {
                LOG(LOG_DEBUG, "Hash table clash!\n");
                // Move on to check the next entry in hash table
                idx_hash = (idx_hash + 1) % kMax_decoded_messages;
            }
        } while (!found_empty_slot && !found_duplicate);

        if (found_empty_slot) {
			// Fill the empty hashtable slot
			memcpy(&decoded[idx_hash].message, &message, sizeof(message));
			decoded_hashtable[idx_hash] = &decoded[idx_hash];
			++num_decoded;

			char text[FTX_MAX_MESSAGE_LENGTH];
			ftx_message_offsets_t spans;
            ftx_message_rc_t unpack_status = ftx_message_decode(&message, &hash_if, text, &spans);
            if (unpack_status != FTX_MESSAGE_RC_OK)
                LOG(LOG_DEBUG, "Error [%d] while unpacking!", (int)unpack_status);
			strncpy(decoded[idx_hash].text, text, FTX_MAX_MESSAGE_LENGTH);
			decoded[idx_hash].time_sec = (raw_ms % 60000) / 1000;
			decoded[idx_hash].snr = cand->snr;
			decoded[idx_hash].pitch = freq_hz;
			decoded[idx_hash].spans = spans;
			decoded[idx_hash].last_qso_age = -1;
			decoded[idx_hash].grid_last_qso_age = -1;

			char buf[64];
			int prefix_len = 8 + snprintf(hmst_time_sprint(buf, raw_ms), sizeof(buf) - 8, " %3d %+03d %4d ~ ", cand->score, cand->snr, freq_hz);
			int line_len = prefix_len + snprintf(buf + prefix_len, sizeof(buf) - prefix_len, "%s", text);

			//For troubleshooting you can display the time offset - n1qm
			//sprintf(buff, "%s %d %+03d %-4.0f ~  %s\n", time_str, cand->time_offset,
			//  cand->snr, freq_hz, message.payload);

			text_span_semantic sem[MAX_CONSOLE_LINE_STYLES];
			memset(sem, 0, sizeof(sem));
			char callsign[20];
			memset(callsign, 0, sizeof(callsign));
			bool my_call_found = false;
			int calls_found = 0;
			int total_calls = message_callsign_count(&spans);
			int span_i = 0;
			int sem_i = 0;
			int col = 0;
			sem[sem_i].length = line_len;
			sem[sem_i++].semantic = STYLE_FT8_RX;
			sem[sem_i].length = 8;
			sem[sem_i++].semantic = STYLE_TIME;
			col = 8 + 5; // skip "score"
			sem[sem_i].start_column = col;
			sem[sem_i].length = 3;
			sem[sem_i++].semantic = STYLE_SNR;
			col += 4;
			sem[sem_i].start_column = col;
			sem[sem_i].length = 4;
			sem[sem_i++].semantic = STYLE_FREQ;

			for (; span_i < FTX_MAX_MESSAGE_FIELDS && sem_i < MAX_CONSOLE_LINE_STYLES &&
					spans.offsets[span_i] >= 0; ++span_i, ++sem_i) {
				sem[sem_i].start_column = prefix_len + spans.offsets[span_i];
				// each span ends where the next starts (ftx_message_offsets_t does not have lengths, so far)
				if (sem_i > 4) {
					sem[sem_i - 1].length = sem[sem_i].start_column - sem[sem_i - 1].start_column;
					// Now that we know the length of the previous field, check for special cases to change style
					switch (sem[sem_i - 1].semantic) {
						case STYLE_CALLER: {
							// Have we had a recent QSO with this caller?
							int start = sem[sem_i - 1].start_column;
							int len = sem[sem_i - 1].length;
							if (buf[start] == ' ')
								++start;
							while (!buf[start + len - 1] || buf[start + len - 1] == ' ')
								--len;
							strncpy(callsign, buf + start, len);
							callsign[len] = 0;
							time_t recent_qso = len > 0 ? logbook_last_qso(callsign, len) : 0ll;
							//~ printf("checking for recent QSO: callsign is at text range %d len %d from '%s': '%s'; last qso @ %lld, %lld secs ago; limit is %d hours\n",
								//~ start, len, buf, callsign, recent_qso, now - recent_qso, recent_qso_age);
							decoded[idx_hash].last_qso_age = recent_qso ? (now - recent_qso) / 60 / 60 : -1; // convert seconds to hours
							if (recent_qso && decoded[idx_hash].last_qso_age < recent_qso_age)
								sem[sem_i - 1].semantic = STYLE_RECENT_CALLER;
							break;
						}
					}
					//~ printf("span %d: start %d len %d - %d = %d; style %d\n", sem_i - 1,
						//~ sem[sem_i].start_column, sem[sem_i].start_column, sem[sem_i - 1].start_column,
						//~ sem[sem_i - 1].length, sem[sem_i - 1].semantic);
				}
				if (spans.types[span_i] == FTX_FIELD_CALL) {
					// detect whether it's my callsign or the caller's
					char *call = text + spans.offsets[span_i];
					char *call_end = strchr(call, ' ');
					if (!call_end)
						call_end = call + strlen(call);
					assert(call_end);
					if (*call == '<')
						++call;
					if (*(call_end - 1) == '>')
						--call_end;
					//~ printf("considering call %d of %d: first %d chars of %s\n", calls_found, total_calls, call_end - call, call);
					if (!strncmp(call, mycallsign_upper, call_end - call)) {
						sem[sem_i].semantic = STYLE_MYCALL;
						my_call_found = true;
					} else if (!calls_found && total_calls > 1) {
						// the first callsign is the callee, unless it's a single-call message (such as CQ):
						// less interesting then, unless it's my call
						sem[sem_i].semantic = STYLE_CALLEE;
					} else {
						// otherwise the callsign is presumably the caller
						// (since we don't support multi-part messages yet)
						sem[sem_i].semantic = STYLE_CALLER;
					}
					++calls_found;
					continue; // with the for loop, so as to skip the next line below
				}
				sem[sem_i].semantic = kFieldType_style_map[spans.types[span_i]];
			}
			// set length of the last span (no next span, but null terminator in text)
			if (span_i > 0) {
				sem[sem_i - 1].length = strlen(text + spans.offsets[span_i - 1]);
				//~ printf("final span '%s' has len %d sem %d\n", text + spans.offsets[span_i - 1], sem[sem_i - 1].length, sem[sem_i - 1].semantic);
				switch (sem[sem_i - 1].semantic) {
					case STYLE_FT8_RX: {
						// RR73 and RRR should not stand out.
						if (!strncmp(buf + sem[sem_i - 1].start_column, "RR73", 4) || !strncmp(buf + sem[sem_i - 1].start_column, "RRR", 4)) {
							sem[sem_i - 1].semantic = STYLE_LOG;
							break;
						}
					}
					case STYLE_GRID: {
						// If RR73 got tagged as a grid, it's not a grid.
						if (!strncmp(buf + sem[sem_i - 1].start_column, "RR73", 4)) {
							sem[sem_i - 1].semantic = STYLE_LOG; // should not stand out
							break;
						}
						// When was the last QSO with someone in this grid?
						time_t recent_grid_qso = sem[sem_i - 1].length > 0 ? logbook_grid_last_qso(buf + sem[sem_i - 1].start_column, sem[sem_i - 1].length) : 0ll;
						//~ printf("checking for QSO: grid is at text range %d len %d from '%s'; exists? %d\n",
							//~ sem[sem_i - 1].start_column, sem[sem_i - 1].length, buf, exists);
						decoded[idx_hash].grid_last_qso_age = recent_grid_qso ? (now - recent_grid_qso) / 60 / 60 : -1; // convert seconds to hours
						if (recent_grid_qso)
							sem[sem_i - 1].semantic = STYLE_EXISTING_GRID;
						break;
					}
					case STYLE_CALLER: {
						// The line could end with the callsign if it's a non-standard call (no grid then).
						// Have we had a recent QSO with this caller?
						int start = sem[sem_i - 1].start_column;
						int len = sem[sem_i - 1].length;
						if (buf[start] == ' ')
							++start;
						while (!buf[start + len - 1] || buf[start + len - 1] == ' ')
							--len;
						strncpy(callsign, buf + start, len);
						callsign[len] = 0;
						time_t recent_qso = len > 0 ? logbook_last_qso(callsign, len) : 0ll;
						decoded[idx_hash].last_qso_age = recent_qso ? (now - recent_qso) / 60 / 60 : -1; // convert seconds to hours
						if (recent_qso && decoded[idx_hash].last_qso_age < recent_qso_age)
							sem[sem_i - 1].semantic = STYLE_RECENT_CALLER;
						break;
					}
				}
			}
			const int message_type = ftx_message_get_i3(&message);
			if (decoded[idx_hash].last_qso_age < 0 && decoded[idx_hash].grid_last_qso_age < 0) {
				LOG(LOG_INFO, "<< %d.%c       %s\n",
					message_type, message_type ? ' ' : '0' + ftx_message_get_n3(&message), buf);
			} else {
				LOG(LOG_INFO, "<< %d.%c       %s; last QSO with %s was %d hours ago, with grid %d hours ago\n",
					message_type, message_type ? ' ' : '0' + ftx_message_get_n3(&message),
					buf, callsign, decoded[idx_hash].last_qso_age, decoded[idx_hash].grid_last_qso_age);
			}
			buf[line_len++] = '\n';
			buf[line_len] = 0;
			write_console_semantic(buf, sem, sem_i);

			if (my_call_found)
				ftx_call_or_continue(buf, line_len, sem);
			n_decodes++;
        }
    }
	if (crc_mismatches)
		LOG(LOG_DEBUG, "Decoded %d messages; %d CRC mismatches\n", num_decoded, crc_mismatches);

    // Here we have a populated hash table with the decoded messages
    // If we are in autorespond mode and in idle state (i.e. no message planned to transmit),
    //  we would like to answer to a CQ call
    // This simple implementation just answers to a random CQ call (if any)
    //  by scanning sequentially the hash table until one is found that
    //  has not been answered since the start of the sbitx program,
    //  with a max number of entries stored into a circular buffer
    // In the future, this could be made more sophisticated, with blacklists or
    //  prioritization criteria or querying the log
    // In theory, we could also start a CQ ourselves,
    //  however this would not be consistent with the idea of auto responder that
    // is shown in the GUI. We may consider this option in the future,
    //  and make it behave more like FT8CN (i.e. a sort of
    // completely autonomous ft8 bot), according to the preferences of the user
	// If that gets done, we can add ROBOT to the list of modes for FTX_AUTO (see sbitx_gtk.c:1094)
	const bool cq_auto_respond = !strcmp(field_str("FTX_AUTO"), "CQRESP");
	if (cq_auto_respond && ftx_tx_text[0])
		LOG(LOG_DEBUG, "skipping auto-responder because of queued message '%s'\n", ftx_tx_text);
	if (cq_auto_respond && !ftx_tx_text[0]) {
		int cand_time_sec = -1;
		int cand_snr = -100;
		int cand_pitch = -1;
		bool prioritized = false;
		char *cand_text = NULL;
		char cand_callsign[12];
		char cand_exch[12];
		memset(cand_callsign, 0, sizeof(cand_callsign));
		memset(cand_exch, 0, sizeof(cand_exch));

		for (int idx = 0; idx < kMax_decoded_messages; idx++) {
			if (decoded_hashtable[idx] && decoded_hashtable[idx]->text) {
				int de_offset = message_last_span_offset(&decoded_hashtable[idx]->spans, FTX_FIELD_CALL);
				if (de_offset < 0)
					continue;
				if (strncmp(decoded_hashtable[idx]->text, "CQ ", 3))
					continue; // ignore it if it's not a CQ
				char callsign[12];
				tokncpy(callsign, decoded_hashtable[idx]->text + de_offset, sizeof(callsign));
				// if our last QSO with this callsign was too recent, don't call again
				if (decoded_hashtable[idx]->last_qso_age >= 0 && decoded_hashtable[idx]->last_qso_age < recent_qso_age) {
					LOG(LOG_DEBUG, "Skipping %s: age %d hours is too recent\n",
						callsign, decoded_hashtable[idx]->last_qso_age);
					continue;
				}

				// Prioritize xOTA, /QRP and /P
				if (!strncmp(decoded_hashtable[idx]->text + 4, "OTA ", 4) ||
					 strstr(callsign, "/QRP") || strstr(callsign, "/P") ) {

					// We try to avoid calling automatically the same stations again and again, at least in this session
					bool found = false;
					for (int ii = 0; ii < MIN(ftx_already_called_n, FTX_CALLED_SIZE); ii++) {
						if (!strcmp(callsign, ftx_already_called[ii]))
							found = true;
					}

					if (!found) {
						cand_time_sec = decoded_hashtable[idx]->time_sec;
						cand_snr = decoded_hashtable[idx]->snr;
						cand_pitch = decoded_hashtable[idx]->pitch;
						cand_text = decoded_hashtable[idx]->text;
						strncpy(cand_callsign, callsign, sizeof(cand_callsign));
						int grid_offset = message_last_span_offset(&decoded_hashtable[idx]->spans, FTX_FIELD_GRID);
						if (grid_offset > 0)
							tokncpy(cand_exch, decoded_hashtable[idx]->text + grid_offset, sizeof(cand_exch));
						prioritized = true;
					}
					break;
				}

				// Otherwise, answer a plain CQ
				if (!cand_text) {
					// We try to avoid calling automatically the same stations again and again, at least in this session
					bool found = false;
					for (int ii = 0; ii < MIN(ftx_already_called_n, FTX_CALLED_SIZE); ii++) {
						if (!strcmp(callsign, ftx_already_called[ii]))
							found = true;
					}

					if (!found) {
						cand_time_sec = decoded_hashtable[idx]->time_sec;
						cand_snr = decoded_hashtable[idx]->snr;
						cand_pitch = decoded_hashtable[idx]->pitch;
						cand_text = decoded_hashtable[idx]->text;
						strncpy(cand_callsign, callsign, sizeof(cand_callsign));
						int grid_offset = message_last_span_offset(&decoded_hashtable[idx]->spans, FTX_FIELD_GRID);
						if (grid_offset > 0)
							tokncpy(cand_exch, decoded_hashtable[idx]->text + grid_offset, sizeof(cand_exch));
						prioritized = false;
					}
				}
			}
		}

		if (cand_text) {
			field_set("CALL", cand_callsign);
			field_set("EXCH", cand_exch);
			{
				// don't use set_field_int for RST: we need a custom format with explicit sign and 2 digits
				char buf[8];
				snprintf(buf, sizeof(buf), "%+03d", cand_snr);
				field_set("SENT", buf);
			}
			set_field_int("ftx_rx_pitch", cand_pitch);
			ft8_call(cand_time_sec); // decide in which slot to transmit, etc.
			LOG(LOG_INFO, "Auto-responding in %s slot to %s'%s' @ '%s' from t %d snr %d f %d '%s'\n",
				ftx_tx1st ? "even" : "odd", prioritized ? "prioritized " : "", cand_callsign,
				cand_exch, cand_time_sec, cand_snr, cand_pitch, cand_text);
			strcpy(ftx_already_called[ftx_already_called_n % FTX_CALLED_SIZE], cand_callsign);
			ftx_already_called_n++;
		}
	}

    monitor_free(&mon);
    hashtable_cleanup(10);

    return n_decodes;
}

static bool encode_xota() {
	const char *xota = field_str("xOTA");
	const char *xota_loc = field_str("LOCATION");
	if (!xota[0] || !xota_loc[0] || !strcmp(xota, "NONE"))
		return false;
	else {
		snprintf(ftx_xota_text, sizeof(ftx_xota_text), "%c%c %s", xota[0], xota[1], xota_loc);
		LOG(LOG_DEBUG, "%05d encode_xota %s '%s'\n", wallclock_day_ms % 60000, xota, ftx_xota_text);
		ftx_message_rc_t rc = ftx_message_encode_free(&ftx_xota_msg, ftx_xota_text);
		if (rc != FTX_MESSAGE_RC_OK)
			LOG(LOG_INFO, "failed to encode xOTA message '%s': %d\n", ftx_xota_text, rc);
	}
	return true;
}

/*!
	Returns \c true if we have anything to send at this moment:
	is ftx_tx_text set and is it the right time to start?
	If so, it updates ftx_pitch and is_cq.
	If ftx_tx_text is a CQ, then it also updates ftx_tx1st, ftx_cq_alt, ftx_xota and ftx_xota_msg
	according to current settings.
*/
static bool ftx_would_send() {
	ftx_update_clock();
	if (!ftx_tx_text[0])
		return false; // nothing to send
	bool start = false;
	is_cq = !strncmp(ftx_tx_text, "CQ ", 3);
	bool is_ft4 = !strcmp(field_str("MODE"), "FT4");
	const char *ftx_cq = field_str("FTX_CQ");
	int slot_time = 0;

	ftx_pitch = field_int("TX_PITCH");

	// the FTX_CQ setting applies only to initiating a CQ call;
	// otherwise, leave ftx_tx1st as set earlier, e.g. in ftx_call_or_continue()
	if (is_cq) {
		// if it's not odd, it's even (slot 0/2)
		ftx_tx1st = strcmp(ftx_cq, "ODD");
		ftx_cq_alt = !strcmp(ftx_cq, "ALT_EVEN");
		ftx_xota = !strcmp(ftx_cq, "XOTA");
	} else {
		ftx_xota = false;
	}

	if (ftx_xota && !ftx_xota_text[0])
		encode_xota();

	if (is_ft4) {
		int two_slot_clock = wallclock_day_ms % 15000;
		int four_slot_clock = wallclock_day_ms % 30000;
		if (two_slot_clock < 7500) {
			if (ftx_tx1st)
				start = true;
		} else {
			if (!ftx_tx1st)
				start = true;
		}
		// if we have a timeslot based on even/odd setting, and we would otherwise send CQ,
		// then decide what to send, or to skip it, in case of xOTA or CQ_alt settings respectively
		if (start) {
			if (is_cq && four_slot_clock > 10000) {
				// wait until next minute for CQ; either send ftx_xota_text instead, or stay silent
				if (ftx_xota) { // set from the setting, above
					start = encode_xota(); // generate ftx_xota_msg
				} else if (ftx_cq_alt) {
					start = false;
				}
			} else {
				ftx_xota = false; // regular message this time
			}
		}
	} else {
		int two_slot_clock = wallclock_day_ms % 30000;
		int four_slot_clock = wallclock_day_ms % 60000;
		if (two_slot_clock < 15000) {
			if (ftx_tx1st)
				start = true;
		} else {
			if (!ftx_tx1st)
				start = true;
		}
		// if we have a timeslot based on even/odd setting, and we would otherwise send CQ,
		// then decide what to send, or to skip it, in case of xOTA or CQ_alt settings respectively
		if (start) {
			if (is_cq && four_slot_clock > 20000) {
				// wait until next minute for CQ; either send ft8_xota_text instead, or stay silent
				if (ftx_xota) { // set from the setting, above
					start = encode_xota(); // generate ftx_xota_msg
				} else if (ftx_cq_alt) {
					start = false;
				}
			} else {
				ftx_xota = false; // regular message this time
			}
		}
	}
	//~ printf("--- %d cq setting '%s' 1st %d alt %d xota %d '%s': start? %d\n",
		//~ wallclock_day_ms % 60000, ftx_cq, ftx_tx1st, ftx_cq_alt, ftx_xota, ftx_xota_text, start);
	return start;
}

static void ftx_start_tx(int offset_ms){
	char buf[100];
	int freq = field_int("TX_PITCH");
	if (freq != ftx_pitch)
		ftx_pitch = freq;
	ftx_tx_nsamples = sbitx_ftx_msg_audio(freq, ftx_tx_buffer);

	snprintf(hmst_wallclock_time_sprint(buf), sizeof(buf) - 8, "  TX     %4d ~ %s\n",
		ftx_pitch, ftx_xota ? ftx_xota_text : ftx_tx_text);
	write_console(STYLE_FT8_TX, buf);

	const int message_type = ftx_message_get_i3(&ftx_tx_msg);
	LOG(LOG_INFO, ">> %d.%c rpt %d %s", message_type,
		message_type ? ' ' : '0' + ftx_message_get_n3(&ftx_tx_msg), ftx_repeat, buf);

	// start at the beginning if at all reasonable
	if (offset_ms < 1000)
		offset_ms = 0;
	ftx_tx_buff_index = offset_ms * 96;
	LOG(LOG_DEBUG, "%05d ftx_start_tx: starting @index %d based on offset_ms %d '%s'\n",
		wallclock_day_ms % 60000, ftx_tx_buff_index, offset_ms, ftx_xota ? ftx_xota_text : ftx_tx_text);
}

/*!
	Encode and schedule \a message for transmission, modulated on \a freq.
	It is picked up by ft8_poll to do the actual transmission.
	\a message may be anything: ft8_lib has to parse it and guess the message type to use.
	So it's better to call ft8_tx_3f(to, de, extra) in all programmatic cases,
	and use this function only when the user is doing the typing.
*/
void ft8_tx(char *message, int freq){
	char buf[64];

	ftx_tx_text[0] = 0;
	for (int i = 0; i < strlen(message); i++)
		message[i] = toupper(message[i]);
	if (sbitx_ft8_encode(message) != FTX_MESSAGE_RC_OK) {
		LOG(LOG_INFO, "failed to encode: nothing to transmit\n");
		return;
	}

	strncpy(ftx_tx_text, message, sizeof(ftx_tx_text));
	const int message_type = ftx_message_get_i3(&ftx_tx_msg);
	const bool send_now = ftx_would_send(); // update wallclock_day_ms, ftx_pitch, ftx_tx1st, ftx_cq_alt, ftx_xota, ftx_xota_text
	if (!freq)
		freq = ftx_pitch;
	snprintf(hmst_wallclock_time_sprint(buf), sizeof(buf) - 8,
		"  TX     %4d ~ %s\n", freq, ftx_xota ? ftx_xota_text : ftx_tx_text);
	if (!send_now)
		write_console(STYLE_FT8_QUEUED, buf);

	int msg_length = strlen(message);
	if (msg_length > 3 && !strcmp(message + msg_length - 3, " 73"))
		ftx_repeat = 1; // no repeat for '73'
	else
		ftx_repeat = field_int("FTX_REPEAT");

	LOG(LOG_INFO, "-> %d.%c rpt %d %s",
		message_type, message_type ? ' ' : '0' + ftx_message_get_n3(&ftx_tx_msg), ftx_repeat, buf);
	//~ printf("%05d ft8_tx '%s' even? %d ftx_cq_alt %d ftx_xota %d '%s' rpt %d\n",
		//~ wallclock_day_ms % 60000, ftx_tx_text, ftx_tx1st, ftx_cq_alt, ftx_xota, ftx_xota_text, ftx_repeat);
}

/*!
	Encode and schedule a message for transmission, composed from the 3 fields
	\a call_to (which may alternatively be things like "CQ", "CQ SOTA", ...),
	\a call_de, and \a extra (which is for grid, RST, RRnn, RRR, 73).
	It is picked up by ft8_poll to do the actual transmission.
	The encoding will be std if possible, falling back to nonstd otherwise,
	and then falling back to free text if all else fails.
*/
void ft8_tx_3f(const char* call_to, const char* call_de, const char* extra) {
	char buf[64];

	snprintf(ftx_tx_text, sizeof(ftx_tx_text), "%s %s %s", call_to, call_de, extra);
	if (sbitx_ft8_encode_3f(call_to, call_de, extra) != FTX_MESSAGE_RC_OK) {
		LOG(LOG_INFO, "failed to encode: nothing to transmit\n");
		return;
	}
	const int message_type = ftx_message_get_i3(&ftx_tx_msg);
	ftx_would_send(); // update ftx_pitch, is_cq, ftx_tx1st, ftx_cq_alt, ftx_xota, ftx_xota_text
	snprintf(hmst_wallclock_time_sprint(buf), sizeof(buf) - 8, "  TX     %4d ~ %s\n", ftx_pitch, ftx_xota ? ftx_xota_text : ftx_tx_text);
	write_console(STYLE_FT8_QUEUED, buf);

	const char *str_73 = strstr(extra, "73");
	if (str_73 && str_73 < extra + 2) // extra could be ' 73' or '73' but not 'RR73'
		ftx_repeat = 1; // no repeat for '73'
	else
		ftx_repeat = field_int("FTX_REPEAT");

	LOG(LOG_INFO, "-> %d.%c rpt %d '%s' '%s' '%s'\n",
		message_type, message_type ? ' ' : '0' + ftx_message_get_n3(&ftx_tx_msg), ftx_repeat, call_to, call_de, extra);
}

void *ftx_thread_function(void *ptr){
	//wake up every 100 msec to see if there is anything to decode
	while(1){
		usleep(1000);

		if (!ftx_do_decode)
			continue;

		ftx_do_decode = 0;
		sbitx_ft8_decode(ftx_rx_buffer, ftx_rx_buff_index);
		//let the next batch begin
		ftx_rx_buff_index = 0;
	}
}

// the ft8 sampling is at 12000, the incoming samples are at
// 96000 samples/sec
void ft8_rx(int32_t *samples, int count) {

	bool is_ft4 = !strcmp(field_str("MODE"), "FT4");
	int decimation_ratio = 96000/12000;

	//if there is an overflow, then reset to the begining
	if (ftx_rx_buff_index + (count/decimation_ratio) >= FT8_MAX_BUFF){
		ftx_rx_buff_index = 0;
		printf("Buffer Overflow\n");
	}

	//down convert to 12000 Hz sampling rate
	for (int i = 0; i < count; i += decimation_ratio)
		ftx_rx_buffer[ftx_rx_buff_index++] = samples[i] / 200000000.0f;

	int time_was = wallclock_day_ms;
	ftx_update_clock();
	// we only need to check every half-second
	if (time_was / 500 == wallclock_day_ms / 500)
		return;

	int slot_time = wallclock_day_ms % 15000;
	int min_secs = 12000;
	int slot_time_decode = 13000;
	if (is_ft4) {
		slot_time = wallclock_day_ms % 7500;
		min_secs = 6000;
		slot_time_decode = 13000 / 2;
	}
	//~ printf("time %d -> %d; slot %d; ftx_rx_buff_index %d\n", time_was % 60000, wallclock_day_ms % 60000, slot_time, ftx_rx_buff_index);

	if (slot_time < 500)
		ftx_rx_buff_index = 0;

	//we should have at least 6 or 12 seconds of samples to decode
	if (ftx_rx_buff_index >= 13 * min_secs && slot_time > slot_time_decode) {
		ftx_do_decode = 1;
		//~ printf("ft8_rx decoding trigger index %d, clock %d, slot_time %d\n", ftx_rx_buff_index, wallclock_day_ms % 60000, slot_time);
	}
}

void ft8_poll(int tx_is_on){
	//if we are already transmitting, we continue
	//until we run out of ft8 sampels
	if (tx_is_on){
		//tx_off should not abort repeats from modem_poll, when called from here
		int ftx_repeat_save = ftx_repeat;
		if (ftx_tx_nsamples == 0){
			tx_off();
			ftx_repeat = ftx_repeat_save;
			if (!ftx_repeat) {
				call_wipe();
				ft8_abort(true);
				ftx_tx_text[0] = 0;
			}
		}
		return;
	}

	if (!ftx_repeat)
		return;

	//we poll for this only once every half-second
	//we are here only if we are rx-ing and we have a pending transmission

	if (ftx_would_send()) {
		const bool is_ft4 = !strcmp(field_str("MODE"), "FT4");
		// FT4: two transmissions take 15 secs; are we interested in the first slot or the second?
		// FT8: two transmissions take 30 secs; are we interested in the first slot or the second?
		const int slot_time = is_ft4 ? wallclock_day_ms % 7500 : wallclock_day_ms % 15000;
		LOG(LOG_DEBUG, "%05d ft8_poll: tx_is_on %d ftx_tx_nsamples %d start '%s'\n",
			wallclock_day_ms % 60000, tx_is_on, ftx_tx_nsamples, ftx_tx_text);
		ftx_start_tx(slot_time); // modulate audio at current frequency setting
		if (ftx_tx_nsamples)
			tx_on(TX_SOFT);
		ftx_repeat--;
	}
}

float ft8_next_sample(){
	float sample = 0;
	if (ftx_tx_buff_index/8 < ftx_tx_nsamples){
		sample = ftx_tx_buffer[ftx_tx_buff_index/8]/7;
		ftx_tx_buff_index++;
	}
	else //stop transmitting ft8
		ftx_tx_nsamples = 0;
	return sample;
}

/*!
	Decide whether to reply on the even or odd timeslot, based on
	the received second \a msg_second within the minute.
	(i.e. \a msg_second is from 0 to 59)
*/
static void set_reply_tx1st(int msg_second)
{
	// When replying to an FT8 message that started in the 0- or 30-second "even" timeslot,
	// send in the "odd" 15- or 45-second timeslot; and vice-versa.
	// FT4 is similar, except we have 8 slots per minute.
	const int slot_len = (!strcmp(field_str("MODE"), "FT4") ? 7500 : 15000);
	const int msg_ms = 1000 * msg_second;
	// integer division is truncation: round to the nearest timeslot instead
	const int slot_in_minute = (msg_ms + slot_len / 2) / slot_len;
	// time 0 is slot 0: that's even, set ftx_tx1st = 0 to reply in odd slot;
	// FT8 15 secs is slot 1: that's odd, set ftx_tx1st = 1 to reply in even slot;
	// FT8 22.5 secs is slot 3: that's odd, set ftx_tx1st = 1 to reply in even slot (e.g. 30 or 45 secs)
	ftx_tx1st = slot_in_minute % 2;
	LOG(LOG_DEBUG, "msg_second %d slot_in_minute %d odd? %d reply tx1st? %d\n", msg_second, slot_in_minute, slot_in_minute % 2, ftx_tx1st);
}

/*!
	start a QSO: call the callsign specified by the "CALL" field,
	based on a previous selected message that occurred at time \a sel_time.
	The "SENT" field may hold previously-observed RST,
	and "EXCH" may hold the recipient's grid.
*/
void ft8_call(int sel_time)
{
	const char *call = field_str("CALL");
	if (!call[0]) {
		printf("CALL field empty: nobody to call\n");
		return;
	}

	const char *mycall = field_str("MYCALLSIGN");
	char mygrid[8];
	strncpy(mygrid, field_str("MYGRID"), 8);
	mygrid[4] = 0; // use only the first 4 letters of the grid
	field_set("NR", mygrid);

	tx_off();
	set_reply_tx1st(sel_time % 100);
	ft8_tx_3f(call, mycall, mygrid);
}

/*!
	Handle clicking a line from the messages list, or auto-responding to such a message:
	first populate all relevant fields, then if FTX_AUTO != "OFF", start or continue a QSO
	as appropriate for the given console \a line with length \a line_len in bytes.
	Assume it's already been parsed, with all tokens identified by the array of
	\a spans (max MAX_CONSOLE_LINE_STYLES).
	Timing is based on the time from the STYLE_TIME span.

	But if FTX_AUTO == "OFF", this function only populates fields.

	You can even click multiple messages in case the QSO was aborted and you
	want to resume it: first click a line that contains the caller's grid,
	then one that has the received signal report.  All 4 fields get re-populated,
	and then you can continue the QSO.
*/
void ftx_call_or_continue(const char* line, int line_len, const text_span_semantic* spans)
{
	if (spans[0].semantic != STYLE_FT8_RX) {
		LOG(LOG_DEBUG, "semantic %d != %d STYLE_FT8_RX: can't reply\n", spans[0].semantic, STYLE_FT8_RX);
		return;
	}
	char caller[16];
	int call_start = extract_semantic(line, line_len,  spans, STYLE_CALLER, caller, sizeof(caller));
	const char *mycall = field_str("MYCALLSIGN");
	// If no caller callsign has been identified, or the caller is me, do nothing here.
	// (Maybe the user clicked on that line by mistake; it's easy to do)
	if (call_start < 0 && strstr(caller, mycall))
		return;

	// Otherwise, we can see that the user clicked on a received line (not an outgoing line).
	// Populate fields appropriately.
	tx_off();
	field_set("CALL", caller);

	LOG(LOG_DEBUG, "ft8_call_or_continue: '%s'\n", line);

	char snr[8], pitch[8], grid[8], rst[8], extra[16], callee[16], time_str[10];
	int snr_start = 0, pitch_start = 0, grid_start = 0, rst_start = 0, callee_start = 0, extra_start = 0;
	int time = -1;
	bool is_rrst = false;
	const bool is_73 = strstr(line + line_len - 5, " 73");
	const bool is_rr73 = strstr(line + line_len - 5, "RR73");
	const bool is_rrr = strstr(line + line_len - 5, "RRR");
	// expect spans[0] to apply to the whole line; start with spans[1] to look at individual "words"
	for (int s = 1; s < MAX_CONSOLE_LINE_STYLES; ++s) {
		char *dbg_label = NULL;
		char *dbg_val = NULL;
		if (!spans[s].length)
			break; // a zero-length span marks the end of the spans array
		switch (spans[s].semantic) {
			case STYLE_TIME:
				extract_single_semantic(line, line_len, spans[s], time_str, sizeof(time_str));
				time = atoi(time_str); // skip tenths of seconds
				dbg_label = "time"; dbg_val = time_str;
				break;
			case STYLE_SNR:
				snr_start = extract_single_semantic(line, line_len, spans[s], snr, sizeof(snr));
				// Update the SENT field with SNR of received message
				// only when it should affect the report that we send;
				// leave it alone near the end of the QSO, so that we log what we have actually sent.
				if (!is_rr73 && !is_73 && !is_rrr)
					field_set("SENT", snr);
				dbg_label = "SNR"; dbg_val = snr;
				break;
			case STYLE_GRID:
			case STYLE_EXISTING_GRID:
				grid_start = extract_single_semantic(line, line_len, spans[s], grid, sizeof(grid));
				// If "RR73" shows up here, it's not a grid.
				if (is_rr73) {
					extra_start = grid_start;
					grid_start = 0;
					strncpy(extra, grid, sizeof(extra));
					grid[0] = 0;
					dbg_label = "not-a-grid"; dbg_val = extra;
				} else {
					field_set("EXCH", grid);
					dbg_label = "grid"; dbg_val = grid;
				}
				break;
			case STYLE_RST:
				rst_start = extract_single_semantic(line, line_len, spans[s], rst, sizeof(rst));
				// If the received line contained e.g. "R-07" it means we received a reply with
				// a signal report of -07. That's how we want to log it, so remove the leading R.
				if (rst[0] == 'R') {
					memmove(rst, rst + 1, 6);
					is_rrst = true;
				}
				field_set("RECV", rst);
				dbg_label = "RST"; dbg_val = rst;
				break;
			case STYLE_FREQ:
				pitch_start = extract_single_semantic(line, line_len, spans[s], pitch, sizeof(pitch));
				field_set("FTX_RX_PITCH", pitch);
				dbg_label = "pitch"; dbg_val = pitch;
				break;
			case STYLE_FT8_RX:
				// "CQ" and "73" currently show up here.
				// But a plain-text message could show up this way too;
				// extra is big enough to hold it, but there is not much point in clicking on it.
				extra_start = extract_single_semantic(line, line_len, spans[s], extra, sizeof(extra));
				dbg_label = "extra"; dbg_val = extra;
				break;
			case STYLE_LOG: {
				char buf[8];
				int start = extract_single_semantic(line, line_len, spans[s], buf, sizeof(buf));
				// "RR73" normally shows up here (we don't want it to stand out in the log).
				if (is_rr73) {
					extra_start = start;
					strncpy(extra, buf, sizeof(extra));
					dbg_label = "plain"; dbg_val = extra;
				}
				break;
			}
			case STYLE_MYCALL:
			case STYLE_CALLEE:
				callee_start = extract_single_semantic(line, line_len, spans[s], callee, sizeof(callee));
				break;
			case STYLE_CALLER:
			case STYLE_RECENT_CALLER:
				// skip it; we know the user's callsign, and we already extracted the caller's callsign above
				break;
		}
		if (dbg_label)
			LOG(LOG_DEBUG, "   span @ %d len %d: %d %s '%s'\n", spans[s].start_column, spans[s].length, spans[s].semantic, dbg_label, dbg_val);
	}

	if (time < 0) {
		LOG(LOG_ERROR, "failed to get time from '%s'\n", line);
		return;
	}
	char mygrid[8];
	set_reply_tx1st(time % 100);
	strncpy(mygrid, field_str("MYGRID"), sizeof(mygrid));
	mygrid[4] = 0; // use only the first 4 letters of the grid
	field_set("NR", mygrid);

	// We populated the fields above.
	// If FTX_AUTO == "OFF", the user gets to decide what to do next
	// and operate the macro buttons manually.
	const char *ftx_auto_str = field_str("FTX_AUTO");
	if (!strcmp(ftx_auto_str, "OFF"))
		return;

	// Otherwise, decide how to respond.

	if (callee_start && strncmp(callee, mycall, sizeof(callee))) {
		// The callee is not me, so this is not a continuation of a QSO.
		// Clear the received RST (if populated, it was for someone else) and start a new QSO.
		field_set("RECV", "");
		ft8_tx_3f(caller, mycall, mygrid);
		return;
	}
	if (is_rr73 || is_rrr) {
		LOG(LOG_DEBUG, "received RR73, send 73\n");
		ftx_repeat = 1;
		ft8_tx_3f(caller, mycall, "73");
		enter_qso();
		return;
	}
	if (is_73) {
		LOG(LOG_DEBUG, "received 73\n");
		enter_qso();
		call_wipe();
		ft8_abort(true);
		ftx_tx_text[0] = 0;
		return;
	}
	if (is_rrst) {
		LOG(LOG_DEBUG, "received roger-report; send RR73\n");
		ft8_tx_3f(caller, mycall, "RR73");
		return;
	}
	if (rst_start && !grid_start) {
		LOG(LOG_DEBUG, "received report; send roger-report\n");
		char report[5];
		snprintf(report, sizeof(report), "R%s", snr);
		ft8_tx_3f(caller, mycall, report);
		return;
	}
	// If it's just an incoming call, with no CQ, send a signal report.
	if (strncmp(extra, "CQ", 2)) {
		LOG(LOG_DEBUG, "not a CQ, no other cases: send a signal report\n");
		ft8_tx_3f(caller, mycall, snr);
		return;
	}

	// It wasn't a reply situation: start a new QSO then.
	ft8_tx_3f(caller, mycall, mygrid);
}

void ft8_init(){
	ftx_rx_buff_index = 0;
	ftx_tx_buff_index = 0;
	ftx_tx_nsamples = 0;
	hashtable_init();
	pthread_create( &ftx_thread, NULL, ftx_thread_function, (void*)NULL);
	memset(ftx_rx_buffer, 0, sizeof(ftx_rx_buffer));
	memset(ftx_tx_buffer, 0, sizeof(ftx_tx_buffer));
	memset(ftx_tx_text, 0, sizeof(ftx_tx_text));
	memset(ftx_xota_text, 0, sizeof(ftx_xota_text));
}

void ft8_abort(bool terminate_qso){
	ftx_tx_nsamples = 0;
	ftx_repeat = 0;
	if (terminate_qso)
		ftx_tx_text[0] = 0; // ftx_would_send() will now return false: nothing left to send
}
