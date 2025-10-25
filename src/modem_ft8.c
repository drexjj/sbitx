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

#include "ft8_lib/common/common.h"
#include "ft8_lib/common/wave.h"
#include "ft8_lib/ft8/debug.h"
#include "ft8_lib/ft8/decode.h"
#include "ft8_lib/ft8/encode.h"
#include "ft8_lib/ft8/constants.h"
#include "ft8_lib/fft/kiss_fftr.h"

// We try to avoid calling automatically the same stations again and again, at least in this session
#define min(x, y) (((x) < (y)) ? (x) : (y))
#define FT8_CALLED_SIZE 64
static char ft8_already_called[FT8_CALLED_SIZE][32];
static int ft8_already_called_n = 0;

static int32_t ft8_rx_buff[FT8_MAX_BUFF];
static float ft8_rx_buffer[FT8_MAX_BUFF];
static float ft8_tx_buff[FT8_MAX_BUFF];
static char ft8_tx_text[128];
ftx_message_t ftx_tx_msg;
static int ft8_rx_buff_index = 0;
static int ft8_tx_buff_index = 0;
static int	ft8_tx_nsamples = 0;
static int ft8_do_decode = 0;
static int	ft8_do_tx = 0;
static int	ft8_pitch = 0;
static int	ft8_mode = FT8_SEMI;
static pthread_t ft8_thread;
static int ft8_tx1st = 1;
void ft8_tx(char *message, int freq);
void ft8_interpret(char *received, char *transmit);
extern void call_wipe();

// how to handle a command option
#define FT8_START_QSO 1
#define FT8_CONTINUE_QSO 0
static unsigned int wallclock =0;
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
	STYLE_LOG		// FTX_FIELD_RST
};

#define LOG_LEVEL LOG_INFO

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
	Encode ftx_tx_msg.payload onto audio carrier \a freq and output to \a signal.
	\a is_ft4 chooses FT4 encoding instead of FT8.
	@return the number of audio samples
*/
int sbitx_ftx_msg_audio(int32_t freq, float *signal, bool is_ft4)
{
	if (!freq)
		freq = field_int("TX_PITCH");
    float frequency = 1.0 * freq;

    int num_tones = (is_ft4) ? FT4_NN : FT8_NN;
    float symbol_period = (is_ft4) ? FT4_SYMBOL_PERIOD : FT8_SYMBOL_PERIOD;
    float symbol_bt = (is_ft4) ? FT4_SYMBOL_BT : FT8_SYMBOL_BT;
    float slot_time = (is_ft4) ? FT4_SLOT_TIME : FT8_SLOT_TIME;

    // Second, encode the binary message as a sequence of FSK tones
    uint8_t tones[num_tones]; // Array of 79 tones (symbols)
    if (is_ft4)
        ft4_encode(ftx_tx_msg.payload, tones);
    else
        ft8_encode(ftx_tx_msg.payload, tones);

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
    synth_gfsk(tones, num_tones, frequency, symbol_bt, symbol_period, sample_rate, signal + num_silence);
    return num_total_samples;
}

/*!
	Encode \a message into ftx_tx_msg.
	\a is_ft4 chooses FT4 encoding instead of FT8.
	This should only be used when the user has typed \a message;
	for programmatic cases, prefer sbitx_ft8_encode_3f'()
	@return the return value from ftx_message_encode (see enum ftx_message_rc_t in ft8_lib/message.h)
*/
int sbitx_ft8_encode(char *message, bool is_ft4)
{
    ftx_message_rc_t rc = ftx_message_encode(&ftx_tx_msg, &hash_if, message);
    if (rc != FTX_MESSAGE_RC_OK)
        printf("Cannot encode FTx message! RC = %d\n", (int)rc);

	return rc;
}

/*!
	Compose a message from the 3 fields \a call_to, \a call_de and \a extra into ftx_tx_msg.
	\a is_ft4 chooses FT4 encoding instead of FT8.
	@return the return value from ftx_message_encode_std/nonstd/free
	(see enum ftx_message_rc_t in ft8_lib/message.h)
*/
int sbitx_ft8_encode_3f(const char* call_to, const char* call_de, const char* extra, bool is_ft4)
{
	ftx_message_rc_t rc = ftx_message_encode_std(&ftx_tx_msg, &hash_if, call_to, call_de, extra);
	if (rc != FTX_MESSAGE_RC_OK) {
		LOG(LOG_DEBUG, "   ftx_message_encode_std failed: %d\n", rc);
		rc = ftx_message_encode_nonstd(&ftx_tx_msg, &hash_if, call_to, call_de, extra);
		if (rc != FTX_MESSAGE_RC_OK) {
			LOG(LOG_DEBUG, "   ftx_message_encode_nonstd failed: %d\n", rc);
			rc = ftx_message_encode_free(&ftx_tx_msg, ft8_tx_text);
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

static void monitor_reset(monitor_t* me)
{
    me->wf.num_blocks = 0;
    me->max_mag = 0;
}

static int message_callsign_count(const ftx_message_offsets_t *spans)
{
	int ret = 0;
	for (int i = 0; i < FTX_MAX_MESSAGE_FIELDS; ++i)
		if (spans->types[i] == FTX_FIELD_CALL)
			++ret;
	return ret;
}

static int sbitx_ft8_decode(float *signal, int num_samples, bool is_ft8)
{
    int sample_rate = 12000;

    LOG(LOG_DEBUG, "Sample rate %d Hz, %d samples, %.3f seconds\n", sample_rate, num_samples, (double)num_samples / sample_rate);

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

		//timestamp the packets
		//the time is shifted back by the time it took to capture these sameples
		time_t	rawtime = (time_sbitx() / 15) * 15; //round to the earlier slot
		char time_str[20], response[100];
		struct tm *t = gmtime(&rawtime);
		sprintf(time_str, "%02d%02d%02d", t->tm_hour, t->tm_min, t->tm_sec);

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
		char text[FTX_MAX_MESSAGE_LENGTH]; // message text as decoded
		ftx_message_offsets_t spans; // locations/lengths of fields in text
		char displaytext[64]; // text as written to the console
		ftx_message_t message; // encoded form
	} decoded_message_t;

	int num_decoded = 0;
    decoded_message_t decoded[kMax_decoded_messages];
    decoded_message_t* decoded_hashtable[kMax_decoded_messages];

    // Initialize hash table pointers
    for (int i = 0; i < kMax_decoded_messages; ++i)
    {
        decoded_hashtable[i] = NULL;
    }

		int n_decodes = 0;

    bool processingqso = false;

    // Go over candidates and attempt to decode messages
    for (int idx = 0; idx < num_candidates; ++idx)
    {
        const ftx_candidate_t* cand = &candidate_list[idx];
        if (cand->score < kMin_score)
            continue;

        int freq_hz = lroundf((cand->freq_offset + (float)cand->freq_sub / mon.wf.freq_osr) / mon.symbol_period);
        float time_sec = (cand->time_offset + (float)cand->time_sub / mon.wf.time_osr) * mon.symbol_period;

        ftx_message_t message;
        ftx_decode_status_t status;
        if (!ftx_decode_candidate(&mon.wf, cand, kLDPC_iterations, &message, &status)){
            // printf("000000 %3d %+4.2f %4.0f ~  ---\n", cand->score, time_sec, freq_hz);
            if (status.ldpc_errors > 0)
                LOG(LOG_DEBUG, "LDPC decode: %d errors\n", status.ldpc_errors);
            else if (status.crc_calculated != status.crc_extracted)
                LOG(LOG_DEBUG, "CRC mismatch!\n");
            continue;
        }

        LOG(LOG_DEBUG, "Checking hash table for %4.1fs / %4.1fHz [%d]...\n", time_sec, freq_hz, cand->score);
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
			decoded[idx_hash].spans = spans;

			char buf[64];
			int prefix_len = snprintf(buf, sizeof(buf), "%s %3d %+03d %4d ~ ", time_str, cand->score, cand->snr, freq_hz);
			int line_len = prefix_len + snprintf(buf + prefix_len, sizeof(buf) - prefix_len, "%s\n", text);
			const int message_type = ftx_message_get_i3(&message);
			//~ char type_utf8[4] = {0xE2, message_type ? 0x91 : 0x93, message_type ? 0xA0 + message_type - 1 : 0xAA, 0 };
			LOG(LOG_INFO, ">> %d.%c %s", message_type, message_type ? ' ' : '0' + ftx_message_get_n3(&message), buf);

			//For troubleshooting you can display the time offset - n1qm
			//sprintf(buff, "%s %d %+03d %-4.0f ~  %s\n", time_str, cand->time_offset,
			//  cand->snr, freq_hz, message.payload);

			text_span_semantic sem[FTX_MAX_MESSAGE_FIELDS + 4];
			memset(sem, 0, sizeof(sem));
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
			col = 8 + 3; // skip "score"
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
			if (span_i > 0)
				sem[sem_i - 1].length = strlen(text + spans.offsets[span_i - 1]);
			write_console_semantic(buf, sem, sem_i);

			if (my_call_found) {
				write_console(STYLE_FT8_QUEUED, buf);
				processingqso |= ft8_process(buf, FT8_CONTINUE_QSO);
			}

			// Store a string that may need to be parsed again in the future
			// For compatibility with other parts of the software, this historically
			// has to be the same that is shown in the GUI (and is clickable)
			strncpy(decoded[idx_hash].displaytext, buf, FTX_MAX_MESSAGE_LENGTH);
			decoded[idx_hash].displaytext[line_len] = 0;

			n_decodes++;
        }
    }
    //LOG(LOG_INFO, "Decoded %d messages\n", num_decoded);

    // Here we have a populated hash table with the decoded messages
    // If we are in autorespond mode and in idle state (i.e. no QSO ongoing),
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
    if (!strcmp(field_str("FT8_AUTO"), "ON") && !strlen(field_str("CALL")) && !processingqso) {
       char *candmsg = NULL;
       char *candtext = NULL;
       printf("Looking for a CQ to answer to\n");

       for (int idx = 0; idx < kMax_decoded_messages; idx++) {
           // We prioritize POTA and SOTA and /QRP and /P
           if ( decoded_hashtable[idx] && decoded_hashtable[idx]->text) {

              if ( !strncmp(decoded_hashtable[idx]->text, "CQ POTA ", 8) ||
                  !strncmp(decoded_hashtable[idx]->text, "CQ SOTA ", 8) ||
                  ( !strncmp(decoded_hashtable[idx]->text, "CQ ", 3) &&
                    (strstr(decoded_hashtable[idx]->text, "/QRP") ||
                     strstr(decoded_hashtable[idx]->text, "/P")) ) ) {

                 // We try to avoid calling automatically the same stations again and again, at least in this session
                 bool found = false;
                 for (int ii = 0; ii < min(ft8_already_called_n, FT8_CALLED_SIZE); ii++) {
                     if (!strcmp(decoded_hashtable[idx]->text, ft8_already_called[ii]))
                        found = true;
                 }

                 if (!found) {
                    candmsg = decoded_hashtable[idx]->displaytext;
                    candtext = decoded_hashtable[idx]->text;
                 }
                 break;
              }

              if ( !strncmp(decoded_hashtable[idx]->text, "CQ ", 3) ) {

                 // We try to avoid calling automatically the same stations again and again, at least in this session
                 bool found = false;
                 for (int ii = 0; ii < min(ft8_already_called_n, FT8_CALLED_SIZE); ii++) {
                     if (!strcmp(decoded_hashtable[idx]->text, ft8_already_called[ii]))
                        found = true;
                 }

                 if (!found) {
                    candmsg = decoded_hashtable[idx]->displaytext;
                    candtext = decoded_hashtable[idx]->text;
                 }

              }

           }
       }

       if (candmsg) {
          printf("Maybe we should respond to '%s' ... ", candmsg);
          if (ft8_process(candmsg, FT8_START_QSO)) {
             strcpy(ft8_already_called[ft8_already_called_n % FT8_CALLED_SIZE], candtext);
             ft8_already_called_n++;
             printf("yes, doing it\n");
          }
          else
             printf("no\n");
       }
    }

    monitor_free(&mon);
    hashtable_cleanup(10);

    return n_decodes;
}

// number of repetitions left for the current message, counting down from the user setting
static int ft8_repeat = 5;

int sbitx_ft8_encode(char *message, bool is_ft4);
int sbitx_ft8_encode_3f(const char* call_to, const char* call_de, const char* extra, bool is_ft4);

void ft8_setmode(int config){
	switch(config){
		case FT8_MANUAL:
			ft8_mode = FT8_MANUAL;
			write_console(STYLE_LOG, "FT8 is manual now.\nSend messages through the keyboard\n");
			break;
		case FT8_SEMI:
			write_console(STYLE_LOG, "FT8 is semi-automatic.\nClick on the callsign to start the QSO\n");
			ft8_mode = FT8_SEMI;
			break;
		case FT8_AUTO:
			write_console(STYLE_LOG, "FT8 is automatic.\nIt will call CQ and QSO with the first reply.\n");
			ft8_mode = FT8_AUTO;
			break;
	}
}

static void ft8_start_tx(int offset_seconds){
	char buf[100];
	//timestamp the packets for display log
	time_t	rawtime = time_sbitx();
	struct tm *t = gmtime(&rawtime);

	int freq = field_int("TX_PITCH");
	if (freq != ft8_pitch)
		ft8_pitch = freq;
	ft8_tx_nsamples = sbitx_ftx_msg_audio(freq,  ft8_tx_buff, /* is_ft4*/ false);

	snprintf(buf, sizeof(buf), "%02d%02d%02d  TX     %4d ~ %s\n", t->tm_hour, t->tm_min, t->tm_sec, ft8_pitch, ft8_tx_text);
	write_console(STYLE_FT8_TX, buf);

	const int message_type = ftx_message_get_i3(&ftx_tx_msg);
	LOG(LOG_INFO, "<< %d.%c %s", message_type, message_type ? ' ' : '0' + ftx_message_get_n3(&ftx_tx_msg), buf);

	ft8_tx_buff_index = offset_seconds * 96000;
	printf("ft8_start_tx: starting @index %d based on offset_seconds %d\n", ft8_tx_buff_index, offset_seconds);
}

/*!
	Encode and schedule \a message for transmission, modulated on \a freq.
	It is picked up by ft8_poll to do the actual transmission.
	\a message may be anything: ft8_lib has to parse it and guess the message type to use.
	So it's better to call ft8_tx_3f(to, de, extra) in all programmatic cases,
	and use this function only when the user is doing the typing.
*/
void ft8_tx(char *message, int freq){
	char cmd[200], buf[64];
	FILE	*pf;
	time_t	rawtime = time_sbitx();
	struct tm *t = gmtime(&rawtime);

	ft8_tx_text[0] = 0;
	for (int i = 0; i < strlen(message); i++)
		message[i] = toupper(message[i]);
	if (sbitx_ft8_encode(message, false) != FTX_MESSAGE_RC_OK) {
		LOG(LOG_INFO, "failed to encode: nothing to transmit\n");
		return;
	}

	strncpy(ft8_tx_text, message, sizeof(ft8_tx_text));
	if (!freq) {
		freq = field_int("TX_PITCH");
		ft8_pitch = freq;
	}
	const int message_type = ftx_message_get_i3(&ftx_tx_msg);

	snprintf(buf, sizeof(buf), "%02d%02d%02d  TX     %4d ~ %s\n", t->tm_hour, t->tm_min, t->tm_sec, freq, ft8_tx_text);
	write_console(STYLE_FT8_QUEUED, buf);
	LOG(LOG_INFO, "<- %d.%c %s", message_type, message_type ? ' ' : '0' + ftx_message_get_n3(&ftx_tx_msg), buf);

	//also set the times of transmission
	char str_tx1st[10], str_repeat[10];
	get_field_value_by_label("FT8_TX1ST", str_tx1st);
	get_field_value_by_label("FT8_REPEAT", str_repeat);
	int slot_second = time_sbitx() % 15;

	//no repeat for '73'
	int msg_length = strlen(message);
	if (msg_length > 3 && !strcmp(message + msg_length - 3, " 73")){
		ft8_repeat = 1;
	}
	else
		ft8_repeat = atoi(str_repeat);

	// the FT8_TX1ST setting applies only to initiating a CQ call;
	// otherwise, leave ft8_tx1st as set earlier, e.g. in ft8_process()
	// if it is a CQ message, then wait for the slot
	if (!strncmp(message, "CQ ", 3)) {
		ft8_tx1st = !strcmp(str_tx1st, "ON");
		return;
	}
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
	char cmd[200], buff[1000];
	FILE	*pf;
	time_t	rawtime = time_sbitx();
	struct tm *t = gmtime(&rawtime);

	ft8_pitch = field_int("TX_PITCH");

	snprintf(ft8_tx_text, sizeof(ft8_tx_text), "%s %s %s", call_to, call_de, extra);
	sprintf(buff, "%02d%02d%02d  TX     %4d ~ %s\n", t->tm_hour, t->tm_min, t->tm_sec, ft8_pitch, ft8_tx_text);
	write_console(STYLE_FT8_QUEUED, buff);

	sbitx_ft8_encode_3f(call_to, call_de, extra, false);

	// also set the times of transmission
	char str_tx1st[10], str_repeat[10];
	get_field_value_by_label("FT8_TX1ST", str_tx1st);
	get_field_value_by_label("FT8_REPEAT", str_repeat);
	int slot_second = time_sbitx() % 15;

	// no repeat for '73'
	if (!strcmp(extra, " 73"))
		ft8_repeat = 1;
	else
		ft8_repeat = atoi(str_repeat);

	// the FT8_TX1ST setting applies only to initiating a CQ call;
	// otherwise, leave ft8_tx1st as set earlier, e.g. in ft8_process()
	// if it is a CQ message, then wait for the slot
	if (!strncmp(call_to, "CQ ", 3)) {
		ft8_tx1st = !strcmp(str_tx1st, "ON");
		return;
	}
}

void *ft8_thread_function(void *ptr){
	FILE *pf;
	char buff[1000], mycallsign_upper[20]; //there are many ways to crash sbitx, bufferoverflow of callsigns is 1

	//wake up every 100 msec to see if there is anything to decode
	while(1){
		usleep(1000);

		if (!ft8_do_decode)
			continue;

		ft8_do_decode = 0;
		sbitx_ft8_decode(ft8_rx_buffer, ft8_rx_buff_index, true);
		//let the next batch begin
		ft8_rx_buff_index = 0;
	}
}

// the ft8 sampling is at 12000, the incoming samples are at
// 96000 samples/sec
void ft8_rx(int32_t *samples, int count){

	int decimation_ratio = 96000/12000;

	//if there is an overflow, then reset to the begining
	if (ft8_rx_buff_index + (count/decimation_ratio) >= FT8_MAX_BUFF){
		ft8_rx_buff_index = 0;
		printf("Buffer Overflow\n");
	}

	//down convert to 12000 Hz sampling rate
	for (int i = 0; i < count; i += decimation_ratio)
		//ft8_rx_buff[ft8_rx_buff_index++] = samples[i];
		ft8_rx_buffer[ft8_rx_buff_index++] = samples[i] / 200000000.0f;

	int now = time_sbitx();
	if (now != wallclock)
		wallclock = now;
	else
		return;

	int slot_second = wallclock % 15;
	if (slot_second == 0)
		ft8_rx_buff_index = 0;

//	printf("ft8 decoding trigger index %d, slot_second %d\n", ft8_rx_buff_index, slot_second);
	//we should have atleast 12 seconds of samples to decode
	if (ft8_rx_buff_index >= 13 * 12000 && slot_second > 13)
		ft8_do_decode = 1;
}

void ft8_poll(int seconds, int tx_is_on){
	static int last_second = 0;

	//if we are already transmitting, we continue
	//until we run out of ft8 sampels
	if (tx_is_on){
		//tx_off should not abort repeats from modem_poll, when called from here
		int ft8_repeat_save = ft8_repeat;
		if (ft8_tx_nsamples == 0){
			tx_off();
			ft8_repeat = ft8_repeat_save;
		}
		return;
	}

	if (!ft8_repeat || seconds == last_second)
            return;

	//we poll for this only once every second
	//we are here only if we are rx-ing and we have a pending transmission
	last_second = seconds = seconds % 60;

	if (
		(ft8_tx1st == 1 && ((seconds >= 0  && seconds < 15) ||
			(seconds >=30 && seconds < 45))) ||
		(ft8_tx1st == 0 && ((seconds >= 15 && seconds < 30)||
			(seconds >= 45 && seconds < 59)))){
		ft8_start_tx(seconds % 15);
		if (ft8_tx_nsamples)
			tx_on(TX_SOFT);
		ft8_repeat--;
		if (!ft8_repeat)
                   call_wipe();
	}
}

float ft8_next_sample(){
		float sample = 0;
		if (ft8_tx_buff_index/8 < ft8_tx_nsamples){
			sample = ft8_tx_buff[ft8_tx_buff_index/8]/7;
			ft8_tx_buff_index++;
		}
		else //stop transmitting ft8
			ft8_tx_nsamples = 0;
		return sample;
}

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
	for (; c < dsize && is_token_char(*src); ++c)
		*dst++ = *src++;
	*dst = 0;
	return c;
}

/* these are used to process the current message */
static char m1[32], m2[32], m3[32], m4[32], signal_strength[10], mygrid[10],
	reply_message[100];
static int rx_pitch, confidence_score, msg_time;
static const char *call = NULL, *exchange = NULL,
	*report_send = NULL, *report_received = NULL, *mycall = NULL;

int ft8_message_tokenize(char *message){
	char *p;

	//tokenize the message
	p = strtok(message, " \r\n");
	if (!p) return -1;
	msg_time = atoi(p);

	p = strtok(NULL, " \r\n");
	if (!p) return -1;
	confidence_score = atoi(p);

	p = strtok(NULL, " \r\n");
	if (!p) return -1;
	strcpy(signal_strength, p);

	p = strtok(NULL, " \r\n");
	if (!p) return -1;
	rx_pitch = atoi(p);

	//santiy check, we should get a tilde '~' now
	p = strtok(NULL, " \r\n");
	if (!p)
		return -1;
	if (strcmp(p, "~"))
		return -1;

	p = strtok(NULL, " \r\n");
	if (!p) return -1;
	tokncpy(m1, p, sizeof(m1));

	p = strtok(NULL, " \r\n");
	if (!p) return -1;
	tokncpy(m2, p, sizeof(m2));

	p = strtok(NULL, " \r\n");
	if (p){
		tokncpy(m3, p, sizeof(m3));

		p = strtok(NULL, " \r\n");
		if (p)
			tokncpy(m4, p, sizeof(m4));
		else
			m4[0] = 0;
	}
	else
		m3[0] = 0;

	return 0;
}

// this kicks stars a new qso either as a CQ message or
// as a reply to someone's cq or as a 'break' with signal report to
// a concluding qso
void ft8_on_start_qso(char *message){
	modem_abort();
	tx_off();
	call_wipe();

	//for cq message that started on 0 or 30th second, use the 15 or 45 and
	//vice versa
	int msg_second = msg_time % 100;
	if (msg_second < 15 || (msg_second >= 30 && msg_second < 45))
		ft8_tx1st = 0; //we tx on 2nd and 4ht slots for msgs on 1st and 3rd
	else
		ft8_tx1st = 1;

	if (!strcmp(m1, "CQ")){
		if (m4[0]){
			field_set("CALL", m3);
			field_set("EXCH", m4);
			field_set("SENT", signal_strength);
		}
		else {
			field_set("CALL", m2);
			field_set("EXCH", m3);
			field_set("SENT", signal_strength);
		}
		LOG(LOG_DEBUG, "ft8_on_start_qso CQ: rst s %s\n", signal_strength);
		sprintf(reply_message, "%s %s %s", call, mycall, mygrid);
	}
	//whoa, someone cold called us
	else if (!strcmp(m1, mycall)){
		field_set("CALL", m2);
		field_set("SENT", signal_strength);
		LOG(LOG_DEBUG, "ft8_on_start_qso cold call: rst s %s\n", signal_strength);
		//they might have directly sent us a signal report
		if (isalpha(m3[0]) && isalpha(m3[1]) && strncmp(m3,"RR",2)!=0){ // R- RR are not EXCH
			field_set("EXCH", m3);
			sprintf(reply_message, "%s %s %s", call, mycall, signal_strength);
		}
		else {
			field_set("RECV", m3);
			sprintf(reply_message, "%s %s R%s", call, mycall, signal_strength);
		}
	}
	else { //we are breaking into someone else's qso
		field_set("CALL", m2);
		if (isalpha(m3[0]) && isalpha(m3[1]) && strncmp(m3,"RR",2)!=0){ // R- RR are not EXCH
			field_set("EXCH", m3); // the gridId is valid - use it
		} else {
			field_set("EXCH", "");
		}
		field_set("SENT", signal_strength);
		LOG(LOG_DEBUG, "ft8_on_start_qso break-in: rst s %s\n", signal_strength);
		sprintf(reply_message, "%s %s %s", call, mycall, signal_strength);
	}
	field_set("NR", mygrid);
	ft8_tx(reply_message, ft8_pitch);
}

void ft8_on_signal_report(){
	field_set("CALL", m2);
	if (m3[0] == 'R'){
		//skip the 'R'
		field_set("RECV", m3+1);
		ft8_tx_3f(call, mycall, "RR73");
	}
	else{
		field_set("RECV", m3);
		// in case ft8_on_start_qso() was not called: ensure that we send some numeric signal report
		if (!field_str("SENT")[0]) {
			field_set("SENT", signal_strength);
			report_send = field_str("SENT");
		}
		char report[5];
		snprintf(report, sizeof(report), "R%s", report_send);
		ft8_tx_3f(call, mycall, report);
	}

	//Disabled this because of early logging - W9JES
	//enter_qso();
}

/*!
	start a QSO: call the callsign specified by the "CALL" field,
	based on a previous selected message that occurred at time \a sel_time.
	The "SENT" field may hold previously-observed RST,
	and "EXCH" may hold the recipient's grid.
*/
void ft8_call(int sel_time)
{
	call = field_str("CALL");
	if (!call[0]) {
		printf("CALL field empty: nobody to call\n");
		return;
	}

	modem_abort();
	tx_off();

	exchange = field_str("EXCH");
	report_send = field_str("SENT");
	mycall = field_str("MYCALLSIGN");
	// initial pitch; but it can also be adjusted between timeslots
	// (audio is re-generated in ft8_start_tx())
	ft8_pitch = field_int("TX_PITCH");
	//use only the first 4 letters of the grid
	strcpy(mygrid, field_str("MYGRID"));
	mygrid[4] = 0;
	field_set("NR", mygrid);

	// for CQ (or other) message that started in the 0 or 30-second timeslot,
	// send reply in the 15 or 45 second; and vice-versa
	// i.e. tx on 2nd and 4th slots for msgs on 1st and 3rd
	const int msg_second = sel_time % 100;
	ft8_tx1st = !(msg_second < 15 || (msg_second >= 30 && msg_second < 45));

	ft8_tx_3f(call, mycall, mygrid);
}

/*!
	Start or continue a QSO as appropriate for the \a message:
	\a operation may be FT8_START_QSO or FT8_CONTINUE_QSO
	This should mostly not be used, because it throws away information that we already have:
	if \a message came from ft8_lib, we also have spans to identify the fields;
	or if we want to start a QSO, call ft8_call() above (which depends
	on fields containing information that we already have).
	The remaining legitimate usecase is only when the user types the message in the "TEXT" field.
*/
int ft8_process(char *message, int operation)
{
	char buff[100], reply_message[100], *p;
	int auto_respond = 0;

	if (ft8_message_tokenize(message) == -1)
		return 0;

	call = field_str("CALL");
	exchange = field_str("EXCH");
	report_send = field_str("SENT");
	report_received = field_str("RECV");
	mycall = field_str("MYCALLSIGN");
	ft8_pitch = field_int("TX_PITCH");
	if (!strcmp(field_str("FT8_AUTO"), "ON"))
		auto_respond = 1;

	//use only the first 4 letters of the grid
	strcpy(mygrid, field_str("MYGRID"));
	mygrid[4] = 0;

	//we can start call in reply to a cq, cq dx or anyone else ending the call
	if (operation == FT8_START_QSO){
		ft8_on_start_qso(message);
		return 1;
	}

	// see if you are on auto responder, the logger is empty and we are the called party
	if (auto_respond && !strlen(call) && !strcmp(m1, mycall)){
		ft8_on_start_qso(message);
		return 2;
	}

	//by now, any message that comes to us should have our callsign as m1
	if (strcmp(m1, mycall)){
		printf("FT8: Not a message for %s\n", mycall);
		return 0;
	}


	if (!strcmp(m3, "73")){
		ft8_abort();
		enter_qso(); // W9JES
		ft8_repeat = 0;
		return 1;
	}

	//the other station has sent either an RRR or an RR73
	//this maybe arriving after we have cleared the log
	//we don't check it against any fields of the logger
	if (!strcmp(m3, "RR73") || !strcmp(m3, "RRR")){
		ft8_tx_3f(m2, mycall, "73");
		enter_qso();
		call_wipe();
		ft8_repeat = 1;
        return 1;
	}

	//beyond this point, we need to have a call filled up in the logger
	if (!strlen(call))
		return 0;

	//this is a signal report, at times, some other call can just send their sig report, even if we are in the
        // middle of a different qso. We shall not overwrite the fields relative to the current qso, mixing things up,
	// hence we stick to the already ongoing one
	if (!strcmp(call, m2)) {
		if (m3[0] == '-' || (m3[0] == 'R' && m3[1] == '-') || m3[0] == '+' || (m3[0] == 'R' && m3[1] == '+')) {
			printf("FT8: Got a signal report '%s' '%s' '%s'\n", m1, m2, m3);
			ft8_on_signal_report();
		}
	} else {
		printf("FT8: Ignoring an unsolicited signal report '%s' '%s' '%s'. Current call was '%s'\n", m1, m2, m3, call);
	}
	return 0;
}

void ft8_init(){
	ft8_rx_buff_index = 0;
	ft8_tx_buff_index = 0;
	ft8_tx_nsamples = 0;
	hashtable_init();
	pthread_create( &ft8_thread, NULL, ft8_thread_function, (void*)NULL);
}

void ft8_abort(){
	ft8_tx_nsamples = 0;
	ft8_repeat = 0;
}
