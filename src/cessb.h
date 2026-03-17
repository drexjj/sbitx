// CESSB (Controlled Envelope Single Sideband) Processing Header
//
// Reference: "Controlled Envelope Single Sideband" by David Hershberger, W9GR
//            QEX November/December 2014
//
// Added by Mike KB2ML and Bob KD8CGH

#include <stdint.h>

// ============================================================================
// CONFIGURATION PARAMETERS
// ============================================================================

#define CESSB_SAMPLE_RATE 96000.0f

// --- AGC Parameters ---
// Target level for AGC output
#define AGC_TARGET_LEVEL 1.65f
// Maximum gain boost (40 dB)
#define AGC_MAX_GAIN 100.0f
// Maximum gain cut (20 dB)
#define AGC_MIN_GAIN 0.1f
// AGC attack time in milliseconds (fast to catch loud passages)
#define AGC_ATTACK_MS 100.0f
// AGC release time in milliseconds (slow to avoid pumping)
#define AGC_RELEASE_MS 100.0f
// AGC gain smoothing time constant (very slow for smooth gain changes)
#define AGC_GAIN_SMOOTH_MS 50.0f

// --- CESSB Thresholds ---
// Clip level: initial hard clip threshold (as fraction of 1.0)
#define CESSB_CLIP_LEVEL 0.95f
// Envelope limit: final limiter ceiling
#define CESSB_ENVELOPE_LIMIT 0.93f

// --- Filter Parameters ---
// Hilbert transform filter length (must be odd)
#define HILBERT_TAPS 127
// Delay to match Hilbert filter group delay
#define HILBERT_DELAY_LEN ((HILBERT_TAPS - 1) / 2)

// Overshoot control lowpass filter length
#define OVERSHOOT_FILTER_TAPS 65
// Delay to match overshoot filter group delay
#define OVERSHOOT_DELAY_LEN ((OVERSHOOT_FILTER_TAPS - 1) / 2)

// Post-limiter lowpass filter (6th-order = 3 biquad stages)
#define POST_LPF_BIQUAD_STAGES 3

// --- Look-ahead Limiter Parameters ---
// Maximum look-ahead buffer size (samples)
#define LOOKAHEAD_MAX_SAMPLES 512
// Default look-ahead (~2 ms at 96 kHz)
#define LOOKAHEAD_DEFAULT_SAMPLES 192
// Default attack time (ms)
#define LOOKAHEAD_DEFAULT_ATTACK_MS 0.5f
// Default release time (ms)
#define LOOKAHEAD_DEFAULT_RELEASE_MS 50.0f

// --- Debug Audio Recording ---
// Set to 1 to enable recording of input/output audio for analysis
#define REC_AUDIO 0
#define REC_AUDIO_SEGMENT_SECONDS 5

// --- Enable/Disable ---
#define CESSB_ENABLED 1
#define CESSB_DISABLED 0

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Biquad filter state (for post-limiter LPF)
typedef struct {
    float x1, x2;  // Input delay
    float y1, y2;  // Output delay
} biquad_state_t;

// Look-ahead limiter state (vector version for I/Q)
typedef struct {
    float delay_i[LOOKAHEAD_MAX_SAMPLES];
    float delay_q[LOOKAHEAD_MAX_SAMPLES];
    float envelope[LOOKAHEAD_MAX_SAMPLES];
    int write_index;
    int lookahead_samples;
    float current_gain;
    float peak_hold;
    float attack_coeff;
    float release_coeff;
} lookahead_limiter_t;

// Input AGC state (NEW)
typedef struct {
    float peak_tracker;      // Smoothed peak estimate
    float agc_gain;          // Current AGC gain (smoothed)
    float target_level;      // Where we want peaks to land
    float attack_coeff;      // Peak tracker attack coefficient
    float release_coeff;     // Peak tracker release coefficient
    float gain_smooth_coeff; // Gain smoothing coefficient
    float max_gain;          // Maximum gain boost
    float min_gain;          // Minimum gain (maximum cut)
} input_agc_t;

// Main CESSB processor state
typedef struct {
    int enabled;
    float clip_level;
    float envelope_limit;
    float sample_rate;

    // Input AGC (NEW)
    input_agc_t input_agc;

    // Hilbert transform state
    float hilbert_delay[HILBERT_TAPS];
    int hilbert_index;

    // Delay line for I channel (matches Hilbert group delay)
    float delay_line[HILBERT_DELAY_LEN];
    int delay_index;

    // Overshoot control filter state (I and Q branches)
    float overshoot_i_delay[OVERSHOOT_FILTER_TAPS];
    float overshoot_q_delay[OVERSHOOT_FILTER_TAPS];
    int overshoot_i_index;
    int overshoot_q_index;

    // Second delay lines for envelope detection after overshoot filter
    float delay2_i[OVERSHOOT_DELAY_LEN];
    float delay2_q[OVERSHOOT_DELAY_LEN];
    int delay2_i_index;
    int delay2_q_index;

    // Look-ahead limiter
    lookahead_limiter_t lookahead;

    // Post-limiter lowpass filter state
    biquad_state_t post_lpf_state[POST_LPF_BIQUAD_STAGES];

    // Statistics for monitoring
    float peak_input;           // Peak before AGC
    float peak_after_agc;       // Peak after AGC (NEW)
    float peak_after_clip;
    float peak_after_overshoot;
    float peak_output;
    float average_power_in;
    float average_power_out;
    float min_limiter_gain;
    uint64_t sample_count;
} cessb_state_t;

// ============================================================================
// GLOBAL INSTANCE
// ============================================================================

extern int cessb_enabled;
extern cessb_state_t cessb_processor;

// ============================================================================
// API FUNCTIONS
// ============================================================================

// Initialize CESSB processor (call once at startup)
void cessb_init(cessb_state_t *state, float sample_rate);

// Enable/disable CESSB processing
void cessb_set_enabled(cessb_state_t *state, int enabled);
int cessb_is_enabled(cessb_state_t *state);

// CESSB parameter control
void cessb_set_clip_level(cessb_state_t *state, float level);
void cessb_set_envelope_limit(cessb_state_t *state, float limit);

// Look-ahead limiter control
void cessb_set_lookahead_samples(cessb_state_t *state, int samples);
void cessb_set_lookahead_ms(cessb_state_t *state, float milliseconds);
void cessb_set_attack_ms(cessb_state_t *state, float attack_ms);
void cessb_set_release_ms(cessb_state_t *state, float release_ms);
int cessb_get_lookahead_samples(cessb_state_t *state);

// AGC control (NEW)
void cessb_set_agc_target(cessb_state_t *state, float target);
void cessb_set_agc_max_gain(cessb_state_t *state, float max_gain);
void cessb_set_agc_min_gain(cessb_state_t *state, float min_gain);
float cessb_get_agc_gain(cessb_state_t *state);  // For metering display

// Main processing functions
void cessb_process(cessb_state_t *state, float *samples, int num_samples);
void cessb_process_int32(cessb_state_t *state, int32_t *samples, int num_samples);

// Statistics
void cessb_get_stats(cessb_state_t *state, float *peak_reduction_db,
                     float *avg_power_gain_db, float *talk_power_db);
void cessb_reset_stats(cessb_state_t *state);
void cessb_debug_print_stats(cessb_state_t *state);
