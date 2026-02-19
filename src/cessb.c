// CESSB (Controlled Envelope Single Sideband) Processing Implementation
//
// Reference: "Controlled Envelope Single Sideband" by David Hershberger, W9GR
//            QEX November/December 2014
//
// Concept: CESSB increases average SSB transmit power without increasing peak
// envelope power (PEP). This is achieved by controlling envelope overshoot that
// normally occurs when clipped audio is filtered.
//
// Processing chain:
// - Input audio arrives as int32 and is normalized to float in [-1, +1]
// - Automatically adjust audio input levels into working range
// - Compute analytic signal using Hilbert (HILBERT_TAPS):
//     I = delayed real sample
//     Q = Hilbert output (quadrature)
// - Measure instantaneous envelope: envelope = sqrt(I*I + Q*Q)
// - Envelope-based hard clip (vector clipping):
//     If envelope > clip_level, scale BOTH I and Q by (clip_level / envelope)
//     (preserves phase and applies radial limiting in the complex plane)
// - Overshoot-control FIR LPF (OVERSHOOT_FILTER_TAPS, Blackman–Harris, ~3 kHz):
//     Apply the same FIR to both I and Q (separate delay buffers)
//     This prevents overshoot regeneration while keeping analytic symmetry
// - Second envelope measurement:
//     Use delayed filtered I and filtered Q
//     envelope2 = sqrt(i2_delayed*i2_delayed + q2_delayed*q2_delayed)
// - Vector look-ahead limiter (configurable up to LOOKAHEAD_MAX_SAMPLES):
//     Maintain lookahead buffers for I, Q and envelope; compute peak over window,
//     derive a target gain (ceiling = envelope_limit) and smooth it with attack/release.
//     Apply the SAME time-aligned gain to the delayed I and Q to preserve phase.
// - Collapse analytic pair to real transmit waveform (use limited I for SSB)
// - Post-limiter 6th-order lowpass (3 biquad stages) @ 3 kHz applied to the real output
// - Convert float back to int32 before returning processed data
// - Statistics (peaks, average power, AGC gain, min limiter gain) accumulated for monitoring
//
// Key configuration parameters (see cessb.h)
//
//   AGC_TARGET_LEVEL              Target level for AGC
//   AGC_MAX_GAIN                  Maximum AGC boost (default 100 = 40 dB)
//   AGC_MIN_GAIN                  Maximum AGC cut (default 0.1 = -20 dB)
//   CESSB_CLIP_LEVEL              Hard clip threshold
//   CESSB_ENVELOPE_LIMIT          Final limiter ceiling
//   LOOKAHEAD_DEFAULT_SAMPLES     Default look-ahead (~2 ms at 96 kHz)
//   LOOKAHEAD_DEFAULT_ATTACK_MS   Limiter attack (default 0.5 ms)
//   LOOKAHEAD_DEFAULT_RELEASE_MS  Limiter release (default 50 ms)
//
// Added by Mike KB2ML and Bob KD8CGH

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cessb.h"

// ============================================================================
// PRECOMPUTED FILTER COEFFICIENTS
// Generated for: 96000 Hz sample rate, 3000 Hz audio cutoff
// ============================================================================

// Hilbert transform filter (127 taps, Blackman window)
static const float hilbert_coeffs[HILBERT_TAPS] = {
     1.40236097e-19f,  0.00000000e+00f, -9.37617092e-06f,  0.00000000e+00f, -3.91882957e-05f,
     0.00000000e+00f, -9.28426066e-05f,  0.00000000e+00f, -1.75022280e-04f,  0.00000000e+00f,
    -2.91799050e-04f,  0.00000000e+00f, -4.50725486e-04f,  0.00000000e+00f, -6.60910489e-04f,
     0.00000000e+00f, -9.33083247e-04f,  0.00000000e+00f, -1.27965419e-03f,  0.00000000e+00f,
    -1.71478569e-03f,  0.00000000e+00f, -2.25449075e-03f,  0.00000000e+00f, -2.91678511e-03f,
     0.00000000e+00f, -3.72192860e-03f,  0.00000000e+00f, -4.69280630e-03f,  0.00000000e+00f,
    -5.85552231e-03f,  0.00000000e+00f, -7.24031348e-03f,  0.00000000e+00f, -8.88294600e-03f,
     0.00000000e+00f, -1.08268500e-02f,  0.00000000e+00f, -1.31264051e-02f,  0.00000000e+00f,
    -1.58520727e-02f,  0.00000000e+00f, -1.90985932e-02f,  0.00000000e+00f, -2.29984892e-02f,
     0.00000000e+00f, -2.77452162e-02f,  0.00000000e+00f, -3.36349290e-02f,  0.00000000e+00f,
    -4.11468657e-02f,  0.00000000e+00f, -5.11114405e-02f,  0.00000000e+00f, -6.51024086e-02f,
     0.00000000e+00f, -8.65011541e-02f,  0.00000000e+00f, -1.24114929e-01f,  0.00000000e+00f,
    -2.10267285e-01f,  0.00000000e+00f, -6.35971008e-01f,  0.00000000e+00f,  6.35971008e-01f,
     0.00000000e+00f,  2.10267285e-01f,  0.00000000e+00f,  1.24114929e-01f,  0.00000000e+00f,
     8.65011541e-02f,  0.00000000e+00f,  6.51024086e-02f,  0.00000000e+00f,  5.11114405e-02f,
     0.00000000e+00f,  4.11468657e-02f,  0.00000000e+00f,  3.36349290e-02f,  0.00000000e+00f,
     2.77452162e-02f,  0.00000000e+00f,  2.29984892e-02f,  0.00000000e+00f,  1.90985932e-02f,
     0.00000000e+00f,  1.58520727e-02f,  0.00000000e+00f,  1.31264051e-02f,  0.00000000e+00f,
     1.08268500e-02f,  0.00000000e+00f,  8.88294600e-03f,  0.00000000e+00f,  7.24031348e-03f,
     0.00000000e+00f,  5.85552231e-03f,  0.00000000e+00f,  4.69280630e-03f,  0.00000000e+00f,
     3.72192860e-03f,  0.00000000e+00f,  2.91678511e-03f,  0.00000000e+00f,  2.25449075e-03f,
     0.00000000e+00f,  1.71478569e-03f,  0.00000000e+00f,  1.27965419e-03f,  0.00000000e+00f,
     9.33083247e-04f,  0.00000000e+00f,  6.60910489e-04f,  0.00000000e+00f,  4.50725486e-04f,
     0.00000000e+00f,  2.91799050e-04f,  0.00000000e+00f,  1.75022280e-04f,  0.00000000e+00f,
     9.28426066e-05f,  0.00000000e+00f,  3.91882957e-05f,  0.00000000e+00f,  9.37617092e-06f,
     0.00000000e+00f, -1.40236097e-19f
};

// Overshoot control filter (65 taps, Blackman-Harris window)
// Lowpass, cutoff 3000 Hz, normalized fc = 0.031250
static const float overshoot_coeffs[OVERSHOOT_FILTER_TAPS] = {
    -1.55789393e-22f, -4.25971371e-07f, -2.84081374e-06f, -1.00466862e-05f, -2.62075167e-05f,
    -5.70680120e-05f, -1.09643317e-04f, -1.91176866e-04f, -3.07228071e-04f, -4.58862990e-04f,
    -6.39078438e-04f, -8.28780584e-04f, -9.92837463e-04f, -1.07689533e-03f, -1.00574960e-03f,
    -6.84053239e-04f,  5.64658657e-19f,  1.16767022e-03f,  2.93941582e-03f,  5.42196095e-03f,
     8.69377583e-03f,  1.27905065e-02f,  1.76921839e-02f,  2.33141094e-02f,  2.95031125e-02f,
     3.60404042e-02f,  4.26515434e-02f,  4.90231806e-02f,  5.48253530e-02f,  5.97373084e-02f,
     6.34742558e-02f,  6.58121708e-02f,  6.66078871e-02f,  6.58121708e-02f,  6.34742558e-02f,
     5.97373084e-02f,  5.48253530e-02f,  4.90231806e-02f,  4.26515434e-02f,  3.60404042e-02f,
     2.95031125e-02f,  2.33141094e-02f,  1.76921839e-02f,  1.27905065e-02f,  8.69377583e-03f,
     5.42196095e-03f,  2.93941582e-03f,  1.16767022e-03f,  5.64658657e-19f, -6.84053239e-04f,
    -1.00574960e-03f, -1.07689533e-03f, -9.92837463e-04f, -8.28780584e-04f, -6.39078438e-04f,
    -4.58862990e-04f, -3.07228071e-04f, -1.91176866e-04f, -1.09643317e-04f, -5.70680120e-05f,
    -2.62075167e-05f, -1.00466862e-05f, -2.84081374e-06f, -4.25971371e-07f, -1.55789393e-22f
};

// Post-limiter LPF (6th-order Butterworth, 3000 Hz)
// Each row: b0, b1, b2, a1, a2
static const float post_lpf_coeffs[POST_LPF_BIQUAD_STAGES][5] = {
    {  8.08399021e-03f,  1.61679804e-02f,  8.08399021e-03f, -1.65053850e+00f,  6.82874458e-01f },
    {  8.44269293e-03f,  1.68853859e-02f,  8.44269293e-03f, -1.72377617e+00f,  7.57546944e-01f },
    {  9.14557162e-03f,  1.82911432e-02f,  9.14557162e-03f, -1.86728554e+00f,  9.03867829e-01f }
};

// Global instance
int cessb_enabled = CESSB_DISABLED;
cessb_state_t cessb_processor;

// ============================================================================
// TIME CONSTANT CALCULATION
// ============================================================================

static float time_constant_to_coeff(float time_ms, float sample_rate) {
    if (time_ms <= 0.0f) return 1.0f;
    float time_samples = (time_ms / 1000.0f) * sample_rate;
    return 1.0f - expf(-1.0f / time_samples);
}

// ============================================================================
// INPUT AGC
// Take tiny float values and get them to fill the range +/- 1 for cessb processing
// Relieves requirement for operator to accurately set gains for cessb
// clipper and limiter to perform
// ============================================================================

static void input_agc_init(input_agc_t *agc, float sample_rate) {
    agc->peak_tracker = 0.0f;
    agc->agc_gain = 1.0f;
    agc->target_level = AGC_TARGET_LEVEL;  // consider clip and limiter limits
    agc->max_gain = AGC_MAX_GAIN;
    agc->min_gain = AGC_MIN_GAIN;
    agc->attack_coeff = time_constant_to_coeff(AGC_ATTACK_MS, sample_rate);
    agc->release_coeff = time_constant_to_coeff(AGC_RELEASE_MS, sample_rate);
    agc->gain_smooth_coeff = time_constant_to_coeff(AGC_GAIN_SMOOTH_MS, sample_rate);
}

static float apply_input_agc(input_agc_t *agc, float input) {
    float abs_in = fabsf(input);

    // Track peak with fast attack, slow release
    if (abs_in > agc->peak_tracker) {
        agc->peak_tracker += agc->attack_coeff * (abs_in - agc->peak_tracker);
    } else {
        agc->peak_tracker += agc->release_coeff * (abs_in - agc->peak_tracker);
    }

    // Calculate desired gain to hit target level
    float desired_gain = 1.0f;
    if (agc->peak_tracker > 1e-6f) {
        desired_gain = agc->target_level / agc->peak_tracker;
    }

    // Clamp gain to reasonable range
    if (desired_gain > agc->max_gain) desired_gain = agc->max_gain;
    if (desired_gain < agc->min_gain) desired_gain = agc->min_gain;

    // Smooth gain changes (prevents pumping)
    agc->agc_gain += agc->gain_smooth_coeff * (desired_gain - agc->agc_gain);

    return input * agc->agc_gain;
}

// ============================================================================
// FIR FILTER PROCESSING
// ============================================================================

static float apply_fir_filter(const float *coeffs, float *delay, int *index, int num_taps,
                              float input) {
    delay[*index] = input;

    float output = 0.0f;
    int idx = *index;

    for (int i = 0; i < num_taps; i++) {
        output += coeffs[i] * delay[idx];
        idx--;
        if (idx < 0) idx = num_taps - 1;
    }

    (*index)++;
    if (*index >= num_taps) *index = 0;

    return output;
}

static float get_delayed_sample(float *delay, int *index, int delay_length, float input) {
    float output = delay[*index];
    delay[*index] = input;

    (*index)++;
    if (*index >= delay_length) *index = 0;

    return output;
}

// ============================================================================
// BIQUAD FILTER PROCESSING
// ============================================================================

static float apply_biquad(const float *coeffs, biquad_state_t *state, float input) {
    float output = coeffs[0] * input + coeffs[1] * state->x1 + coeffs[2] * state->x2 -
                   coeffs[3] * state->y1 - coeffs[4] * state->y2;

    state->x2 = state->x1;
    state->x1 = input;
    state->y2 = state->y1;
    state->y1 = output;

    return output;
}

static float apply_biquad_cascade(const float coeffs[][5], biquad_state_t *states,
                                  int num_stages, float input) {
    float output = input;
    for (int i = 0; i < num_stages; i++) {
        output = apply_biquad(coeffs[i], &states[i], output);
    }
    return output;
}

// ============================================================================
// LOOK-AHEAD LIMITER
// Carry I and Q both through the limiter together
// ============================================================================

static void lookahead_limiter_init_vec(lookahead_limiter_t *lim, float sample_rate) {
    memset(lim->delay_i, 0, sizeof(lim->delay_i));
    memset(lim->delay_q, 0, sizeof(lim->delay_q));
    memset(lim->envelope, 0, sizeof(lim->envelope));
    lim->write_index = 0;
    lim->lookahead_samples = LOOKAHEAD_DEFAULT_SAMPLES;
    lim->current_gain = 1.0f;
    lim->peak_hold = 0.0f;

    lim->attack_coeff = time_constant_to_coeff(LOOKAHEAD_DEFAULT_ATTACK_MS, sample_rate);
    lim->release_coeff = time_constant_to_coeff(LOOKAHEAD_DEFAULT_RELEASE_MS, sample_rate);
}

static float find_peak_in_window_vec(lookahead_limiter_t *lim) {
    float peak = 0.0f;

    int start_index = lim->write_index - lim->lookahead_samples + 1;
    if (start_index < 0) start_index += LOOKAHEAD_MAX_SAMPLES;

    int idx = start_index;
    for (int i = 0; i < lim->lookahead_samples; i++) {
        if (lim->envelope[idx] > peak) peak = lim->envelope[idx];
        idx++;
        if (idx >= LOOKAHEAD_MAX_SAMPLES) idx = 0;
    }
    return peak;
}

static void lookahead_limiter_process_vec(lookahead_limiter_t *lim,
                                          float input_i, float input_q, float envelope, float limit,
                                          float *out_i, float *out_q) {
    lim->delay_i[lim->write_index] = input_i;
    lim->delay_q[lim->write_index] = input_q;
    lim->envelope[lim->write_index] = envelope;

    int read_index = lim->write_index - lim->lookahead_samples;
    if (read_index < 0) read_index += LOOKAHEAD_MAX_SAMPLES;

    float peak_envelope = find_peak_in_window_vec(lim);

    float target_gain = 1.0f;
    if (peak_envelope > limit && peak_envelope > 1e-10f) {
        target_gain = limit / peak_envelope;
    }

    if (target_gain < lim->current_gain) {
        lim->current_gain += lim->attack_coeff * (target_gain - lim->current_gain);
    } else {
        lim->current_gain += lim->release_coeff * (target_gain - lim->current_gain);
    }

    if (lim->current_gain < 0.0f) lim->current_gain = 0.0f;
    if (lim->current_gain > 1.0f) lim->current_gain = 1.0f;

    *out_i = lim->delay_i[read_index] * lim->current_gain;
    *out_q = lim->delay_q[read_index] * lim->current_gain;

    lim->write_index++;
    if (lim->write_index >= LOOKAHEAD_MAX_SAMPLES) lim->write_index = 0;
}

// ============================================================================
// CESSB INITIALIZATION AND CONFIGURATION
// ============================================================================

void cessb_init(cessb_state_t *state, float sample_rate) {
    memset(state, 0, sizeof(cessb_state_t));

    state->enabled = CESSB_DISABLED;
    state->clip_level = CESSB_CLIP_LEVEL;
    state->envelope_limit = CESSB_ENVELOPE_LIMIT;
    state->sample_rate = sample_rate;

    state->hilbert_index = 0;
    state->delay_index = 0;

    state->overshoot_i_index = 0;
    state->overshoot_q_index = 0;

    state->delay2_i_index = 0;
    state->delay2_q_index = 0;

    // Initialize input AGC
    input_agc_init(&state->input_agc, sample_rate);

    // Initialize look-ahead limiter
    lookahead_limiter_init_vec(&state->lookahead, sample_rate);

    cessb_reset_stats(state);
}

// ============================================================================
// EXTERNAL CONTROL API
// these are here for some future possible UI controls to use
// ============================================================================

void cessb_set_enabled(cessb_state_t *state, int enabled) {
    state->enabled = enabled;
    cessb_enabled = enabled;
}

void cessb_set_clip_level(cessb_state_t *state, float level) {
    if (level > 0.0f && level <= 1.0f) {
        state->clip_level = level;
    }
}

void cessb_set_envelope_limit(cessb_state_t *state, float limit) {
    if (limit > 0.0f && limit <= 1.5f) {
        state->envelope_limit = limit;
    }
}

void cessb_set_lookahead_samples(cessb_state_t *state, int samples) {
    if (samples < 1) samples = 1;
    if (samples > LOOKAHEAD_MAX_SAMPLES) samples = LOOKAHEAD_MAX_SAMPLES;
    state->lookahead.lookahead_samples = samples;
}

void cessb_set_lookahead_ms(cessb_state_t *state, float milliseconds) {
    int samples = (int)((milliseconds / 1000.0f) * state->sample_rate + 0.5f);
    cessb_set_lookahead_samples(state, samples);
}

void cessb_set_attack_ms(cessb_state_t *state, float attack_ms) {
    state->lookahead.attack_coeff = time_constant_to_coeff(attack_ms, state->sample_rate);
}

void cessb_set_release_ms(cessb_state_t *state, float release_ms) {
    state->lookahead.release_coeff = time_constant_to_coeff(release_ms, state->sample_rate);
}

int cessb_get_lookahead_samples(cessb_state_t *state) {
    return state->lookahead.lookahead_samples;
}

int cessb_is_enabled(cessb_state_t *state) {
    return state->enabled;
}

// AGC control functions
void cessb_set_agc_target(cessb_state_t *state, float target) {
    if (target > 0.0f && target <= 1.0f) {
        state->input_agc.target_level = target;
    }
}

void cessb_set_agc_max_gain(cessb_state_t *state, float max_gain) {
    if (max_gain >= 1.0f) {
        state->input_agc.max_gain = max_gain;
    }
}

void cessb_set_agc_min_gain(cessb_state_t *state, float min_gain) {
    if (min_gain > 0.0f && min_gain <= 1.0f) {
        state->input_agc.min_gain = min_gain;
    }
}

float cessb_get_agc_gain(cessb_state_t *state) {
    return state->input_agc.agc_gain;
}

// Debug print
void cessb_debug_print_stats(cessb_state_t *state) {
    float peak_reduction_db = 0.0f;
    float avg_power_gain_db = 0.0f;
    float talk_power_db = 0.0f;

    cessb_get_stats(state, &peak_reduction_db, &avg_power_gain_db, &talk_power_db);

    printf("CESSB stats:\n");
    printf("  samples processed      : %ld\n", (long)state->sample_count);
    printf("  AGC gain               : %8.4f (%.1f dB)\n",
           state->input_agc.agc_gain, 20.0f * log10f(state->input_agc.agc_gain));
    printf("  peak in (raw)          : %8.4f\n", state->peak_input);
    printf("  peak after AGC         : %8.4f\n", state->peak_after_agc);
    printf("  peak after clip        : %8.4f\n", state->peak_after_clip);
    printf("  peak after overshoot   : %8.4f\n", state->peak_after_overshoot);
    printf("  peak out               : %8.4f\n", state->peak_output);
    printf("  min limiter gain       : %8.4f\n", state->min_limiter_gain);
    printf("  peak reduction (dB)    : %8.2f dB\n", peak_reduction_db);
    printf("  avg power gain (dB)    : %8.2f dB\n", avg_power_gain_db);
    printf("  avg power @ equal PEP  : %8.2f dB\n", talk_power_db);
}

// ============================================================================
// AUDIO CAPTURE CODE (for debugging)
// currently captures audio in and out for first 5 seconds for development use
// ============================================================================

#if REC_AUDIO
#include <time.h>

static FILE *rec_in_file  = NULL;
static FILE *rec_out_file = NULL;
static unsigned long rec_samples_in_segment = 0;
static int rec_done = 0;

static inline unsigned long rec_segment_max_samples(void) {
    return (unsigned long)(CESSB_SAMPLE_RATE * REC_AUDIO_SEGMENT_SECONDS);
}

static void rec_make_filenames(char *in_name, size_t in_sz,
                               char *out_name, size_t out_sz) {
    time_t now = time(NULL);
    struct tm *tm = localtime(&now);
    strftime(in_name,  in_sz,  "cessb_in_%Y%m%d_%H%M%S.raw", tm);
    strftime(out_name, out_sz, "cessb_out_%Y%m%d_%H%M%S.raw", tm);
}

static void rec_open_segment(void) {
    if (rec_done || rec_in_file || rec_out_file) return;

    char in_name[128], out_name[128];
    rec_make_filenames(in_name, sizeof(in_name), out_name, sizeof(out_name));

    rec_in_file  = fopen(in_name,  "wb");
    rec_out_file = fopen(out_name, "wb");

    if (!rec_in_file || !rec_out_file) {
        if (rec_in_file)  { fclose(rec_in_file);  rec_in_file = NULL; }
        if (rec_out_file) { fclose(rec_out_file); rec_out_file = NULL; }
        rec_done = 1;
        return;
    }

    rec_samples_in_segment = 0;
}

static void rec_close_segment(void) {
    if (rec_in_file)  { fclose(rec_in_file);  rec_in_file = NULL; }
    if (rec_out_file) { fclose(rec_out_file); rec_out_file = NULL; }
    rec_done = 1;
}

static inline void rec_note_block(int num_samples) {
    if (rec_done) return;
    rec_samples_in_segment += (unsigned long)num_samples;
    if (rec_samples_in_segment >= rec_segment_max_samples()) {
        rec_close_segment();
    }
}
#endif

// ============================================================================
// MAIN CESSB PROCESSING (float version - internal)
// ============================================================================

void cessb_process(cessb_state_t *state, float *samples, int num_samples) {
    if (!state->enabled) {
        return;
    }

    int hilbert_delay_len = HILBERT_DELAY_LEN;
    int overshoot_delay_len = OVERSHOOT_DELAY_LEN;

    for (int i = 0; i < num_samples; i++) {
        float sample = samples[i];

        // STAGE 1: Hilbert envelope detection
        float q = apply_fir_filter(hilbert_coeffs, state->hilbert_delay,
                                   &state->hilbert_index, HILBERT_TAPS, sample);

        float i_delayed = get_delayed_sample(state->delay_line, &state->delay_index,
                                             hilbert_delay_len, sample);

        float envelope = sqrtf(i_delayed * i_delayed + q * q);

        // STAGE 2: Hard clip based on envelope
        float clipped_i;
        float clipped_q;
        if (envelope > state->clip_level && envelope > 1e-10f) {
            float gain = state->clip_level / envelope;
            clipped_i = i_delayed * gain;
            clipped_q = q * gain;
        } else {
            clipped_i = i_delayed;
            clipped_q = q;
        }

        float abs_clip = sqrtf(clipped_i * clipped_i + clipped_q * clipped_q);
        if (abs_clip > state->peak_after_clip) {
            state->peak_after_clip = abs_clip;
        }

        // STAGE 3: Overshoot control filter — operate on both I and Q
        float filtered_i =
            apply_fir_filter(overshoot_coeffs, state->overshoot_i_delay,
                             &state->overshoot_i_index, OVERSHOOT_FILTER_TAPS, clipped_i);
        float filtered_q =
            apply_fir_filter(overshoot_coeffs, state->overshoot_q_delay,
                             &state->overshoot_q_index, OVERSHOOT_FILTER_TAPS, clipped_q);

        float abs_filt = sqrtf(filtered_i * filtered_i + filtered_q * filtered_q);
        if (abs_filt > state->peak_after_overshoot) {
            state->peak_after_overshoot = abs_filt;
        }

        // STAGE 4: Second envelope detection — use delayed I and Q
        float i2_delayed = get_delayed_sample(state->delay2_i, &state->delay2_i_index,
                                              overshoot_delay_len, filtered_i);
        float q2_delayed = get_delayed_sample(state->delay2_q, &state->delay2_q_index,
                                              overshoot_delay_len, filtered_q);
        float envelope2 = sqrtf(i2_delayed * i2_delayed + q2_delayed * q2_delayed);

        // STAGE 5: Look-ahead limiter (for I and Q)
        float limited_i, limited_q;
        lookahead_limiter_process_vec(&state->lookahead, i2_delayed, q2_delayed, envelope2,
                                      state->envelope_limit, &limited_i, &limited_q);

        if (state->lookahead.current_gain < state->min_limiter_gain) {
            state->min_limiter_gain = state->lookahead.current_gain;
        }

        // STAGE 6: Post-limiter lowpass filter
        float output = apply_biquad_cascade(post_lpf_coeffs, state->post_lpf_state,
                                            POST_LPF_BIQUAD_STAGES, limited_i);

        state->sample_count++;

        // Measure output
        float abs_out = fabsf(output);
        if (abs_out > state->peak_output) {
            state->peak_output = abs_out;
        }
        state->average_power_out += output * output;

        samples[i] = output;
    }
}

// ============================================================================
// MAIN CESSB PROCESSING (int32 version - public API)
// ============================================================================

void cessb_process_int32(cessb_state_t *state, int32_t *samples, int num_samples) {
    if (num_samples < 0 || num_samples > 1024 || !state->enabled) return;

#if REC_AUDIO
    rec_open_segment();
    if (!rec_done && rec_in_file) {
        (void)fwrite(samples, sizeof(int32_t), num_samples, rec_in_file);
    }
#endif

    float temp_buffer[1024];

    // Convert int32 to float normalized to [-1, 1]
    for (int i = 0; i < num_samples; i++) {
        float s = (float)samples[i] / 2147483648.0f;
        temp_buffer[i] = s;

        // Track peak input (before AGC)
        float abs_s = fabsf(s);
        if (abs_s > state->peak_input) {
            state->peak_input = abs_s;
        }
        state->average_power_in += s * s;
    }

    // Apply input AGC to normalize levels
    for (int i = 0; i < num_samples; i++) {
        temp_buffer[i] = apply_input_agc(&state->input_agc, temp_buffer[i]);

        // Track peak after AGC
        float abs_agc = fabsf(temp_buffer[i]);
        if (abs_agc > state->peak_after_agc) {
            state->peak_after_agc = abs_agc;
        }
    }

    // Run CESSB processing
    cessb_process(state, temp_buffer, num_samples);

    // Convert float back to int32 and
    // scale output to match downstream limiter threshold (0.04 float equivalent)
    // Without this, CESSB output would be clipped by downstream processing
    for (int i = 0; i < num_samples; i++) {
        float out = temp_buffer[i] * 0.04f * 2147483647.0f;
        // we will never come close to these limits
        if (out > 2147483647.0f) out = 2147483647.0f;
        if (out < -2147483648.0f) out = -2147483648.0f;
        samples[i] = (int32_t)out;
    }

#if REC_AUDIO
    if (!rec_done && rec_out_file) {
        (void)fwrite(samples, sizeof(int32_t), num_samples, rec_out_file);
    }
    rec_note_block(num_samples);
#endif
}

// ============================================================================
// STATISTICS
// ============================================================================

void cessb_get_stats(cessb_state_t *state, float *peak_reduction_db,
                     float *avg_power_gain_db, float *talk_power_db) {
    if (state->sample_count == 0) {
        *peak_reduction_db = 0.0f;
        *avg_power_gain_db = 0.0f;
        *talk_power_db = 0.0f;
        return;
    }

    // Peak reduction (comparing raw input to output)
    if (state->peak_input > 1e-10f && state->peak_output > 1e-10f) {
        *peak_reduction_db = 20.0f * log10f(state->peak_output / state->peak_input);
    } else {
        *peak_reduction_db = 0.0f;
    }

    // Average power gain
    float avg_in  = state->average_power_in  / state->sample_count;
    float avg_out = state->average_power_out / state->sample_count;
    if (avg_in > 1e-10f && avg_out > 1e-10f) {
        *avg_power_gain_db = 10.0f * log10f(avg_out / avg_in);
    } else {
        *avg_power_gain_db = 0.0f;
    }

    // Talk power @ equal PEP
    if (avg_in > 1e-10f && avg_out > 1e-10f &&
        state->peak_input > 1e-10f && state->peak_output > 1e-10f) {
        float peak_norm = state->peak_input / state->peak_output;
        float scaled_out = avg_out * peak_norm * peak_norm;
        *talk_power_db = 10.0f * log10f(scaled_out / avg_in);
    } else {
        *talk_power_db = 0.0f;
    }
}

void cessb_reset_stats(cessb_state_t *state) {
    state->peak_input = 0.0f;
    state->peak_after_agc = 0.0f;
    state->peak_output = 0.0f;
    state->peak_after_clip = 0.0f;
    state->peak_after_overshoot = 0.0f;
    state->average_power_in = 0.0f;
    state->average_power_out = 0.0f;
    state->min_limiter_gain = 1.0f;
    state->sample_count = 0;
}


