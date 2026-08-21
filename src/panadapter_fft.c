#include "panadapter_fft.h"
#include "sdr.h"

#include <complex.h>
#include <fftw3.h>
#include <math.h>
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define MIN_ANALYSIS_BANDWIDTH 5000
#define FILTER_TAPS 127
/*
 * Post-decimation sample rate as a percentage of displayed bandwidth.
 * 250 means 2.5x; raise it for more anti-alias guard band or lower it for
 * finer FFT resolution.
 */
#define DECIMATION_GUARD_PERCENT 250
#define MAX_DECIMATION ((100 * SDR_SAMPLE_RATE) / \
	(DECIMATION_GUARD_PERCENT * MIN_ANALYSIS_BANDWIDTH))
#define MAX_RAW_SAMPLES ((PANADAPTER_FFT_MAX_BINS - 1) * MAX_DECIMATION + FILTER_TAPS)
#define WORK_SIZE (PANADAPTER_FFT_MAX_BINS * (MAX_DECIMATION + 1))
/*
 * The I/Q retention ring is sized per context at create() time (see
 * panadapter_fft_ring_size()); PANADAPTER_FFT_MAX_HISTORY_SAMPLES is about 10.9
 * seconds and 8 MiB at 96 ksample/s. The retention window limits how much
 * existing waterfall history can be reanalyzed and rerendered after zooming or
 * panning, and history_limit narrows it further at runtime.
 */
#define HISTORY_FLOOR_SAMPLES (MAX_RAW_SAMPLES + PANADAPTER_FFT_MAX_BINS)
/*
 * Display-level calibration expressed as an equivalent FFT length. The value
 * 2048 makes the variable-length, decimated analysis match the noise floor of
 * the original 2048-bin analyzer. It only affects length_scale, not the FFT
 * size or resolution: increasing it raises every displayed bin, with each
 * doubling adding about 3 dB; decreasing it lowers them by the same amount.
 */
#define FFT_LEVEL_REFERENCE_BINS 2048
// Blend 30% of each new non-CW spectrum into the displayed frame.
#define NON_CW_NEW_FRAME_WEIGHT 0.3f

_Static_assert(WORK_SIZE >= MAX_RAW_SAMPLES, "panadapter FFT work area must hold the largest analysis window");
_Static_assert(PANADAPTER_FFT_MIN_HISTORY_SAMPLES >= HISTORY_FLOOR_SAMPLES,
  "smallest panadapter FFT history must still satisfy the history retention guard");
_Static_assert((PANADAPTER_FFT_MIN_HISTORY_SAMPLES & (PANADAPTER_FFT_MIN_HISTORY_SAMPLES - 1)) == 0,
  "panadapter FFT history size must be a power of two");
_Static_assert((PANADAPTER_FFT_MAX_HISTORY_SAMPLES & (PANADAPTER_FFT_MAX_HISTORY_SAMPLES - 1)) == 0,
  "panadapter FFT history size must be a power of two");

struct panadapter_fft {
  fftwf_complex *sample_ring;
  uint64_t ring_size;  // Power of two, fixed at create(); the mask is ring_size - 1.
  uint64_t ring_mask;
  /*
   * How far back reconstructed history may reach, in samples. Written from any
   * thread, read by the worker. Relaxed access is sufficient: the ring is always
   * written in full with the fixed mask, so every index the worker computes is a
   * slot the audio thread genuinely maintains. A stale read only changes how many
   * rows one batch renders, never whether an index is valid.
   */
  _Atomic uint64_t history_limit;
  _Atomic uint64_t samples_written;
  _Atomic uint64_t history_epoch;

  fftwf_complex raw_work[WORK_SIZE];
  fftwf_complex *fft_data;
  fftwf_plan fft_plan;
  int planned_fft_bins;
  float filter_coeff[FILTER_TAPS];
  int filter_bandwidth_hz;
  float smoothed_bins[PANADAPTER_FFT_FRAME_BINS];
  struct panadapter_fft_config smoothed_config;
  bool smoothed_config_valid;

  pthread_t worker_thread;
  pthread_mutex_t state_mutex;
  pthread_cond_t request_cond;
  struct panadapter_fft_config pending_config;
  uint64_t pending_serial;
  uint64_t last_request_ms;
  bool request_pending;
  bool last_request_valid;
  bool stop_worker;
  bool reset_smoothing;
  struct panadapter_fft_frame published_frame;

  struct panadapter_fft_config history_batch_config;
  uint64_t *history_batch_ends;
  int history_batch_count;
  uint64_t history_batch_generation;
  uint64_t history_batch_epoch;
  bool history_batch_pending;
  bool history_batch_busy;
  struct panadapter_fft_frame *published_history_batch;
  int published_history_batch_count;
  uint64_t published_history_batch_generation;
};

/** Return whether two configurations produce equivalent spectrum frames. */
static bool fft_configs_equal(const struct panadapter_fft_config *a, const struct panadapter_fft_config *b) {
  return a->display_span_hz == b->display_span_hz
         && a->center_hz == b->center_hz
         && a->is_cw == b->is_cw
         && a->is_tx == b->is_tx
         && a->display_width_px == b->display_width_px
         && a->max_bins == b->max_bins
         && (!a->is_cw || a->wpm == b->wpm);
}

static uint64_t monotonic_ms(void) {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return (uint64_t) now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

/** Return the sample count to observe, capped at one dit for CW. */
static int observation_samples(const struct panadapter_fft_config *config, int decimation, int fft_bins) {
  if (!config->is_cw)
    return fft_bins;

  const int wpm = config->wpm > 0 ? config->wpm : 1;
  const double sample_rate = (double) SDR_SAMPLE_RATE / decimation;
  int count = (int) lround(sample_rate * 1.2 / wpm);
  if (count < 1) count = 1;
  if (count > fft_bins) count = fft_bins;
  return count;
}

/** Reuse or create the in-place FFTW buffer and plan for the requested size. */
static bool ensure_fft_plan(struct panadapter_fft *state, int fft_bins) {
  if (fft_bins == state->planned_fft_bins)
    return true;

  fftwf_complex *const new_data = fftwf_alloc_complex(fft_bins);
  if (!new_data)
    return false;
  fftwf_plan new_plan = fftwf_plan_dft_1d(fft_bins, new_data, new_data, FFTW_FORWARD, FFTW_ESTIMATE);
  if (!new_plan) {
    fftwf_free(new_data);
    return false;
  }

  if (state->fft_plan) fftwf_destroy_plan(state->fft_plan);
  if (state->fft_data) fftwf_free(state->fft_data);
  state->fft_data = new_data;
  state->fft_plan = new_plan;
  state->planned_fft_bins = fft_bins;
  printf("Panadapter FFT bins: %d\n", fft_bins);
  return true;
}

/** Design a normalized Hamming-windowed sinc low-pass filter. */
static void make_filter(struct panadapter_fft *state, int bandwidth_hz) {
  const int middle = FILTER_TAPS / 2;
  const double cutoff = 0.55 * bandwidth_hz / SDR_SAMPLE_RATE;
  double sum = 0.0;

  for (int tap = 0; tap < FILTER_TAPS; tap++) {
    const int offset = tap - middle;
    const double sinc = offset == 0
                          ? 2.0 * cutoff
                          : sin(2.0 * M_PI * cutoff * offset) / (M_PI * offset);
    const double window = 0.54 - 0.46 * cos(2.0 * M_PI * tap / (FILTER_TAPS - 1));
    state->filter_coeff[tap] = (float) (sinc * window);
    sum += state->filter_coeff[tap];
  }

  for (int tap = 0; tap < FILTER_TAPS; tap++)
    state->filter_coeff[tap] /= (float) sum;
}

/*
 * Derived geometry and scaling for one analysis configuration.
 * prepare_analysis() converts the requested span, display width and CW timing
 * into these FFT, decimation and visible-bin parameters once, so live frames
 * and reconstructed waterfall history use identical calculations.
 */
struct analysis_layout {
  int decimation;
  int target_bins;
  int fft_bins;
  int observed;
  int raw_count;
  int half_bins;
  int visible_bins;
  double bin_hz;
  float length_scale;
};

/** Derive and prepare the analysis parameters shared by live and historical frames. */
static bool prepare_analysis(struct panadapter_fft *state,
                             const struct panadapter_fft_config *config,
                             struct analysis_layout *layout) {
  if (config->display_span_hz < 1 || config->display_width_px < 1)
    return false;

  const int bandwidth = config->display_span_hz > MIN_ANALYSIS_BANDWIDTH
                          ? config->display_span_hz
                          : MIN_ANALYSIS_BANDWIDTH;
  const int guarded_decimation = (100 * SDR_SAMPLE_RATE) / (DECIMATION_GUARD_PERCENT * bandwidth);
  layout->decimation = guarded_decimation > 0 ? guarded_decimation : 1;
  const double output_rate = (double) SDR_SAMPLE_RATE / layout->decimation;
  layout->target_bins = config->display_width_px < PANADAPTER_FFT_FRAME_BINS
                          ? config->display_width_px
                          : PANADAPTER_FFT_FRAME_BINS;

  int max_bins = config->max_bins > 0 ? config->max_bins : PANADAPTER_FFT_MAX_BINS;
  if (max_bins > PANADAPTER_FFT_MAX_BINS) max_bins = PANADAPTER_FFT_MAX_BINS;
  if (max_bins < PANADAPTER_FFT_MIN_BINS) max_bins = PANADAPTER_FFT_MIN_BINS;

  layout->fft_bins = PANADAPTER_FFT_MIN_BINS;
  while (layout->fft_bins < max_bins) {
    const double bin_hz = output_rate / layout->fft_bins;
    const int half_bins = (int) floor(config->display_span_hz / (2.0 * bin_hz));
    if (2 * half_bins + 1 >= layout->target_bins)
      break;
    layout->fft_bins *= 2;
  }
  if (!ensure_fft_plan(state, layout->fft_bins))
    return false;

  layout->observed = observation_samples(config, layout->decimation, layout->fft_bins);
  layout->raw_count = (layout->observed - 1) * layout->decimation + FILTER_TAPS;
  layout->bin_hz = output_rate / layout->fft_bins;
  layout->half_bins = (int) floor(config->display_span_hz / (2.0 * layout->bin_hz));
  const int max_half_bins = layout->fft_bins > 1 ? layout->fft_bins / 2 - 1 : 0;
  if (layout->half_bins > max_half_bins)
    layout->half_bins = max_half_bins;
  layout->visible_bins = 2 * layout->half_bins + 1;
  layout->length_scale = sqrtf((float) (FFT_LEVEL_REFERENCE_BINS * layout->decimation) / layout->observed);

  if (bandwidth != state->filter_bandwidth_hz) {
    make_filter(state, bandwidth);
    state->filter_bandwidth_hz = bandwidth;
  }
  return true;
}

/**
 * Convert the current FFT output into a display frame. Live spectrum frames
 * enable smoothing to blend successive updates. Reconstructed waterfall rows
 * disable it so each row represents only its own historical sample window.
 */
static void fill_frame(struct panadapter_fft *state,
                       const struct panadapter_fft_config *config,
                       const struct analysis_layout *layout, bool smooth,
                       uint64_t sample_end, uint64_t sample_end_ms,
                       struct panadapter_fft_frame *frame) {
  frame->count = layout->visible_bins < layout->target_bins
                   ? layout->visible_bins
                   : layout->target_bins;
  frame->first_hz = config->center_hz + layout->half_bins * layout->bin_hz;
  frame->bin_step_hz = frame->count > 1
                         ? -(layout->visible_bins - 1) * layout->bin_hz / (frame->count - 1)
                         : -layout->bin_hz;
  frame->decimation = layout->decimation;
  frame->observation_samples = layout->observed;
  frame->fft_bins = layout->fft_bins;
  frame->sample_end = sample_end;
  frame->sample_end_ms = sample_end_ms;
  frame->config = *config;

  if (smooth && (!state->smoothed_config_valid || !fft_configs_equal(config, &state->smoothed_config))) {
    memset(state->smoothed_bins, 0, sizeof(state->smoothed_bins));
    state->smoothed_config = *config;
    state->smoothed_config_valid = true;
  }

  const float new_frame_weight = !smooth || config->is_cw ? 1.0f : NON_CW_NEW_FRAME_WEIGHT;
  for (int output = 0; output < frame->count; output++) {
    const int first_visible = output * layout->visible_bins / frame->count;
    const int end_visible = (output + 1) * layout->visible_bins / frame->count;
    float magnitude = 0.0f;
    for (int visible = first_visible; visible < end_visible; visible++) {
      const int signed_bin = layout->half_bins - visible;
      const int fft_bin = signed_bin >= 0 ? signed_bin : layout->fft_bins + signed_bin;
      magnitude = fmaxf(magnitude, cabsf(state->fft_data[fft_bin]));
    }
    if (smooth) {
      state->smoothed_bins[output] = (1.0f - new_frame_weight) *
                                     state->smoothed_bins[output] + new_frame_weight * magnitude;
      magnitude = state->smoothed_bins[output];
    }
    frame->bins[output] = (int) lroundf(20.0f * log10f(fmaxf(magnitude, 1.0e-12f)));
  }
}

static float hann_window(int index, int count) {
  return count == 1 ? 1.0f : 0.5f - 0.5f * cosf(2.0f * (float) M_PI * index / (count - 1));
}

/** Copy the newest complete analysis window without blocking audio. */
static bool snapshot_samples(struct panadapter_fft *state, int count,
                             uint64_t *first_sample, uint64_t *sample_end,
                             uint64_t *sample_end_ms) {
  const uint64_t current = atomic_load_explicit(&state->samples_written, memory_order_acquire);
  if (current < (uint64_t) count || (uint64_t) count > state->ring_size)
    return false;
  *sample_end = current;
  *sample_end_ms = monotonic_ms();

  const uint64_t start = current - (uint64_t) count;
  const size_t sample_count = (size_t) count;
  const size_t ring_index = (size_t) (start & state->ring_mask);
  size_t first_count = state->ring_size - ring_index;
  if (first_count > sample_count) first_count = sample_count;
  memcpy(state->raw_work, state->sample_ring + ring_index, first_count * sizeof(state->raw_work[0]));
  memcpy(state->raw_work + first_count, state->sample_ring, (sample_count - first_count) * sizeof(state->raw_work[0]));

  if (atomic_load_explicit(&state->samples_written, memory_order_acquire) - start > state->ring_size)
    return false;
  *first_sample = start;
  return true;
}

/** Mix the requested analysis center down to DC in place. */
static void shift_buffer(fftwf_complex *samples, int count, int center_hz, uint64_t first_sample) {
  if (center_hz == 0) {
    return;
  }

  const double step = -2.0 * M_PI * center_hz / SDR_SAMPLE_RATE;
  const double phase = fmod(step * (double) (first_sample % SDR_SAMPLE_RATE), 2.0 * M_PI);
  float oscillator_i = cosf((float) phase);
  float oscillator_q = sinf((float) phase);
  const float step_i = cosf((float) step);
  const float step_q = sinf((float) step);

  for (int index = 0; index < count; index++) {
    samples[index] *= oscillator_i + I * oscillator_q;

    const float next_i = oscillator_i * step_i - oscillator_q * step_q;
    oscillator_q = oscillator_i * step_q + oscillator_q * step_i;
    oscillator_i = next_i;
    if ((index & 1023) == 1023) {
      const float magnitude = hypotf(oscillator_i, oscillator_q);
      oscillator_i /= magnitude;
      oscillator_q /= magnitude;
    }
  }
}

/** Produce one cropped, smoothed display spectrum from the newest samples. */
static bool analyze(struct panadapter_fft *state,
                    const struct panadapter_fft_config *config,
                    struct panadapter_fft_frame *frame) {
  struct analysis_layout layout;
  if (!prepare_analysis(state, config, &layout))
    return false;

  uint64_t first_sample;
  uint64_t sample_end;
  uint64_t sample_end_ms;

  if (!snapshot_samples(state, layout.raw_count, &first_sample, &sample_end, &sample_end_ms))
    return false;

  shift_buffer(state->raw_work, layout.raw_count, config->center_hz, first_sample);
  memset(state->fft_data, 0, sizeof(*state->fft_data) * layout.fft_bins);

  /* Keep the level stable across observation lengths and decimation. */
  for (int output = 0; output < layout.observed; output++) {
    const int newest = FILTER_TAPS - 1 + output * layout.decimation;
    fftwf_complex sum = 0.0f;
    for (int tap = 0; tap < FILTER_TAPS; tap++) {
      const int input = newest - tap;
      sum += state->filter_coeff[tap] * state->raw_work[input];
    }
    state->fft_data[output] = sum * hann_window(output, layout.observed) * layout.length_scale;
  }

  fftwf_execute(state->fft_plan);
  fill_frame(state, config, &layout, true, sample_end, sample_end_ms, frame);
  return true;
}

static void free_history_work(fftwf_complex *input, fftwf_complex *filtered, unsigned char *computed) {
  if (input) fftwf_free(input);
  if (filtered) fftwf_free(filtered);
  free(computed);
}

/** Build historical rows together, sharing the I/Q copy, mixer and FIR results. */
static struct panadapter_fft_frame *analyze_history_batch(
  struct panadapter_fft *state, const struct panadapter_fft_config *config,
  const uint64_t *sample_ends, int count, uint64_t history_epoch) {
  if (count < 1)
    return NULL;
  struct panadapter_fft_frame *frames = calloc((size_t) count, sizeof(*frames));
  if (!frames)
    return frames;
  for (int row = 0; row < count; row++) {
    frames[row].sample_end = sample_ends[row];
    frames[row].config = *config;
  }
  if (atomic_load_explicit(&state->history_epoch, memory_order_relaxed) != history_epoch)
    return frames;

  struct analysis_layout layout;
  if (!prepare_analysis(state, config, &layout))
    return frames;

  const uint64_t current = atomic_load_explicit(&state->samples_written, memory_order_acquire);
  const uint64_t limit = atomic_load_explicit(&state->history_limit, memory_order_relaxed);
  uint64_t first_sample = UINT64_MAX;
  uint64_t last_sample = 0;
  for (int row = 0; row < count; row++) {
    const uint64_t end = sample_ends[row];
    if (end > current || end < (uint64_t) layout.raw_count ||
        current - end + layout.raw_count + PANADAPTER_FFT_MAX_BINS > limit)
      continue;
    const uint64_t start = end - layout.raw_count;
    if (start < first_sample) first_sample = start;
    if (end > last_sample) last_sample = end;
  }
  if (first_sample == UINT64_MAX)
    return frames;

  const size_t input_count = last_sample - first_sample;
  const size_t filtered_count = input_count - FILTER_TAPS + 1;
  fftwf_complex *input = fftwf_alloc_complex(input_count);
  fftwf_complex *filtered = fftwf_alloc_complex(filtered_count);
  unsigned char *computed = calloc(filtered_count, 1);
  if (!input || !filtered || !computed) {
    free_history_work(input, filtered, computed);
    return frames;
  }

  const size_t ring_index = first_sample & state->ring_mask;
  size_t first_count = state->ring_size - ring_index;
  if (first_count > input_count) first_count = input_count;
  memcpy(input, state->sample_ring + ring_index, first_count * sizeof(*input));
  memcpy(input + first_count, state->sample_ring, (input_count - first_count) * sizeof(*input));
  if (atomic_load_explicit(&state->samples_written, memory_order_acquire) - first_sample > state->ring_size) {
    free_history_work(input, filtered, computed);
    return frames;
  }

  shift_buffer(input, (int) input_count, config->center_hz, first_sample);
  if (atomic_load_explicit(&state->history_epoch, memory_order_relaxed) !=
      history_epoch) {
    free_history_work(input, filtered, computed);
    return frames;
  }

  for (int row = 0; row < count; row++) {
    if (atomic_load_explicit(&state->history_epoch, memory_order_relaxed) != history_epoch)
      break;
    const uint64_t end = sample_ends[row];
    if (end > last_sample || end < (uint64_t) layout.raw_count || end - layout.raw_count < first_sample)
      continue;
    memset(state->fft_data, 0, sizeof(*state->fft_data) * layout.fft_bins);
    for (int output = 0; output < layout.observed; output++) {
      const uint64_t newest = end - 1 - (uint64_t) (layout.observed - 1 - output) * layout.decimation;
      const size_t filtered_index = newest - (first_sample + FILTER_TAPS - 1);
      if (!computed[filtered_index]) {
        const size_t input_index = newest - first_sample;
        fftwf_complex sum = 0.0f;
        for (int tap = 0; tap < FILTER_TAPS; tap++)
          sum += state->filter_coeff[tap] * input[input_index - tap];
        filtered[filtered_index] = sum;
        computed[filtered_index] = 1;
      }
      state->fft_data[output] = filtered[filtered_index] *
                                hann_window(output, layout.observed) * layout.length_scale;
    }
    fftwf_execute(state->fft_plan);
    fill_frame(state, config, &layout, false, end, 0, &frames[row]);
  }

  free_history_work(input, filtered, computed);
  return frames;
}

/** Process coalesced requests and publish only the newest analysis result. */
static void *panadapter_fft_worker(void *context) {
  struct panadapter_fft *const state = context;
  while (true) {
    pthread_mutex_lock(&state->state_mutex);
    while (!state->request_pending && !state->history_batch_pending && !state->stop_worker)
      pthread_cond_wait(&state->request_cond, &state->state_mutex);
    if (state->stop_worker) {
      pthread_mutex_unlock(&state->state_mutex);
      break;
    }
    const bool batch = !state->request_pending && state->history_batch_pending;
    const struct panadapter_fft_config config = batch ? state->history_batch_config : state->pending_config;
    const uint64_t serial = batch ? state->history_batch_generation : state->pending_serial;
    uint64_t *batch_ends = NULL;
    int batch_count = 0;
    const uint64_t history_epoch = batch
                                     ? state->history_batch_epoch
                                     : atomic_load_explicit(&state->history_epoch, memory_order_relaxed);
    const bool reset = !batch && state->reset_smoothing;
    if (batch) {
      batch_ends = state->history_batch_ends;
      batch_count = state->history_batch_count;
      state->history_batch_ends = NULL;
      state->history_batch_pending = false;
      state->history_batch_busy = true;
    } else {
      state->reset_smoothing = false;
      state->request_pending = false;
    }
    pthread_mutex_unlock(&state->state_mutex);
    if (reset)
      state->smoothed_config_valid = false;

    struct panadapter_fft_frame frame = {.count = 0};
    struct panadapter_fft_frame *batch_frames = batch
                                                  ? analyze_history_batch( state, &config, batch_ends, batch_count, history_epoch)
                                                  : NULL;
    free(batch_ends);
    const bool analyzed = batch ? false : analyze(state, &config, &frame);

    pthread_mutex_lock(&state->state_mutex);
    if (batch) {
      state->history_batch_busy = false;
      free(state->published_history_batch);
      state->published_history_batch = batch_frames;
      state->published_history_batch_count = batch_count;
      state->published_history_batch_generation = serial;
    } else if (analyzed && serial == state->pending_serial) {
      frame.generation = serial;
      state->published_frame = frame;
    }
    pthread_mutex_unlock(&state->state_mutex);
  }
  if (state->fft_plan) fftwf_destroy_plan(state->fft_plan);
  if (state->fft_data) fftwf_free(state->fft_data);
  return NULL;
}

/** Round a requested retention to the nearest usable power-of-two ring size. */
uint64_t panadapter_fft_ring_size(uint64_t requested_samples) {
  uint64_t size = PANADAPTER_FFT_MIN_HISTORY_SAMPLES;
  while (size < PANADAPTER_FFT_MAX_HISTORY_SAMPLES &&
         requested_samples >= size + size / 2)
    size *= 2;
  return size;
}

void panadapter_fft_set_history_limit(struct panadapter_fft *state, uint64_t samples) {
  if (!state)
    return;
  if (samples < HISTORY_FLOOR_SAMPLES)
    samples = HISTORY_FLOOR_SAMPLES;
  if (samples > state->ring_size)
    samples = state->ring_size;
  atomic_store_explicit(&state->history_limit, samples, memory_order_relaxed);
}

uint64_t panadapter_fft_history_limit(const struct panadapter_fft *state) {
  if (!state)
    return 0;
  return atomic_load_explicit(&state->history_limit, memory_order_relaxed);
}

struct panadapter_fft *panadapter_fft_create(uint64_t history_samples) {
  struct panadapter_fft *const state = calloc(1, sizeof(*state));
  if (!state)
    return NULL;
  state->ring_size = panadapter_fft_ring_size(history_samples);
  state->ring_mask = state->ring_size - 1;
  state->sample_ring = fftwf_alloc_complex(state->ring_size);
  if (!state->sample_ring) {
    free(state);
    return NULL;
  }
  // fftwf_alloc_complex() does not zero. Uninitialised bit patterns can be NaN,
  // and lroundf(log10f(NaN)) in fill_frame() is undefined, so clear it up front.
  memset(state->sample_ring, 0, state->ring_size * sizeof(*state->sample_ring));

  atomic_init(&state->history_limit, state->ring_size);
  atomic_init(&state->samples_written, 0);
  atomic_init(&state->history_epoch, 0);
  if (pthread_mutex_init(&state->state_mutex, NULL) != 0) {
    fftwf_free(state->sample_ring);
    free(state);
    return NULL;
  }
  if (pthread_cond_init(&state->request_cond, NULL) != 0) {
    pthread_mutex_destroy(&state->state_mutex);
    fftwf_free(state->sample_ring);
    free(state);
    return NULL;
  }
  if (pthread_create(&state->worker_thread, NULL, panadapter_fft_worker, state) != 0) {
    pthread_cond_destroy(&state->request_cond);
    pthread_mutex_destroy(&state->state_mutex);
    fftwf_free(state->sample_ring);
    free(state);
    return NULL;
  }
  return state;
}

void panadapter_fft_destroy(struct panadapter_fft *state) {
  if (!state)
    return;

  pthread_mutex_lock(&state->state_mutex);
  state->stop_worker = true;
  pthread_cond_signal(&state->request_cond);
  pthread_mutex_unlock(&state->state_mutex);
  pthread_join(state->worker_thread, NULL);
  free(state->history_batch_ends);
  free(state->published_history_batch);
  pthread_cond_destroy(&state->request_cond);
  pthread_mutex_destroy(&state->state_mutex);
  fftwf_free(state->sample_ring);
  free(state);
}

/** Append complex input samples to the lock-free analysis ring. */
void panadapter_fft_push(struct panadapter_fft *state, const double *i_samples, const double *q_samples, int count) {
  if (!state)
    return;

  const uint64_t start = atomic_load_explicit(&state->samples_written, memory_order_relaxed);
  for (int index = 0; index < count; index++) {
    const uint64_t ring_index = (start + index) & state->ring_mask;
    state->sample_ring[ring_index] = (float) i_samples[index] + I * (float) q_samples[index];
  }
  atomic_store_explicit(&state->samples_written, start + count, memory_order_release);
}

/** Queue the newest analysis request, subject to the configured refresh rate. */
void panadapter_fft_request(struct panadapter_fft *state, const struct panadapter_fft_config *config) {
  if (!state || !config)
    return;

  const uint64_t now = monotonic_ms();
  pthread_mutex_lock(&state->state_mutex);
  if (state->last_request_valid
      && fft_configs_equal(config, &state->pending_config)
      && now - state->last_request_ms < (uint64_t) (config->refresh_ms > 0 ? config->refresh_ms : 0)) {
    pthread_mutex_unlock(&state->state_mutex);
    return;
  }
  state->pending_config = *config;
  state->last_request_valid = true;
  state->last_request_ms = now;
  state->pending_serial++;
  state->request_pending = true;
  pthread_cond_signal(&state->request_cond);
  pthread_mutex_unlock(&state->state_mutex);
}

/** Non-blockingly copy the latest frame matching the requested analysis. */
bool panadapter_fft_get_frame(struct panadapter_fft *state,
                              const struct panadapter_fft_config *config,
                              struct panadapter_fft_frame *frame) {
  if (!state || !config || !frame || pthread_mutex_trylock(&state->state_mutex) != 0)
    return false;
  const bool available = state->published_frame.generation != 0 && fft_configs_equal(
                           config, &state->published_frame.config);
  if (available)
    *frame = state->published_frame;
  pthread_mutex_unlock(&state->state_mutex);
  return available;
}

/** Queue all visible historical rows as one shared-preprocessing job. */
bool panadapter_fft_request_history_batch(struct panadapter_fft *state,
                                          const struct panadapter_fft_config *config,
                                          const uint64_t *sample_ends, int count,
                                          uint64_t generation) {
  if (!state || !config || !sample_ends || count < 1 || !generation)
    return false;
  uint64_t *ends = malloc((size_t) count * sizeof(*ends));
  if (!ends)
    return false;
  memcpy(ends, sample_ends, (size_t) count * sizeof(*ends));

  pthread_mutex_lock(&state->state_mutex);
  if (state->history_batch_pending || state->history_batch_busy) {
    pthread_mutex_unlock(&state->state_mutex);
    free(ends);
    return false;
  }
  state->history_batch_config = *config;
  state->history_batch_ends = ends;
  state->history_batch_count = count;
  state->history_batch_generation = generation;
  state->history_batch_epoch = atomic_load_explicit(&state->history_epoch, memory_order_relaxed);
  state->history_batch_pending = true;
  pthread_cond_signal(&state->request_cond);
  pthread_mutex_unlock(&state->state_mutex);
  return true;
}

/** Transfer ownership of a completed historical batch to the caller. */
struct panadapter_fft_frame *panadapter_fft_take_history_batch(struct panadapter_fft *state, uint64_t generation, int *count) {
  if (!state || !generation || !count || pthread_mutex_trylock(&state->state_mutex) != 0)
    return NULL;
  struct panadapter_fft_frame *frames = NULL;
  if (state->published_history_batch_generation == generation) {
    frames = state->published_history_batch;
    *count = state->published_history_batch_count;
    state->published_history_batch = NULL;
    state->published_history_batch_count = 0;
    state->published_history_batch_generation = 0;
  }
  pthread_mutex_unlock(&state->state_mutex);
  return frames;
}

/** Estimate the age of the frame's effective observation center. */
int panadapter_fft_frame_latency_ms(const struct panadapter_fft_frame *frame) {
  if (!frame || !frame->sample_end_ms || frame->observation_samples < 1 || frame->decimation < 1)
    return -1;

  const uint64_t now = monotonic_ms();
  const uint64_t age_ms = now > frame->sample_end_ms ? now - frame->sample_end_ms : 0;
  const double center_samples = (frame->observation_samples - 1) * frame->decimation / 2.0 + (FILTER_TAPS - 1) / 2.0;
  return (int) lround((double) age_ms + center_samples * 1000.0 / SDR_SAMPLE_RATE);
}

/** Discard pending and published frames and reset display smoothing. */
void panadapter_fft_reset(struct panadapter_fft *state) {
  if (!state)
    return;
  atomic_fetch_add_explicit(&state->history_epoch, 1, memory_order_relaxed);
  pthread_mutex_lock(&state->state_mutex);
  state->reset_smoothing = true;
  state->pending_serial++;
  state->request_pending = false;
  state->last_request_valid = false;
  state->published_frame.generation = 0;
  pthread_mutex_unlock(&state->state_mutex);
}
