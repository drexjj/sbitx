#ifndef PANADAPTER_FFT_H
#define PANADAPTER_FFT_H

#include <stdbool.h>
#include <stdint.h>

#define PANADAPTER_FFT_MIN_BINS 1024
#define PANADAPTER_FFT_MAX_BINS 16384
#define PANADAPTER_FFT_FRAME_BINS 2048

/*
 * Bounds for the I/Q retention ring. The maximum is the historical fixed size
 * (about 10.9 seconds at 96 ksample/s, 8 MiB). The minimum has to hold the
 * largest analysis window plus a full FFT of slack, because that is what
 * analyze_history_batch()'s retention guard requires; anything smaller would
 * silently reject every history row at high zoom while the live spectrum kept
 * working.
 */
#define PANADAPTER_FFT_MAX_HISTORY_SAMPLES (1024 * 1024)
#define PANADAPTER_FFT_MIN_HISTORY_SAMPLES (256 * 1024)

struct panadapter_fft;

struct panadapter_fft_config {
  int display_span_hz;
  int center_hz;
  int is_cw;
  int is_tx;  // Keep RX and TX frames and smoothing state separate.
  int wpm;
  int refresh_ms;
  int display_width_px;
  int max_bins;  // Cap on the analysis FFT size; 0 means PANADAPTER_FFT_MAX_BINS.
};

struct panadapter_fft_frame {
  int bins[PANADAPTER_FFT_FRAME_BINS];
  int count;
  double first_hz;
  double bin_step_hz;
  int decimation;
  int observation_samples;
  int fft_bins;
  uint64_t sample_end;
  uint64_t sample_end_ms;
  uint64_t generation;
  struct panadapter_fft_config config;
};

/*
 * Round a requested I/Q retention (in samples) to a usable ring size: the
 * nearest power of two within
 * [PANADAPTER_FFT_MIN_HISTORY_SAMPLES, PANADAPTER_FFT_MAX_HISTORY_SAMPLES].
 * Pure function, exposed for unit testing.
 */
uint64_t panadapter_fft_ring_size(uint64_t requested_samples);

struct panadapter_fft *panadapter_fft_create(uint64_t history_samples);

void panadapter_fft_destroy(struct panadapter_fft *context);

void panadapter_fft_push(struct panadapter_fft *context, const double *i_samples, const double *q_samples, int count);

void panadapter_fft_request(struct panadapter_fft *context, const struct panadapter_fft_config *config);

bool panadapter_fft_get_frame(struct panadapter_fft *context, const struct panadapter_fft_config *config, struct panadapter_fft_frame *frame);

bool panadapter_fft_request_history_batch(struct panadapter_fft *context,
  const struct panadapter_fft_config *config, const uint64_t *sample_ends,
  int count, uint64_t generation);

struct panadapter_fft_frame *panadapter_fft_take_history_batch( struct panadapter_fft *context, uint64_t generation, int *count);

int panadapter_fft_frame_latency_ms(const struct panadapter_fft_frame *frame);

/*
 * Limit how far back reconstructed waterfall history may reach, in samples.
 * Clamped to the analysis floor and to the allocated ring size. Safe to call
 * from any thread at any time: it only ever makes the retention guard stricter
 * or re-admits samples the ring never stopped holding.
 */
void panadapter_fft_set_history_limit(struct panadapter_fft *context, uint64_t samples);

/** Return the retention limit currently in effect, in samples. */
uint64_t panadapter_fft_history_limit(const struct panadapter_fft *context);

void panadapter_fft_reset(struct panadapter_fft *context);

#endif
