#ifndef PANADAPTER_FFT_H
#define PANADAPTER_FFT_H

#include <stdbool.h>
#include <stdint.h>

#define PANADAPTER_FFT_MIN_BINS 1024
#define PANADAPTER_FFT_MAX_BINS 16384
#define PANADAPTER_FFT_FRAME_BINS 2048

struct panadapter_fft;

struct panadapter_fft_config {
  int display_span_hz;
  int center_hz;
  int is_cw;
  int is_tx;  // Keep RX and TX frames and smoothing state separate.
  int wpm;
  int refresh_ms;
  int display_width_px;
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

struct panadapter_fft *panadapter_fft_create(void);

void panadapter_fft_destroy(struct panadapter_fft *context);

void panadapter_fft_push(struct panadapter_fft *context, const double *i_samples, const double *q_samples, int count);

void panadapter_fft_request(struct panadapter_fft *context, const struct panadapter_fft_config *config);

bool panadapter_fft_get_frame(struct panadapter_fft *context, const struct panadapter_fft_config *config, struct panadapter_fft_frame *frame);

bool panadapter_fft_request_history_batch(struct panadapter_fft *context,
  const struct panadapter_fft_config *config, const uint64_t *sample_ends,
  int count, uint64_t generation);

struct panadapter_fft_frame *panadapter_fft_take_history_batch( struct panadapter_fft *context, uint64_t generation, int *count);

int panadapter_fft_frame_latency_ms(const struct panadapter_fft_frame *frame);

void panadapter_fft_reset(struct panadapter_fft *context);

#endif
