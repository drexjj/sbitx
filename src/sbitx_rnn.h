#ifndef SBITX_RNN_H
#define SBITX_RNN_H

#include <stdint.h>

/* RNNoise-based noise reduction for the RX speaker audio path.
 *
 * rnn_process_speaker() processes a block of demodulated speaker samples
 * in place. It handles 96 kHz <-> 48 kHz rate conversion, amplitude
 * scaling between sbitx's int32 speaker range and RNNoise's int16-float
 * range, and reframing into RNNoise's fixed 480-sample (10 ms) frames.
 * Introduces roughly one RNNoise frame (~10-20 ms) of latency while
 * enabled.
 *
 * rnn_reset() drops all buffered state; call when the mode changes or
 * the feature is toggled off so stale audio doesn't leak into the next
 * session.
 */
void rnn_process_speaker(int32_t *samples, int n_samples);
void rnn_reset(void);

/* Wet/dry strength, 0-100. See sbitx_rnn.c. */
extern int rnn_strength;

#endif
