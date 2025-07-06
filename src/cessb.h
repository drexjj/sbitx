// cessb.h - Header for CESSB processing
#ifndef SBITX_CESSB_H
#define SBITX_CESSB_H

// Initialize CESSB processing with given sample rate
void cessb_init(int rate);

// Applies CESSB shaping to an audio buffer (float PCM)
void cessb_process(float* buf, int len);

// Resets the CESSB processing state
void cessb_reset(void);

// Enable/disable debug output
void cessb_set_debug(int enable);

// Enable or disable look-ahead limiting (this has not been exposed to sbitx_gtk.c yet)
void cessb_set_lookahead(int enable);

// Enable or disable multiband processing (this has not been exposed to sbitx_gtk.c yet)
void cessb_set_multiband(int enable);

// Enable or disable pre-emphasis/de-emphasis (this has not been exposed to sbitx_gtk.c yet)
void cessb_set_preemphasis(int enable);

// Set the clipping threshold (0.0-1.0)
void cessb_set_clip_level(float level);

#endif
