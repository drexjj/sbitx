// cfc.h - Header for Continuous Frequency Compression processing
#ifndef SBITX_CFC_H
#define SBITX_CFC_H

// Initialize CFC processing with given sample rate
void cfc_init(int rate);

// Applies CFC processing to an audio buffer (float PCM)
void cfc_process(float* buf, int len);

// Resets the CFC processing state
void cfc_reset(void);

// Enable/disable debug output
void cfc_set_debug(int enable);

// Set the compression ratio (1.0 = no compression, 2.0 = 2:1 compression, etc.)
void cfc_set_compression_ratio(float ratio);

// Alias for cfc_set_compression_ratio for UI consistency
void cfc_set_ratio(float ratio);

// Set the frequency above which compression begins (in Hz)
void cfc_set_knee_frequency(float freq);

// Set the maximum frequency to preserve (in Hz)
void cfc_set_max_frequency(float freq);

// Enable or disable CFC processing
void cfc_set_enabled(int enable);

#endif
