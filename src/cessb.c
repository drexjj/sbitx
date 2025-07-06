// cessb.c - CESSB Envelope Shaping for SSB Transmission
// Enhanced implementation with soft clipping and high-pass filtering
// For sBitx / zBitx W2JON
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "cessb.h"

// External variables
extern double cessb_clip_level;  // Global clip level from sbitx.c

// Global variables
static int g_sample_rate = 96000;
static int g_debug_enabled = 0;      // Set to 0 to disable debug output
static float g_clip_level = 0.8f;   // Soft clipping threshold (0.0-1.0) 80% default Can be adjusted with GCLIP control in sbitx_gtk.c
        // Lower values (0.6-0.7): More aggressive compression, higher average power, but potentially more distortion
        // Higher values (0.8-0.9): More gentle compression, less distortion, but lower average power
        // Very low values (<0.5): Not recommended, would cause excessive compression and distortion
        // Values at 1.0: Minimal compression effect, mostly just applies the high-pass filter

// Look-ahead buffer for smoother compression
#define LOOKAHEAD_SIZE 32  // Number of samples to look ahead
static float g_lookahead_buffer[LOOKAHEAD_SIZE] = {0};
static int g_buffer_index = 0;
static int g_lookahead_enabled = 1;  // Set to 1 to enable look-ahead limiting (this control has not been exposed to sbitx_gtk.c yet)

// Multiband compression settings
static int g_multiband_enabled = 1;  // Set to 1 to enable multiband processing (this has not been exposed to sbitx_gtk.c yet)

// Simple IIR filter state variables for multiband processing
static float g_low_pass_state = 0.0f;
static float g_high_pass_state = 0.0f;
static float g_band_pass_state1 = 0.0f;
static float g_band_pass_state2 = 0.0f;

// Pre-emphasis/de-emphasis settings
static int g_preemphasis_enabled = 1;  // Set to 1 to enable pre-emphasis (this has not been exposed to sbitx_gtk.c yet)
static float g_preemphasis_state = 0.0f;
static float g_deemphasis_state = 0.0f;


// Enhanced soft clipper with more aggressive compression and better audio quality
static inline float soft_clip(float sample) {
    // Use the global clip level from the UI element GCLIP
    float clip_level = (float)cessb_clip_level;
    
    float abs_sample = fabsf(sample);
    float sign = (sample >= 0) ? 1.0f : -1.0f;
    
    // Apply compression to all signals, more aggressive for louder signals
    if (abs_sample <= clip_level * 0.3f) {
        // Light compression for quiet signals
        return sample * 1.5f; // Boost quiet signals more
    } else if (abs_sample <= clip_level) {
        // Medium compression for moderate signals
        float ratio = abs_sample / clip_level;
        // Smooth transition curve for better audio quality
        return sign * abs_sample * (1.2f - ratio * 0.3f);
    } else {
        // Heavy compression for loud signals
        // Use a smoother transition for better audio quality
        float excess = abs_sample - clip_level;
        float compression = 0.15f + 0.05f / (1.0f + excess * 5.0f); // Asymptotic compression
        return sign * (clip_level + excess * compression);
    }
}

// Simple high-pass filter to enhance voice clarity
static float g_prev_input = 0.0f;
static float g_prev_output = 0.0f;

static inline float high_pass_filter(float sample) {
    // Simple first-order high-pass filter (100Hz cutoff at 96kHz)
    // To adjust the cutoff frequency, modify the alpha value:
    // - Higher alpha (closer to 1.0): Lower cutoff frequency
    // - Lower alpha (e.g., 0.995): Higher cutoff frequency
    // Current setting (0.998) provides approximately 100Hz cutoff at 96kHz sample rate
    // Formula: alpha = exp(-2π * cutoff / sample_rate)
    const float alpha = 0.998f;
    float output = alpha * (g_prev_output + sample - g_prev_input);
    g_prev_input = sample;
    g_prev_output = output;
    return output;
}

// Pre-emphasis filter to boost high frequencies before processing
static inline float apply_preemphasis(float sample) {
    // Pre-emphasis coefficient - adjusted for cleaner spectral characteristics
    // Reduced from 0.94f to 0.92f for gentler high-frequency boost
    // This helps prevent excessive high-frequency content that could cause splatter
    const float alpha = 0.92f;
    float output = sample - alpha * g_preemphasis_state;
    g_preemphasis_state = sample;
    return output;
}

// De-emphasis filter to restore frequency balance after processing
static inline float apply_deemphasis(float sample) {
    // De-emphasis coefficient (must match pre-emphasis)
    const float alpha = 0.92f;
    float output = sample + alpha * g_deemphasis_state;
    g_deemphasis_state = output;
    
    // Apply a very gentle low-pass filter to ensure smooth high-frequency rolloff
    // This helps prevent any residual high-frequency artifacts
    static float prev_output = 0.0f;
    output = output * 0.95f + prev_output * 0.05f;
    prev_output = output;
    
    return output;
}

// Simple low-pass filter for multiband processing
static inline float low_pass_filter(float sample) {
    // Cutoff around 300Hz at 96kHz
    const float alpha = 0.98f;
    g_low_pass_state = g_low_pass_state * alpha + sample * (1.0f - alpha);
    return g_low_pass_state;
}

// Simple high-pass filter for multiband processing
static inline float high_pass_filter_mb(float sample) {
    // Cutoff around 3000Hz at 96kHz
    const float alpha = 0.90f;
    g_high_pass_state = g_high_pass_state * alpha + sample * (1.0f - alpha);
    return sample - g_high_pass_state;
}

// Band-pass filter for multiband processing (mid frequencies)
static inline float band_pass_filter(float sample) {
    float low = low_pass_filter(sample);
    float high = high_pass_filter_mb(sample);
    return sample - low - high;
}

// Multiband processing function
static void process_multiband(float *buffer, int buffer_size) {
    float low_band[buffer_size];
    float mid_band[buffer_size];
    float high_band[buffer_size];
    
    // Split into frequency bands
    for (int i = 0; i < buffer_size; i++) {
        // Apply pre-emphasis if enabled
        if (g_preemphasis_enabled) {
            buffer[i] = apply_preemphasis(buffer[i]);
        }
        
        // Split into three bands
        low_band[i] = low_pass_filter(buffer[i]);
        high_band[i] = high_pass_filter_mb(buffer[i]);
        mid_band[i] = buffer[i] - low_band[i] - high_band[i];
    }
    
    // Process each band with different compression settings
    // Reduced boost values to decrease distortion while maintaining effect
    for (int i = 0; i < buffer_size; i++) {
        // Low band: moderate compression
        low_band[i] *= 1.4f; // Moderate boost (reduced from 1.8f)
        if (fabsf(low_band[i]) > 0.65f * (float)cessb_clip_level) { // Slightly higher threshold
            low_band[i] = soft_clip(low_band[i] * 0.95f); // Less aggressive
        }
        
        // Mid band: balanced compression (voice frequencies)
        mid_band[i] *= 2.2f; // Moderate boost for mid frequencies (reduced from 3.0f)
        mid_band[i] = soft_clip(mid_band[i]);
        
        // High band: moderate compression
        high_band[i] *= 1.7f; // Moderate boost (reduced from 2.2f)
        high_band[i] = soft_clip(high_band[i] * 1.1f); // Less aggressive
    }
    
    // Recombine the bands
    for (int i = 0; i < buffer_size; i++) {
        // Mix with different weights - adjusted for cleaner spectral characteristics
        // Reduced high band weight slightly to minimize potential splatter
        buffer[i] = low_band[i] * 0.65f + mid_band[i] * 1.0f + high_band[i] * 0.7f;
        
        // Apply de-emphasis if pre-emphasis was used
        if (g_preemphasis_enabled) {
            buffer[i] = apply_deemphasis(buffer[i]);
        }
        
        // Final limiter to catch any intersample peaks
        // This ensures no samples exceed the clip level, preventing splatter
        float clip_level = (float)cessb_clip_level;
        if (fabsf(buffer[i]) > clip_level) {
            buffer[i] = (buffer[i] > 0) ? clip_level : -clip_level;
        }
    }
}

// Look-ahead limiting function to anticipate and smooth out peaks
static void process_with_lookahead(float *buffer, int buffer_size) {
    float output_buffer[buffer_size];
    float peak_values[buffer_size + LOOKAHEAD_SIZE];
    
    // First pass: find peak values with lookahead window
    for (int i = 0; i < buffer_size + LOOKAHEAD_SIZE; i++) {
        if (i < buffer_size) {
            // For samples in the current buffer
            peak_values[i] = fabsf(buffer[i]);
        } else {
            // For lookahead samples beyond current buffer, use zeros
            // In a real-time system, these would be from future samples
            peak_values[i] = 0.0f;
        }
    }
    
    // Second pass: find maximum peak in each lookahead window
    for (int i = 0; i < buffer_size; i++) {
        float max_peak = peak_values[i];
        for (int j = 1; j < LOOKAHEAD_SIZE; j++) {
            if (peak_values[i + j] > max_peak) {
                max_peak = peak_values[i + j];
            }
        }
        
        // Apply soft clipping based on the maximum peak in the window
        float gain = 1.0f;
        if (max_peak > 0.0f) {
            float clip_level = (float)cessb_clip_level;
            if (max_peak > clip_level) {
                // Calculate a gain reduction that would bring the peak to the clip level
                gain = clip_level / max_peak;
                // Apply a more balanced gain reduction curve to reduce distortion
                // Previously was: gain = 0.5f + 0.5f * gain; (too aggressive)
                gain = 0.65f + 0.35f * gain; // More balanced compression
            }
        }
        
        // Apply the calculated gain to the current sample
        output_buffer[i] = buffer[i] * gain;
    }
    
    // Copy the processed samples back to the input buffer
    memcpy(buffer, output_buffer, buffer_size * sizeof(float));
}

// Main CESSB processing function
void cessb_process(float *buffer, int buffer_size) {
    // Calculate max level before processing by finding the sample with the highest absolute amplitude
    // This is used to measure the input signal level and calculate compression ratio for debug output
    // The algorithm simply iterates through all samples, takes the absolute value of each,
    // and keeps track of the highest value found
    float max_before = 0.0f;
    for (int i = 0; i < buffer_size; i++) {
        float abs_val = fabsf(buffer[i]); // Get absolute value (magnitude) of the sample
        if (abs_val > max_before) max_before = abs_val; // Update max if this sample is larger
    }
    
    // Apply high-pass filter to enhance voice clarity
    for (int i = 0; i < buffer_size; i++) {
        buffer[i] = high_pass_filter(buffer[i]);
    }
    
    // Boost the signal before processing to make the effect more noticeable
    // This intentionally drives the signal harder into the soft clipper
    // Reduced from 2.5x to 2.0x to decrease distortion while still showing effect
    // - Higher values create a more aggressive CESSB effect with more visible clipping
    // - Lower values create a more subtle effect with less compression
    for (int i = 0; i < buffer_size; i++) {
        buffer[i] *= 2.0f; // Boost by 100% (balanced setting)
    }
    
    // Process with multiband, look-ahead limiting, or standard soft clipping
    if (g_multiband_enabled) {
        // Multiband processing with different compression settings per band
        process_multiband(buffer, buffer_size);
    } else if (g_lookahead_enabled) {
        // Look-ahead limiting for smoother compression
        process_with_lookahead(buffer, buffer_size);
    } else {
        // Standard soft clipping (original implementation)
        for (int i = 0; i < buffer_size; i++) {
            buffer[i] = soft_clip(buffer[i]);
        }
    }
    
    // Calculate max level after processing by finding the sample with the highest absolute amplitude
    // This is used to measure the output signal level and calculate compression ratio for debug output
    // The algorithm simply iterates through all samples, takes the absolute value of each,
    // and keeps track of the highest value found
    float max_after = 0.0f;
    for (int i = 0; i < buffer_size; i++) {
        float abs_val = fabsf(buffer[i]); // Get absolute value (magnitude) of the sample
        if (abs_val > max_after) max_after = abs_val; // Update max if this sample is larger
    }
    
    // Print debug info occasionally
    static int debug_counter = 0;
    debug_counter++;
    if (g_debug_enabled && debug_counter >= 100) { // More frequent debug output
        debug_counter = 0;
        printf("CESSB: Active, buffer=%d, before=%.3f, after=%.3f, ratio=%.2f\n", 
               buffer_size, max_before, max_after, max_after/max_before);
    }
}

// Reset CESSB processing state - called when switching between TX and RX modes
void cessb_reset() {
    // Reset filter state variables
    g_prev_input = 0.0f;
    g_prev_output = 0.0f;
    
    if (g_debug_enabled) {
        printf("CESSB: State reset\n");
    }
}

// Initialize CESSB with the given sample rate
void cessb_init(int sample_rate) {
    g_sample_rate = sample_rate;
    
    // Adjust filter parameters based on sample rate if needed
    // Currently using fixed parameters that work well at 96kHz
    
    if (g_debug_enabled) {
        printf("CESSB: Initialized with sample rate %d Hz\n", sample_rate);
    }
}

// Enable/disable debug output
void cessb_set_debug(int enable) {
    g_debug_enabled = enable;
    if (g_debug_enabled) {
        printf("CESSB: Debug output enabled\n");
    }
}

// Enable or disable look-ahead limiting
void cessb_set_lookahead(int enable) {
    g_lookahead_enabled = enable;
    if (g_debug_enabled) {
        printf("CESSB: Look-ahead limiting %s\n", enable ? "enabled" : "disabled");
    }
}

// Enable or disable multiband processing
void cessb_set_multiband(int enable) {
    g_multiband_enabled = enable;
    if (g_debug_enabled) {
        printf("CESSB: Multiband processing %s\n", enable ? "enabled" : "disabled");
    }
}

// Enable or disable pre-emphasis/de-emphasis
void cessb_set_preemphasis(int enable) {
    g_preemphasis_enabled = enable;
    if (g_debug_enabled) {
        printf("CESSB: Pre-emphasis/de-emphasis %s\n", enable ? "enabled" : "disabled");
    }
    
    // Reset filter states when changing modes
    g_preemphasis_state = 0.0f;
    g_deemphasis_state = 0.0f;
}

// Set the clipping threshold (0.0-1.0)
void cessb_set_clip_level(float level) {
    if (level > 0.0f && level <= 1.0f) {
        // Update both the local and global clip level
        g_clip_level = level;
        cessb_clip_level = (double)level;

        if (g_debug_enabled) {
            printf("CESSB: Clip level set to %.2f\n", level);
        }
    }
}
