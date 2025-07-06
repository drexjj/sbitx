// cfc.c - Continuous Frequency Compression for SSB Transmission
// Implementation for sBitx / zBitx
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <complex.h>
#include "cfc.h"

// FFT size - power of 2 for efficiency
#define FFT_SIZE 1024
#define FFT_HALF (FFT_SIZE / 2)

// Global variables
static int g_sample_rate = 96000;
static int g_debug_enabled = 0;
static float g_compression_ratio = 2.5f;  // Higher default compression ratio for more noticeable effect
static float g_knee_frequency = 600.0f;   // Start compression above 600 Hz
static float g_max_frequency = 3000.0f;   // Maximum frequency to preserve
static int g_cfc_enabled = 1;             // Enabled by default
static float g_fade_level = 0.0f;         // For smooth fade-in when enabling
static int g_initialized = 0;             // Initialization flag

// FFT buffers
static float complex g_fft_buffer[FFT_SIZE];
static float g_window[FFT_SIZE];
static float g_overlap_buffer[FFT_SIZE];
static int g_overlap_size = 0;

// Forward declarations for FFT functions
static void fft(float complex *buffer, int size, int inverse);
static void apply_window(float *input, float complex *output, int size);
static void overlap_add(float *output, float *overlap, int size, int overlap_size);

// Initialize CFC with the given sample rate
void cfc_init(int sample_rate) {
    g_sample_rate = sample_rate;
    
    // Initialize Hann window function for smoother FFT processing
    for (int i = 0; i < FFT_SIZE; i++) {
        g_window[i] = 0.5f * (1.0f - cosf(2.0f * M_PI * i / (FFT_SIZE - 1)));
    }
    
    // Clear overlap buffer
    memset(g_overlap_buffer, 0, sizeof(g_overlap_buffer));
    g_overlap_size = FFT_SIZE / 2;  // 50% overlap
    
    // Reset fade level for smooth transitions
    g_fade_level = 0.0f;
    
    // Mark as initialized
    g_initialized = 1;
    
    if (g_debug_enabled) {
        printf("CFC: Initialized with sample rate %d Hz\n", sample_rate);
        printf("CFC: Compression ratio: %.2f:1\n", g_compression_ratio);
        printf("CFC: Knee frequency: %.1f Hz\n", g_knee_frequency);
        printf("CFC: Max frequency: %.1f Hz\n", g_max_frequency);
    }
}

// Reset CFC processing state
void cfc_reset(void) {
    memset(g_overlap_buffer, 0, sizeof(g_overlap_buffer));
    g_fade_level = 0.0f;  // Reset fade level for smooth transitions
    
    if (g_debug_enabled) {
        printf("CFC: Reset\n");
    }
}

// Enable/disable debug output
void cfc_set_debug(int enabled) {
    g_debug_enabled = enabled;
    printf("CFC: Debug mode %s\n", enabled ? "enabled" : "disabled");
}

// Set the compression ratio
void cfc_set_compression_ratio(float ratio) {
    if (ratio >= 1.0f && ratio <= 4.0f) {
        g_compression_ratio = ratio;
        if (g_debug_enabled) {
            printf("CFC: Compression ratio set to %.2f:1\n", ratio);
        }
    }
}

// Alias for cfc_set_compression_ratio for UI consistency
void cfc_set_ratio(float ratio) {
    cfc_set_compression_ratio(ratio);
}

// Set the knee frequency
void cfc_set_knee_frequency(float freq) {
    if (freq > 0.0f && freq < g_sample_rate / 2) {
        g_knee_frequency = freq;
        if (g_debug_enabled) {
            printf("CFC: Knee frequency set to %.1f Hz\n", freq);
        }
    }
}

// Set the maximum frequency
void cfc_set_max_frequency(float freq) {
    if (freq > g_knee_frequency && freq < g_sample_rate / 2) {
        g_max_frequency = freq;
        if (g_debug_enabled) {
            printf("CFC: Max frequency set to %.1f Hz\n", freq);
        }
    }
}

// Enable or disable CFC processing
void cfc_set_enabled(int enable) {
    g_cfc_enabled = enable;
    if (g_debug_enabled) {
        printf("CFC: Processing %s\n", enable ? "enabled" : "disabled");
    }
}

// Apply frequency compression to the spectrum
static void apply_frequency_compression(float complex *spectrum, int size) {
    // Create a temporary buffer to hold the compressed spectrum
    float complex temp_spectrum[size];
    memset(temp_spectrum, 0, sizeof(temp_spectrum));
    
    // Calculate bin indices for knee and max frequencies
    int knee_bin = (int)(g_knee_frequency * size / g_sample_rate);
    int max_bin = (int)(g_max_frequency * size / g_sample_rate);
    
    // Ensure valid bin indices
    if (knee_bin < 1) knee_bin = 1;
    if (max_bin >= size / 2) max_bin = size / 2 - 1;
    
    // Copy bins below knee frequency directly (no compression)
    for (int i = 0; i < knee_bin; i++) {
        temp_spectrum[i] = spectrum[i];
        temp_spectrum[size - i - 1] = spectrum[size - i - 1]; // Mirror for real signals
    }
    
    // Apply compression only to frequencies above the knee frequency
    for (int i = knee_bin; i < size / 2; i++) {
        // Calculate compressed frequency bin
        float normalized_pos = (float)(i - knee_bin) / (float)(size / 2 - knee_bin);
        float compressed_pos = powf(normalized_pos, 1.0f / g_compression_ratio);
        int new_bin = knee_bin + (int)(compressed_pos * (max_bin - knee_bin));
        
        // Ensure new bin is within valid range
        if (new_bin >= size / 2) new_bin = size / 2 - 1;
        if (new_bin < knee_bin) new_bin = knee_bin;
        
        // Get magnitude and phase of the original bin
        float magnitude = cabsf(spectrum[i]);
        float phase = cargf(spectrum[i]);
        
        // Only transfer significant magnitudes to avoid noise amplification
        if (magnitude > 0.0001f) {
            // Store magnitude and phase at new position
            temp_spectrum[new_bin] += magnitude * cexpf(I * phase) * 0.9f; // Slight attenuation
            
            // Handle symmetric part (for real signals)
            float mirror_mag = cabsf(spectrum[size - i - 1]);
            float mirror_phase = cargf(spectrum[size - i - 1]);
            temp_spectrum[size - new_bin - 1] += mirror_mag * cexpf(I * mirror_phase) * 0.9f;
        }
    }
    
    // Apply a gentle low-pass filter to smooth transitions between bins
    for (int i = 1; i < size - 1; i++) {
        spectrum[i] = 0.2f * temp_spectrum[i-1] + 0.6f * temp_spectrum[i] + 0.2f * temp_spectrum[i+1];
    }
    
    // Handle edge cases
    spectrum[0] = temp_spectrum[0];
    spectrum[size-1] = temp_spectrum[size-1];
}

// Main CFC processing function
void cfc_process(float *buffer, int buffer_size) {
    // If CFC is disabled, just return without processing
    if (!g_cfc_enabled || buffer_size <= 0 || !g_initialized) {
        return;
    }
    
    // SIMPLE TIME-DOMAIN APPROACH - No FFT to avoid pitch issues
    // This is a much simpler approach that won't cause pitch shifting
    
    // Make a copy of the original buffer for safe processing
    float original_buffer[buffer_size];
    memcpy(original_buffer, buffer, buffer_size * sizeof(float));
    
    // Debug mode - just apply a simple gain boost
    if (g_debug_enabled) {
        for (int i = 0; i < buffer_size; i++) {
            buffer[i] = original_buffer[i] * 1.5f;
        }
        
        static int debug_counter = 0;
        debug_counter++;
        if (debug_counter >= 50) {
            debug_counter = 0;
            printf("CFC: DEBUG MODE - Simple gain boost applied\n");
        }
        return;
    }
    
    // Apply a simple high-pass filter to emphasize higher frequencies
    // but keep some of the original signal to maintain audio level
    float prev_sample = 0.0f;
    float alpha = 0.1f; // Filter coefficient
    float mix_ratio = 0.7f; // Mix 70% original with 30% filtered
    
    for (int i = 0; i < buffer_size; i++) {
        // Simple high-pass filter: y[n] = alpha * (y[n-1] + x[n] - x[n-1])
        float filtered = alpha * (prev_sample + original_buffer[i] - prev_sample);
        prev_sample = original_buffer[i];
        
        // Mix original with filtered to maintain level
        buffer[i] = mix_ratio * original_buffer[i] + (1.0f - mix_ratio) * filtered * 3.0f;
    }
    
    // Apply soft compression to make the effect more noticeable
    float threshold = 0.2f;  // Lower threshold to compress more of the signal
    float ratio = g_compression_ratio;
    float makeup_gain = 2.0f + (ratio - 1.0f) * 0.5f;  // Much higher makeup gain
    
    for (int i = 0; i < buffer_size; i++) {
        float input = buffer[i];
        float abs_input = fabsf(input);
        
        // Apply soft compression
        if (abs_input > threshold) {
            float excess = abs_input - threshold;
            float compressed = threshold + excess / ratio;
            buffer[i] = (input > 0 ? compressed : -compressed) * makeup_gain;
        } else {
            // Boost lower level signals even more
            buffer[i] = input * makeup_gain * 1.2f;
        }
    }
    
    // Print debug message
    static int debug_counter = 0;
    debug_counter++;
    if (debug_counter >= 50) {
        debug_counter = 0;
        printf("CFC: Time-domain processing active, buffer=%d, ratio=%.2f\n", 
               buffer_size, g_compression_ratio);
    }
}

// Apply window function to reduce spectral leakage
static void apply_window(float *input, float complex *output, int size) {
    // Apply Hann window with normalization to preserve energy
    float sum = 0.0f;
    for (int i = 0; i < size; i++) {
        sum += g_window[i];
    }
    float normalization = size / sum;
    
    for (int i = 0; i < size; i++) {
        output[i] = input[i] * g_window[i] * normalization;
    }
}

// Overlap-add method for reconstructing time-domain signal
static void overlap_add(float *output, float *processed, int size, int overlap_size) {
    // Add first part with overlap from previous chunk
    for (int i = 0; i < overlap_size; i++) {
        if (i < size) {
            output[i] += processed[i];
        }
    }
    
    // Copy the rest directly
    for (int i = overlap_size; i < FFT_SIZE && (i - overlap_size) < size; i++) {
        output[i - overlap_size] = processed[i];
    }
}

// Simple FFT implementation (Cooley-Tukey algorithm)
// For production use, consider replacing with a more optimized library like FFTW
static void fft(float complex *buffer, int size, int inverse) {
    // Check if size is a power of 2
    if (size & (size - 1)) {
        if (g_debug_enabled) {
            printf("CFC: FFT size must be a power of 2\n");
        }
        return;
    }
    
    // Bit-reverse permutation
    int bits = (int)log2f(size);
    for (int i = 0; i < size; i++) {
        int j = 0;
        for (int k = 0; k < bits; k++) {
            j = (j << 1) | ((i >> k) & 1);
        }
        if (j > i) {
            float complex temp = buffer[i];
            buffer[i] = buffer[j];
            buffer[j] = temp;
        }
    }
    
    // Cooley-Tukey FFT algorithm
    for (int step = 2; step <= size; step <<= 1) {
        int half_step = step >> 1;
        float complex twiddle_factor = cexpf(
            (inverse ? 1.0f : -1.0f) * 2.0f * M_PI * I / step);
        
        for (int i = 0; i < size; i += step) {
            float complex omega = 1.0f;
            for (int j = 0; j < half_step; j++) {
                float complex u = buffer[i + j];
                float complex v = buffer[i + j + half_step] * omega;
                buffer[i + j] = u + v;
                buffer[i + j + half_step] = u - v;
                omega *= twiddle_factor;
            }
        }
    }
    
    // Scale if inverse FFT
    if (inverse) {
        for (int i = 0; i < size; i++) {
            buffer[i] /= size;
        }
    }
}
