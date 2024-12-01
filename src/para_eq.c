#include "para_eq.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

// Function to copy files
int copy_file(const char *src, const char *dst) {
    FILE *source = fopen(src, "rb");
    if (source == NULL) {
        return -1;
    }

    FILE *dest = fopen(dst, "wb");
    if (dest == NULL) {
        fclose(source);
        return -1;
    }

    char buffer[BUFSIZ];
    size_t n;
    while ((n = fread(buffer, 1, sizeof(buffer), source)) > 0) {
        if (fwrite(buffer, 1, n, dest) != n) {
            fclose(source);
            fclose(dest);
            return -1;
        }
    }

    fclose(source);
    fclose(dest);
    return 0;
}

// Function to read value from file
float read_value(FILE* file, const char* key, float default_value) {
    char line[256];
    char* found;
    float value = default_value;

    rewind(file); // Reset the file pointer to the beginning
    while (fgets(line, sizeof(line), file)) {
        if ((found = strstr(line, key)) != NULL) {
            sscanf(found + strlen(key) + 1, "%f", &value);
            break;
        }
    }
    return value;
}

// Function to initialize EQ parameters
void init_eq(parametriceq* eq, const char* section) {
    char *home = getenv("HOME");
    if (home == NULL) {
        fprintf(stderr, "Error: HOME environment variable is not set.\n");
        exit(EXIT_FAILURE);
    }

    char user_settings_path[200];
    char default_settings_path[200];
    snprintf(user_settings_path, sizeof(user_settings_path), "%s/sbitx/data/user_settings.ini", home);
    snprintf(default_settings_path, sizeof(default_settings_path), "%s/sbitx/data/default_settings.ini", home);

    FILE *file = fopen(user_settings_path, "r");
    if (file == NULL) {
        printf("user_settings.ini not found. Attempting to create from default_settings.ini...\n");
        if (copy_file(default_settings_path, user_settings_path) == 0) {
            printf("Successfully copied default_settings.ini to user_settings.ini.\n");
            file = fopen(user_settings_path, "r");
            if (file == NULL) {
                perror("Failed to open user_settings.ini after copying");
                exit(EXIT_FAILURE);
            }
        } else {
            perror("Failed to copy default_settings.ini to user_settings.ini");
            exit(EXIT_FAILURE);
        }
    }

    char key[20];

    // Load default values for the specified section (e.g., TX or RX)
    for (int i = 0; i < NUM_BANDS; i++) {
        snprintf(key, sizeof(key), "#%s_eq_b%df", section, i);
        eq->bands[i].frequency = read_value(file, key, eq->bands[i].frequency);

        snprintf(key, sizeof(key), "#%s_eq_b%dg", section, i);
        eq->bands[i].gain = read_value(file, key, eq->bands[i].gain);

        snprintf(key, sizeof(key), "#%s_eq_b%db", section, i);
        eq->bands[i].bandwidth = read_value(file, key, eq->bands[i].bandwidth);
    }

    fclose(file);
}


// Structure for Biquad filter
typedef struct {
    double a0, a1, a2, b0, b1, b2;
    double x1, x2, y1, y2;
} Biquad;

// Function to calculate filter coefficients
void calculate_coefficients(EQBand* band, double sample_rate, Biquad* filter) {
    double clamped_gain = fmax(fmin(band->gain, 24.0), -24.0); // Clamp gain to ±24 dB
    double A = pow(10.0, clamped_gain / 40.0);
    double omega = 2.0 * M_PI * band->frequency / sample_rate;
    double sin_omega = sin(omega);
    double cos_omega = cos(omega);
    double alpha = sin_omega * sinh(log(2.0) / 2.0 * band->bandwidth * omega / fmax(sin_omega, 1e-10));

    // Ensure the filter coefficients are stable
    filter->b0 = 1.0 + alpha * A;
    filter->b1 = -2.0 * cos_omega;
    filter->b2 = 1.0 - alpha * A;
    filter->a0 = 1.0 + alpha / A;
    filter->a1 = -2.0 * cos_omega;
    filter->a2 = 1.0 - alpha / A;

    // Check if a0 is close to zero to prevent division by zero
    if (fabs(filter->a0) < 1e-10) {
        filter->a0 = 1e-10; // Prevent division by zero
    }

    // Normalize the coefficients
    filter->b0 /= filter->a0;
    filter->b1 /= filter->a0;
    filter->b2 /= filter->a0;
    filter->a1 /= filter->a0;
    filter->a2 /= filter->a0;

    // a0 should always be normalized to 1.0
    filter->a0 = 1.0;

    // Initialize filter states
    filter->x1 = filter->x2 = 0.0;
    filter->y1 = filter->y2 = 0.0;
}

// Function to process a single sample through the filter
int32_t process_sample(Biquad* filter, int32_t sample) {
    double result = filter->b0 * sample + filter->b1 * filter->x1 + filter->b2 * filter->x2
        - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    
    // Update filter states
    filter->x2 = filter->x1;
    filter->x1 = sample;
    filter->y2 = filter->y1;
    filter->y1 = result;

    // Ensure result is within valid range
    if (result > INT32_MAX) result = INT32_MAX;
    if (result < INT32_MIN) result = INT32_MIN;

    return (int32_t)result;
}

// Function to remove DC offset
void remove_dc_offset(int32_t* samples, int num_samples) {
    int64_t sum = 0;
    for (int i = 0; i < num_samples; i++) {
        sum += samples[i];
    }
    int32_t average = (int32_t)(sum / num_samples);
    for (int i = 0; i < num_samples; i++) {
        samples[i] -= average;
    }
}

// Function to scale samples by a gain factor
void scale_samples(int32_t* samples, int num_samples, float gain_factor) {
    for (int i = 0; i < num_samples; i++) {
        // Apply the gain factor and clamp values to avoid overflow/underflow
        int64_t scaled_sample = (int64_t)(samples[i] * gain_factor);
        if (scaled_sample > INT32_MAX) scaled_sample = INT32_MAX;
        if (scaled_sample < INT32_MIN) scaled_sample = INT32_MIN;
        samples[i] = (int32_t)scaled_sample;
    }
}

// Function to apply EQ and gain scaling
void apply_eq(parametriceq* eq, int32_t* samples, int num_samples, double sample_rate) {
    Biquad filters[NUM_BANDS];

    // Step 1: Calculate coefficients for each band
    for (int i = 0; i < NUM_BANDS; i++) {
        calculate_coefficients(&eq->bands[i], sample_rate, &filters[i]);
    }

    // Step 2: Initialize output buffer
    int32_t output_samples[num_samples];
    memset(output_samples, 0, sizeof(output_samples));

    // Step 3: Process samples for each band
    for (int i = 0; i < NUM_BANDS; i++) {
        double band_gain = pow(10.0, eq->bands[i].gain / 20.0); // Convert dB to linear scale

        for (int n = 0; n < num_samples; n++) {
            // Apply the filter to the current sample
            double filtered_sample = process_sample(&filters[i], samples[n]);

            // Accumulate the scaled filtered output
            output_samples[n] += (int32_t)(filtered_sample * band_gain);
        }
    }

    // Step 4: Normalize the summed output
    for (int n = 0; n < num_samples; n++) {
        output_samples[n] /= NUM_BANDS; // Prevent unintentional gain
    }

    // Step 5: Scale and clamp using scale_samples
    scale_samples(output_samples, num_samples, 1.0);

    // Step 6: Copy normalized output back to input buffer
    memcpy(samples, output_samples, sizeof(int32_t) * num_samples);
}







