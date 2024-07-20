// para_eq.h

#ifndef PARA_EQ_H_
#define PARA_EQ_H_
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <glib.h>
#define NUM_BANDS 5  // Let's start out with 5 bands in the parametric EQ

// Define Band structure
typedef struct {
    double frequency;
    double gain;
    double bandwidth;
} EQBand;

// Define ParametricEQ structure
typedef struct {
    EQBand bands[NUM_BANDS];
} ParametricEQ;

extern ParametricEQ eq;

// Function declarations
extern void init_eq(ParametricEQ *eq);
extern void modify_eq_band_frequency(ParametricEQ *eq, int band_index, double new_frequency);
extern void modify_eq_band_gain(ParametricEQ *eq, int band_index, double new_gain);
extern void modify_eq_band_bandwidth(ParametricEQ *eq, int band_index, double new_bandwidth);
extern void print_eq_int(const ParametricEQ *eq);
extern void apply_eq(ParametricEQ* eq, int32_t* samples, int num_samples, double sample_rate);
extern int eq_is_enabled;
gboolean check_eq_control(gpointer data);  

#endif /* PARA_EQ_H_ */
