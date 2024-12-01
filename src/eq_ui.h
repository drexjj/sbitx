#ifndef EQ_UI_H_
#define EQ_UI_H_

#include <gtk/gtk.h>
#include "para_eq.h"

#define NUM_BANDS 5

int field_set(const char *label, const char *new_value);

// Declare the modify functions
void modify_eq_band_frequency(parametriceq *eq, int band_index, double new_frequency);
void modify_eq_band_gain(parametriceq *eq, int band_index, double new_gain);
void modify_eq_band_bandwidth(parametriceq *eq, int band_index, double new_bandwidth);

// GTK Callback functions for TX EQ
void on_tx_gain_value_changed(GtkScale *scale, gpointer data);
void on_tx_freq_value_changed(GtkScale *scale, gpointer data);

// GTK Callback functions for RX EQ
void on_rx_gain_value_changed(GtkScale *scale, gpointer data);
void on_rx_freq_value_changed(GtkScale *scale, gpointer data);

// Function to initialize EQ UI
void get_print_and_set_values(GtkWidget *freq_sliders[], GtkWidget *gain_sliders[], const char *prefix);
void eq_ui(GtkWidget* parent);

// Declare the instances for TX and RX EQ
extern parametriceq tx_eq;
extern parametriceq rx_eq;

#endif // EQ_UI_H_
