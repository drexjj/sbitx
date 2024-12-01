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

// GTK Callback functions
void on_gain_value_changed(GtkScale *scale, gpointer data);
void on_freq_value_changed(GtkScale *scale, gpointer data);
void get_print_and_set_values(GtkWidget *freq_sliders[], GtkWidget *gain_sliders[]);
void eq_ui(GtkWidget* parent);

extern parametriceq eq; // Declare the instance


#endif // EQ_UI_H_
