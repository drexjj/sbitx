#ifndef EQ_UI_H_
#define EQ_UI_H_

#include <gtk/gtk.h>

#define NUM_BANDS 5

int field_set(const char *label, const char *new_value);

void on_gain_value_changed(GtkScale *scale, gpointer data);
void on_freq_value_changed(GtkScale *scale, gpointer data);
void get_print_and_set_values(GtkWidget *freq_sliders[], GtkWidget *gain_sliders[]);
void eq_ui(GtkWidget* parent);

#endif // EQ_UI_H_
