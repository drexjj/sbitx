#include <gtk/gtk.h>
#include <stdlib.h>
#include <string.h>
#include "eq_ui.h"

// Define or declare the parametric EQ instance
extern parametriceq eq; // Assuming eq is defined elsewhere
void on_gain_value_changed(GtkScale *scale, gpointer user_data) {
    gint index = GPOINTER_TO_INT(user_data);
    gdouble value = gtk_range_get_value(GTK_RANGE(scale));
    
    // Update the EQ band gain
    modify_eq_band_gain(&eq, index, value);
    
    // Update the field value
    gchar field_name[10];
    g_snprintf(field_name, sizeof(field_name), "B%dG", index);
    gchar *value_str = g_strdup_printf("%f", value);
    field_set(field_name, value_str);
    g_free(value_str);
}

void on_freq_value_changed(GtkScale *scale, gpointer user_data) {
    gint index = GPOINTER_TO_INT(user_data);
    gdouble value = gtk_range_get_value(GTK_RANGE(scale));
    
    // Update the EQ band frequency
    modify_eq_band_frequency(&eq, index, value);
    
    // Update the field value
    gchar field_name[10];
    g_snprintf(field_name, sizeof(field_name), "B%dF", index);
    gchar *value_str = g_strdup_printf("%f", value);
    field_set(field_name, value_str);
    g_free(value_str);
}


// Callback function to hide the window
void on_window_close(GtkWidget *widget, gpointer data) {
    gtk_widget_hide(widget);
}

// Callback function for the close button
void on_close_button_clicked(GtkWidget *widget, gpointer window) {
    gtk_widget_hide(GTK_WIDGET(window));
}

// TXEQ GTK Form
void eq_ui(GtkWidget* parent){
    GtkWidget *window;
    GtkWidget *grid;
    GtkWidget *gain_sliders[5];
    GtkWidget *freq_sliders[5];
    GtkWidget *close_button;
    GtkStyleContext *context;
    GtkCssProvider *provider;
    gint i;
    const gint freq_min[5] = {40, 150, 250, 600, 1500};
    const gint freq_max[5] = {160, 500, 1000, 2400, 3500};
    const gint freq_initial[5] = {100, 300, 500, 1200, 2500};
    gint wx, wy, ww, wh;
    gint nx, ny, nw, nh;
    //gtk_init(&argc, &argv);

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "TX Audio EQ");
    gtk_window_set_decorated(GTK_WINDOW(window), FALSE);
    gtk_container_set_border_width(GTK_CONTAINER(window), 5);


    // Connect the custom close handler to the window destroy event
    g_signal_connect(window, "destroy", G_CALLBACK(on_window_close), NULL);

    grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(grid), 0);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
    gtk_container_add(GTK_CONTAINER(window), grid);

    // Create CSS provider and load stylesheet
    provider = gtk_css_provider_new();
    gtk_css_provider_load_from_path(provider, "src/images/eq_style.css", NULL);

    // Apply CSS to the window
    context = gtk_widget_get_style_context(GTK_WIDGET(window));
    gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    // Add CSS to use a monospaced font for the labels
    GtkCssProvider *css_provider = gtk_css_provider_new();
    gtk_css_provider_load_from_data(css_provider, "label { font-family: Monospace; }", -1, NULL);
    gtk_style_context_add_provider_for_screen(gdk_screen_get_default(), GTK_STYLE_PROVIDER(css_provider), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    for (i = 0; i < 5; i++) {
        // Create Gain Slider
        gain_sliders[i] = gtk_scale_new_with_range(GTK_ORIENTATION_VERTICAL, -16, 16, 1);
        gtk_scale_set_value_pos(GTK_SCALE(gain_sliders[i]), GTK_POS_RIGHT); // Position value on the right
        gtk_scale_set_digits(GTK_SCALE(gain_sliders[i]), 0);
        gtk_range_set_value(GTK_RANGE(gain_sliders[i]), 0); // Center value
        gtk_widget_set_size_request(gain_sliders[i], 100, 150); //150x200 seems to be a good size
        gtk_range_set_inverted(GTK_RANGE(gain_sliders[i]), TRUE); // Invert slider
        g_signal_connect(gain_sliders[i], "value-changed", G_CALLBACK(on_gain_value_changed), GINT_TO_POINTER(i));
     
        // Attach the gain slider to the grid
        gtk_grid_attach(GTK_GRID(grid), gain_sliders[i], i, 0, 1, 1);

        // Create Frequency Slider
        freq_sliders[i] = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, freq_min[i], freq_max[i], 1);
        gtk_range_set_value(GTK_RANGE(freq_sliders[i]), freq_initial[i]);
        gtk_scale_set_value_pos(GTK_SCALE(freq_sliders[i]), GTK_POS_BOTTOM);
        g_signal_connect(freq_sliders[i], "value-changed", G_CALLBACK(on_freq_value_changed), GINT_TO_POINTER(i));

        // Attach the frequency slider to the grid
        gtk_grid_attach(GTK_GRID(grid), freq_sliders[i], i, 1, 1, 1);
     
        // Apply CSS to sliders
        GtkStyleContext *gain_slider_style_context = gtk_widget_get_style_context(gain_sliders[i]);
        gtk_style_context_add_provider(gain_slider_style_context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);
        gtk_style_context_add_class(gain_slider_style_context, "gain-slider");
     
        GtkStyleContext *freq_slider_style_context = gtk_widget_get_style_context(freq_sliders[i]);
        gtk_style_context_add_provider(freq_slider_style_context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);
        gtk_style_context_add_class(freq_slider_style_context, "freq-slider");
    }

    // Close Button
    close_button = gtk_button_new_with_label("Close");
    g_signal_connect(close_button, "clicked", G_CALLBACK(on_close_button_clicked), window);
    gtk_grid_attach(GTK_GRID(grid), close_button, 0, 2, 5, 1);

    // Apply CSS to the close button
    context = gtk_widget_get_style_context(GTK_WIDGET(close_button));
    gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    gtk_widget_show_all(window);
         
    // Set the sliders to the initial values from the external controls
    get_print_and_set_values(freq_sliders, gain_sliders);

    // Get main window size and new window's size so we can center new window
    gdk_window_get_geometry(gtk_widget_get_window(parent),&wx, &wy, &ww, &wh);
    gdk_window_get_geometry(gtk_widget_get_window(window),&nx, &ny, &nw, &nh);

    gtk_window_move(GTK_WINDOW(window), wx + (ww-nw)/2, wy + (wh - nh)/2);//position window

}

