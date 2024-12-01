#include <gtk/gtk.h>
#include <stdlib.h>
#include <string.h>
#include "eq_ui.h"
#include "sdr_ui.h"

// Define or declare the parametric EQ instances for TX and RX
extern parametriceq tx_eq; // TX EQ defined elsewhere
extern parametriceq rx_eq; // RX EQ defined elsewhere

// Callback for TX EQ Gain Slider
void on_tx_gain_value_changed(GtkScale *scale, gpointer user_data)
{
    gint index = GPOINTER_TO_INT(user_data);
    gdouble value = gtk_range_get_value(GTK_RANGE(scale));

    modify_eq_band_gain(&tx_eq, index, value);

    gchar field_name[20];
    g_snprintf(field_name, sizeof(field_name), "B%dG", index);
    gchar *value_str = g_strdup_printf("%.0f", value);
    field_set(field_name, value_str);
    g_free(value_str);
    
}

// Callback for TX EQ Frequency Slider
void on_tx_freq_value_changed(GtkScale *scale, gpointer user_data)
{
    gint index = GPOINTER_TO_INT(user_data);
    gdouble value = gtk_range_get_value(GTK_RANGE(scale));

    modify_eq_band_frequency(&tx_eq, index, value);

    gchar field_name[20];
    g_snprintf(field_name, sizeof(field_name), "B%dF", index);
    gchar *value_str = g_strdup_printf("%.0f", value);
    field_set(field_name, value_str);
    g_free(value_str);
    
}

// Callback for RX EQ Gain Slider
void on_rx_gain_value_changed(GtkScale *scale, gpointer user_data)
{
    gint index = GPOINTER_TO_INT(user_data);
    gdouble value = gtk_range_get_value(GTK_RANGE(scale));

    modify_eq_band_gain(&rx_eq, index, value);

    gchar field_name[20];
    g_snprintf(field_name, sizeof(field_name), "R%dG", index);
    gchar *value_str = g_strdup_printf("%.0f", value);
    field_set(field_name, value_str);
    g_free(value_str);
    
}

// Callback for RX EQ Frequency Slider
void on_rx_freq_value_changed(GtkScale *scale, gpointer user_data)
{
    gint index = GPOINTER_TO_INT(user_data);
    gdouble value = gtk_range_get_value(GTK_RANGE(scale));

    modify_eq_band_frequency(&rx_eq, index, value);

    gchar field_name[20];
    g_snprintf(field_name, sizeof(field_name), "R%dF", index);
    gchar *value_str = g_strdup_printf("%.0f", value);
    field_set(field_name, value_str);
    g_free(value_str);
    
}

// Callback function to hide the window
void on_window_close(GtkWidget *widget, gpointer data)
{
    gtk_widget_hide(widget);
}

// Callback function for the close button
void on_close_button_clicked(GtkWidget *widget, gpointer window)
{
    save_user_settings(1);
    gtk_widget_hide(GTK_WIDGET(window));
}

// TX and RX EQ GTK Form
void eq_ui(GtkWidget *parent)
{
    GtkWidget *window;
    GtkWidget *notebook;
    GtkWidget *tx_frame, *rx_frame;
    GtkWidget *tx_grid, *rx_grid;
    GtkWidget *tx_gain_sliders[5], *tx_freq_sliders[5];
    GtkWidget *rx_gain_sliders[5], *rx_freq_sliders[5];
    GtkWidget *close_button;
    GtkCssProvider *provider;
    GtkCssProvider *slider_provider;
    gint wx, wy, ww, wh; // Parent window position and dimensions
    gint nx, ny, nw, nh; // New window position and dimensions
    gint i;

    const gint freq_min[5] = {40, 150, 250, 600, 1500};
    const gint freq_max[5] = {160, 500, 1000, 2400, 3500};
    const gint freq_initial[5] = {100, 300, 500, 1200, 2500};

    // Create the main window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Audio EQ");
    gtk_container_set_border_width(GTK_CONTAINER(window), 10);
    g_signal_connect(window, "destroy", G_CALLBACK(on_window_close), NULL);

    // Load CSS
    provider = gtk_css_provider_new();
    gtk_css_provider_load_from_path(provider, "src/images/eq_style.css", NULL);

    GtkStyleContext *context = gtk_widget_get_style_context(GTK_WIDGET(window));
    gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    // Additional CSS for sliders
    slider_provider = gtk_css_provider_new();
    gtk_css_provider_load_from_data(
        slider_provider,
        "scale.gain-slider { background-color: lightgray; } "
        "scale.freq-slider { background-color: lightblue; }",
        -1, NULL);

    // Create a notebook for tabs
    notebook = gtk_notebook_new();
    gtk_container_add(GTK_CONTAINER(window), notebook);

    // TX EQ Section
    tx_frame = gtk_frame_new("TX EQ");
    gtk_container_set_border_width(GTK_CONTAINER(tx_frame), 5);
    tx_grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(tx_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(tx_grid), 5);
    gtk_container_add(GTK_CONTAINER(tx_frame), tx_grid);

    for (i = 0; i < 5; i++)
{
    // TX Gain Slider
    tx_gain_sliders[i] = gtk_scale_new_with_range(GTK_ORIENTATION_VERTICAL, -16, 16, 1);
    gtk_scale_set_value_pos(GTK_SCALE(tx_gain_sliders[i]), GTK_POS_RIGHT);
    gtk_range_set_value(GTK_RANGE(tx_gain_sliders[i]), 0);
    gtk_widget_set_size_request(tx_gain_sliders[i], 100, 150);
    gtk_range_set_inverted(GTK_RANGE(tx_gain_sliders[i]), TRUE);
    g_signal_connect(tx_gain_sliders[i], "value-changed", G_CALLBACK(on_tx_gain_value_changed), GINT_TO_POINTER(i));

    // Assign CSS class "gain-slider" to TX Gain Slider
    GtkStyleContext *gain_context = gtk_widget_get_style_context(tx_gain_sliders[i]);
    gtk_style_context_add_class(gain_context, "gain-slider");
    gtk_style_context_add_provider(gain_context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    gtk_grid_attach(GTK_GRID(tx_grid), tx_gain_sliders[i], i, 0, 1, 1);

    // TX Frequency Slider
    tx_freq_sliders[i] = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, freq_min[i], freq_max[i], 1);
    gtk_scale_set_value_pos(GTK_SCALE(tx_freq_sliders[i]), GTK_POS_BOTTOM);
    gtk_range_set_value(GTK_RANGE(tx_freq_sliders[i]), freq_initial[i]);
    g_signal_connect(tx_freq_sliders[i], "value-changed", G_CALLBACK(on_tx_freq_value_changed), GINT_TO_POINTER(i));

    // Assign CSS class "freq-slider" to TX Frequency Slider
    GtkStyleContext *freq_context = gtk_widget_get_style_context(tx_freq_sliders[i]);
    gtk_style_context_add_class(freq_context, "freq-slider");
    gtk_style_context_add_provider(freq_context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    gtk_grid_attach(GTK_GRID(tx_grid), tx_freq_sliders[i], i, 1, 1, 1);
}


    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), tx_frame, gtk_label_new("TX EQ"));

    // RX EQ Section
    rx_frame = gtk_frame_new("RX EQ");
    gtk_container_set_border_width(GTK_CONTAINER(rx_frame), 5);
    rx_grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(rx_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(rx_grid), 5);
    gtk_container_add(GTK_CONTAINER(rx_frame), rx_grid);

    for (i = 0; i < 5; i++)
{
    // RX Gain Slider
    rx_gain_sliders[i] = gtk_scale_new_with_range(GTK_ORIENTATION_VERTICAL, -16, 16, 1);
    gtk_scale_set_value_pos(GTK_SCALE(rx_gain_sliders[i]), GTK_POS_RIGHT);
    gtk_range_set_value(GTK_RANGE(rx_gain_sliders[i]), 0);
    gtk_widget_set_size_request(rx_gain_sliders[i], 100, 150);
    gtk_range_set_inverted(GTK_RANGE(rx_gain_sliders[i]), TRUE);
    g_signal_connect(rx_gain_sliders[i], "value-changed", G_CALLBACK(on_rx_gain_value_changed), GINT_TO_POINTER(i));

    // Assign CSS class "gain-slider" to RX Gain Slider
    GtkStyleContext *gain_context = gtk_widget_get_style_context(rx_gain_sliders[i]);
    gtk_style_context_add_class(gain_context, "gain-slider");
    gtk_style_context_add_provider(gain_context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    gtk_grid_attach(GTK_GRID(rx_grid), rx_gain_sliders[i], i, 0, 1, 1);

    // RX Frequency Slider
    rx_freq_sliders[i] = gtk_scale_new_with_range(GTK_ORIENTATION_HORIZONTAL, freq_min[i], freq_max[i], 1);
    gtk_scale_set_value_pos(GTK_SCALE(rx_freq_sliders[i]), GTK_POS_BOTTOM);
    gtk_range_set_value(GTK_RANGE(rx_freq_sliders[i]), freq_initial[i]);
    g_signal_connect(rx_freq_sliders[i], "value-changed", G_CALLBACK(on_rx_freq_value_changed), GINT_TO_POINTER(i));

    // Assign CSS class "freq-slider" to RX Frequency Slider
    GtkStyleContext *freq_context = gtk_widget_get_style_context(rx_freq_sliders[i]);
    gtk_style_context_add_class(freq_context, "freq-slider");
    gtk_style_context_add_provider(freq_context, GTK_STYLE_PROVIDER(provider), GTK_STYLE_PROVIDER_PRIORITY_USER);

    gtk_grid_attach(GTK_GRID(rx_grid), rx_freq_sliders[i], i, 1, 1, 1);
}


    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), rx_frame, gtk_label_new("RX EQ"));

    // Close Button
    close_button = gtk_button_new_with_label("Save and Close");
    g_signal_connect(close_button, "clicked", G_CALLBACK(on_close_button_clicked), window);
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook), close_button, gtk_label_new("Close"));

    gtk_widget_show_all(window);

    // Center the window
    // Ensure parent and child windows are realized
    gtk_widget_realize(parent);
    gtk_widget_realize(window);

    // Get the screen dimensions
    GdkScreen *screen = gtk_window_get_screen(GTK_WINDOW(window));

    // Get the size of the parent window (including decorations)
    GdkRectangle parent_geometry;
    gdk_window_get_frame_extents(gtk_widget_get_window(parent), &parent_geometry);

    // Get the size of the child window (including decorations)
    GdkRectangle child_geometry;
    gdk_window_get_frame_extents(gtk_widget_get_window(window), &child_geometry);

    // Calculate the centered position relative to the parent
    int center_x = parent_geometry.x + (parent_geometry.width - child_geometry.width) / 2;
    int center_y = parent_geometry.y + (parent_geometry.height - child_geometry.height) / 2;


    // Move the window to the calculated position
    gtk_window_move(GTK_WINDOW(window), center_x, center_y);

    // Debugging: Print calculated positions
    g_print("Parent: (%d, %d, %d, %d)\n", parent_geometry.x, parent_geometry.y, parent_geometry.width, parent_geometry.height);
    g_print("Child: (%d, %d, %d, %d)\n", child_geometry.x, child_geometry.y, child_geometry.width, child_geometry.height);
    g_print("Center Position: (%d, %d)\n", center_x, center_y);



    get_print_and_set_values(tx_freq_sliders, tx_gain_sliders, "tx");
    get_print_and_set_values(rx_freq_sliders, rx_gain_sliders, "rx");
}
