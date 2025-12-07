#include <gtk/gtk.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <complex.h>
#include <fftw3.h>
#include "calibration_ui.h"
#include "sdr.h"
#include "sdr_ui.h"

// Define power_settings structure (from sbitx.c)
struct power_settings {
    int f_start;
    int f_stop;
    int max_watts;
    double scale;
};

// Define encoder structure (from sbitx_gtk.c)
struct encoder {
    int pin_a;
    int pin_b;
    int speed;
    int history;
    int prev_state;
    uint64_t last_us;
};

// External variables from sbitx.c and sbitx_gtk.c
extern struct power_settings band_power[];
extern int fwdpower;
extern struct encoder enc_a;
extern int in_tx;

// External functions
extern void set_rx1(int freq);
extern void tr_switch(int tx_on);
extern void set_tx_power_levels(void);
extern void save_hw_settings(void);
extern int enc_read(struct encoder *e);
extern int main_ui_encoders_enabled;  // Flag to disable main UI encoders
extern void sdr_request(char *request, char *response);

// Calibration state
struct calibration_state {
    int selected_band;          // Currently selected band index (0-8)
    int is_transmitting;        // Whether we're currently in TEST mode
    int tx_start_time;          // Timestamp when TX started
    double original_scales[9];  // Backup of original scale values
    GtkWidget *band_buttons[9]; // Band selection button widgets
    GtkWidget *band_label;      // Label showing selected band
    GtkWidget *freq_label;      // Label showing calibration frequency
    GtkWidget *scale_label;     // Label showing scale factor
    GtkWidget *test_button;     // TEST button widget
    GtkWidget *dialog;          // Dialog widget
    guint timeout_id;           // GTK timeout callback ID
};
const int cal_freq[] = {3535000, 5330500,  7035000, 10135000, 14035000, 18068000, 21035000, 24895000, 28035000 };

static struct calibration_state cal_state;

// Get current time in milliseconds
static long get_time_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

// Update band button styles based on modifications
static void update_band_button_styles(void) {
    for (int i = 0; i < 9; i++) {
        GtkStyleContext *context = gtk_widget_get_style_context(cal_state.band_buttons[i]);

        if (i == cal_state.selected_band) {
            // Selected band - remove yellow so blue shows through
            gtk_style_context_remove_class(context, "modified-band");
        } else if (band_power[i].scale != cal_state.original_scales[i]) {
            // Modified but not selected - add yellow highlight
            gtk_style_context_add_class(context, "modified-band");
        } else {
            // Not modified - remove yellow highlight
            gtk_style_context_remove_class(context, "modified-band");
        }
    }
}

// Update the display labels
static void update_display(void) {
    char buff[256];
    int band_idx = cal_state.selected_band;

    // Update band name
    const char *band_names[] = {"80M", "60M", "40M", "30M", "20M", "17M", "15M", "12M", "10M"};
    sprintf(buff, "Selected Band: %s", band_names[band_idx]);
    gtk_label_set_text(GTK_LABEL(cal_state.band_label), buff);

    // Update frequency
    sprintf(buff, "Frequency: %.3f MHz", (cal_freq[band_idx]) / 1000000.0);
    gtk_label_set_text(GTK_LABEL(cal_state.freq_label), buff);

    // Update scale factor - show saved value in parentheses if modified
    double current_scale = band_power[band_idx].scale;
    double saved_scale = cal_state.original_scales[band_idx];

    if (current_scale != saved_scale) {
        sprintf(buff, "Scale Factor: %.4f (%.4f)", current_scale, saved_scale);
    } else {
        sprintf(buff, "Scale Factor: %.4f", current_scale);
    }
    gtk_label_set_text(GTK_LABEL(cal_state.scale_label), buff);

    // Update band button styles
    update_band_button_styles();
}

// Stop transmitting
static void stop_tx(void) {
    if (cal_state.is_transmitting) {
        tr_switch(0);  // Disable TX
        in_tx = TX_OFF;

        cal_state.is_transmitting = 0;
        gtk_button_set_label(GTK_BUTTON(cal_state.test_button), "TEST");
        gtk_widget_set_sensitive(cal_state.test_button, TRUE);
    }
}

// Periodic update callback (called every 100ms)
static gboolean calibration_tick(gpointer user_data) {
    // Check if we're transmitting and if 5 seconds have elapsed
    if (cal_state.is_transmitting) {
        long elapsed = get_time_ms() - cal_state.tx_start_time;
        if (elapsed >= 5000) {  // 5 seconds
            stop_tx();
        }

        // Update power display during TX
        update_display();
    }

    // Read encoder input and adjust scale
    int scroll = enc_read(&enc_a);
    if (scroll != 0) {
        int band_idx = cal_state.selected_band;

        // Adjust scale by ±0.001 per encoder click
        double delta = scroll * 0.001;
        band_power[band_idx].scale += delta;

        // Clamp to reasonable range
        if (band_power[band_idx].scale < 0.001)
            band_power[band_idx].scale = 0.001;
        if (band_power[band_idx].scale > 0.015)
            band_power[band_idx].scale = 0.015;

        // Apply new scale only if transmitting (to avoid setting volume to 95)
        if (cal_state.is_transmitting)
            set_tx_power_levels();

        // Update display
        update_display();

        printf("Calibration: Encoder scroll=%d, new scale=%.4f\n", scroll, band_power[band_idx].scale);
    }

    return TRUE;  // Continue calling this function
}

// Band selection button clicked
static void on_band_selected(GtkWidget *widget, gpointer data) {
    int band_idx = GPOINTER_TO_INT(data);

    // Stop any ongoing transmission
    stop_tx();

    // Remove highlight from previously selected button
    GtkStyleContext *old_context = gtk_widget_get_style_context(cal_state.band_buttons[cal_state.selected_band]);
    gtk_style_context_remove_class(old_context, "selected-band");

    // Update selected band
    cal_state.selected_band = band_idx;

    // Add highlight to newly selected button
    GtkStyleContext *new_context = gtk_widget_get_style_context(cal_state.band_buttons[band_idx]);
    gtk_style_context_add_class(new_context, "selected-band");

    // Update display
    update_display();
}

// Scale adjustment button clicked
static void on_scale_adjust(GtkWidget *widget, gpointer data) {
    int direction = GPOINTER_TO_INT(data);  // +1 or -1
    int band_idx = cal_state.selected_band;

    // Adjust scale by ±0.001
    double delta = direction * 0.001;
    band_power[band_idx].scale += delta;

    // Clamp to reasonable range
    if (band_power[band_idx].scale < 0.001)
        band_power[band_idx].scale = 0.001;
    if (band_power[band_idx].scale > 0.015)
        band_power[band_idx].scale = 0.015;

    // Apply new scale only if transmitting (to avoid setting volume to 95)
    if (cal_state.is_transmitting)
        set_tx_power_levels();

    // Update display
    update_display();

    printf("Calibration: Button %s clicked, new scale=%.4f\n",
           direction > 0 ? "UP" : "DOWN", band_power[band_idx].scale);
}

// TEST button clicked
static void on_test_clicked(GtkWidget *widget, gpointer data) {
    if (cal_state.is_transmitting) {
        // Already transmitting, stop it
        stop_tx();
        return;
    }

    int band_idx = cal_state.selected_band;
    
    char response[100];
    // Set drive to 100 for calibration
    sdr_request("tx_power=100", response);

    // Set frequency to calibration frequency
    set_rx1(cal_freq[band_idx]);

    // Set calibration mode
    extern struct rx *tx_list;
    tx_list->mode = MODE_CALIBRATE;

    // Apply power levels with new drive setting
    set_tx_power_levels();

    // Enable transmit
    tr_switch(1);
    in_tx = TX_SOFT;

    // Mark as transmitting and record start time
    cal_state.is_transmitting = 1;
    cal_state.tx_start_time = get_time_ms();

    // Update button label to show it can be clicked to stop
    gtk_button_set_label(GTK_BUTTON(cal_state.test_button), "STOP");
}

// Save button clicked
static void on_save_clicked(GtkWidget *widget, gpointer data) {
    // Stop any ongoing transmission
    stop_tx();

    // Save settings to hw_settings.ini
    save_hw_settings();

    // Update original_scales to match the saved values
    // This removes the "modified" (yellow) highlighting from bands after saving
    for (int i = 0; i < 9; i++) {
        cal_state.original_scales[i] = band_power[i].scale;
    }

    // Update the display to refresh band button styles
    update_display();

    // Show confirmation
    GtkWidget *msg_dialog = gtk_message_dialog_new(
        GTK_WINDOW(cal_state.dialog),
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        GTK_MESSAGE_INFO,
        GTK_BUTTONS_OK,
        "Calibration settings saved successfully.");
    gtk_dialog_run(GTK_DIALOG(msg_dialog));
    gtk_widget_destroy(msg_dialog);
}

// Restore original values and stop transmission
static void restore_and_cleanup(void) {
    // Stop any ongoing transmission
    stop_tx();

    // Restore original scale values (discard unsaved changes)
    for (int i = 0; i < 9; i++) {
        band_power[i].scale = cal_state.original_scales[i];
    }

    printf("Calibration: Closed without saving, scale values restored\n");
}

// Close button clicked
static void on_close_clicked(GtkWidget *widget, gpointer data) {
    restore_and_cleanup();

    // Close the dialog by sending response
    gtk_dialog_response(GTK_DIALOG(cal_state.dialog), GTK_RESPONSE_CLOSE);
}

// Handle window delete event (X button)
static gboolean on_delete_event(GtkWidget *widget, GdkEvent *event, gpointer data) {
    restore_and_cleanup();

    // Allow the window to close
    return FALSE;
}

// Main calibration UI function
void calibration_ui(GtkWidget *parent) {
    GtkWidget *dialog, *grid, *label, *button;
    int row = 0;

    // Check if we're in USB mode
    const char *current_mode = field_str("MODE");
    if (!current_mode || strcmp(current_mode, "USB") != 0) {
        GtkWidget *msg_dialog = gtk_message_dialog_new(
            GTK_WINDOW(parent),
            GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
            GTK_MESSAGE_WARNING,
            GTK_BUTTONS_OK,
            "Switch to USB mode and then reopen this dialog.");
        gtk_dialog_run(GTK_DIALOG(msg_dialog));
        gtk_widget_destroy(msg_dialog);
        return;
    }

    // Initialize calibration state
    memset(&cal_state, 0, sizeof(cal_state));
    cal_state.selected_band = 0;  // Default to 80M

    // Save original scale values for all bands
    for (int i = 0; i < 9; i++) {
        cal_state.original_scales[i] = band_power[i].scale;
    }

    // Disable main UI encoders while calibration is active
    main_ui_encoders_enabled = 0;

    // Create modal dialog
    dialog = gtk_dialog_new_with_buttons(
        "Power Calibration",
        GTK_WINDOW(parent),
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        NULL);

    cal_state.dialog = dialog;
    gtk_window_set_default_size(GTK_WINDOW(dialog), 180, 300);

    // Position window on the left side of the screen
    gtk_window_move(GTK_WINDOW(dialog), 10, 50);

    // Remove window control buttons (close, maximize, minimize)
    gtk_window_set_deletable(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);

    // Connect delete-event handler in case X button appears despite deletable=FALSE
    g_signal_connect(dialog, "delete-event", G_CALLBACK(on_delete_event), NULL);

    // Create grid for layout
    grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 10);
    gtk_box_pack_start(GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(dialog))),
                       grid, TRUE, TRUE, 0);

    // Add CSS for selected and modified band buttons
    GtkCssProvider *css_provider = gtk_css_provider_new();
    const char *css_data =
        ".selected-band { "
        "  background-image: none; "
        "  background-color: #3584e4; "  // Blue background
        "  color: white; "
        "  font-weight: bold; "
        "}"
        ".modified-band { "
        "  background-image: none; "
        "  background-color: #f9f06b; "  // Yellow background
        "  color: black; "
        "}";
    gtk_css_provider_load_from_data(css_provider, css_data, -1, NULL);
    gtk_style_context_add_provider_for_screen(
        gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(css_provider),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    // Band selection buttons in 3x3 grid
    const char *band_names[] = {"80M", "60M", "40M", "30M", "20M", "17M", "15M", "12M", "10M"};

    // 3x3 grid layout: 80M-40M (row 0), 30M-17M (row 1), 15M-10M (row 2)
    int band_indices[3][3] = {
        {0, 1, 2},  // Row 0: 80M, 60M, 40M
        {3, 4, 5},  // Row 1: 30M, 20M, 17M
        {6, 7, 8}   // Row 2: 15M, 12M, 10M
    };

    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            int i = band_indices[r][c];
            button = gtk_button_new_with_label(band_names[i]);
            gtk_widget_set_size_request(button, 70, 40);
            g_signal_connect(button, "clicked", G_CALLBACK(on_band_selected), GINT_TO_POINTER(i));
            gtk_grid_attach(GTK_GRID(grid), button, c, row + r, 1, 1);
            cal_state.band_buttons[i] = button;  // Store button reference
        }
    }
    row += 3;

    // Highlight 80M button by default (index 0)
    GtkStyleContext *context = gtk_widget_get_style_context(cal_state.band_buttons[0]);
    gtk_style_context_add_class(context, "selected-band");

    // Add spacing
    label = gtk_label_new("");
    gtk_grid_attach(GTK_GRID(grid), label, 0, row, 3, 1);
    row++;

    // Display labels
    cal_state.band_label = gtk_label_new("Selected Band: 80M");
    gtk_label_set_xalign(GTK_LABEL(cal_state.band_label), 0.0);
    gtk_grid_attach(GTK_GRID(grid), cal_state.band_label, 0, row, 3, 1);
    row++;

    cal_state.freq_label = gtk_label_new("Frequency: 0 kHz");
    gtk_label_set_xalign(GTK_LABEL(cal_state.freq_label), 0.0);
    gtk_grid_attach(GTK_GRID(grid), cal_state.freq_label, 0, row, 3, 1);
    row++;

    cal_state.scale_label = gtk_label_new("Scale Factor: 0.0000");
    gtk_label_set_xalign(GTK_LABEL(cal_state.scale_label), 0.0);
    gtk_grid_attach(GTK_GRID(grid), cal_state.scale_label, 0, row, 3, 1);
    row++;

    // Scale adjustment buttons
    button = gtk_button_new_with_label("Scale -");
    gtk_widget_set_size_request(button, 70, 35);
    g_signal_connect(button, "clicked", G_CALLBACK(on_scale_adjust), GINT_TO_POINTER(-1));
    gtk_grid_attach(GTK_GRID(grid), button, 0, row, 1, 1);

    button = gtk_button_new_with_label("Scale +");
    gtk_widget_set_size_request(button, 70, 35);
    g_signal_connect(button, "clicked", G_CALLBACK(on_scale_adjust), GINT_TO_POINTER(1));
    gtk_grid_attach(GTK_GRID(grid), button, 1, row, 1, 1);

    // TEST button
    cal_state.test_button = gtk_button_new_with_label("TEST");
    gtk_widget_set_size_request(cal_state.test_button, 70, 35);
    g_signal_connect(cal_state.test_button, "clicked", G_CALLBACK(on_test_clicked), NULL);
    gtk_grid_attach(GTK_GRID(grid), cal_state.test_button, 2, row, 1, 1);
    row++;

    // Save and Close buttons
    button = gtk_button_new_with_label("Save");
    gtk_widget_set_size_request(button, 100, 35);
    g_signal_connect(button, "clicked", G_CALLBACK(on_save_clicked), NULL);
    gtk_grid_attach(GTK_GRID(grid), button, 0, row, 1, 1);

    button = gtk_button_new_with_label("Close");
    gtk_widget_set_size_request(button, 100, 35);
    g_signal_connect(button, "clicked", G_CALLBACK(on_close_clicked), NULL);
    gtk_grid_attach(GTK_GRID(grid), button, 2, row, 1, 1);

    // Initial display update
    update_display();

    // Start periodic update timer (100ms interval)
    cal_state.timeout_id = g_timeout_add(1, calibration_tick, NULL);

    // Show dialog
    gtk_widget_show_all(dialog);
    gtk_dialog_run(GTK_DIALOG(dialog));

    // Clean up
    stop_tx();
    g_source_remove(cal_state.timeout_id);
    gtk_widget_destroy(dialog);

    // Re-enable main UI encoders
    main_ui_encoders_enabled = 1;
}
