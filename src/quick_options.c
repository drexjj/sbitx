#include <gtk/gtk.h>
#include <wiringPi.h>
#include "quick_options.h"

// External dependencies from sbitx_gtk.c
extern GtkWidget *window;
extern int is_fullscreen;
extern void on_fullscreen_toggle(const int requested_state);
extern void on_power_down_button_click(GtkWidget *widget, gpointer data);
extern unsigned long sbitx_millis();

#define ENC1_SW (14)
#define ENC2_SW (3)

// Callback functions for Quick Options dialog
static void on_dialog_fullscreen_clicked(GtkButton *button, gpointer user_data) {
	gtk_dialog_response(GTK_DIALOG(user_data), 1);
}

static void on_dialog_close_clicked(GtkButton *button, gpointer user_data) {
	gtk_dialog_response(GTK_DIALOG(user_data), 2);
}

static void on_dialog_shutdown_clicked(GtkButton *button, gpointer user_data) {
	gtk_dialog_response(GTK_DIALOG(user_data), 3);
}

static void on_dialog_cancel_clicked(GtkButton *button, gpointer user_data) {
	gtk_dialog_response(GTK_DIALOG(user_data), GTK_RESPONSE_CANCEL);
}

// Function to handle dual encoder button press
void handleDualButtonPress() {
	static int enc1_was_pressed = 0;
	static int enc2_was_pressed = 0;
	static unsigned long enc1_press_time = 0;
	static unsigned long enc2_press_time = 0;
	static unsigned long both_pressed_time = 0;
	static int dialog_shown = 0;

	int enc1_pressed = (digitalRead(ENC1_SW) == 0);
	int enc2_pressed = (digitalRead(ENC2_SW) == 0);
	unsigned long now = sbitx_millis();

	// Track individual button press times
	if (enc1_pressed && !enc1_was_pressed) {
		enc1_press_time = now;
	}
	if (enc2_pressed && !enc2_was_pressed) {
		enc2_press_time = now;
	}

	enc1_was_pressed = enc1_pressed;
	enc2_was_pressed = enc2_pressed;

	// Check if both buttons are currently pressed
	if (enc1_pressed && enc2_pressed) {
		// Check if they were pressed within 200ms of each other
		long time_diff = (long)(enc1_press_time - enc2_press_time);
		if (time_diff < 0) time_diff = -time_diff;

		if (time_diff <= 200) {
			// Buttons pressed close enough together - start/check long press timer
			if (both_pressed_time == 0) {
				both_pressed_time = now;
			} else if (now - both_pressed_time >= 1000 && !dialog_shown) {
				// Both buttons held together for more than 1 second - show Quick Options dialog
				dialog_shown = 1;
			GtkWidget *dialog = gtk_dialog_new();
			gtk_window_set_title(GTK_WINDOW(dialog), "Quick Options");
			gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(window));
			gtk_window_set_modal(GTK_WINDOW(dialog), TRUE);
			gtk_window_set_default_size(GTK_WINDOW(dialog), 300, 250);

			// Get content area and create vertical box for buttons
			GtkWidget *content = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
			GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
			gtk_container_set_border_width(GTK_CONTAINER(vbox), 20);
			gtk_container_add(GTK_CONTAINER(content), vbox);

			// Create buttons with consistent size
			GtkWidget *btn_fullscreen = gtk_button_new_with_label("Toggle Fullscreen");
			GtkWidget *btn_close = gtk_button_new_with_label("Close sBitx");
			GtkWidget *btn_shutdown = gtk_button_new_with_label("Shutdown Pi");
			GtkWidget *btn_cancel = gtk_button_new_with_label("Cancel");

			gtk_widget_set_size_request(btn_fullscreen, 250, 40);
			gtk_widget_set_size_request(btn_close, 250, 40);
			gtk_widget_set_size_request(btn_shutdown, 250, 40);
			gtk_widget_set_size_request(btn_cancel, 250, 40);

			gtk_box_pack_start(GTK_BOX(vbox), btn_fullscreen, FALSE, FALSE, 0);
			gtk_box_pack_start(GTK_BOX(vbox), btn_close, FALSE, FALSE, 0);
			gtk_box_pack_start(GTK_BOX(vbox), btn_shutdown, FALSE, FALSE, 0);
			gtk_box_pack_start(GTK_BOX(vbox), btn_cancel, FALSE, FALSE, 0);

			// Connect button signals
			g_signal_connect(btn_fullscreen, "clicked",
				G_CALLBACK(on_dialog_fullscreen_clicked), dialog);
			g_signal_connect(btn_close, "clicked",
				G_CALLBACK(on_dialog_close_clicked), dialog);
			g_signal_connect(btn_shutdown, "clicked",
				G_CALLBACK(on_dialog_shutdown_clicked), dialog);
			g_signal_connect(btn_cancel, "clicked",
				G_CALLBACK(on_dialog_cancel_clicked), dialog);

			gtk_widget_show_all(dialog);

			gint response = gtk_dialog_run(GTK_DIALOG(dialog));
			gtk_widget_destroy(dialog);

			switch(response) {
				case 1:  // Toggle Fullscreen
					on_fullscreen_toggle(!is_fullscreen);
					break;
				case 2:  // Close sBitx
					{
						GtkWidget *confirm = gtk_message_dialog_new(
							GTK_WINDOW(window),
							GTK_DIALOG_MODAL,
							GTK_MESSAGE_QUESTION,
							GTK_BUTTONS_YES_NO,
							"Are you sure you want to close sBitx?"
						);
						gint confirm_response = gtk_dialog_run(GTK_DIALOG(confirm));
						gtk_widget_destroy(confirm);
						if (confirm_response == GTK_RESPONSE_YES) {
							gtk_main_quit();
						}
					}
					break;
				case 3:  // Shutdown Pi
					on_power_down_button_click(NULL, window);
					break;
			}

				// Wait for both buttons to be released
				while (digitalRead(ENC1_SW) == 0 || digitalRead(ENC2_SW) == 0) {
					delay(50);
				}
				// Reset all state
				enc1_was_pressed = 0;
				enc2_was_pressed = 0;
				enc1_press_time = 0;
				enc2_press_time = 0;
				both_pressed_time = 0;
				dialog_shown = 0;
			}
		}
	} else {
		// At least one button not pressed - reset long press timer
		both_pressed_time = 0;
		dialog_shown = 0;

		// Reset press times when buttons are released
		if (!enc1_pressed) {
			enc1_press_time = 0;
		}
		if (!enc2_pressed) {
			enc2_press_time = 0;
		}
	}
}
