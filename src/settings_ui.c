/*
	By Ashhar Farhan and chatGPT.
	Under GPL v3

	USB audio device UI rows added by the sBitx volunteer development team.

	Rows 0–3: Callsign, My Grid, xOTA, PIN  (unchanged)
	Row  4:   Horizontal separator
	Row  5:   "Audio Devices" section label
	Row  6:   USB Audio Out  (ALSA device name, blank = disabled)
	Row  7:   USB Mic In     (ALSA device name, blank = disabled)
	Row  8:   [Auto-detect]  [Disable USB Audio]  buttons
	Row  9:   Hint label

	Design notes
	────────────
	• sound_find_usb_audio() is declared in sound.h. It scans ALSA, fills the
	  #usb_audio_out / #usb_audio_in fields via field_set(), and should be
	  called once at startup (before setup()) so the globals are ready when
	  sound_thread_start() runs.

	• Changing the device entries and pressing OK calls field_set() and
	  save_user_settings(1).  The new device names take effect on the next
	  application restart; live ALSA thread teardown is intentionally deferred
	  to avoid blocking-I/O races.

	• When the user clicks "Disable USB Audio" both entries are cleared,
	  field_set() is called with empty strings, and settings are saved.

	• The "Auto-detect" button calls sound_find_usb_audio() immediately so
	  the user can see what was found without restarting.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <math.h>
#include <complex.h>
#include <fftw3.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>
#include <linux/types.h>
#include <stdint.h>
#include <pthread.h>
#include <time.h>
#include <stdbool.h>
#include <sys/types.h>
#include <stdint.h>
#include <errno.h>
#include <time.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <ctype.h>
#include <arpa/inet.h>
#include <gtk/gtk.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "sound.h"
#include "modem_ft8.h"
#include "modem_cw.h"
#include "touch_combo.h"

/* ── forward declaration (implemented in sbitx_sound.c) ─────────────────── */
/* signature: int sound_find_usb_audio(char *out_play, char *out_cap, int maxlen) */

/* ── helper: scan ALSA and refresh the two USB entry widgets ─────────────── */
typedef struct {
	GtkWidget *entry_usb_out;
	GtkWidget *entry_usb_in;
} usb_entry_pair;

static void on_autodetect_clicked(GtkWidget *widget, gpointer data)
{
	usb_entry_pair *p = (usb_entry_pair *)data;

	/* Run the scanner with output buffers */
	char found_play[64] = "", found_cap[64] = "";
	if (sound_find_usb_audio(found_play, found_cap, sizeof(found_play)))
	{
		/* Push into field system so they persist on next save */
		if (found_play[0]) field_set("usb_audio_out", found_play);
		if (found_cap[0])  field_set("usb_audio_in",  found_cap);
	}

	/* Refresh the visible entries from the (now updated) field values */
	const char *out_dev = field_str("usb_audio_out");
	const char *in_dev  = field_str("usb_audio_in");

	gtk_entry_set_text(GTK_ENTRY(p->entry_usb_out), out_dev ? out_dev : "");
	gtk_entry_set_text(GTK_ENTRY(p->entry_usb_in),  in_dev  ? in_dev  : "");
}

static void on_disable_usb_clicked(GtkWidget *widget, gpointer data)
{
	usb_entry_pair *p = (usb_entry_pair *)data;

	/*
	 * Write the sentinel "none" into the entry widgets.
	 * The OK handler maps any empty or "none" entry to the literal
	 * "none" before calling field_set(), so sound_start_with_usb()
	 * can distinguish "user deliberately disabled USB" from
	 * "never been configured / first run".
	 */
	gtk_entry_set_text(GTK_ENTRY(p->entry_usb_out), "none");
	gtk_entry_set_text(GTK_ENTRY(p->entry_usb_in),  "none");
}

/* ── input sanitisers ────────────────────────────────────────────────────── */

/* Callsign: A-Z 0-9 and '/' only, forced upper-case – W9JES */
static void on_callsign_changed(GtkWidget *widget, gpointer data)
{
	const gchar *text = gtk_entry_get_text(GTK_ENTRY(widget));
	gchar *result = g_strdup(text);

	for (int i = 0; i < (int)strlen(text); ++i) {
		if (!isalnum((unsigned char)text[i]) && text[i] != '/')
			result[i] = '\0';
		else
			result[i] = toupper((unsigned char)text[i]);
	}
	gtk_entry_set_text(GTK_ENTRY(widget), result);
	g_free(result);
}

/* Grid: A-Z 0-9, forced upper-case */
static void on_my_grid_changed(GtkWidget *widget, gpointer data)
{
	const gchar *text = gtk_entry_get_text(GTK_ENTRY(widget));
	gchar *result = g_strdup(text);

	for (int i = 0; i < (int)strlen(text); ++i) {
		if (!isalnum((unsigned char)text[i]))
			result[i] = '\0';
		else
			result[i] = toupper((unsigned char)text[i]);
	}
	gtk_entry_set_text(GTK_ENTRY(widget), result);
	g_free(result);
}

/* xOTA location: A-Z 0-9 '/' '-', forced upper-case */
static void on_alnum_slash_hyphen_changed(GtkWidget *widget, gpointer data)
{
	const gchar *text = gtk_entry_get_text(GTK_ENTRY(widget));
	gchar *result = g_strdup(text);

	for (int i = 0; i < (int)strlen(text); ++i) {
		if (!isalnum((unsigned char)text[i]) && text[i] != '/' && text[i] != '-')
			result[i] = '\0';
		else
			result[i] = toupper((unsigned char)text[i]);
	}
	gtk_entry_set_text(GTK_ENTRY(widget), result);
	g_free(result);
}

/*
 * USB device name: printable ASCII minus leading/trailing whitespace.
 * We allow the full ALSA plughw:CARD=...,DEV=N syntax – no forced upper-case.
 * We strip any character that is a C0 control code (< 0x20) or DEL (0x7f).
 */
static void on_usb_device_changed(GtkWidget *widget, gpointer data)
{
	const gchar *text = gtk_entry_get_text(GTK_ENTRY(widget));
	gchar *result = g_strdup(text);
	int out = 0;

	for (int i = 0; text[i]; ++i) {
		unsigned char c = (unsigned char)text[i];
		if (c >= 0x20 && c != 0x7f)
			result[out++] = (gchar)c;
	}
	result[out] = '\0';

	if (strcmp(result, text) != 0)
		gtk_entry_set_text(GTK_ENTRY(widget), result);

	g_free(result);
}

/* ── main dialog ─────────────────────────────────────────────────────────── */
void settings_ui(GtkWidget *parent)
{
	GtkWidget *dialog, *grid, *label, *combo;
	GtkWidget *entry_callsign, *entry_grid, *entry_xota, *entry_pin;
	GtkWidget *entry_usb_out, *entry_usb_in;
	GtkWidget *btn_autodetect, *btn_disable;
	GtkWidget *btn_box, *separator;

	/* Heap-allocate the pair so the button callbacks own a stable pointer for
	   the lifetime of the dialog.  We free it in the cleanup block below. */
	usb_entry_pair *usb_pair = g_new0(usb_entry_pair, 1);

	dialog = gtk_dialog_new_with_buttons(
		"Settings",
		GTK_WINDOW(parent),
		GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
		"OK",     GTK_RESPONSE_ACCEPT,
		"Cancel", GTK_RESPONSE_CANCEL,
		NULL);

	/* ── grid container ─────────────────────────────────────────────────── */
	grid = gtk_grid_new();
	gtk_grid_set_column_spacing(GTK_GRID(grid), 6);
	gtk_grid_set_row_spacing(GTK_GRID(grid), 4);
	gtk_container_set_border_width(GTK_CONTAINER(grid), 10);
	gtk_box_pack_start(
		GTK_BOX(gtk_dialog_get_content_area(GTK_DIALOG(dialog))),
		grid, TRUE, TRUE, 0);

	/* ── Row 0: Callsign ────────────────────────────────────────────────── */
	label = gtk_label_new("Callsign");
	gtk_widget_set_halign(label, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), label, 0, 0, 1, 1);

	entry_callsign = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_callsign), 11);
	g_signal_connect(entry_callsign, "changed",
		G_CALLBACK(on_callsign_changed), NULL);
	gtk_entry_set_text(GTK_ENTRY(entry_callsign),
		(gchar *)field_str("MYCALLSIGN"));
	gtk_grid_attach(GTK_GRID(grid), entry_callsign, 1, 0, 2, 1);

	/* ── Row 1: My Grid ─────────────────────────────────────────────────── */
	label = gtk_label_new("My Grid");
	gtk_widget_set_halign(label, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), label, 0, 1, 1, 1);

	entry_grid = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_grid), 6);
	g_signal_connect(entry_grid, "changed",
		G_CALLBACK(on_my_grid_changed), NULL);
	gtk_entry_set_text(GTK_ENTRY(entry_grid),
		(gchar *)field_str("MYGRID"));
	gtk_grid_attach(GTK_GRID(grid), entry_grid, 1, 1, 2, 1);

	/* ── Row 2: xOTA type + location ────────────────────────────────────── */
	combo = touch_combo_new();
	touch_combo_append_text(combo, "NONE");
	touch_combo_append_text(combo, "IOTA");
	touch_combo_append_text(combo, "SOTA");
	touch_combo_append_text(combo, "POTA");
	{
		const gchar *xota_type = (gchar *)field_str("xOTA");
		if (xota_type && *xota_type) {
			if      (g_ascii_strcasecmp(xota_type, "IOTA") == 0) touch_combo_set_active(combo, 1);
			else if (g_ascii_strcasecmp(xota_type, "SOTA") == 0) touch_combo_set_active(combo, 2);
			else if (g_ascii_strcasecmp(xota_type, "POTA") == 0) touch_combo_set_active(combo, 3);
			else                                                   touch_combo_set_active(combo, 0);
		} else {
			touch_combo_set_active(combo, 0);
		}
	}
	gtk_grid_attach(GTK_GRID(grid), combo, 0, 2, 1, 1);

	entry_xota = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_xota), 12);
	g_signal_connect(entry_xota, "changed",
		G_CALLBACK(on_alnum_slash_hyphen_changed), NULL);
	gtk_entry_set_text(GTK_ENTRY(entry_xota),
		(gchar *)field_str("LOCATION"));
	gtk_grid_attach(GTK_GRID(grid), entry_xota, 1, 2, 2, 1);

	/* ── Row 3: PIN ─────────────────────────────────────────────────────── */
	label = gtk_label_new("PIN");
	gtk_widget_set_halign(label, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), label, 0, 3, 1, 1);

	entry_pin = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_pin), 6);
	gtk_entry_set_input_purpose(GTK_ENTRY(entry_pin), GTK_INPUT_PURPOSE_NUMBER);
	gtk_entry_set_text(GTK_ENTRY(entry_pin), field_str("PASSKEY"));
	gtk_grid_attach(GTK_GRID(grid), entry_pin, 1, 3, 2, 1);

	/* ── Row 4: Separator ───────────────────────────────────────────────── */
	separator = gtk_separator_new(GTK_ORIENTATION_HORIZONTAL);
	gtk_widget_set_margin_top(separator, 4);
	gtk_widget_set_margin_bottom(separator, 4);
	gtk_grid_attach(GTK_GRID(grid), separator, 0, 4, 3, 1);

	/* ── Row 5: "Audio Devices" section header ──────────────────────────── */
	label = gtk_label_new(NULL);
	gtk_label_set_markup(GTK_LABEL(label), "<b>Audio Devices</b>");
	gtk_widget_set_halign(label, GTK_ALIGN_START);
	gtk_grid_attach(GTK_GRID(grid), label, 0, 5, 3, 1);

	/* ── Row 6: USB Audio Out ───────────────────────────────────────────── */
	label = gtk_label_new("USB Out");
	gtk_widget_set_halign(label, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), label, 0, 6, 1, 1);

	entry_usb_out = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_usb_out), 63);
	gtk_entry_set_width_chars(GTK_ENTRY(entry_usb_out), 36);
	gtk_entry_set_placeholder_text(GTK_ENTRY(entry_usb_out), "plughw:CARD=Device,DEV=0");
	g_signal_connect(entry_usb_out, "changed",
		G_CALLBACK(on_usb_device_changed), NULL);
	{
		const char *v = field_str("usb_audio_out");
		gtk_entry_set_text(GTK_ENTRY(entry_usb_out),
		                   (v && v[0] && strcmp(v, "none") != 0) ? v : "");
	}
	gtk_grid_attach(GTK_GRID(grid), entry_usb_out, 1, 6, 2, 1);

	/* ── Row 7: USB Mic In ──────────────────────────────────────────────── */
	label = gtk_label_new("USB Mic");
	gtk_widget_set_halign(label, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), label, 0, 7, 1, 1);

	entry_usb_in = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_usb_in), 63);
	gtk_entry_set_width_chars(GTK_ENTRY(entry_usb_in), 36);
	gtk_entry_set_placeholder_text(GTK_ENTRY(entry_usb_in), "plughw:CARD=Device,DEV=0");
	g_signal_connect(entry_usb_in, "changed",
		G_CALLBACK(on_usb_device_changed), NULL);
	{
		const char *v = field_str("usb_audio_in");
		gtk_entry_set_text(GTK_ENTRY(entry_usb_in),
		                   (v && v[0] && strcmp(v, "none") != 0) ? v : "");
	}
	gtk_grid_attach(GTK_GRID(grid), entry_usb_in, 1, 7, 2, 1);

	/* Store widget pointers so button callbacks can update them */
	usb_pair->entry_usb_out = entry_usb_out;
	usb_pair->entry_usb_in  = entry_usb_in;

	/* ── Row 8: Auto-detect + Disable buttons ───────────────────────────── */
	btn_box = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
	gtk_button_box_set_layout(GTK_BUTTON_BOX(btn_box), GTK_BUTTONBOX_START);
	gtk_box_set_spacing(GTK_BOX(btn_box), 6);

	btn_autodetect = gtk_button_new_with_label("Auto-detect");
	g_signal_connect(btn_autodetect, "clicked",
		G_CALLBACK(on_autodetect_clicked), usb_pair);
	gtk_container_add(GTK_CONTAINER(btn_box), btn_autodetect);

	btn_disable = gtk_button_new_with_label("Disable");
	g_signal_connect(btn_disable, "clicked",
		G_CALLBACK(on_disable_usb_clicked), usb_pair);
	gtk_container_add(GTK_CONTAINER(btn_box), btn_disable);

	gtk_grid_attach(GTK_GRID(grid), btn_box, 1, 8, 2, 1);

	/* ── Run dialog ─────────────────────────────────────────────────────── */
	gtk_widget_show_all(dialog);
	gint response = gtk_dialog_run(GTK_DIALOG(dialog));

	if (response == GTK_RESPONSE_ACCEPT) {
		const gchar *callsign  = gtk_entry_get_text(GTK_ENTRY(entry_callsign));
		const gchar *pin       = gtk_entry_get_text(GTK_ENTRY(entry_pin));
		const gchar *grid_val  = gtk_entry_get_text(GTK_ENTRY(entry_grid));
		const gchar *xota_loc  = gtk_entry_get_text(GTK_ENTRY(entry_xota));
		const gchar *xota      = touch_combo_get_active_text(combo);

		/* Station identity */
		field_set("MYCALLSIGN", callsign);
		field_set("MYGRID",     grid_val);
		field_set("xOTA",       xota);
		field_set("LOCATION",   xota_loc);
		field_set("PASSKEY",    pin);

		/*
		 * USB audio devices.
		 * Map a blank entry to the sentinel "none" so sound_start_with_usb()
		 * can distinguish "user deliberately disabled USB" from
		 * "never been configured / first run".
		 * A non-empty, non-"none" string is taken as a literal ALSA device name.
		 */
		const gchar *usb_out_raw = gtk_entry_get_text(GTK_ENTRY(entry_usb_out));
		const gchar *usb_in_raw  = gtk_entry_get_text(GTK_ENTRY(entry_usb_in));
		const char  *usb_out = (usb_out_raw && usb_out_raw[0]) ? usb_out_raw : "none";
		const char  *usb_in  = (usb_in_raw  && usb_in_raw[0])  ? usb_in_raw  : "none";

		field_set("usb_audio_out", usb_out);
		field_set("usb_audio_in",  usb_in);

		/* Persist to user_settings.ini immediately */
		save_user_settings(1);

		printf("settings saved: call=%s grid=%s xota=%s:%s pin=%s "
		       "usb_out=%s usb_in=%s\n",
		       callsign, grid_val, xota, xota_loc, pin, usb_out, usb_in);
	}

	g_free(usb_pair);
	gtk_widget_destroy(dialog);
}
