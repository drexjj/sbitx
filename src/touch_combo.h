// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Shawn Rutledge K7IHZ / LB2JK <s@ecloud.org>

/*
 * touch_combo.h - Touch-friendly combo replacement for GTK3
 *
 * A lightweight, reusable widget that provides a touch-friendly alternative
 * to GtkComboBoxText. It shows a popover (or transient popup) with a
 * GtkListBox and tracks pointer/touch motion so rows are highlighted while
 * dragging and selection is performed on release. This avoids the common
 * touch/drag problems of the stock GtkComboBox on many touchscreens.
 *
 * This header declares a simple C API that wraps the widget as a generic
 * GtkWidget pointer so it can be embedded in dialogs and boxes the same way
 * as other widgets.
 *
 * Usage:
 *
 *   GtkWidget *tc = touch_combo_new();
 *   touch_combo_append_text(tc, "Option A");
 *   touch_combo_append_text(tc, "Option B");
 *   g_signal_connect(tc, "changed", G_CALLBACK(on_changed), NULL);
 *   gtk_box_pack_start(GTK_BOX(container), tc, FALSE, FALSE, 0);
 *
 * Signals:
 *   "changed" - emitted when the active item changes (same semantics as
 *               GtkComboBox "changed" signal).
 *
 * Notes:
 * - The active text returned by touch_combo_get_active_text() is owned by
 *   the widget; do not free it.
 * - The widget is implemented in plain GTK widgets (button + popover +
 *   listbox) in the C file. The header exposes a minimal procedural API so
 *   it is easy to reuse in multiple dialogs.
 *
 * Written by ChatGPT 5 Mini
 * Likely inspired by https://forums.raspberrypi.com/viewtopic.php?t=223035
 * Works around https://gitlab.gnome.org/GNOME/gtk/-/issues/7762
 */

#ifndef SBITX_TOUCH_COMBO_H
#define SBITX_TOUCH_COMBO_H

#include <gtk/gtk.h>

G_BEGIN_DECLS

/* Opaque type alias: the widget itself is a GtkWidget *
 * returned by `touch_combo_new()`. Consumers treat it
 * as a GtkWidget and use the procedural API below. */

/* Create a new touch-friendly combo widget.
 *
 * The returned widget should be packed into your container. It is a normal
 * GtkWidget (a small button-like control) that shows a popover when clicked.
 */
GtkWidget* touch_combo_new(void);

/* Append a textual item to the combo.
 *
 * The text is copied internally.
 */
void touch_combo_append_text(GtkWidget *touch_combo, const char *text);

/* Clear all items from the combo.
 *
 * Resets the active index to -1 and removes all stored texts.
 */
void touch_combo_clear(GtkWidget *touch_combo);

/* Get the active index, or -1 if none is selected. */
int touch_combo_get_active(GtkWidget *touch_combo);

/* Set the active index programmatically.
 *
 * If `index` is out of range (<0 or >= item count) the active selection is
 * cleared and the visible label set to the empty string.
 */
void touch_combo_set_active(GtkWidget *touch_combo, int index);

/* Get the active text (NULL if none).
 *
 * The returned string pointer is owned by the widget and must not be freed.
 */
const char* touch_combo_get_active_text(GtkWidget *touch_combo);

/* Set the preferred popup size (width, height in pixels).
 *
 * If either parameter is <= 0 the default size is used for that dimension.
 */
void touch_combo_set_popup_size(GtkWidget *touch_combo, int width, int height);

/* Convenience: append items from a NULL-terminated array of strings */
void touch_combo_append_texts(GtkWidget *touch_combo, const char * const texts[]);

/* Register a callback that will be invoked when the selection changes.
 *
 * The callback will be called with the touch-combo GtkWidget as the first
 * argument and the provided user_data as the second argument.
 */
void touch_combo_set_changed_callback(GtkWidget *touch_combo, GCallback cb, gpointer user_data);

/* Example:
 *
 *   GtkWidget *tc = touch_combo_new();
 *   touch_combo_append_text(tc, "One");
 *   touch_combo_append_text(tc, "Two");
 *   touch_combo_set_popup_size(tc, 280, 220);
 *   touch_combo_set_changed_callback(tc, G_CALLBACK(on_touch_combo_changed), NULL);
 *
 * Signal handler signature:
 *   void on_touch_combo_changed(GtkWidget *widget, gpointer user_data)
 *
 */

G_END_DECLS

#endif /* SBITX_TOUCH_COMBO_H */
