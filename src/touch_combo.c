// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Shawn Rutledge K7IHZ / LB2JK <s@ecloud.org>

/* Touch-friendly combobox replacement for GTK3
 *
 * Provides a compact widget (button + popover + listbox) that behaves well on
 * touchscreens: it highlights rows while dragging and selects on release.
 *
 * Written mostly by ChatGPT 5 mini
 */

#include "touch_combo.h"
#include <string.h>

/* Internal data structure stored on the public widget (root GtkWidget) */
typedef struct {
	GtkWidget *root;         /* the returned widget (container) */
	GtkWidget *button;       /* visible control */
	GtkWidget *label;        /* label showing selected text */
	GtkWidget *arrow;        /* arrow image */
	GtkWidget *popover;      /* GtkPopover containing the list */
	GtkWidget *scrolled;     /* cached GtkScrolledWindow we create */
	GtkWidget *listbox;      /* GtkListBox of items */
	GList     *items;        /* list of char* (g_strdup'd) */
	int        active;       /* selection: active index or -1 */
	GtkListBoxRow *hover_row;/* currently hovered row during drag */
	int        popup_width;  /* optional override width (px) */
	int        popup_height; /* optional override height (px) - used as cap */
	/* convenience callback in addition to standard "changed" signal */
	void     (*changed_cb)(GtkWidget *widget, gpointer user_data);
	gpointer   changed_user_data;
} TouchComboData;

/* Forward declarations */
static TouchComboData *tcd_from_widget(GtkWidget *widget);
static void ensure_popover(TouchComboData *tcd);
static void adjust_popover_size(TouchComboData *tcd);
static void touch_combo_data_free(gpointer data);
static gboolean list_motion_cb(GtkWidget *widget, GdkEventMotion *event, gpointer user_data);
static gboolean list_button_release_cb(GtkWidget *widget, GdkEventButton *event, gpointer user_data);
static void list_row_activated_cb(GtkListBox *box, GtkListBoxRow *row, gpointer user_data);
static void touch_combo_button_clicked_cb(GtkButton *b, gpointer user_data);

/* Helper: retrieve our data from the public root widget */
static TouchComboData *
tcd_from_widget(GtkWidget *widget)
{
	if (!widget) return NULL;
	return (TouchComboData*)g_object_get_data(G_OBJECT(widget), "touch-combo-data");
}

/* Cleanup function passed to g_object_set_data_full */
static void
touch_combo_data_free(gpointer data)
{
	TouchComboData *tcd = (TouchComboData*)data;
	if (!tcd) return;
	if (tcd->items)
		g_list_free_full(tcd->items, g_free);
	/* GTK will free child widgets; only free our struct */
	g_free(tcd);
}

/* Emit convenience callback only.
 * We do not emit a 'changed' signal on the root GtkBox because that type
 * does not declare such a signal.
 * Consumers should register via touch_combo_set_changed_callback().
 */
static void
emit_changed(TouchComboData *tcd)
{
	if (!tcd) return;
	if (tcd->changed_cb)
		tcd->changed_cb(tcd->root, tcd->changed_user_data);
}

/* Motion handler: track pointer and apply prelight to hovered row */
static gboolean
list_motion_cb(GtkWidget *widget, GdkEventMotion *event, gpointer user_data)
{
	TouchComboData *tcd = tcd_from_widget((GtkWidget*)user_data);
	if (!tcd || !tcd->listbox || !event) return FALSE;

	GtkListBoxRow *row = gtk_list_box_get_row_at_y(GTK_LIST_BOX(tcd->listbox), event->y);
	if (row != tcd->hover_row) {
		if (tcd->hover_row)
			gtk_widget_set_state_flags(GTK_WIDGET(tcd->hover_row), GTK_STATE_FLAG_NORMAL, TRUE);
		tcd->hover_row = row;
		if (tcd->hover_row)
			gtk_widget_set_state_flags(GTK_WIDGET(tcd->hover_row), GTK_STATE_FLAG_PRELIGHT, TRUE);
	}
	return FALSE; /* allow further handling */
}

/* Button release: if a row is hovered, select it and close popover */
static gboolean
list_button_release_cb(GtkWidget *widget, GdkEventButton *event, gpointer user_data)
{
	TouchComboData *tcd = tcd_from_widget((GtkWidget*)user_data);
	if (!tcd) return FALSE;

	if (tcd->hover_row) {
		gpointer idxp = g_object_get_data(G_OBJECT(tcd->hover_row), "index");
		if (idxp) {
			int index = GPOINTER_TO_INT(idxp) - 1;
			if (index >= 0 && index < g_list_length(tcd->items)) {
				tcd->active = index;
				const char *s = (const char*)g_list_nth_data(tcd->items, index);
				gtk_label_set_text(GTK_LABEL(tcd->label), s ? s : "");
				emit_changed(tcd);
			}
		}
	}
	if (tcd->popover)
		gtk_popover_popdown(GTK_POPOVER(tcd->popover));
	return TRUE; /* handled */
}

/* Row activated (double-click or Enter): immediate selection */
static void
list_row_activated_cb(GtkListBox *box, GtkListBoxRow *row, gpointer user_data)
{
	TouchComboData *tcd = tcd_from_widget((GtkWidget*)user_data);
	if (!tcd || !row) return;

	gpointer idxp = g_object_get_data(G_OBJECT(row), "index");
	if (!idxp) return;
	int index = GPOINTER_TO_INT(idxp) - 1;
	if (index < 0 || index >= g_list_length(tcd->items)) return;

	tcd->active = index;
	const char *s = (const char*)g_list_nth_data(tcd->items, index);
	gtk_label_set_text(GTK_LABEL(tcd->label), s ? s : "");
	emit_changed(tcd);
	if (tcd->popover)
		gtk_popover_popdown(GTK_POPOVER(tcd->popover));
}

/* Compute preferred size of listed rows and set scrolled window size_request.
 * - Use cached tcd->scrolled when available; otherwise walk parents to find it.
 * - popup_width (if >0) used as width; popup_height (if >0) used as max height.
 */
static void
adjust_popover_size(TouchComboData *tcd)
{
	if (!tcd || !tcd->listbox) return;

	GtkWidget *sw = tcd->scrolled;
	if (!sw) return;

	/* iterate rows and sum natural heights, also track max natural width */
	int total_h = 0;
	int max_w = 0;
	GList *children = gtk_container_get_children(GTK_CONTAINER(tcd->listbox));
	for (GList *l = children; l; l = l->next) {
		GtkWidget *row = GTK_WIDGET(l->data);
		if (!GTK_IS_WIDGET(row)) continue;

		/* ensure widget is shown/realized so preferred size is available */
		gtk_widget_show(row);

		int min_h = 0, nat_h = 0;
		int min_w = 0, nat_w = 0;
		gtk_widget_get_preferred_height(row, &min_h, &nat_h);
		gtk_widget_get_preferred_width(row, &min_w, &nat_w);

		if (nat_h <= 0) nat_h = (min_h > 0) ? min_h : 24;
		if (nat_w <= 0) nat_w = (min_w > 0) ? min_w : 100;

		total_h += nat_h;
		if (nat_w > max_w) max_w = nat_w;
	}
	g_list_free(children);

	/* Add small padding so rows are not clipped */
	total_h += 6;
	max_w += 16; /* horizontal padding */

	/* Determine final width/height to request */
	int width = (tcd->popup_width > 0) ? tcd->popup_width : max_w;
	const int default_cap = 320;
	int cap = (tcd->popup_height > 0) ? tcd->popup_height : default_cap;
	int height = (total_h > cap) ? cap : total_h;

	if (width <= 0) width = 280;

	gtk_widget_set_size_request(sw, width, height);
}

/* Lazily create the popover and listbox and wire handlers */
static void
ensure_popover(TouchComboData *tcd)
{
	if (!tcd || tcd->popover) return;

	tcd->popover = gtk_popover_new(GTK_WIDGET(tcd->button));
	gtk_popover_set_position(GTK_POPOVER(tcd->popover), GTK_POS_BOTTOM);
	gtk_popover_set_modal(GTK_POPOVER(tcd->popover), TRUE);

	GtkWidget *sw = gtk_scrolled_window_new(NULL, NULL);
	/* initial sensible size; will be adjusted after items are added */
	int w = (tcd->popup_width > 0) ? tcd->popup_width : 280;
	int h = (tcd->popup_height > 0) ? tcd->popup_height : 220;
	gtk_widget_set_size_request(sw, w, h);
	gtk_container_add(GTK_CONTAINER(tcd->popover), sw);

	/* cache scrolled window pointer for later sizing */
	tcd->scrolled = sw;

	tcd->listbox = gtk_list_box_new();
	gtk_container_add(GTK_CONTAINER(sw), tcd->listbox);

	/* enable pointer events so we can track dragging */
	gtk_widget_add_events(tcd->listbox, GDK_POINTER_MOTION_MASK | GDK_BUTTON_RELEASE_MASK);

	/* Connect handlers */
	g_signal_connect(tcd->listbox, "motion-notify-event", G_CALLBACK(list_motion_cb), tcd->root);
	g_signal_connect(tcd->listbox, "button-release-event", G_CALLBACK(list_button_release_cb), tcd->root);
	g_signal_connect(tcd->listbox, "row-activated", G_CALLBACK(list_row_activated_cb), tcd->root);

	/* keep popover hidden until needed */
	gtk_widget_show_all(tcd->popover);
	gtk_popover_popdown(GTK_POPOVER(tcd->popover));
}

/* Show popover on button click; adjust size before popping */
static void
touch_combo_button_clicked_cb(GtkButton *b, gpointer user_data)
{
	(void)b;
	GtkWidget *root_widget = GTK_WIDGET(user_data);
	TouchComboData *tcd_inner = tcd_from_widget(root_widget);
	if (!tcd_inner) return;
	ensure_popover(tcd_inner);
	/* Recompute popup size based on current rows */
	adjust_popover_size(tcd_inner);
	gtk_widget_show_all(tcd_inner->popover);
	gtk_popover_popup(GTK_POPOVER(tcd_inner->popover));

	// highlight the previously-existing selection
	GtkListBoxRow *firstRow = gtk_list_box_get_row_at_index(GTK_LIST_BOX(tcd_inner->listbox), 0);
	GtkListBoxRow *row = gtk_list_box_get_row_at_index(GTK_LIST_BOX(tcd_inner->listbox), tcd_inner->active);
	if (firstRow)
		gtk_widget_set_state_flags(GTK_WIDGET(firstRow), GTK_STATE_FLAG_NORMAL, TRUE);
	tcd_inner->hover_row = row;
	if (row)
		gtk_widget_set_state_flags(GTK_WIDGET(row), GTK_STATE_FLAG_SELECTED, TRUE);
}

/* Public API --------------------------------------------------------------- */

GtkWidget *
touch_combo_new(void)
{
	TouchComboData *tcd = g_new0(TouchComboData, 1);
	tcd->active = -1;
	tcd->popup_width = 0;
	tcd->popup_height = 0;
	tcd->changed_cb = NULL;
	tcd->changed_user_data = NULL;
	tcd->items = NULL;
	tcd->hover_row = NULL;
	tcd->scrolled = NULL;
	tcd->popover = NULL;
	tcd->listbox = NULL;

	/* root widget is a simple box */
	GtkWidget *root = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 4);
	tcd->root = root;

	/* button visible control */
	tcd->button = gtk_button_new();
	gtk_box_pack_start(GTK_BOX(root), tcd->button, FALSE, FALSE, 0);

	/* inner layout for label + arrow */
	GtkWidget *inner = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
	tcd->label = gtk_label_new("");
	gtk_label_set_xalign(GTK_LABEL(tcd->label), 0.0);
	gtk_box_pack_start(GTK_BOX(inner), tcd->label, TRUE, TRUE, 2);
	tcd->arrow = gtk_image_new_from_icon_name("pan-down-symbolic", GTK_ICON_SIZE_MENU);
	gtk_box_pack_end(GTK_BOX(inner), tcd->arrow, FALSE, FALSE, 2);
	gtk_container_add(GTK_CONTAINER(tcd->button), inner);

	/* attach our data to root with destructor */
	g_object_set_data_full(G_OBJECT(root), "touch-combo-data", tcd, touch_combo_data_free);

	/* connect button clicked */
	g_signal_connect(tcd->button, "clicked", G_CALLBACK(touch_combo_button_clicked_cb), root);

	return root;
}

void
touch_combo_append_text(GtkWidget *touch_combo, const char *text)
{
	if (!touch_combo || !text) return;
	TouchComboData *tcd = tcd_from_widget(touch_combo);
	if (!tcd) return;

	/* store a copy of the string */
	gchar *dup = g_strdup(text);
	tcd->items = g_list_append(tcd->items, dup);
	int index = g_list_length(tcd->items) - 1;

	/* ensure popover/listbox exist */
	ensure_popover(tcd);
	if (!tcd->listbox) return;

	/* create a row */
	GtkWidget *row = gtk_list_box_row_new();
	GtkWidget *lbl = gtk_label_new(text);
	gtk_label_set_xalign(GTK_LABEL(lbl), 0.0);
	gtk_container_add(GTK_CONTAINER(row), lbl);
	g_object_set_data(G_OBJECT(row), "index", GINT_TO_POINTER(index + 1));
	gtk_list_box_insert(GTK_LIST_BOX(tcd->listbox), row, -1);
	gtk_widget_show_all(row);

	/* adjust popup size to reflect new rows */
	adjust_popover_size(tcd);
}

void
touch_combo_append_texts(GtkWidget *touch_combo, const char * const texts[])
{
	if (!touch_combo || !texts) return;
	for (const char * const *p = texts; *p; ++p)
		touch_combo_append_text(touch_combo, *p);
}

void
touch_combo_clear(GtkWidget *touch_combo)
{
	if (!touch_combo) return;
	TouchComboData *tcd = tcd_from_widget(touch_combo);
	if (!tcd) return;

	/* free internal strings */
	if (tcd->items) {
		g_list_free_full(tcd->items, g_free);
		tcd->items = NULL;
	}
	tcd->active = -1;

	/* clear visible label */
	if (tcd->label)
		gtk_label_set_text(GTK_LABEL(tcd->label), "");

	/* remove rows from listbox if created */
	if (tcd->listbox) {
		GList *children = gtk_container_get_children(GTK_CONTAINER(tcd->listbox));
		for (GList *l = children; l; l = l->next) {
			gtk_widget_destroy(GTK_WIDGET(l->data));
		}
		g_list_free(children);
		/* recompute popup size after clearing rows */
		adjust_popover_size(tcd);
	}
	tcd->hover_row = NULL;
}

int
touch_combo_get_active(GtkWidget *touch_combo)
{
	TouchComboData *tcd = tcd_from_widget(touch_combo);
	if (!tcd) return -1;
	return tcd->active;
}

const char *
touch_combo_get_active_text(GtkWidget *touch_combo)
{
	TouchComboData *tcd = tcd_from_widget(touch_combo);
	if (!tcd) return NULL;
	if (tcd->active < 0) return NULL;
	GList *node = g_list_nth(tcd->items, tcd->active);
	return node ? (const char*)node->data : NULL;
}

void
touch_combo_set_active(GtkWidget *touch_combo, int index)
{
	TouchComboData *tcd = tcd_from_widget(touch_combo);
	if (!tcd) return;

	if (index < 0 || index >= g_list_length(tcd->items)) {
		tcd->active = -1;
		if (tcd->label) gtk_label_set_text(GTK_LABEL(tcd->label), "");
		emit_changed(tcd);
		return;
	}
	tcd->active = index;
	const char *s = (const char*)g_list_nth_data(tcd->items, index);
	if (tcd->label) gtk_label_set_text(GTK_LABEL(tcd->label), s ? s : "");
	emit_changed(tcd);

	/* try to visually reflect the selection inside the listbox */
	if (tcd->listbox) {
		GtkListBoxRow *row = gtk_list_box_get_row_at_index(GTK_LIST_BOX(tcd->listbox), index);
		if (tcd->hover_row && tcd->hover_row != row)
			gtk_widget_set_state_flags(GTK_WIDGET(tcd->hover_row), GTK_STATE_FLAG_NORMAL, TRUE);
		tcd->hover_row = row;
		if (row)
			gtk_widget_set_state_flags(GTK_WIDGET(row), GTK_STATE_FLAG_SELECTED, TRUE);
	}
}

void
touch_combo_set_popup_size(GtkWidget *touch_combo, int width, int height)
{
	TouchComboData *tcd = tcd_from_widget(touch_combo);
	if (!tcd) return;
	if (width > 0) tcd->popup_width = width;
	if (height > 0) tcd->popup_height = height;

	/* if popover already exists, apply new constraints immediately */
	if (tcd->popover && tcd->listbox)
		adjust_popover_size(tcd);
}

void
touch_combo_set_changed_callback(GtkWidget *touch_combo, GCallback cb, gpointer user_data)
{
	TouchComboData *tcd = tcd_from_widget(touch_combo);
	if (!tcd) return;
	tcd->changed_cb = (void (*)(GtkWidget*, gpointer))cb;
	tcd->changed_user_data = user_data;
}
