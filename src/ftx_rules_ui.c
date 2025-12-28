// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2025 Shawn Rutledge K7IHZ / LB2JK <s@ecloud.org>

#include <gtk/gtk.h>
#include <stdbool.h>
#include <stdio.h>
#include "ftx_rules.h"

// List store and tree view columns
enum {
	COL_ID = 0,
	COL_DESC,
	COL_FIELD,
	COL_MIN,
	COL_MAX,
	COL_REGEX,
	COL_CQ_ADJ,
	COL_ANS_ADJ,
	NUM_COLS
};

static GtkListStore *store = NULL;
static GtkWidget *dialog = NULL;
static GtkWidget *grid = NULL;
static GtkWidget *tree = NULL;
static GtkWidget *lbl_desc = NULL;
static GtkWidget *entry_desc = NULL;
static GtkWidget *lbl_field = NULL;
static GtkWidget *combo_field = NULL;
static GtkWidget *lbl_regex = NULL;
static GtkWidget *entry_regex = NULL;
static GtkWidget *lbl_min = NULL;
static GtkAdjustment *adj_min = NULL;
static GtkWidget *spin_min = NULL;
static GtkWidget *lbl_max = NULL;
static GtkAdjustment *adj_max = NULL;
static GtkWidget *spin_max = NULL;
static GtkAdjustment *adj_cq = NULL;
static GtkWidget *lbl_cq = NULL;
static GtkWidget *spin_cq = NULL;
static GtkAdjustment *adj_ans = NULL;
static GtkWidget *lbl_ans = NULL;
static GtkWidget *spin_ans = NULL;
static GtkWidget *hbox_buttons = NULL;
static GtkWidget *btn_add = NULL;
static GtkWidget *btn_update = NULL;
static GtkWidget *btn_delete = NULL;
static GtkWidget *btn_close = NULL;
static bool programmatic_priority_change = false;

static GtkTreeViewColumn* add_text_column(GtkWidget *tree, const char *title, int col, int min_width)
{
	GtkCellRenderer *renderer = gtk_cell_renderer_text_new();
	GtkTreeViewColumn *column = gtk_tree_view_column_new_with_attributes(title, renderer,
		"text", col,
		NULL);
	if (min_width > 0)
		gtk_tree_view_column_set_min_width(column, min_width);
	gtk_tree_view_append_column(GTK_TREE_VIEW(tree), column);
	return column;
}

static gboolean is_numeric_field(int field)
{
	return (field == RULE_FIELD_SNR ||
		field == RULE_FIELD_DISTANCE ||
		field == RULE_FIELD_AZIMUTH);
}

// show/hide regex vs numeric widgets based on field
static void update_field_visibility(int field)
{
	if (is_numeric_field(field)) {
		gtk_widget_set_visible(lbl_regex, FALSE);
		gtk_widget_set_visible(entry_regex, FALSE);
		gtk_widget_set_visible(lbl_min, TRUE);
		gtk_widget_set_visible(spin_min, TRUE);
		gtk_widget_set_visible(lbl_max, TRUE);
		gtk_widget_set_visible(spin_max, TRUE);
	} else {
		gtk_widget_set_visible(lbl_regex, TRUE);
		gtk_widget_set_visible(entry_regex, TRUE);
		gtk_widget_set_visible(lbl_min, FALSE);
		gtk_widget_set_visible(spin_min, FALSE);
		gtk_widget_set_visible(lbl_max, FALSE);
		gtk_widget_set_visible(spin_max, FALSE);
	}
}

/*!
 	If a rule has both priorities set to 0, it is not applied at runtime
    because it has no effect. Grey it out to indicate that.
*/
static void cell_data_func_ignore(GtkTreeViewColumn *col, GtkCellRenderer *renderer,
	GtkTreeModel *model, GtkTreeIter *iter, gpointer data)
{
	int cq_adj, ans_adj;
	gtk_tree_model_get(model, iter,
		COL_CQ_ADJ, &cq_adj,
		COL_ANS_ADJ, &ans_adj,
		-1);
	const bool ignored = (cq_adj == 0 && ans_adj == 0);
	g_object_set(renderer, "foreground", ignored ? "#808080" : NULL, NULL);
}

static void on_selection_changed(GtkTreeSelection *sel, gpointer user_data)
{
	GtkTreeIter iter;
	GtkTreeModel *model;
	if (gtk_tree_selection_get_selected(sel, &model, &iter)) {
		int id, field, cq_adj, ans_adj, minv, maxv;
		char *desc = NULL;
		char *regex = NULL;
		gtk_tree_model_get(model, &iter,
			COL_ID, &id,
			COL_DESC, &desc,
			COL_CQ_ADJ, &cq_adj,
			COL_ANS_ADJ, &ans_adj,
			COL_FIELD, &field,
			COL_MIN, &minv,
			COL_MAX, &maxv,
			COL_REGEX, &regex,
			-1);
		// populate widgets
		gtk_entry_set_text(GTK_ENTRY(entry_desc), desc ? desc : "");
		programmatic_priority_change = true;
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_cq), cq_adj);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_ans), ans_adj);
		programmatic_priority_change = false;
		gtk_combo_box_set_active(GTK_COMBO_BOX(combo_field), field - 1);
		update_field_visibility(field);
		if (is_numeric_field(field)) {
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_min), minv);
			gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_max), maxv);
		} else {
			gtk_entry_set_text(GTK_ENTRY(entry_regex), regex ? regex : "");
		}
		g_free(desc);
		g_free(regex);
		// store selected id on dialog for handlers
		g_object_set_data(G_OBJECT(dialog), "selected-id", GINT_TO_POINTER(id));
	} else {
		// no selection
		gtk_entry_set_text(GTK_ENTRY(entry_desc), "");
		gtk_entry_set_text(GTK_ENTRY(entry_regex), "");
		programmatic_priority_change = true;
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_cq), 0);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_ans), 0);
		programmatic_priority_change = false;
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_min), 0);
		gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_max), 0);
		g_object_set_data(G_OBJECT(dialog), "selected-id", GINT_TO_POINTER(-1));
	}
}

// Refresh the list store from the database
static void refresh_rules_list()
{
	gtk_list_store_clear(store);
	void *q = ftx_rule_prepare_query_all();
	if (!q) return;

	ftx_rule r;
	char desc_buf[256];
	char regex_buf[512];
	while (ftx_next_rule(q, &r, desc_buf, sizeof(desc_buf), regex_buf, sizeof(regex_buf))) {
		GtkTreeIter iter;
		gtk_list_store_append(store, &iter);
		gtk_list_store_set(store, &iter,
			COL_ID, r.id,
			COL_DESC, desc_buf,
			COL_CQ_ADJ, r.cq_resp_pri_adj,
			COL_ANS_ADJ, r.ans_pri_adj,
			COL_FIELD, r.field,
			COL_MIN, r.min_value,
			COL_MAX, r.max_value,
			COL_REGEX, regex_buf,
			-1);
	}
	ftx_rule_end_query(q);
}

static void on_field_changed(GtkComboBox *cb, gpointer user_data)
{
	const ftx_rules_field field = gtk_combo_box_get_active(GTK_COMBO_BOX(cb)) + 1;
	// printf("cb chose field %d: numeric? %d\n", field, is_numeric_field(field));
	update_field_visibility(field);
}

static void on_add_clicked(GtkButton *btn, gpointer user_data)
{
	gtk_entry_set_text(GTK_ENTRY(entry_desc), "");
	programmatic_priority_change = true;
	gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_cq), 0);
	gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_ans), 0);
	programmatic_priority_change = false;
	gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_min), 0);
	gtk_spin_button_set_value(GTK_SPIN_BUTTON(spin_max), 0);
	gtk_entry_set_text(GTK_ENTRY(entry_regex), "");
	GtkTreeSelection *sel = gtk_tree_view_get_selection(GTK_TREE_VIEW(tree));
	gtk_tree_selection_unselect_all(sel);
	g_object_set_data(G_OBJECT(dialog), "selected-id", GINT_TO_POINTER(-1));
}

static void on_priority_changed(GtkSpinButton *spin, gpointer user_data)
{
	if (programmatic_priority_change)
		return;
	int sel_id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(dialog), "selected-id"));
	if (sel_id < 0)
		return;
	int8_t cq_pri = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_cq));
	int8_t ans_pri = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_ans));
	// printf("rule %d: priorities changed to %d %d\n", sel_id, cq_pri, ans_pri);
	ftx_rule_update_priorities(sel_id, cq_pri, ans_pri);

	/* Iterate the list store to find the row with matching ID and update its priorities */
	GtkTreeIter iter;
	gboolean valid;
	gint id;
	valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(store), &iter);
	while (valid) {
		gtk_tree_model_get(GTK_TREE_MODEL(store), &iter, COL_ID, &id, -1);
		if (id == sel_id) {
			gtk_list_store_set(store, &iter,
				COL_CQ_ADJ, cq_pri,
				COL_ANS_ADJ, ans_pri,
				-1);
			break;
		}
		valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(store), &iter);
	}
}

/*!
	To update a rule, delete it and add a new one.
	This can result in it having a different ID:
	it's likely to be a low-valued one.
*/
static void on_update_clicked(GtkButton *btn, gpointer user_data)
{
	int sel_id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(dialog), "selected-id"));
	if (sel_id > 0) {
		// Find the corresponding row in the list store and compare values.
		GtkTreeIter iter;
		gboolean valid = gtk_tree_model_get_iter_first(GTK_TREE_MODEL(store), &iter);
		while (valid) {
			gint id;
			gtk_tree_model_get(GTK_TREE_MODEL(store), &iter, COL_ID, &id, -1);
			if (id == sel_id) {
				char *model_desc = NULL;
				char *model_regex = NULL;
				int model_field = 0;
				int model_cq = 0, model_ans = 0;
				int model_min = 0, model_max = 0;
				gtk_tree_model_get(GTK_TREE_MODEL(store), &iter,
					COL_DESC, &model_desc,
					COL_FIELD, &model_field,
					COL_CQ_ADJ, &model_cq,
					COL_ANS_ADJ, &model_ans,
					COL_MIN, &model_min,
					COL_MAX, &model_max,
					COL_REGEX, &model_regex,
					-1);

				const char *ui_desc = gtk_entry_get_text(GTK_ENTRY(entry_desc));
				int ui_field = gtk_combo_box_get_active(GTK_COMBO_BOX(combo_field)) + 1;
				int ui_cq = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_cq));
				int ui_ans = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_ans));
				bool unchanged = false;

				if (is_numeric_field(ui_field)) {
					int ui_min = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_min));
					int ui_max = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_max));
					unchanged = (g_strcmp0(model_desc, ui_desc) == 0 &&
																		model_field == ui_field &&
																		model_cq == ui_cq &&
																		model_ans == ui_ans &&
																		model_min == ui_min &&
																		model_max == ui_max);
				} else {
					const char *ui_regex = gtk_entry_get_text(GTK_ENTRY(entry_regex));
					unchanged = (g_strcmp0(model_desc, ui_desc) == 0 &&
																		model_field == ui_field &&
																		model_cq == ui_cq &&
																		model_ans == ui_ans &&
																		g_strcmp0(model_regex, ui_regex) == 0);
				}

				g_free(model_desc);
				g_free(model_regex);

				if (unchanged) {
					// Nothing changed, nothing to do.
					return;
				}
				break;
			}
			valid = gtk_tree_model_iter_next(GTK_TREE_MODEL(store), &iter);
		}
	}

	if (sel_id > 0)
		ftx_delete_rule((int8_t)sel_id);

	const char *desc = gtk_entry_get_text(GTK_ENTRY(entry_desc));
	ftx_rules_field field = gtk_combo_box_get_active(GTK_COMBO_BOX(combo_field)) + 1;
	int cq_adj = (int)gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_cq));
	int ans_adj = (int)gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_ans));
	if (is_numeric_field(field)) {
		int minv = (int)gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_min));
		int maxv = (int)gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin_max));
		ftx_add_numeric_rule(desc, field, minv, maxv, (int8_t)cq_adj, (int8_t)ans_adj);
	} else {
		const char *regex = gtk_entry_get_text(GTK_ENTRY(entry_regex));
		ftx_add_regex_rule(desc, field, regex, (int8_t)cq_adj, (int8_t)ans_adj);
	}
	refresh_rules_list();
}

static void on_delete_clicked(GtkButton *btn, gpointer user_data)
{
	int sel_id = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(dialog), "selected-id"));
	if (sel_id <= 0) return;
	ftx_delete_rule((int8_t)sel_id);
	// clear selected id
	g_object_set_data(G_OBJECT(dialog), "selected-id", GINT_TO_POINTER(-1));
	refresh_rules_list();
}

/*!
	Opens up a window for editing FT8/FT4 CQRESP/ANS rules.

	Builds a dialog window with a tree view and edit controls for rules.
	This implementation populates the table using the ftx_rules API and
	allows adding new rules, updating the priority adjustments for an
	existing rule, and deleting a rule.
*/
GtkWidget *ftx_rules_ui(GtkWidget* parentWindow)
{
	dialog = gtk_window_new(GTK_WINDOW_TOPLEVEL);
	if (parentWindow)
		gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(parentWindow));
	gtk_window_set_title(GTK_WINDOW(dialog), "FT8/FT4 auto-responder priority rules");
	gtk_window_set_default_size(GTK_WINDOW(dialog), 640, 400);
	gtk_container_set_border_width(GTK_CONTAINER(dialog), 8);
	GtkWidget *vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
	gtk_container_add(GTK_CONTAINER(dialog), vbox);

	store = gtk_list_store_new(NUM_COLS,
		G_TYPE_INT,    // id
		G_TYPE_STRING, // desc
		G_TYPE_INT,    // field
		G_TYPE_INT,    // min
		G_TYPE_INT,    // max
		G_TYPE_STRING, // regex
		G_TYPE_INT,    // cq adj
		G_TYPE_INT     // ans adj
	);

	// Scrolled tree view
	GtkWidget *scrolled = gtk_scrolled_window_new(NULL, NULL);
	gtk_widget_set_vexpand(scrolled, TRUE);
	gtk_widget_set_hexpand(scrolled, TRUE);
	gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled),
		GTK_POLICY_AUTOMATIC, GTK_POLICY_AUTOMATIC);
	gtk_box_pack_start(GTK_BOX(vbox), scrolled, TRUE, TRUE, 0);
	tree = gtk_tree_view_new_with_model(GTK_TREE_MODEL(store));
	gtk_tree_view_set_headers_visible(GTK_TREE_VIEW(tree), TRUE);
	gtk_container_add(GTK_CONTAINER(scrolled), tree);
	add_text_column(tree, "Description", COL_DESC, 300);
	add_text_column(tree, "CQ Resp", COL_CQ_ADJ, 80);
	add_text_column(tree, "Answer", COL_ANS_ADJ, 80);

	// Replace the simple text renderers with cell data funcs so we can style ignored rows
	for (int ci = 0; ci < 3; ++ci) {
		GtkTreeViewColumn *column = gtk_tree_view_get_column(GTK_TREE_VIEW(tree), ci);
		GList *renderers = gtk_cell_layout_get_cells(GTK_CELL_LAYOUT(column));
		if (renderers && renderers->data) {
			GtkCellRenderer *rend = GTK_CELL_RENDERER(renderers->data);
			// remove existing attributes and use our cell data func
			// gtk_cell_layout_clear_attributes(GTK_CELL_LAYOUT(column));
			gtk_tree_view_column_set_cell_data_func(column, rend, cell_data_func_ignore, GINT_TO_POINTER(ci), NULL);
		}
		g_list_free(renderers);
	}

	// Editing widgets area
	grid = gtk_grid_new();
	gtk_grid_set_row_spacing(GTK_GRID(grid), 3);
	gtk_grid_set_column_spacing(GTK_GRID(grid), 6);
	gtk_box_pack_start(GTK_BOX(vbox), grid, FALSE, FALSE, 0);

	// Description entry
	lbl_desc = gtk_label_new("Description");
	gtk_widget_set_halign(lbl_desc, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), lbl_desc, 0, 0, 1, 1);
	entry_desc = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_desc), 256);
	gtk_grid_attach(GTK_GRID(grid), entry_desc, 1, 0, 3, 1);

	// Add button
	btn_add = gtk_button_new_with_label("+");
	gtk_grid_attach(GTK_GRID(grid), btn_add, 4, 0, 1, 1);

	// Field combobox
	lbl_field = gtk_label_new("Field");
	gtk_widget_set_halign(lbl_field, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), lbl_field, 0, 1, 1, 1);
	combo_field = gtk_combo_box_text_new();
	// populate with names from ftx_rule_field_name()
	for (int f = RULE_FIELD_CALLSIGN; f < RULE_FIELD_COUNT; ++f)
			gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(combo_field),
				NULL, ftx_rule_field_name((ftx_rules_field)f));
	gtk_grid_attach(GTK_GRID(grid), combo_field, 1, 1, 1, 1);

	// Regex entry (shown only for regex rules)
	lbl_regex = gtk_label_new("Regular Expression");
	gtk_widget_set_halign(lbl_regex, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), lbl_regex, 2, 1, 1, 1);
	entry_regex = gtk_entry_new();
	gtk_entry_set_max_length(GTK_ENTRY(entry_regex), 512);
	gtk_grid_attach(GTK_GRID(grid), entry_regex, 3, 1, 1, 1);

	// Numeric min/max (shown only for numeric rules)
	lbl_min = gtk_label_new("Min Value");
	gtk_widget_set_halign(lbl_min, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), lbl_min, 0, 2, 1, 1);
	adj_min = gtk_adjustment_new(0, -32768, 32767, 1, 10, 0);
	spin_min = gtk_spin_button_new(adj_min, 1, 0);
	gtk_grid_attach(GTK_GRID(grid), spin_min, 1, 2, 1, 1);

	lbl_max = gtk_label_new("Max Value");
	gtk_widget_set_halign(lbl_max, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), lbl_max, 2, 2, 1, 1);
	adj_max = gtk_adjustment_new(0, -32768, 32767, 1, 10, 0);
	spin_max = gtk_spin_button_new(adj_max, 1, 0);
	gtk_grid_attach(GTK_GRID(grid), spin_max, 3, 2, 1, 1);

	// Priority spinboxes
	adj_cq = gtk_adjustment_new(0, -128, 127, 1, 10, 0);
	lbl_cq = gtk_label_new("CQ response priority");
	gtk_widget_set_halign(lbl_cq, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), lbl_cq, 0, 3, 1, 1);
	spin_cq = gtk_spin_button_new(adj_cq, 1, 0);
	gtk_spin_button_set_range(GTK_SPIN_BUTTON(spin_cq), -127, 127);
	gtk_grid_attach(GTK_GRID(grid), spin_cq, 1, 3, 1, 1);

	adj_ans = gtk_adjustment_new(0, -128, 127, 1, 10, 0);
	lbl_ans = gtk_label_new("Answer priority");
	gtk_widget_set_halign(lbl_ans, GTK_ALIGN_END);
	gtk_grid_attach(GTK_GRID(grid), lbl_ans, 2, 3, 1, 1);
	spin_ans = gtk_spin_button_new(adj_ans, 1, 0);
	gtk_spin_button_set_range(GTK_SPIN_BUTTON(spin_ans), -127, 127);
	gtk_grid_attach(GTK_GRID(grid), spin_ans, 3, 3, 1, 1);

	// Buttons row
	hbox_buttons = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
	gtk_box_pack_start(GTK_BOX(vbox), hbox_buttons, FALSE, FALSE, 0);
	btn_update = gtk_button_new_with_label("Save Changes");
	btn_delete = gtk_button_new_with_label("Delete Rule");
	btn_close = gtk_button_new_with_label("Close");
	gtk_box_pack_end(GTK_BOX(hbox_buttons), btn_close, FALSE, FALSE, 0);
	gtk_box_pack_end(GTK_BOX(hbox_buttons), btn_delete, FALSE, FALSE, 0);
	gtk_box_pack_end(GTK_BOX(hbox_buttons), btn_update, FALSE, FALSE, 0);

	// Wire up signals
	GtkTreeSelection *sel = gtk_tree_view_get_selection(GTK_TREE_VIEW(tree));
	gtk_tree_selection_set_mode(sel, GTK_SELECTION_SINGLE);
	g_signal_connect(sel, "changed", G_CALLBACK(on_selection_changed), NULL);
	g_signal_connect(combo_field, "changed", G_CALLBACK(on_field_changed), NULL);
	g_signal_connect(btn_add, "clicked", G_CALLBACK(on_add_clicked), NULL);
	g_signal_connect(btn_update, "clicked", G_CALLBACK(on_update_clicked), NULL);
	g_signal_connect(btn_delete, "clicked", G_CALLBACK(on_delete_clicked), NULL);
	g_signal_connect_swapped(btn_close, "clicked", G_CALLBACK(gtk_widget_destroy), dialog);
	g_signal_connect(spin_cq, "value-changed", G_CALLBACK(on_priority_changed), NULL);
	g_signal_connect(spin_ans, "value-changed", G_CALLBACK(on_priority_changed), NULL);

	// Initial state
	g_object_set_data(G_OBJECT(dialog), "selected-id", GINT_TO_POINTER(-1));
	refresh_rules_list();

	gtk_widget_show_all(dialog);
	return dialog;
}
