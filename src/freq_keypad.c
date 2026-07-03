/*
 * freq_keypad.c — Integrated GTK frequency-entry keypad for sBitx.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gtk/gtk.h>
#include <cairo.h>

#include "freq_keypad.h"

#define MAX_FIELD_LENGTH 128

#define FIELD_NUMBER    0
#define FIELD_BUTTON    1
#define FIELD_TOGGLE    2
#define FIELD_SELECTION 3
#define FIELD_TEXT      4
#define FIELD_STATIC    5
#define FIELD_CONSOLE   6
#define FIELD_DROPDOWN  7

#define FIELD_DRAW  0
#define FIELD_EDIT  2

struct field {
    char *cmd;
    int (*fn)(struct field *f, cairo_t *gfx, int event,
              int param_a, int param_b, int param_c);
    int x, y, width, height;
    char label[30];
    int  label_width;
    char value[MAX_FIELD_LENGTH];
    char value_type;
    int  font_index;
    char selection[1000];
    long int min, max;
    int  step;
    int  section;
    char is_dirty;
    char update_remote;
    int  dropdown_columns;
    void *data;
};

extern int           screen_width, screen_height;
extern struct field *get_field(const char *cmd);
extern int           set_field(const char *id, const char *value);
extern void          update_field(struct field *f);
extern void          invalidate_rect(int x, int y, int width, int height);

#define KEYPAD_POSITION_FILE "/home/pi/sbitx/data/keypad_position.txt"

static GtkWidget *s_keypad_window = NULL;  /* NULL when keypad is closed */

typedef struct {
    GtkWidget *window;
    GtkWidget *display;    /* GtkLabel showing digits being typed */
    char       digits[32];
} KeypadData;

typedef struct {
    gdouble  ox, oy;
    gboolean active;
} KpDrag;

static void keypad_save_position(GtkWidget *window)
{
    gint x, y;
    gtk_window_get_position(GTK_WINDOW(window), &x, &y);
    FILE *fp = fopen(KEYPAD_POSITION_FILE, "w");
    if (fp) { fprintf(fp, "%d,%d", x, y); fclose(fp); }
}

static void keypad_load_position(GtkWidget *window)
{
    FILE *fp = fopen(KEYPAD_POSITION_FILE, "r");
    if (!fp) return;
    gint x = 0, y = 0;
    if (fscanf(fp, "%d,%d", &x, &y) == 2)
        gtk_window_move(GTK_WINDOW(window), x, y);
    fclose(fp);
}

static void keypad_tune_and_close(KeypadData *kd, const char *khz_str)
{
    if (!khz_str || khz_str[0] == '\0') return;
    long khz = atol(khz_str);
    if (khz <= 0) return;

    /* r1:freq is stored in Hz */
    char hz_buf[32];
    snprintf(hz_buf, sizeof(hz_buf), "%ld", khz * 1000L);
    set_field("r1:freq", hz_buf);

    struct field *ff = get_field("r1:freq");
    if (ff) update_field(ff);
    invalidate_rect(0, 0, screen_width, screen_height);

    keypad_save_position(kd->window);
    gtk_widget_destroy(kd->window);   /* fires on_keypad_destroy -> frees kd */
}

static void on_keypad_destroy(GtkWidget *w, gpointer user_data)
{
    (void)w;
    g_free((KeypadData *)user_data);
    s_keypad_window = NULL;
}

/* Digit keys, backspace ("<-"), and Enter */
static void on_kp_key_clicked(GtkButton *btn, gpointer user_data)
{
    KeypadData *kd  = (KeypadData *)user_data;
    const char *lbl = gtk_button_get_label(btn);

    if (g_strcmp0(lbl, "Enter") == 0) {
        keypad_tune_and_close(kd, kd->digits);
        return;
    }
    if (g_strcmp0(lbl, "<-") == 0) {
        size_t len = strlen(kd->digits);
        if (len > 0) kd->digits[len - 1] = '\0';
    } else {
        size_t len = strlen(kd->digits);
        if (len < sizeof(kd->digits) - 1) {
            kd->digits[len]     = lbl[0];
            kd->digits[len + 1] = '\0';
        }
    }
    gtk_label_set_text(GTK_LABEL(kd->display),
                       kd->digits[0] ? kd->digits : "Enter kHz");
}

/* Band shortcut buttons — kHz value stored as GObject data */
static void on_kp_band_clicked(GtkButton *btn, gpointer user_data)
{
    KeypadData *kd  = (KeypadData *)user_data;
    const char *khz = (const char *)g_object_get_data(G_OBJECT(btn), "khz");
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "-> %s kHz", khz);
    gtk_label_set_text(GTK_LABEL(kd->display), tmp);
    keypad_tune_and_close(kd, khz);
}

/* Close button */
static void on_kp_close_clicked(GtkButton *btn, gpointer user_data)
{
    (void)btn;
    KeypadData *kd = (KeypadData *)user_data;
    keypad_save_position(kd->window);
    gtk_widget_destroy(kd->window);
}

/* Drag-to-move */
static gboolean on_kp_drag_press(GtkWidget *w, GdkEventButton *ev, gpointer ud)
{
    (void)w;
    KpDrag *d = (KpDrag *)ud;
    if (ev->button == 1) { d->active = TRUE; d->ox = ev->x; d->oy = ev->y; }
    return FALSE;
}

static gboolean on_kp_drag_release(GtkWidget *w, GdkEventButton *ev, gpointer ud)
{
    KpDrag     *d  = (KpDrag *)ud;
    KeypadData *kd = (KeypadData *)g_object_get_data(G_OBJECT(w), "kp_data");
    if (ev->button == 1) {
        d->active = FALSE;
        if (kd) keypad_save_position(kd->window);
    }
    return FALSE;
}

static gboolean on_kp_drag_motion(GtkWidget *w, GdkEventMotion *ev, gpointer ud)
{
    KpDrag *d = (KpDrag *)ud;
    if (!d->active) return FALSE;
    gint wx, wy;
    gtk_window_get_position(GTK_WINDOW(w), &wx, &wy);
    gtk_window_move(GTK_WINDOW(w),
                    wx + (gint)(ev->x - d->ox),
                    wy + (gint)(ev->y - d->oy));
    return FALSE;
}

/* -------------------------------------------------------------------------
 * CSS — black/green/teal theme
 * ------------------------------------------------------------------------- */
static void keypad_apply_css(GtkWidget *window)
{
    static const char *css =
        "window#keypad_win  { background-color: #000000; }\n"
        "#kp_display        {\n"
        "  color: #00ff00; font-size: 18px; font-weight: bold;\n"
        "  background-color: #001111; padding: 8px;\n"
        "  border-radius: 6px; margin: 4px 8px; }\n"
        "#kp_band           {\n"
        "  background-color: #003333; color: #00ff00;\n"
        "  font-size: 13px; font-weight: bold;\n"
        "  min-width: 52px; min-height: 28px;\n"
        "  border-radius: 5px; padding: 0; }\n"
        "#kp_band:hover     { background-color: #00ff00; color: #000000; }\n"
        "#kp_digit          {\n"
        "  background-color: #006666; color: #ffffff;\n"
        "  font-size: 16px; font-weight: bold;\n"
        "  min-width: 56px; min-height: 36px; border-radius: 5px; }\n"
        "#kp_digit:hover    { background-color: #009999; }\n"
        "#kp_enter          { background-color: #008800; }\n"
        "#kp_enter:hover    { background-color: #00bb00; }\n"
        "#kp_back           { background-color: #aa5500; }\n"
        "#kp_back:hover     { background-color: #cc7700; }\n"
        "#kp_close          {\n"
        "  background-color: #cc0000; color: #ffffff;\n"
        "  font-size: 13px; min-width: 32px; border-radius: 5px; }\n"
        "#kp_close:hover    { background-color: #ff4444; }\n";

    GtkCssProvider *prov = gtk_css_provider_new();
    gtk_css_provider_load_from_data(prov, css, -1, NULL);
    gtk_style_context_add_provider_for_screen(
        gtk_widget_get_screen(window),
        GTK_STYLE_PROVIDER(prov),
        GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);
    g_object_unref(prov);
    gtk_widget_set_name(window, "keypad_win");
}

void open_freq_keypad(void)
{
    /* Singleton: raise existing window instead of opening a second one */
    if (s_keypad_window) {
        gtk_window_present(GTK_WINDOW(s_keypad_window));
        return;
    }

    KeypadData *kd  = g_new0(KeypadData, 1);
    GtkWidget  *win = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    kd->window      = win;
    s_keypad_window = win;

    gtk_window_set_title(GTK_WINDOW(win), "Freq Keypad");
    gtk_window_set_decorated(GTK_WINDOW(win), FALSE);
    gtk_window_set_resizable(GTK_WINDOW(win), FALSE);
    gtk_window_set_keep_above(GTK_WINDOW(win), TRUE);
    gtk_container_set_border_width(GTK_CONTAINER(win), 6);
    gtk_window_set_default_size(GTK_WINDOW(win), 300, 300);

    keypad_apply_css(win);
    keypad_load_position(win);

    g_signal_connect(win, "destroy", G_CALLBACK(on_keypad_destroy), kd);

    /* Drag support */
    KpDrag *drag = g_new0(KpDrag, 1);
    g_object_set_data_full(G_OBJECT(win), "kp_drag", drag, g_free);
    g_object_set_data(G_OBJECT(win), "kp_data", kd);
    gtk_widget_add_events(win,
        GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK | GDK_POINTER_MOTION_MASK);
    g_signal_connect(win, "button-press-event",   G_CALLBACK(on_kp_drag_press),   drag);
    g_signal_connect(win, "button-release-event", G_CALLBACK(on_kp_drag_release), drag);
    g_signal_connect(win, "motion-notify-event",  G_CALLBACK(on_kp_drag_motion),  drag);

    /* ---- Layout: [band column] | [close + display + digit grid] ---- */
    GtkWidget *hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_container_add(GTK_CONTAINER(win), hbox);

    /* Band shortcut column */
    static const struct { const char *label; const char *khz; } bands[] = {
        { "BC",    "1100"  }, { "WWV2",  "2500"  }, { "WWV5",  "5000"  },
        { "WWV10", "10000" }, { "WWV15", "15000" }, { "WWV20", "20000" },
        { "49M",   "6000"  }, { "31M",   "9700"  }, { "25M",   "11900" },
    };
    GtkWidget *band_col = gtk_box_new(GTK_ORIENTATION_VERTICAL, 3);
    gtk_box_pack_start(GTK_BOX(hbox), band_col, FALSE, FALSE, 0);
    for (int i = 0; i < (int)(sizeof(bands) / sizeof(bands[0])); i++) {
        GtkWidget *b = gtk_button_new_with_label(bands[i].label);
        gtk_widget_set_name(b, "kp_band");
        g_object_set_data(G_OBJECT(b), "khz", (gpointer)bands[i].khz);
        g_signal_connect(b, "clicked", G_CALLBACK(on_kp_band_clicked), kd);
        gtk_box_pack_start(GTK_BOX(band_col), b, TRUE, TRUE, 0);
    }

    /* Right column */
    GtkWidget *right = gtk_box_new(GTK_ORIENTATION_VERTICAL, 5);
    gtk_box_pack_start(GTK_BOX(hbox), right, TRUE, TRUE, 0);

    /* Close button row */
    GtkWidget *top_row   = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    GtkWidget *close_btn = gtk_button_new_with_label("X");
    gtk_widget_set_name(close_btn, "kp_close");
    g_signal_connect(close_btn, "clicked", G_CALLBACK(on_kp_close_clicked), kd);
    gtk_box_pack_end(GTK_BOX(top_row), close_btn, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(right), top_row, FALSE, FALSE, 0);

    /* Frequency display label */
    kd->display = gtk_label_new("Enter kHz");
    gtk_widget_set_name(kd->display, "kp_display");
    gtk_label_set_xalign(GTK_LABEL(kd->display), 0.5f);
    gtk_box_pack_start(GTK_BOX(right), kd->display, FALSE, FALSE, 0);

    /* Digit grid  1-2-3 / 4-5-6 / 7-8-9 / <-  0  Enter */
    GtkWidget *grid = gtk_grid_new();
    gtk_grid_set_row_homogeneous(GTK_GRID(grid), TRUE);
    gtk_grid_set_column_homogeneous(GTK_GRID(grid), TRUE);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 4);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 4);
    gtk_box_pack_start(GTK_BOX(right), grid, TRUE, TRUE, 0);

    static const char *keys[4][3] = {
        { "1", "2", "3"     },
        { "4", "5", "6"     },
        { "7", "8", "9"     },
        { "<-","0", "Enter" },
    };
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 3; c++) {
            GtkWidget *b = gtk_button_new_with_label(keys[r][c]);
            if      (g_strcmp0(keys[r][c], "Enter") == 0) gtk_widget_set_name(b, "kp_enter");
            else if (g_strcmp0(keys[r][c], "<-")    == 0) gtk_widget_set_name(b, "kp_back");
            else                                           gtk_widget_set_name(b, "kp_digit");
            g_signal_connect(b, "clicked", G_CALLBACK(on_kp_key_clicked), kd);
            gtk_grid_attach(GTK_GRID(grid), b, c, r, 1, 1);
        }
    }

    gtk_widget_show_all(win);
}


