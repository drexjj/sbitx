#ifndef STYLE_CONFIG_H
#define STYLE_CONFIG_H

#include <stdint.h>

// Maximum number of colors and fonts that can be configured
#define MAX_COLORS 32
#define MAX_FONTS 32
#define MAX_FONT_NAME 32
#define MAX_COLOR_NAME 32

#define COLOR_SELECTED_TEXT 0
#define COLOR_TEXT 1
#define COLOR_TEXT_MUTED 2
#define COLOR_SELECTED_BOX 3
#define COLOR_BACKGROUND 4
#define COLOR_FREQ 5
#define COLOR_LABEL 6
#define SPECTRUM_BACKGROUND 7
#define SPECTRUM_GRID 8
#define SPECTRUM_PLOT 9
#define SPECTRUM_NEEDLE 10
#define COLOR_CONTROL_BOX 11
#define SPECTRUM_BANDWIDTH 12
#define COLOR_RX_PITCH 13
#define SELECTED_LINE 14
#define COLOR_FIELD_SELECTED 15
#define COLOR_TX_PITCH 16
#define COLOR_TOGGLE_ACTIVE 17
#define WATERFALL_LOW 18
#define WATERFALL_MID 19
#define WATERFALL_HIGH 20

// We use a look-up table to define the fonts used.
// Each field indexes into this table.
struct font_style {
    int index;
    float r, g, b;
    char name[32];
    int height;
    int weight;
    int type;
};

extern float palette[][3];
extern struct font_style font_table[];

// Structure to hold a color definition
typedef struct {
    char name[MAX_COLOR_NAME];
    float r, g, b;
    int index; // Maps to the original palette index
} ColorConfig;

// Structure to hold a font style definition
typedef struct {
    char name[MAX_FONT_NAME];
    char font_family[MAX_FONT_NAME];
    int size;
    float r, g, b;
    int weight;  // CAIRO_FONT_WEIGHT_NORMAL or CAIRO_FONT_WEIGHT_BOLD
    int slant;   // CAIRO_FONT_SLANT_NORMAL or CAIRO_FONT_SLANT_ITALIC
    int index;   // Maps to the original font_table index
} FontConfig;

// Structure to hold all style configuration
typedef struct {
    ColorConfig colors[MAX_COLORS];
    int color_count;
    FontConfig fonts[MAX_FONTS];
    int font_count;
    char ui_font[MAX_FONT_NAME];
    int field_font_size;
} StyleConfig;

// Function prototypes
int load_style_config(const char *filename, StyleConfig *config);
void apply_style_config(StyleConfig *config);
int save_default_style_config(const char *filename);
int parse_color_value(const char *value, float *r, float *g, float *b);

#endif // STYLE_CONFIG_H
