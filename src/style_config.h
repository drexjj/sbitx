#ifndef STYLE_CONFIG_H
#define STYLE_CONFIG_H

#include <stdint.h>

// Maximum number of colors and fonts that can be configured
#define MAX_COLORS 32
#define MAX_FONTS 32
#define MAX_FONT_NAME 32
#define MAX_COLOR_NAME 32

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
