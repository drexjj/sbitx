#include "style_config.h"
#include "ini.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <cairo.h>

// Forward declaration of font_style structure (from sbitx_gtk.c)
struct font_style
{
    int index;
    float r, g, b;
    char name[32];
    int height;
    int weight;
    int type;
};

// External references to the original palette and font_table
extern float palette[][3];
extern struct font_style font_table[];
extern char *ui_font;
extern int field_font_size;

// Context structure for INI parsing
typedef struct {
    StyleConfig *config;
    char current_section[64];
    int current_color_index;
    int current_font_index;
    ColorConfig *current_color;
    FontConfig *current_font;
} ParserContext;

// Helper function to parse color values
// Supports formats: "1.0, 0.5, 0.0" or "255, 128, 0" or "#FF8000"
int parse_color_value(const char *value, float *r, float *g, float *b) {
    char cleaned[256];
    strncpy(cleaned, value, sizeof(cleaned) - 1);
    cleaned[sizeof(cleaned) - 1] = '\0';
    
    // Remove whitespace
    char *dst = cleaned;
    for (const char *src = value; *src; src++) {
        if (!isspace(*src)) {
            *dst++ = *src;
        }
    }
    *dst = '\0';
    
    // Check for hex format (#RRGGBB)
    if (cleaned[0] == '#' && strlen(cleaned) == 7) {
        unsigned int hex;
        if (sscanf(cleaned + 1, "%x", &hex) == 1) {
            *r = ((hex >> 16) & 0xFF) / 255.0f;
            *g = ((hex >> 8) & 0xFF) / 255.0f;
            *b = (hex & 0xFF) / 255.0f;
            return 1;
        }
    }
    
    // Try float format (0.0-1.0)
    float fr, fg, fb;
    if (sscanf(cleaned, "%f,%f,%f", &fr, &fg, &fb) == 3) {
        if (fr <= 1.0f && fg <= 1.0f && fb <= 1.0f) {
            *r = fr;
            *g = fg;
            *b = fb;
            return 1;
        }
        // Try integer format (0-255)
        if (fr <= 255.0f && fg <= 255.0f && fb <= 255.0f) {
            *r = fr / 255.0f;
            *g = fg / 255.0f;
            *b = fb / 255.0f;
            return 1;
        }
    }
    
    return 0; // Parse failed
}

// Helper to parse font weight
static int parse_font_weight(const char *value) {
    if (!strcasecmp(value, "bold")) {
        return CAIRO_FONT_WEIGHT_BOLD;
    }
    return CAIRO_FONT_WEIGHT_NORMAL;
}

// Helper to parse font slant
static int parse_font_slant(const char *value) {
    if (!strcasecmp(value, "italic")) {
        return CAIRO_FONT_SLANT_ITALIC;
    } else if (!strcasecmp(value, "oblique")) {
        return CAIRO_FONT_SLANT_OBLIQUE;
    }
    return CAIRO_FONT_SLANT_NORMAL;
}

// INI parser callback
static int style_config_handler(void *user, const char *section, 
                                const char *name, const char *value) {
    ParserContext *ctx = (ParserContext *)user;
    StyleConfig *config = ctx->config;
    
    // Check if we're starting a new section
    if (strcmp(ctx->current_section, section) != 0) {
        strncpy(ctx->current_section, section, sizeof(ctx->current_section) - 1);
        ctx->current_section[sizeof(ctx->current_section) - 1] = '\0';
        
        // Setup context for new section
        if (!strncmp(section, "color:", 6)) {
            if (config->color_count >= MAX_COLORS) {
                fprintf(stderr, "Warning: Too many colors defined, ignoring %s\n", section);
                return 1;
            }
            ctx->current_color = &config->colors[config->color_count];
            ctx->current_color_index = config->color_count;
            strncpy(ctx->current_color->name, section + 6, MAX_COLOR_NAME - 1);
            ctx->current_color->name[MAX_COLOR_NAME - 1] = '\0';
            ctx->current_color->index = -1; // Mark as unset
            config->color_count++;
        }
        else if (!strncmp(section, "font:", 5)) {
            if (config->font_count >= MAX_FONTS) {
                fprintf(stderr, "Warning: Too many fonts defined, ignoring %s\n", section);
                return 1;
            }
            ctx->current_font = &config->fonts[config->font_count];
            ctx->current_font_index = config->font_count;
            strncpy(ctx->current_font->name, section + 5, MAX_FONT_NAME - 1);
            ctx->current_font->name[MAX_FONT_NAME - 1] = '\0';
            ctx->current_font->index = -1; // Mark as unset
            config->font_count++;
        }
    }
    
    // Parse fields based on section type
    if (!strcmp(section, "general")) {
        if (!strcmp(name, "ui_font")) {
            strncpy(config->ui_font, value, MAX_FONT_NAME - 1);
            config->ui_font[MAX_FONT_NAME - 1] = '\0';
        } else if (!strcmp(name, "field_font_size")) {
            config->field_font_size = atoi(value);
        }
    }
    else if (!strncmp(section, "color:", 6)) {
        if (!strcmp(name, "index")) {
            ctx->current_color->index = atoi(value);
        } else if (!strcmp(name, "rgb")) {
            parse_color_value(value, &ctx->current_color->r, 
                            &ctx->current_color->g, &ctx->current_color->b);
        }
    }
    else if (!strncmp(section, "font:", 5)) {
        if (!strcmp(name, "index")) {
            ctx->current_font->index = atoi(value);
        } else if (!strcmp(name, "family")) {
            strncpy(ctx->current_font->font_family, value, MAX_FONT_NAME - 1);
            ctx->current_font->font_family[MAX_FONT_NAME - 1] = '\0';
        } else if (!strcmp(name, "size")) {
            ctx->current_font->size = atoi(value);
        } else if (!strcmp(name, "color")) {
            parse_color_value(value, &ctx->current_font->r, 
                            &ctx->current_font->g, &ctx->current_font->b);
        } else if (!strcmp(name, "weight")) {
            ctx->current_font->weight = parse_font_weight(value);
        } else if (!strcmp(name, "slant")) {
            ctx->current_font->slant = parse_font_slant(value);
        }
    }
    
    return 1;
}

// Load style configuration from file
int load_style_config(const char *filename, StyleConfig *config) {
    memset(config, 0, sizeof(StyleConfig));
    
    // Set defaults
    strncpy(config->ui_font, "Sans", MAX_FONT_NAME - 1);
    config->field_font_size = 12;
    
    // Create parser context
    ParserContext ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.config = config;
    
    if (ini_parse(filename, style_config_handler, &ctx) < 0) {
        fprintf(stderr, "Warning: Could not load style config from %s\n", filename);
        return -1;
    }
    
    printf("Loaded style config: %d colors, %d fonts\n", 
           config->color_count, config->font_count);
    return 0;
}

// Apply the loaded configuration to the running system
void apply_style_config(StyleConfig *config) {
    // Apply general settings
    if (config->ui_font[0]) {
        ui_font = strdup(config->ui_font);
    }
    if (config->field_font_size > 0) {
        field_font_size = config->field_font_size;
    }
    
    // Apply color palette
    for (int i = 0; i < config->color_count; i++) {
        ColorConfig *color = &config->colors[i];
        if (color->index >= 0 && color->index < 18) { // 18 = max palette entries
            palette[color->index][0] = color->r;
            palette[color->index][1] = color->g;
            palette[color->index][2] = color->b;
            printf("Applied color %s (index %d): %.2f, %.2f, %.2f\n",
                   color->name, color->index, color->r, color->g, color->b);
        } else if (color->index == -1) {
            fprintf(stderr, "Warning: Color %s has no index defined\n", color->name);
        }
    }
    
    // Apply font styles
    for (int i = 0; i < config->font_count; i++) {
        FontConfig *font = &config->fonts[i];
        if (font->index >= 0 && font->index < 31) { // 31 = max font_table entries
            if (font->font_family[0]) {
                strncpy(font_table[font->index].name, font->font_family, 
                        sizeof(font_table[font->index].name) - 1);
            }
            if (font->size > 0) {
                font_table[font->index].height = font->size;
            }
            font_table[font->index].r = font->r;
            font_table[font->index].g = font->g;
            font_table[font->index].b = font->b;
            font_table[font->index].weight = font->weight;
            font_table[font->index].type = font->slant;
            
            printf("Applied font %s (index %d): %s, size %d\n",
                   font->name, font->index, font->font_family, font->size);
        } else if (font->index == -1) {
            fprintf(stderr, "Warning: Font %s has no index defined\n", font->name);
        }
    }
}

// Define style names - these should match the names used in your code
static const char *font_style_names[] = {
    "STYLE_LOG",              // 0
    "STYLE_MYCALL",           // 1
    "STYLE_CALLER",           // 2
    "STYLE_RECENT_CALLER",    // 3
    "STYLE_CALLEE",           // 4
    "STYLE_GRID",             // 5
    "STYLE_EXISTING_GRID",    // 6
    "STYLE_RST",              // 7
    "STYLE_TIME",             // 8
    "STYLE_SNR",              // 9
    "STYLE_FREQ",             // 10
    "STYLE_COUNTRY",          // 11
    "STYLE_DISTANCE",         // 12
    "STYLE_AZIMUTH",          // 13
    "STYLE_FT8_RX",           // 14
    "STYLE_FT8_TX",           // 15
    "STYLE_FT8_QUEUED",       // 16
    "STYLE_FT8_REPLY",        // 17
    "STYLE_CW_RX",            // 18
    "STYLE_CW_TX",            // 19
    "STYLE_FLDIGI_RX",        // 20
    "STYLE_FLDIGI_TX",        // 21
    "STYLE_TELNET",           // 22
    "STYLE_HIGHLIGHT",        // 23
    "STYLE_FIELD_LABEL",      // 24
    "STYLE_FIELD_VALUE",      // 25
    "STYLE_LARGE_FIELD",      // 26
    "STYLE_LARGE_VALUE",      // 27
    "STYLE_SMALL",            // 28
    "STYLE_SMALL_FIELD_VALUE",// 29
    "STYLE_BLACK"             // 30
};

// Save current style configuration to file
int save_default_style_config(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f) {
        fprintf(stderr, "Error: Could not create %s\n", filename);
        return -1;
    }
    
    fprintf(f, "; sBitx Style Configuration\n");
    fprintf(f, "; This file controls the colors and fonts used in the UI\n");
    fprintf(f, "; Color formats: RGB floats (0.0-1.0), RGB ints (0-255), or hex (#RRGGBB)\n\n");
    
    fprintf(f, "[general]\n");
    fprintf(f, "ui_font = %s\n", ui_font ? ui_font : "Sans");
    fprintf(f, "field_font_size = %d\n\n", field_font_size);
    
    fprintf(f, "; Color palette definitions\n");
    fprintf(f, "; Format: [color:NAME] then index=N and rgb=r,g,b\n\n");
    
    const char *color_names[] = {
        "COLOR_SELECTED_TEXT", "COLOR_TEXT", "COLOR_TEXT_MUTED", 
        "COLOR_SELECTED_BOX", "COLOR_BACKGROUND", "COLOR_FREQ",
        "COLOR_LABEL", "SPECTRUM_BACKGROUND", "SPECTRUM_GRID",
        "SPECTRUM_PLOT", "SPECTRUM_NEEDLE", "COLOR_CONTROL_BOX",
        "SPECTRUM_BANDWIDTH", "COLOR_RX_PITCH", "SELECTED_LINE",
        "COLOR_FIELD_SELECTED", "COLOR_TX_PITCH", "COLOR_TOGGLE_ACTIVE"
    };
    
    for (int i = 0; i < 18; i++) {
        fprintf(f, "[color:%s]\n", color_names[i]);
        fprintf(f, "index = %d\n", i);
        fprintf(f, "rgb = %.2f, %.2f, %.2f\n\n", 
                palette[i][0], palette[i][1], palette[i][2]);
    }
    
    fprintf(f, "; Font style definitions\n");
    fprintf(f, "; Format: [font:STYLE_NAME] then index=N and other properties\n\n");
    
    // Save all font styles using their style names from the font_style_names array
    for (int i = 0; i < 31; i++) {
        if (font_table[i].index == i) {
            fprintf(f, "[font:%s]\n", font_style_names[i]);
            fprintf(f, "index = %d\n", i);
            fprintf(f, "family = %s\n", font_table[i].name);
            fprintf(f, "size = %d\n", font_table[i].height);
            fprintf(f, "color = %.2f, %.2f, %.2f\n", 
                    font_table[i].r, font_table[i].g, font_table[i].b);
            fprintf(f, "weight = %s\n", 
                    font_table[i].weight == CAIRO_FONT_WEIGHT_BOLD ? "bold" : "normal");
            fprintf(f, "slant = %s\n",
                    font_table[i].type == CAIRO_FONT_SLANT_ITALIC ? "italic" : "normal");
            fprintf(f, "\n");
        }
    }
    
    fclose(f);
    printf("Style configuration saved to %s\n", filename);
    return 0;
}
