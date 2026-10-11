#include "panadapter_renderer.h"

#include "panadapter_view.h"
#include "sdr_ui.h"
#include "style_config.h"

#include <cairo.h>
#include <math.h>
#include <stdio.h>

extern float ui_scale;
extern float scope_alpha_plus;

static void set_color(cairo_t *gfx, int color) {
  cairo_set_source_rgb(gfx, palette[color][0], palette[color][1], palette[color][2]);
}

void panadapter_renderer_draw_grid(cairo_t *gfx, int x, int y, int width,
                                   int grid_height, int64_t view_start,
                                   int span_hz) {
  cairo_set_line_width(gfx, 1);
  set_color(gfx, SPECTRUM_GRID);

  for (int division = 0; division <= 10; division++) {
    const int grid_y = y + grid_height * division / 10;
    cairo_move_to(gfx, x, grid_y);
    cairo_line_to(gfx, x + width, grid_y);
  }

  cairo_move_to(gfx, x, y);
  cairo_line_to(gfx, x, y + grid_height);
  cairo_move_to(gfx, x + width, y);
  cairo_line_to(gfx, x + width, y + grid_height);
  const int grid_step = panadapter_grid_step_hz(span_hz);
  const int64_t view_stop = view_start + span_hz;
  for (int64_t nominal = panadapter_grid_first_hz(view_start - grid_step, grid_step);
       nominal <= view_stop + grid_step; nominal += grid_step) {
    const int64_t frequency = panadapter_grid_label_hz(nominal, span_hz, grid_step);
    if (frequency <= view_start || frequency >= view_stop)
      continue;
    const int grid_x = panadapter_view_frequency_x(x, width, frequency, view_start, span_hz);
    cairo_move_to(gfx, grid_x, y);
    cairo_line_to(gfx, grid_x, y + grid_height);
  }
  cairo_stroke(gfx);
}

void panadapter_renderer_draw_labels(cairo_t *gfx, int x, int y, int width, int grid_height, int64_t view_start,
                                     int span_hz) {
  const struct font_style *font = &font_table[STYLE_SMALL];
  cairo_select_font_face(gfx, font->name, font->type, font->weight);
  cairo_set_font_size(gfx, font->height);
  cairo_set_source_rgb(gfx, font->r, font->g, font->b);

  const int grid_step = panadapter_grid_step_hz(span_hz);
  const int64_t view_stop = view_start + span_hz;
  for (int64_t nominal = panadapter_grid_first_hz(view_start - grid_step, grid_step);
       nominal <= view_stop + grid_step; nominal += grid_step) {
    const int64_t frequency = panadapter_grid_label_hz(nominal, span_hz, grid_step);
    if (frequency <= view_start || frequency >= view_stop)
      continue;

    char text[20];
    if (span_hz >= 10000)
      snprintf(text, sizeof(text), "%ld", (long) frequency / 1000);
    else {
      const double khz = (frequency % 1000000) / 1000.0;
      if (grid_step >= 100)
        snprintf(text, sizeof(text), "%5.1f", khz);
      else if (grid_step >= 10)
        snprintf(text, sizeof(text), "%6.2f", khz);
      else
        snprintf(text, sizeof(text), "%7.3f", khz);
    }

    const int label_x = panadapter_view_frequency_x(x, width, frequency, view_start, span_hz);
    cairo_text_extents_t extents;
    cairo_text_extents(gfx, text, &extents);
    cairo_move_to(gfx, label_x - (int) extents.x_advance / 2, y + grid_height + font->height);
    cairo_show_text(gfx, text);
  }
}

static int scaled(int value) {
  return (int) (ui_scale * value);
}

void panadapter_renderer_control_rect(int x, int y, int height, int control, int *control_x, int *control_y,
                                      int *size) {
  const int available = height - scaled(10);
  *size = scaled(34) < available ? scaled(34) : available;
  *control_x = x + scaled(5) + control * (*size + scaled(4));
  *control_y = y + height - *size - scaled(5);
}

static void draw_control(cairo_t *gfx, int x, int y, int height, int control, bool enabled) {
  int control_x, control_y, size;
  panadapter_renderer_control_rect(x, y, height, control, &control_x, &control_y, &size);
  const double cx = control_x + size / 2.0;
  const double cy = control_y + size / 2.0;

  cairo_save(gfx);
  cairo_set_source_rgba(gfx,
                        palette[COLOR_BACKGROUND][0], palette[COLOR_BACKGROUND][1], palette[COLOR_BACKGROUND][2],
                        0.82);
  cairo_rectangle(gfx, control_x, control_y, size, size);
  cairo_fill_preserve(gfx);
  set_color(gfx, COLOR_CONTROL_BOX);
  cairo_set_line_width(gfx, fmax(1.5, size / 16.0));
  cairo_set_line_cap(gfx, CAIRO_LINE_CAP_ROUND);
  cairo_set_line_join(gfx, CAIRO_LINE_JOIN_ROUND);
  cairo_stroke(gfx);
  if (enabled) {
    const struct font_style *font = &font_table[STYLE_SMALL_FIELD_VALUE];
    cairo_set_source_rgb(gfx, font->r, font->g, font->b);
  } else
    set_color(gfx, COLOR_TEXT_MUTED);

  if (control == PANADAPTER_LEFT || control == PANADAPTER_RIGHT) {
    const double direction = control == PANADAPTER_LEFT ? -1.0 : 1.0;
    const double tip = cx + direction * size * 0.27;
    const double base = cx + direction * size * 0.02;
    cairo_move_to(gfx, cx - direction * size * 0.25, cy);
    cairo_line_to(gfx, tip, cy);
    cairo_stroke(gfx);
    cairo_move_to(gfx, tip, cy);
    cairo_line_to(gfx, base, cy - size * 0.22);
    cairo_line_to(gfx, base, cy + size * 0.22);
    cairo_close_path(gfx);
    cairo_fill(gfx);
  } else if (control == PANADAPTER_ZOOM_OUT || control == PANADAPTER_ZOOM_IN) {
    const double radius = size * 0.20;
    cairo_arc(gfx, cx - size * 0.07, cy - size * 0.07, radius, 0, 2 * M_PI);
    cairo_move_to(gfx, cx + size * 0.08, cy + size * 0.08);
    cairo_line_to(gfx, cx + size * 0.27, cy + size * 0.27);
    cairo_move_to(gfx, cx - size * 0.18, cy - size * 0.07);
    cairo_line_to(gfx, cx + size * 0.04, cy - size * 0.07);
    if (control == PANADAPTER_ZOOM_IN) {
      cairo_move_to(gfx, cx - size * 0.07, cy - size * 0.18);
      cairo_line_to(gfx, cx - size * 0.07, cy + size * 0.04);
    }
    cairo_stroke(gfx);
  } else if (control == PANADAPTER_FILTER) {
    cairo_move_to(gfx, cx - size * 0.30, cy + size * 0.23);
    cairo_curve_to(gfx, cx - size * 0.20, cy + size * 0.23,
                   cx - size * 0.19, cy - size * 0.22,
                   cx - size * 0.08, cy - size * 0.22);
    cairo_line_to(gfx, cx + size * 0.08, cy - size * 0.22);
    cairo_curve_to(gfx, cx + size * 0.19, cy - size * 0.22,
                   cx + size * 0.20, cy + size * 0.23,
                   cx + size * 0.30, cy + size * 0.23);
    cairo_stroke(gfx);
  } else {
    const double inner = size * 0.04;
    const double outer = size * 0.26;
    for (int sx = -1; sx <= 1; sx += 2) {
      for (int sy = -1; sy <= 1; sy += 2) {
        cairo_move_to(gfx, cx + sx * inner, cy + sy * outer);
        cairo_line_to(gfx, cx + sx * outer, cy + sy * outer);
        cairo_line_to(gfx, cx + sx * outer, cy + sy * inner);
      }
    }
    cairo_stroke(gfx);
  }
  cairo_restore(gfx);
}

void panadapter_renderer_draw_controls(cairo_t *gfx, int x, int y, int height, int span_hz, unsigned enabled) {
  for (int control = 0; control < PANADAPTER_CONTROL_COUNT; control++)
    draw_control(gfx, x, y, height, control, enabled & (1u << control));

  int control_x, control_y, size;
  panadapter_renderer_control_rect(x, y, height, PANADAPTER_CONTROL_COUNT - 1, &control_x, &control_y, &size);
  char bandwidth[24];
  if (span_hz >= 1000 && span_hz % 1000 == 0)
    snprintf(bandwidth, sizeof(bandwidth), "%d kHz", span_hz / 1000);
  else if (span_hz >= 1000)
    snprintf(bandwidth, sizeof(bandwidth), "%.1f kHz", span_hz / 1000.0);
  else
    snprintf(bandwidth, sizeof(bandwidth), "%d Hz", span_hz);

  cairo_save(gfx);
  const struct font_style *font = &font_table[STYLE_SMALL_FIELD_VALUE];
  cairo_select_font_face(gfx, font->name, font->type, font->weight);
  cairo_set_font_size(gfx, font->height);
  cairo_text_extents_t extents;
  cairo_text_extents(gfx, bandwidth, &extents);
  const double text_x = control_x + size + scaled(7);
  const double text_y = control_y + (size - extents.height) / 2.0 - extents.y_bearing;
  cairo_set_source_rgba(gfx,
                        palette[COLOR_BACKGROUND][0], palette[COLOR_BACKGROUND][1], palette[COLOR_BACKGROUND][2],
                        0.82);
  cairo_rectangle(gfx, text_x - scaled(4),
                  text_y + extents.y_bearing - scaled(3),
                  extents.width + scaled(8),
                  extents.height + scaled(6));
  cairo_fill(gfx);
  cairo_set_source_rgb(gfx, font->r, font->g, font->b);
  cairo_move_to(gfx, text_x, text_y);
  cairo_show_text(gfx, bandwidth);
  cairo_restore(gfx);
}

void panadapter_renderer_draw_spectrum(cairo_t *gfx, int x, int y, int width,
                                       int height, int grid_height,
                                       const int *bins,
                                       const int *averaged_bins, int count,
                                       int waterfall_offset, float scope_gain,
                                       bool auto_scope, float baseline,
                                       int *waterfall) {
  if (count < 1 || width < 1 || height < 1 || grid_height < 1)
    return;

  const double x_step = (double) width / count;
  cairo_pattern_t *gradient = cairo_pattern_create_linear(0, y + grid_height, 0, y);
  cairo_pattern_add_color_stop_rgba(gradient, 0.0,
                                    palette[WATERFALL_LOW][0], palette[WATERFALL_LOW][1], palette[WATERFALL_LOW][2],
                                    0.5 + scope_alpha_plus);
  cairo_pattern_add_color_stop_rgba(gradient, 0.5,
                                    palette[WATERFALL_MID][0], palette[WATERFALL_MID][1], palette[WATERFALL_MID][2],
                                    0.7 + scope_alpha_plus);
  cairo_pattern_add_color_stop_rgba(gradient, 1.0,
                                    palette[WATERFALL_HIGH][0], palette[WATERFALL_HIGH][1], palette[WATERFALL_HIGH][2],
                                    0.9 + scope_alpha_plus);

  cairo_set_antialias(gfx, CAIRO_ANTIALIAS_FAST);
  cairo_move_to(gfx, x + width, y + grid_height);
  static float baseline_offset;
  double bin_x = 0;
  for (int bin = 0; bin < count; bin++) {
    int waterfall_y = (bins[bin] + waterfall_offset) * height / 80;
    if (waterfall_y < 0) waterfall_y = 0;
    if (waterfall_y > height) waterfall_y = height - 1;

    float value = averaged_bins[bin];
    value += auto_scope ? -baseline_offset : waterfall_offset;
    int spectrum_y = (int) (value * scope_gain * height / 80 + 1);
    if (spectrum_y < 0) spectrum_y = 0;
    if (spectrum_y > grid_height) spectrum_y = grid_height;
    cairo_line_to(gfx, x + width - bin_x - x_step / 2.0, y + grid_height - spectrum_y);

    const int pixel_left = fmax(0, width - bin_x - x_step);
    const int pixel_right = fmin(width - 1, width - 1 - bin_x);
    for (int pixel = pixel_left; waterfall && pixel <= pixel_right; pixel++)
      waterfall[pixel] = waterfall_y * 100 / grid_height;
    bin_x += x_step;
  }
  baseline_offset -= (baseline_offset - baseline) / 5;

  cairo_line_to(gfx, x, y + grid_height);
  cairo_close_path(gfx);
  cairo_set_source(gfx, gradient);
  cairo_fill(gfx);
  cairo_pattern_destroy(gradient);
  set_color(gfx, SPECTRUM_PLOT);
  cairo_stroke(gfx);
}

void panadapter_renderer_waterfall_pixel(float value, float min_db,
                                         float max_db, float offset,
                                         bool auto_scope,
                                         uint8_t *pixel) {
  float normalized = auto_scope
                       ? (value * 2.4f - offset) / (max_db - offset) * 100.0f
                       : (value * 2.4f - min_db) / (max_db - min_db) * 100.0f;
  normalized = fmaxf(0.0f, fminf(100.0f, normalized));

  const int level = (int) normalized;
  float red, green, blue;
  if (level < 34) {
    const float mix = level / 33.0f;
    red = palette[WATERFALL_LOW][0] * mix;
    green = palette[WATERFALL_LOW][1] * mix;
    blue = palette[WATERFALL_LOW][2] * mix;
  } else if (level < 67) {
    const float mix = (level - 33) / 34.0f;
    red = palette[WATERFALL_LOW][0] + (palette[WATERFALL_MID][0] - palette[WATERFALL_LOW][0]) * mix;
    green = palette[WATERFALL_LOW][1] + (palette[WATERFALL_MID][1] - palette[WATERFALL_LOW][1]) * mix;
    blue = palette[WATERFALL_LOW][2] + (palette[WATERFALL_MID][2] - palette[WATERFALL_LOW][2]) * mix;
  } else {
    const float mix = (level - 67) / 33.0f;
    red = palette[WATERFALL_MID][0] + (palette[WATERFALL_HIGH][0] - palette[WATERFALL_MID][0]) * mix;
    green = palette[WATERFALL_MID][1] + (palette[WATERFALL_HIGH][1] - palette[WATERFALL_MID][1]) * mix;
    blue = palette[WATERFALL_MID][2] + (palette[WATERFALL_HIGH][2] - palette[WATERFALL_MID][2]) * mix;
  }
  pixel[0] = red * 255;
  pixel[1] = green * 255;
  pixel[2] = blue * 255;
}
