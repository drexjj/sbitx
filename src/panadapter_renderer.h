#ifndef PANADAPTER_RENDERER_H
#define PANADAPTER_RENDERER_H

#include <stdbool.h>
#include <stdint.h>

typedef struct _cairo cairo_t;

enum panadapter_control {
  PANADAPTER_ZOOM_OUT,
  PANADAPTER_ZOOM_IN,
  PANADAPTER_LEFT,
  PANADAPTER_RIGHT,
  PANADAPTER_FILTER,
  PANADAPTER_FULL,
  PANADAPTER_CONTROL_COUNT,
};

void panadapter_renderer_draw_grid(cairo_t *gfx, int x, int y, int width, int grid_height, int64_t view_start,
                                   int span_hz);

void panadapter_renderer_draw_labels(cairo_t *gfx, int x, int y, int width, int grid_height, int64_t view_start,
                                     int span_hz);

void panadapter_renderer_control_rect(int x, int y, int height, int control, int *control_x, int *control_y, int *size);

void panadapter_renderer_draw_controls(cairo_t *gfx, int x, int y, int height, int span_hz, unsigned enabled);

void panadapter_renderer_draw_spectrum(cairo_t *gfx, int x, int y, int width, int height, int grid_height,
                                       const int *bins, const int *averaged_bins, int count, int waterfall_offset,
                                       float scope_gain, bool auto_scope, float baseline, int *waterfall);

void panadapter_renderer_waterfall_pixel(float value, float min_db, float max_db, float offset, bool auto_scope,
                                         uint8_t *pixel);

#endif
