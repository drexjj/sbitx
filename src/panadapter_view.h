#ifndef PANADAPTER_VIEW_H
#define PANADAPTER_VIEW_H

#include <stdbool.h>
#include <stdint.h>

#define PANADAPTER_VIEW_MAX_ZOOM 500.0

struct panadapter_view {
  double zoom;
  double center;
};

void panadapter_view_reset(struct panadapter_view *view);

bool panadapter_view_is_default(const struct panadapter_view *view);

int panadapter_view_span_hz(const struct panadapter_view *view, int base_span_hz);

int panadapter_view_center_hz(const struct panadapter_view *view, int base_span_hz);

void panadapter_view_zoom_at(struct panadapter_view *view, double factor, double position);

void panadapter_view_pan(struct panadapter_view *view, double visible_fraction);

void panadapter_view_fit(struct panadapter_view *view, double start, double stop);

double panadapter_view_map_position(const struct panadapter_view *old_view, const struct panadapter_view *new_view,
                                    double new_position);

int panadapter_grid_step_hz(int span_hz);

int64_t panadapter_grid_first_hz(int64_t view_start_hz, int step_hz);

int64_t panadapter_grid_label_hz(int64_t frequency_hz, int span_hz, int step_hz);

int panadapter_view_frequency_x(int x, int width, int64_t frequency, int64_t view_start, int span_hz);

#endif
