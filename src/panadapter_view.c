#include "panadapter_view.h"

#include <math.h>

static void clamp_view(struct panadapter_view *view) {
  if (view->zoom < 1.0) view->zoom = 1.0;
  if (view->zoom > PANADAPTER_VIEW_MAX_ZOOM) view->zoom = PANADAPTER_VIEW_MAX_ZOOM;

  const double limit = 0.5 - 0.5 / view->zoom;
  if (view->center < -limit) view->center = -limit;
  if (view->center > limit) view->center = limit;
}

void panadapter_view_reset(struct panadapter_view *view) {
  view->zoom = 1.0;
  view->center = 0.0;
}

bool panadapter_view_is_default(const struct panadapter_view *view) {
  // Pan and zoom use floating point, so treat values very close to reset as default.
  return fabs(view->zoom - 1.0) < 1.0e-6 && fabs(view->center) < 1.0e-6;
}

int panadapter_view_span_hz(const struct panadapter_view *view, int base_span_hz) {
  const int span = (int) lround(base_span_hz / view->zoom);
  return span > 0 ? span : 1;
}

int panadapter_view_center_hz(const struct panadapter_view *view, int base_span_hz) {
  return (int) lround(view->center * base_span_hz);
}

void panadapter_view_zoom_at(struct panadapter_view *view, double factor, double position) {
  const double old_zoom = view->zoom;
  view->zoom *= factor;
  clamp_view(view);
  view->center += (position - 0.5) * (1.0 / old_zoom - 1.0 / view->zoom);
  clamp_view(view);
}

void panadapter_view_pan(struct panadapter_view *view, double visible_fraction) {
  view->center += visible_fraction / view->zoom;
  clamp_view(view);
}

void panadapter_view_fit(struct panadapter_view *view, double start, double stop) {
  if (stop <= start)
    return;
  view->zoom = 1.0 / (stop - start);
  view->center = (start + stop) / 2.0 - 0.5;
  clamp_view(view);
}

// Find where a point in the new view came from in the old view. This lets the
// existing waterfall image be resampled immediately while new history renders.
double panadapter_view_map_position(const struct panadapter_view *old_view,
                                    const struct panadapter_view *new_view,
                                    double new_position) {
  const double frequency = new_view->center + (new_position - 0.5) / new_view->zoom;
  return 0.5 + (frequency - old_view->center) * old_view->zoom;
}

// Choose a readable 1/2/2.5/5/10 Hz-decade interval near 8-10 grid divisions.
// If that would produce more than 12 ticks, promote the interval so adjacent
// labels do not get too close together to distinguish and read easily.
int panadapter_grid_step_hz(int span_hz) {
  if (span_hz < 1)
    return 1;
  const double target = span_hz / (span_hz >= 10000 ? 8.0 : 10.0);
  const double magnitude = pow(10.0, floor(log10(target)));
  const double normalized = target / magnitude;
  double nice;
  if (normalized < 1.5)
    nice = 1.0;
  else if (normalized < 2.25)
    nice = 2.0;
  else if (normalized < 3.75)
    nice = 2.5;
  else if (normalized < 7.5)
    nice = 5.0;
  else
    nice = 10.0;
  int step = (int) lround(nice * magnitude);
  if ((int64_t) step * 12 < span_hz) {
    if (nice < 2.0)
      nice = 2.0;
    else if (nice < 2.5)
      nice = 2.5;
    else if (nice < 5.0)
      nice = 5.0;
    else
      nice = 10.0;
    step = (int) lround(nice * magnitude);
  }
  return step;
}

// Return the first absolute step boundary at or above the view's left edge.
int64_t panadapter_grid_first_hz(int64_t view_start_hz, int step_hz) {
  if (step_hz < 1)
    return view_start_hz;
  int64_t remainder = view_start_hz % step_hz;
  if (remainder < 0)
    remainder += step_hz;
  return remainder ? view_start_hz + step_hz - remainder : view_start_hz;
}

// Snap a tick to the exact frequency shown by its rounded label so the grid
// line cannot drift away from that label or a matching pitch marker.
int64_t panadapter_grid_label_hz(int64_t frequency_hz, int span_hz, int step_hz) {
  int resolution;
  if (span_hz >= 10000)
    resolution = 1000;
  else if (step_hz >= 100)
    resolution = 100;
  else if (step_hz >= 10)
    resolution = 10;
  else
    resolution = 1;
  return (int64_t) llround((double) frequency_hz / resolution) * resolution;
}

int panadapter_view_frequency_x(int x, int width, int64_t frequency, int64_t view_start, int span_hz) {
  int64_t offset = frequency - view_start;
  if (offset < 0) offset = 0;
  if (offset > span_hz) offset = span_hz;
  return x + (int) (offset * width / span_hz);
}
