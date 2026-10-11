#ifndef PANADAPTER_PRESETS_H
#define PANADAPTER_PRESETS_H

#include <stdbool.h>

/*
 * Concrete field values a PANPERF preset expands to. They are kept as the
 * strings the #pan_* fields hold, so the caller can hand them straight to
 * set_field() without a second mapping, and so this table stays the single
 * source of truth for what LOW/MED/HIGH mean.
 */
struct panadapter_preset {
  const char *bins;        // #pan_bins
  const char *hist_secs;   // #pan_hist_secs
  const char *wf_rebuild;  // #pan_wf_rebuild
  const char *web_bins;    // #pan_web_bins
};

/*
 * Resolve a PANPERF value. Returns false for "CUSTOM" and for any unrecognised
 * name, leaving *out untouched -- CUSTOM means "the individual knobs are
 * authoritative", so there is deliberately nothing to apply.
 */
bool panadapter_preset_lookup(const char *preset, struct panadapter_preset *out);

/*
 * Whether `values` is exactly what `preset` expands to. False when preset does
 * not resolve (notably "CUSTOM"), so a caller can use this as "the knobs still
 * agree with the named preset".
 *
 * This is what stops a settings reload from mislabelling a preset as CUSTOM:
 * the four knobs are persisted alongside the preset name and loaded after it,
 * so each one re-fires the field handler with a value the preset itself just
 * wrote. Only a genuine disagreement should clear the name.
 */
bool panadapter_preset_matches(const char *preset,
                               const struct panadapter_preset *values);

#endif
