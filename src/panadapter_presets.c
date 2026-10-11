#include "panadapter_presets.h"

#include <string.h>

bool panadapter_preset_lookup(const char *preset, struct panadapter_preset *out)
{
	static const struct {
		const char *name;
		struct panadapter_preset values;
	} presets[] = {
		// Deep zoom is what makes the FIR expensive, so LOW caps bins hardest,
		// drops waterfall re-analysis, and stops feeding the web context at all.
		{"LOW",  {"2048",  "3",  "OFF", "OFF"}},
		// MED is a bit of a happy medium that works well all-around. Decent zoom, without a huge cost
		{"MED",  {"4096",  "6",  "ON",  "512"}},
		// HIGH is high cost, but you can zoom in to less than 1 Hz per bin and have a large history buffer when moving around
		{"HIGH", {"16384", "11", "ON",  "2048"}},
	};

	if (!preset || !out)
		return false;
	for (unsigned i = 0; i < sizeof(presets) / sizeof(*presets); i++)
		if (!strcmp(preset, presets[i].name)) {
			*out = presets[i].values;
			return true;
		}
	return false;
}

static bool same(const char *a, const char *b)
{
	return a && b && !strcmp(a, b);
}

bool panadapter_preset_matches(const char *preset,
                               const struct panadapter_preset *values)
{
	struct panadapter_preset expected;

	if (!values || !panadapter_preset_lookup(preset, &expected))
		return false;
	// field_str() yields NULL for a label that is not in the layout, so a
	// missing knob counts as "does not match" rather than crashing.
	return same(values->bins, expected.bins)
	       && same(values->hist_secs, expected.hist_secs)
	       && same(values->wf_rebuild, expected.wf_rebuild)
	       && same(values->web_bins, expected.web_bins);
}
