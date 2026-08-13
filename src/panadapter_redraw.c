#include "panadapter_redraw.h"

#include <string.h>

static void mark_spectrum(struct panadapter_redraw_state *state)
{
	if (state)
		state->spectrum_dirty = true;
}

bool panadapter_redraw_tick(struct panadapter_redraw_state *state)
{
	if (!state)
		return false;
	const bool dirty = state->spectrum_dirty;
	state->spectrum_dirty = false;
	return dirty;
}

static bool field_affects_spectrum(const char *label)
{
	return label && (!strcmp(label, "PITCH") ||
		!strcmp(label, "FTX_RX_PITCH") ||
		!strcmp(label, "TX_PITCH"));
}

void panadapter_redraw_field_updated(struct panadapter_redraw_state *state,
	const char *label)
{
	if (field_affects_spectrum(label))
		mark_spectrum(state);
}
