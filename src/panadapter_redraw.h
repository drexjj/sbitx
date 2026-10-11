#ifndef PANADAPTER_REDRAW_H
#define PANADAPTER_REDRAW_H

#include <stdbool.h>

struct panadapter_redraw_state {
	bool spectrum_dirty;
};

void panadapter_redraw_field_updated(struct panadapter_redraw_state *state,
	const char *label);
bool panadapter_redraw_tick(struct panadapter_redraw_state *state);

#endif
