#ifndef WATERFALL_RING_H
#define WATERFALL_RING_H

// Pure index math for the waterfall's ring-buffered row storage. The
// buffers themselves (and the head/storage_height state) live in
// sbitx_gtk.c; this module only computes indices so the logic can be
// unit-tested without a GTK/cairo dependency.

struct waterfall_segment {
	int offset;
	int count;
};

// Returns the head after retreating by one row (the new row becomes
// logical row 0).
int waterfall_ring_advance(int head, int storage_height);

// Maps a logical row (0 = newest) to its physical row in the ring.
int waterfall_ring_physical_row(int head, int storage_height, int logical_row);

// Splits the logical rows [0, count) into up to two physically-contiguous
// segments, in logical order, since the ring wraps at storage_height.
// Returns the number of segments (0-2).
int waterfall_ring_segments(int head, int storage_height, int count,
	struct waterfall_segment out[2]);

#endif
