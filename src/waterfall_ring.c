#include "waterfall_ring.h"

int waterfall_ring_advance(int head, int storage_height)
{
	if (storage_height <= 0)
		return 0;
	return (head - 1 + storage_height) % storage_height;
}

int waterfall_ring_physical_row(int head, int storage_height, int logical_row)
{
	if (storage_height <= 0)
		return 0;
	return (head + logical_row) % storage_height;
}

int waterfall_ring_segments(int head, int storage_height, int count,
	struct waterfall_segment out[2])
{
	if (count <= 0 || storage_height <= 0)
		return 0;
	// The ring cannot yield more rows than it holds. Callers pass the widget's
	// current height, which can briefly exceed storage_height when a layout
	// change lands before the next resize_waterfall() -- without this clamp the
	// second segment would run off the end of the buffer.
	if (count > storage_height)
		count = storage_height;
	const int start = head % storage_height;
	const int before_wrap = storage_height - start;
	if (count <= before_wrap) {
		out[0] = (struct waterfall_segment){ start, count };
		return 1;
	}
	out[0] = (struct waterfall_segment){ start, before_wrap };
	out[1] = (struct waterfall_segment){ 0, count - before_wrap };
	return 2;
}
