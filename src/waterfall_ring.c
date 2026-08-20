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
