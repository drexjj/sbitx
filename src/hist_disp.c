#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "sdr_ui.h"
#include "logbook.h"
#include "hist_disp.h"

bool isLetter(char c) {
    return c >= 'A' && c <= 'Z';
}

bool isDigit(char c) {
    return c >= '0' && c <= '9';
}

bool isValidGridId(char* gridId) {
	return strlen(gridId) == 4 &&
		isLetter(gridId[0]) && isLetter(gridId[1]) &&
        isDigit(gridId[2]) && isDigit(gridId[3]);
}

static FILE* onfFout;

void addGridToFile(char * gridId, int cnt) {
    if (isValidGridId(gridId)) {
		if (onfFout != NULL) {
				fwrite(gridId,1,4,onfFout);
        }
    }
}

void hd_createGridList() {
	onfFout = fopen("./web/grids.txt", "w");

	logbook_open();
	logbook_get_grids(addGridToFile);

	if (onfFout != NULL) {
		fwrite("\0\0", 1, 2, onfFout);
		fclose(onfFout);
	}
}

char ff_char(int style) {

	/* used to be 'A' + style, where style came from these:
	#define FONT_FIELD_LABEL 0
	#define FONT_FIELD_VALUE 1
	#define FONT_LARGE_FIELD 2
	#define FONT_LARGE_VALUE 3
	#define FONT_SMALL 4
	#define FONT_LOG 5
	#define FONT_FT8_RX 6
	#define FONT_FT8_TX 7
	#define FONT_SMALL_FIELD_VALUE 8
	#define FONT_CW_RX 9
	#define FONT_CW_TX 10
	#define FONT_FLDIGI_RX 11
	#define FONT_FLDIGI_TX 12
	#define FONT_TELNET 13
	#define FONT_FT8_QUEUED 14
	#define FONT_FT8_REPLY 15

	#define FF_MYCALL 16
	#define FF_CALLER 17
	#define FF_GRID 18
	#define FONT_BLACK 19
	*/

	switch (style) {
		// console styles
		case STYLE_LOG:
		case STYLE_RST:
			return 'A' + 5;
		case STYLE_MYCALL:
			return 'A' + 16;
		case STYLE_CALLER:
			return 'A' + 17;
		case STYLE_CALLEE:
		case STYLE_RECENT_CALLER:
		case STYLE_EXISTING_GRID:
			return 'A' + 5;
		case STYLE_GRID:
			return 'A' + 18;
		case STYLE_TIME:
		case STYLE_FREQ:
		case STYLE_FT8_RX:
			return 'A' + 6;
		case STYLE_SNR:
		case STYLE_FT8_TX:
			return 'A' + 7;
		case STYLE_FT8_QUEUED:
			return 'A' + 14;
		case STYLE_FT8_REPLY:
			return 'A' + 15;
		case STYLE_CW_RX:
			return 'A' + 9;
		case STYLE_CW_TX:
			return 'A' + 10;
		case STYLE_FLDIGI_RX:
			return 'A' + 11;
		case STYLE_FLDIGI_TX:
			return 'A' + 12;
		case STYLE_TELNET:
			return 'A' + 13;

		// field styles
		case STYLE_FIELD_LABEL:
			return 'A' + 0;
		case STYLE_FIELD_VALUE:
			return 'A' + 1;
		case STYLE_LARGE_FIELD:
			return 'A' + 2;
		case STYLE_LARGE_VALUE:
			return 'A' + 3;
		case STYLE_SMALL:
			return 'A' + 4;
		case STYLE_SMALL_FIELD_VALUE:
			return 'A' + 8;
		case STYLE_BLACK:
			return 'A' + 19;
		default:
			printf("warning: unhandled style %d treated as \"log\"\n", style);
			return 'A' + 5;
	}
}

int hd_decorate(int style, const char * message, const text_span_semantic *sem, int sem_count, char * decorated) {

	if (!sem_count) {
		strcpy(decorated, message);
		return 0;
	}

	// similar algorithm as in draw_console() in sbitx_gtk.c
	char *d = decorated;
	char default_style = ff_char(STYLE_LOG);
	char cur_style = default_style;
	int span = 0;
	int col = 0;
	// The first span may be a fallback. If the second span is valid and overlaps it, start with that one.
	if (sem_count >= 2 && sem[1].start_column == 0 && sem[1].length) {
		span = 1;
		default_style = ff_char(sem[0].semantic);
		//~ printf("-  first span had length %d; starting with span 1: col %d len %d: '%s'\n",
				//~ sem[0].length, sem[1].start_column, sem[1].length, message);
	}
	for (; span < MAX_CONSOLE_LINE_STYLES && span < sem_count && sem[span].length; ++span) {
		//~ printf("-  span %d col %d len %d style %d @ col %d\n",
			//~ span, sem[span].start_column, sem[span].length, sem[span].semantic, col);
		if (sem[span].start_column > col) {
			// output the default-styled text to the left of this span
			const int len = sem[span].start_column - col;
			if (len > 1 && cur_style != default_style) {
				*d++ = '#'; *d++ = default_style; cur_style = default_style;
			}
			d = stpncpy(d, message + col, len);
			col += len;
			//~ printf("   nabbed text to left of %d,  len %d; end @ col %d\n",
				//~ sem[span].start_column, len, col);
		}
		char style = ff_char(sem[span].semantic);
		if (cur_style != style) {
			cur_style = style; *d++ = '#'; *d++ = style;
		}
		d = stpncpy(d, message + sem[span].start_column, sem[span].length);
		//~ printf("   output span %d col %d len %d style %d, output offset %d\n",
			//~ span, sem[span].start_column, sem[span].length, sem[span].semantic, d - decorated);
		col += sem[span].length;
	}
	if (message[col] && message[col] != '\n') {
		// draw the default-styled text to the right of the last span
		int remainder = strlen(message + col);
		if (remainder > 1 && cur_style != default_style) {
			*d++ = '#'; *d++ = default_style;
		}
		d = stpncpy(d, message + col, remainder);
		//~ printf("   nabbed text to right of %d,  len %d\n", col, remainder);
		col += remainder;
	}
	*d = 0;

	//~ printf("hd_decorate ends with '%s' len %d %d\n", decorated, col, d - decorated);
	return d - decorated;
}
