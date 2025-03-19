
#pragma once
#define HD_MARKUP_CHAR '#'

void hd_createGridList();
int hd_decorate(int style, const char * message, char * decorated);
void hd_strip_decoration(char * ft8_message, char * decorated);
int hd_length_no_decoration(char * decorated);
