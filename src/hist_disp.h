
#pragma once
#define HD_MARKUP_CHAR '#'

void hd_createGridList();
int hd_decorate(int style, const char * message, const text_span_semantic *sem, int sem_count, char * decorated);
