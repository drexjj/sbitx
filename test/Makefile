CFLAGS = `pkg-config --cflags gtk+-3.0` -I../src -ggdb3 -fsanitize=address
LIBS = -lm -lsqlite3 -lpcre2-8 `pkg-config --libs gtk+-3.0`
LIBS += -fsanitize=address -static-libasan
CC = gcc
LINK = gcc

ftxrules: ../src/ftx_rules_ui.c ../src/ftx_rules.c ../src/touch_combo.c ftxrules_main.c
	gcc -o ftxrules $(CFLAGS) $(LFLAGS) $(LIBS) $^
