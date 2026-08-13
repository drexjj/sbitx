TARGET = sbitx
SOURCES = $(wildcard src/*.c)
CLU_SOURCES = clu/src/awards_enum.c clu/src/dxcc.c clu/src/locator.c
ALL_SOURCES = $(SOURCES) $(CLU_SOURCES)
OBJECTS = $(ALL_SOURCES:.c=.o)
FFTOBJ = ft8_lib/.build/fft/kiss_fft.o ft8_lib/.build/fft/kiss_fftr.o
HEADERS = $(wildcard src/*.h)
GTK_CFLAGS := $(shell pkg-config --cflags gtk+-3.0)
GTK_LIBS := $(shell pkg-config --libs gtk+-3.0)
CFLAGS = $(GTK_CFLAGS) -I. -Iclu/src
LIBS = -lwiringPi -lasound -lm -lfftw3 -lfftw3f -pthread -lsqlite3 -lnsl -lrt -lssl -lcrypto -lpcre2-8 ft8_lib/libft8.a $(GTK_LIBS)
ifdef SBITX_DEBUG
CFLAGS += -ggdb3 -fsanitize=address
LIBS += -fsanitize=address -static-libasan
endif
CC = gcc
LINK = gcc
STRIP = strip
# Define Mongoose SSL flags: ensure OpenSSL is properly enabled
MONGOOSE_FLAGS = -DMG_ENABLE_OPENSSL=1 -DMG_ENABLE_MBEDTLS=0 -DMG_ENABLE_LINES=1 -DMG_TLS=MG_TLS_OPENSSL -DMG_ENABLE_SSI=0 -DMG_ENABLE_IPV6=0
TEST_C_SOURCES = $(wildcard tests/test_*.c)
TEST_C_TARGETS = $(patsubst tests/test_%.c,/tmp/sbitx-test_%,$(TEST_C_SOURCES))
TEST_JS_SOURCES = $(wildcard tests/test_*.js)
TEST_LIBS = -lfftw3f -lm -pthread

$(TARGET): $(OBJECTS) ft8_lib/libft8.a
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(FFTOBJ) $(LIBPATH) $(LIBS)
	sudo setcap CAP_SYS_TIME+ep $(TARGET) # Provide capability to adjust the local system time -W2JON

src/mongoose.o: src/mongoose.c
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $(INCPATH) $(MONGOOSE_FLAGS) -o $@ $<

ifndef SBITX_DEBUG
src/panadapter_fft.o: CFLAGS += -O3
endif

.c.o:
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $(INCPATH) -o $@ $<

ft8_lib/libft8.a:
ifdef SBITX_DEBUG
	$(MAKE) FT8_DEBUG=1 -C ft8_lib
else
	$(MAKE) -C ft8_lib
endif

clean:
	-rm -f $(OBJECTS)
	-rm -f *~ core *.core
	-rm -f $(TARGET)

.PHONY: test
test: $(TEST_C_TARGETS)
	@set -e; for test in $(TEST_C_TARGETS); do echo "Running $$test"; $$test; done
	@set -e; for test in $(TEST_JS_SOURCES); do echo "Running $$test"; node $$test; done

/tmp/sbitx-test_%: tests/test_%.c src/%.c
	$(CC) -O2 -Isrc -o $@ $^ $(TEST_LIBS)
