TARGET = sbitx
SOURCES = $(wildcard src/*.c)
CLU_SOURCES = clu/src/awards_enum.c clu/src/dxcc.c clu/src/locator.c
ALL_SOURCES = $(SOURCES) $(CLU_SOURCES)
OBJECTS = $(ALL_SOURCES:.c=.o)
FFTOBJ = ft8_lib/.build/fft/kiss_fft.o ft8_lib/.build/fft/kiss_fftr.o
HEADERS = $(wildcard src/*.h)
CFLAGS = `pkg-config --cflags gtk+-3.0` -I. -Iclu/src
LIBS = -lwiringPi -lasound -lm -lfftw3 -lfftw3f -pthread -lsqlite3 -lnsl -lrt -lssl -lcrypto -lpcre2-8 ft8_lib/libft8.a `pkg-config --libs gtk+-3.0`
ifdef SBITX_DEBUG
CFLAGS += -ggdb3 -fsanitize=address
LIBS += -fsanitize=address -static-libasan
endif
CC = gcc
LINK = gcc
STRIP = strip
# Define Mongoose SSL flags: ensure OpenSSL is properly enabled
MONGOOSE_FLAGS = -DMG_ENABLE_OPENSSL=1 -DMG_ENABLE_MBEDTLS=0 -DMG_ENABLE_LINES=1 -DMG_TLS=MG_TLS_OPENSSL -DMG_ENABLE_SSI=0 -DMG_ENABLE_IPV6=0

$(TARGET): $(OBJECTS) ft8_lib/libft8.a
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(FFTOBJ) $(LIBPATH) $(LIBS)
	sudo setcap CAP_SYS_TIME+ep $(TARGET) # Provide capability to adjust the local system time -W2JON

src/mongoose.o: src/mongoose.c
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $(INCPATH) $(MONGOOSE_FLAGS) -o $@ $<

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

test:
	echo $(ALL_SOURCES)
	echo $(OBJECTS)
