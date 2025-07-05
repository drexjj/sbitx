TARGET = sbitx
SOURCES = $(wildcard src/*.c)
OBJECTS = $(SOURCES:.c=.o)
FFTOBJ = ft8_lib/.build/fft/kiss_fft.o ft8_lib/.build/fft/kiss_fftr.o
HEADERS = $(wildcard src/*.h)
CFLAGS = -g `pkg-config --cflags gtk+-3.0` -I.
LIBS = -lwiringPi -lasound -lm -lfftw3 -lfftw3f -pthread -lncurses -lsqlite3 -lnsl -lrt -lssl -lcrypto src/ft8_lib/libft8.a `pkg-config --libs gtk+-3.0`
CC = gcc
LINK = gcc
STRIP = strip
# Define Mongoose SSL flags: ensure OpenSSL is properly enabled
MONGOOSE_FLAGS = -DMG_ENABLE_OPENSSL=1 -DMG_ENABLE_MBEDTLS=0 -DMG_ENABLE_LINES=1 -DMG_TLS=MG_TLS_OPENSSL -DMG_ENABLE_SSI=0 -DMG_ENABLE_IPV6=0

$(TARGET): $(OBJECTS) ft8_lib/libft8.a
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(FFTOBJ) $(LIBPATH) $(LIBS)

src/mongoose.o: src/mongoose.c
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $(INCPATH) $(MONGOOSE_FLAGS) -o $@ $<

.c.o: $(HEADERS)
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $(INCPATH) -o $@ $<

ft8_lib/libft8.a:
	$(MAKE) -C ft8_lib

clean:
	-rm -f $(OBJECTS)
	-rm -f *~ core *.core
	-rm -f $(TARGET)

test:
	echo $(OBJECTS)
