TARGET = sbitx
SOURCES = $(wildcard src/*.c)
OBJECTS = $(SOURCES:.c=.o)
HEADERS = $(wildcard src/*.h)
CFLAGS = -g `pkg-config --cflags gtk+-3.0`
LIBS = -lwiringPi -lasound -lm -lfftw3 -lfftw3f -pthread -lncurses -lsqlite3 -lnsl -lrt -lssl -lcrypto src/ft8_lib/libft8.a `pkg-config --libs gtk+-3.0`
CC = gcc
LINK = gcc
STRIP = strip
# Define Mongoose SSL flags: ensure OpenSSL is properly enabled
MONGOOSE_FLAGS = -DMG_ENABLE_OPENSSL=1 -DMG_ENABLE_MBEDTLS=0 -DMG_ENABLE_LINES=1 -DMG_TLS=MG_TLS_OPENSSL -DMG_ENABLE_SSI=0 -DMG_ENABLE_IPV6=0

$(TARGET): $(OBJECTS)
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(LIBPATH) $(LIBS)
	sudo setcap CAP_SYS_TIME+ep $(TARGET) # Provide capability to adjust the local system time -W2JON

src/mongoose.o: src/mongoose.c
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $(INCPATH) $(MONGOOSE_FLAGS) -o $@ $<

.c.o: $(HEADERS)
	$(CC) -c $(CFLAGS) $(DEBUGFLAGS) $(INCPATH) -o $@ $<

clean:
	-rm -f $(OBJECTS)
	-rm -f *~ core *.core
	-rm -f $(TARGET)

test:
	echo $(OBJECTS)
