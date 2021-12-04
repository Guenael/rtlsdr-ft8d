CC = gcc
CFLAGS= -Wall -O3 -ffast-math -std=gnu17
LDFLAGS = -L/usr/lib
LIBS = -lusb-1.0 -lrtlsdr -lpthread -lfftw3f -lcurl -lm

OBJS = rtlsdr_ft8d.o kiss_fft/kiss_fft.o ft8/decode.o ft8/encode.o ft8/crc.o ft8/ldpc.o ft8/unpack.o ft8/text.o ft8/constants.o

TARGETS = rtlsdr_ft8d

.PHONY: all clean

all: $(TARGETS)

%.o: %.c
	${CC} ${CFLAGS} -c $< -o $@

rtlsdr_ft8d: $(OBJS)
	$(CC) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -f *.o ft8/*.o common/*.o fft/*.o $(TARGETS)

install:
	install rtlsdr_ft8d /usr/local/lib/rtlsdr_ft8d