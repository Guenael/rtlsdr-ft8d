CC = clang
CFLAGS= -O3 -std=gnu17 -Wall # -fsanitize=address
LIBS = -lusb-1.0 -lrtlsdr -lpthread -lfftw3f -lcurl -lm # -fsanitize=address

OBJS = rtlsdr_ft8d.o ft8_lib/ft8/constants.o ft8_lib/ft8/pack.o ft8_lib/ft8/unpack.o ft8_lib/ft8/text.o ft8_lib/ft8/ldpc.o ft8_lib/ft8/crc.o ft8_lib/ft8/encode.o ft8_lib/ft8/decode.o

TARGETS = rtlsdr_ft8d

.PHONY: all clean

all: $(TARGETS)

%.o: %.c
	${CC} ${CFLAGS} -c $< -o $@

rtlsdr_ft8d: $(OBJS)
	$(CC) -o $@ $^ $(LIBS)

clean:
	rm -f *.o ft8_lib/ft8/*.o $(TARGETS) fftw_wisdom.dat selftest.iq

install:
	install rtlsdr_ft8d /usr/local/bin/rtlsdr_ft8d
