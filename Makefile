CC=gcc
CFLAGS= -g -o

all:
	$(CC) $(CFLAGS) powerduino_PC powerduino_PC.c;
	rm -rf *.dSYM
clean:
	rm -rf powerduino_PC