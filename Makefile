# Makefile for rv64sim project

CC=gcc -g -Wall
TARGET=rv64sim

all: $(TARGET)

sim: $(TARGET)
	./rv64sim localbuild.bin 64mb.dtb

$(TARGET): cpu.c soc.c rv64sim.c
	$(CC) $(CFLAGS) $^ -o $@

clean:
	rm -f $(TARGET) *.o

