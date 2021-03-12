include ../Makefile.inc

OPTS=-O4
CFLAGS=-Wall -I../include -std=c++11 $(OPTS) $(CVCFLAGS)
LIBS=-laruco -lrealsense2 $(CVLINK)
PROGS=vision

all: $(PROGS)

vision: vision.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)

