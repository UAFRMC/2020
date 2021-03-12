include ../Makefile.inc

OPTS=-O4
CFLAGS=-I../include -std=c++11 $(OPTS) $(CVCFLAGS)
LIBS=$(CVLINK)
PROGS=cartographer

all: $(PROGS)

cartographer: cartographer.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)
