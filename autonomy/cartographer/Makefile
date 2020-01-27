OPTS=-O4
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=cartographer

all: $(PROGS)

cartographer: cartographer.cpp ../include/aurora/*
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)
