OPTS=-O4
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=pathplanner

all: $(PROGS)

pathplanner: pathplanner.cpp pathplanner.h ../include/gridnav/* ../include/aurora/*
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

