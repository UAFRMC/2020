OPTS=-g
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=andretti

all: $(PROGS)

andretti: andretti.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)