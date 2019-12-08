OPTS=-g
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=backend

all: $(PROGS)

backend: backend.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)