OPTS=-g
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=cvs

all: $(PROGS)

cvs: cvs.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

