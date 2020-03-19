OPTS=-g
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=kend

all: $(PROGS)

kend: kend.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)
