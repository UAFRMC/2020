OPTS=-g
CFLAGS=-I../../include -std=c++11 $(OPTS)
PROGS=millitime latcheck 

all: $(PROGS)

millitime: millitime.cpp
	g++ $(CFLAGS) $< -o $@

latcheck: latcheck.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

