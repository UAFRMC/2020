OPTS=-g
CFLAGS=-I../../include -std=c++11 $(OPTS)
PROGS=send millitime latcheck

all: $(PROGS)

send: send.cpp
	g++ $(CFLAGS) $< -o $@

recv: recv.cpp
	g++ $(CFLAGS) $< -o $@

millitime: millitime.cpp
	g++ $(CFLAGS) $< -o $@

latcheck: latcheck.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

