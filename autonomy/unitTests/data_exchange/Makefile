OPTS=-O
CFLAGS=-I../../include -std=c++11 $(OPTS)
PROGS=millitime latcheck atomic_exchange

all: $(PROGS)

millitime: millitime.cpp
	g++ $(CFLAGS) $< -o $@

atomic_exchange: atomic_exchange.cpp
	g++ $(CFLAGS) $< -o $@

latcheck: latcheck.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

