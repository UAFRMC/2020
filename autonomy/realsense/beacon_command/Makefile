CFLAGS=-I../../include
LIBS=-lzmq 


all: beacon

beacon: beacon.cpp
	g++ $(OPTS) -std=c++14 $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm beacon
