OPTS=-O4
CFLAGS=-I../include -std=c++11 $(OPTS)
LIBS=-lpthread  `pkg-config opencv --cflags --libs`
PROGS=cartographer

all: $(PROGS)

cartographer: cartographer.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)
