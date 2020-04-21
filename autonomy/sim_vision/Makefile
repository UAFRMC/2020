OPTS=-O
CFLAGS=-Wall -I../include -std=c++11 $(OPTS)
PROG=sim_vision

all: $(PROG)

$(PROG): sim.cpp
	g++ $(CFLAGS) $< -o $@

run: $(PROG)
	./$(PROG)

clean:
	- rm $(PROG)
