OPTS=-g
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=stepper

all: $(PROGS)

stepper: stepper.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)
