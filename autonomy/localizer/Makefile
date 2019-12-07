OPTS=-g
CFLAGS=-I../include -std=c++11 $(OPTS)
PROGS=localizer

all: $(PROGS)

localizer: localizer.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

