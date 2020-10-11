OPTS=-O4
CFLAGS=-Wall -I../include -std=c++11 $(OPTS)
LIBS= -lrealsense2 
PROGS=t265

all: $(PROGS)

t265: t265.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)