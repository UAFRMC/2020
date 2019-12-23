OPTS=-O4
CFLAGS=-Wall -I../include -std=c++11 $(OPTS)
LIBS=-lpthread -laruco -lrealsense2 `pkg-config opencv --cflags --libs`
PROGS=vision

all: $(PROGS)

vision: vision.cpp ../include/*/*
	g++ $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)

