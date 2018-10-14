OPTS=-O3
CFLAGS= -std=c++11 $(OPTS) -I. -I../include -I/usr/local/include/libfreenect
LDFLAGS=  -lfreenect -lpthread -lusb-1.0 -lGL -lglut 

all: kinect

kinect: main.cpp
	g++ $(CFLAGS) $< $(LDFLAGS)  -o $@

clean:
	- rm kinect


