CFLAGS=-I../../autonomy/include
OPTS=-g -O4


all: realsense

realsense: realsense.cpp
	g++ $(OPTS) -std=c++14 $(CFLAGS) $< -o $@ -lrealsense2 `pkg-config opencv --cflags --libs` 

clean:
	- rm realsense
