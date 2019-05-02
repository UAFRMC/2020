CFLAGS=-I../../autonomy/include
LIBS=-lzmq -lpthread -laruco -lrealsense2 `pkg-config opencv --cflags --libs`
OPTS=-g -O4


all: realsense

realsense: realsense.cpp aruco_localize.cpp ../include/*/*
	g++ $(OPTS) -std=c++14 $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm realsense
