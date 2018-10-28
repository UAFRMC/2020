all: realsense

realsense: realsense.cpp
	g++ -O -g -std=c++14 $< -o $@ -lrealsense2 `pkg-config opencv --cflags --libs` 

clean:
	- rm realsense
