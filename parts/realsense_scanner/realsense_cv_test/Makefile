all: realsense

realsense: realsense.cpp
	g++ $< -o $@ -lrealsense2 `pkg-config opencv --cflags --libs` -ljpeg

clean:
	- rm realsense
