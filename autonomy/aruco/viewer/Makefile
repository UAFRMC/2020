OPTS=-g -O3 -std=c++14
ARUCO=/usr/local
OPENCV=/usr/lib/x86_64-linux-gnu
CFLAGS=$(OPTS) -L$(OPENCV) -I$(ARUCO)/include/aruco 
LOPENCV=-lopencv_core -lopencv_calib3d -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lopencv_video -lopencv_videoio -lopencv_imgcodecs -lpthread -lm
LFLAGS=$(OPTS) -L$(ARUCO)/lib -laruco -Wl,-rpath,$(ARUCO)/lib -Wl,-rpath,$(OPENCV)/lib $(LOPENCV)


all: camera

camera: main.cpp
	g++ $< -o $@ $(CFLAGS) $(LFLAGS)

clean:
	- rm camera

cat_marker: cat_marker.cpp
	g++ $(OPTS) $< -o $@ -I.

