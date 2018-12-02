CFLAGS=-I../../autonomy/include
CFLAGS += `pkg-config opencv --cflags --libs` 
OPTS=-g -O4


all: main

main: main.cpp
	g++ $(OPTS) -std=c++14 $< -o $@ $(CFLAGS) 

clean:
	- rm main
