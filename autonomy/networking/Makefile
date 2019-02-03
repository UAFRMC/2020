OPTS=-O3
CFLAGS= -std=c++11 $(OPTS)
LDFLAGS=  -L/usr/local/lib -lzmq

all: network-client network-server

network-client: hello_client.cpp
	g++ $(CFLAGS) $< $(LDFLAGS)  -o $@

network-server: hello_server.cpp
	g++ $(CFLAGS) $< $(LDFLAGS)  -o $@

clean:
	- rm kinect


