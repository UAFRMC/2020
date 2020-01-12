OPTS=-O3
CFLAGS= -std=c++14 $(OPTS)
LDFLAGS=  -L/usr/local/lib -lzmq
PROGS=network-client network-server network-sub network-pub


all: $(PROGS)

network-client: hello_client.cpp
	g++ $(CFLAGS) $< $(LDFLAGS)  -o $@

network-server: hello_server.cpp
	g++ $(CFLAGS) $< $(LDFLAGS)  -o $@

network-sub: subscribe_test.cpp
	g++ $(CFLAGS) $< $(LDFLAGS)  -o $@

network-pub: publisher_test.cpp
	g++ $(CFLAGS) $< $(LDFLAGS)  -o $@

clean:
	- rm $(PROGS)


