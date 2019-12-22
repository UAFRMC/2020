OPTS=-g -O4
CFLAGS=-I../include  -Wall  -std=c++14  $(OPTS)
LIBS=`pkg-config opencv --cflags --libs`
PROGS=lunaview

all: $(PROGS)

lunaview: lunaview.cpp ../include/*/*
	g++ $(OPTS) $(CFLAGS) $< -o $@ $(LIBS)

clean:
	- rm $(PROGS)
