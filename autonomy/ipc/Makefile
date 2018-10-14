OPTS=-g
CFLAGS=-std=c++11  $(OPTS) -I../include


all: dump_tf dump_grid

dump_tf: dump_tf.cpp
	g++ $(CFLAGS) $< -o $@ 

dump_grid: dump_grid.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm dump_tf dump_grid
