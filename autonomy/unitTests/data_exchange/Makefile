OPTS=-g
CFLAGS=-I../../include -std=c++11 $(OPTS)
PROGS=millitime latcheck lunatic_print_drive lunatic_print_encoders lunatic_print_stepper

all: $(PROGS)

millitime: millitime.cpp
	g++ $(CFLAGS) $< -o $@

latcheck: latcheck.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_print_drive: lunatic_print_drive.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_print_encoders: lunatic_print_encoders.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_print_stepper: lunatic_print_stepper.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)

