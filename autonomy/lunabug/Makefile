OPTS=-O4
CFLAGS=-I../include  -Wall  -std=c++14  $(OPTS)
LIBS=`pkg-config opencv --cflags --libs`
PROGS=lunaview lunatic_print_drive lunatic_print_encoders lunatic_print_stepper lunatic_print_2Dpos

all: $(PROGS)

lunaview: lunaview.cpp ../include/*/*
	g++ $(OPTS) $(CFLAGS) $< -o $@ $(LIBS)

lunatic_print_drive: lunatic_print_drive.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_print_encoders: lunatic_print_encoders.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_print_stepper: lunatic_print_stepper.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_print_2Dpos: lunatic_print_2Dpos.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)
