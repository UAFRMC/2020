OPTS=-O4
CFLAGS=-I../include  -Wall  -std=c++14  $(OPTS)
LIBS=`pkg-config opencv --cflags --libs`
PROGS=lunaview lunatic_print_drive lunatic_print_encoders lunatic_print_stepper lunatic_print_2Dpos lunatic_print_3Dpos lunatic_print_target lunatic_set_target lunatic_set_stepper exchange_read exchange_write

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

lunatic_print_3Dpos: lunatic_print_3Dpos.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_print_target: lunatic_print_target.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_set_target: lunatic_set_target.cpp
	g++ $(CFLAGS) $< -o $@

lunatic_set_stepper: lunatic_set_stepper.cpp
	g++ $(CFLAGS) $< -o $@

exchange_read: exchange_read.cpp
	g++ $(CFLAGS) $< -o $@

exchange_write: exchange_write.cpp
	g++ $(CFLAGS) $< -o $@

clean:
	- rm $(PROGS)
