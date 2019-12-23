PROGS=backend frontend localizer pathplanner stepper vision lunabug

all: 
	for dir in $(PROGS); do make -C $$dir; done

clean: 
	for dir in $(PROGS); do make -C $$dir clean; done

install:
	sudo apt-get install freeglut3-dev g++ make libzmq3-dev
	sudo apt-get install cmake libopencv-dev 
	cd aruco/aruco-3.0.11; cmake .; make; sudo make install

