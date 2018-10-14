


install:
	sudo apt-get install freeglut3-dev g++ make libsoil-dev
	sudo apt-get install cmake libopencv-dev libcv-dev libhighgui-dev
	cd aruco; tar xvf aruco-*tgz; cd aruco-*/; mkdir build; cd build; cmake ..; make; sudo make install
	cd aruco/viewer; make
	cd backend; make
	cd frontend; make

