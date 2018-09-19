install: install_udev install_lib

install_udev:
	sudo cp 66-kinect.rules /etc/udev/rules.d/

install_lib:
	sudo apt-get install libxi-dev libxmu-dev freeglut3-dev build-essential cmake usbutils libusb-1.0-0-dev git-core
	- git clone git://github.com/OpenKinect/libfreenect.git
	cd libfreenect; cmake .; make; sudo make install
	sudo ldconfig


