The "Autonomy" system is separated into two halves:

The "Front End" runs on a control booth PC, and it:
	- Accepts keyboard presses, joystick events, etc in manual mode.  
	- Connects to the back end over the network (probably TCP to start with; should also try UDP subnet broadcast).

The "Back End" runs on an on-robot PC, and it:
	- Talks to the front end via a network socket
	- Logs telemetry for future analysis
	- Performs autonomous responses, like raising the mining head when it stalls
	- Interfaces with sensors, like Kinect or webcam
	- Talks to the Arduino via a USB serial connection
		- Resets the connection if it is ever lost


Build instructions:
	sudo apt-get install freeglut3-dev g++ make

For the vive viewer:
	sudo cp vive/91-vive.rules /etc/udev/rules.d/
	cd vive
	make

For the kinect classifier:
	sudo cp kinect/66-kinect.rules /etc/udev/rules.d/
	sudo apt-get install libfreenect-dev libusb-1.0-0-dev
	cd kinect
	make

For the aruco marker viewer:
	sudo apt-get install cmake libopencv-dev
	cd aruco/aruco-3.0.11
	cmake .
	make
	sudo make install
	cd ../../viewer
	make

For the realsense viewer:
  sudo apt-get install libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libzmq3-dev cmake libssl-dev
  git clone https://github.com/IntelRealSense/librealsense
  cd librealsense/
  ./scripts/patch-realsense-ubuntu-lts.sh 
  cmake .




---------------
2017 Hardware info:

Magnetic encoders:
	36 encoder counts per motor revolution

Mining head:
	15 mining scoops.  
	8 holes in mining head drive, but only 4 scoops.

Drive tracks:
	16 100mm track segments per track.
	Chain drive: 19 teeth on drive sprocket (aluminum), 27 teeth on driven sprocket (pet or ABS)



-----------------
OBSOLETE 2015 Hardware info:
Drive wheels: 16 encoder edges per revolution, 25mm per encoder edge
Mining head: 
	16 encoder sectors
	8 encoder edges per scoop
	15 scoops in entire bucket

Invariants:
	- Deploy front wheels before doing any driving
	- Must raise mining head (to about 15%) before doing any driving (keep from high-centering)
	- Must lower mining head (below 20%) before doing any driving (keep from flipping)
	- To enter mining mode, lower head to 10%


----------
Shipping:
To avoid extra charges for oversize or overweight baggage, your checked bag must:
 weigh 50 pounds (23 kg) or less
 not exceed 62 inches (157 cm) when you total length + width + height

	
-----------
3D printing:
	- Sprockets
	- Motor caps


------------
Rex Engineering DC motors
	- Stock motors: 6 amp idle
	- 11K RPM motors: 3amp idle

