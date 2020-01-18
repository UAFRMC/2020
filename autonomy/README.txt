UPDATE 2020:

The "Autonomy" system is separated into two halves:

The "Front End" runs on a control booth PC, and it:
	- Accepts keyboard presses, joystick events, etc in manual mode.  
	- Connects to the back end over the network (probably TCP to start with; should also try UDP subnet broadcast).
	
The "Backend" runs on an on-robot PC,
The System is divided into 5 parts, these parts include the Bac-Kend, Pathplanner, Vision, Stepper, and Localizer.
These systems are designed to have comms via MMap. With dedicated files to communicate between systems. The files defined are located in the lunatic.h. This style of system is replacing the monolithic backend from previous iterations. 

Logically UNcoupled Architecture Technology for Intra-robot Communication

Lunatic manages data exchange between all on-robot 
software components, including these programs:

Vision manages the realsense camera
    -Reads color and depth frames from the camera
    -Passes color to aruco to look for computer vision markers
        -Any detected markers create position estimates for the localizer
    -Passes depth to obstacle detector subsystem to look for rocks
        -Any detected obstacles get written to the obstacle grid

Localizer integrates the robot position
    -Reads drive encoder counts from the backend
    -Reads aruco data from the vision subsystem
    -Reads camera pointing from the stepper motor
    -Publishes coordinates used by all other components

Stepper talks to the camera pointing stepper motor
    -Reads requested view angle from backend
    -Publishes current view angle to the localizat

Path planner computes an obstacle-free drive path
    -Reads the robot location from the localizer
    -Reads the target location from the backend
    -Reads obstacle grid from the obstacle detector
    -Publishes drive commands to the backend
    -Uses an A* to plan paths. 
    	-A* is a goal area rather than a point.

backend talks to the robot, using two subsystems:
    BAC: Basic Autonomy Control 
        -manages the autonomy state machine and pilot comms (over UDP)
	
    KEND: Keep Electronics Not Dying 
	- Talks to the front end via a network socket
	- Interfaces with sensors, like Kinect or webcam
	- Talks to the Arduino via a USB serial connection
		- Resets the connection if it is ever lost


Build instructions:
	sudo apt-get install freeglut3-dev g++ make libzmq3-dev
	cd backend
	make
	./backend --sim
(The backend without --sim tries to connect to the Arduino mega.)


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


For the vive viewer:
	sudo cp vive/91-vive.rules /etc/udev/rules.d/
	cd vive
	make

For the kinect classifier:
	sudo cp kinect/66-kinect.rules /etc/udev/rules.d/
	sudo apt-get install libfreenect-dev libusb-1.0-0-dev
	cd kinect
	make



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

