/**
 Aurora Robotics general robot code.

 This is shared between the frontend and backed, but not the Arduino.
*/
#ifndef __AURORA_ROBOTICS__ROBOT_H
#define __AURORA_ROBOTICS__ROBOT_H

#include "coords.h" // coordinate systems
#include "../firmware/robot_base.h" // microcontroller code and states

// Command letters:
enum {
  aurora_beacon_command_scan='T', // take a scan for obstacles
  aurora_beacon_command_point='P', // point sensor this direction
  aurora_beacon_command_home='H', // point sensor this direction
  aurora_beacon_command_off='O', // power off the beacon
};
typedef signed short aurora_beacon_command_angle_t;

struct aurora_beacon_command {
  // Command letter
  char letter;

  // Angle, used by scan and point commands.
  //   Angle is in degrees up from the +X axis.
  aurora_beacon_command_angle_t angle;
};

struct aurora_detected_obstacle {
  signed short x,y; // location on field (cm, field coords)
  signed short height; // height above/below neighbors
};


// Comms to the realsense camera stepper motor
class robot_realsense_comms{
public:
	signed char requested_angle;
	char command;
	
	robot_realsense_comms()
	{
		command = 0;
		requested_angle = (unsigned char)-1;
	}
};



// Everything about the visible markers
class robot_markers_all {
public:
  // Integrated robot position
  robot_localization pose;

  // Raw sensed pose for each marker
  enum {NMARKER=2}; // <- all tag25h9 markers
  robot_localization markers[NMARKER];
  
  // Angle of receiver beacon, in degrees, up from horizontal (+X)
  float beacon;
};

/**
 This class contains everything we currently know about the robot.
*/
class robot_base {
public:
	robot_state_t state; ///< Current control state
	robot_status_bits status; ///< Current software status bits
	robot_sensors_arduino sensor;  ///< Current hardware sensor values
	robot_localization loc; ///< Location
	robot_power power; // Current drive commands
	robot_realsense_comms realsense_comms;

	bool autonomous;
};

#endif

