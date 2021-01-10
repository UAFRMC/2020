/**
 Aurora Robotics general robot code.

 This is shared between the Arduino and the PC.

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__ROBOT_BASE_H
#define __AURORA_ROBOTICS__ROBOT_BASE_H
#include <stdint.h> /* for uint32_t */
#include "field_geometry.h"

/** This is the Arduino's AREF analog reference voltage.
  It's the scale factor that gives true voltage output,
  and should be measured from the AREF pin against Arduino ground. */
#define AD_AREF_voltage (4.78)

/** This scale factor converts an
	Arduino Analog/Digital Data Number (0-1023)
	to a real voltage, assuming direct feed-in. */
#define AD_DN2low_voltage (AD_AREF_voltage/(1024.0))

/** This scale factor converts an
	Arduino Analog/Digital Data Number (0-1023)
	to a real voltage, after the resistor divider scaling.
*/
#define AD_DN2high_voltage ((AD_AREF_voltage)*(11.0)/1024.0)


/** This class contains all the robot's sensors, on Arduino, backend, or front end.
Raw sensor values go as bitfields, because many of them are 10-bit quantities:
	- Arduino A/D values are 10 bits each
	- Arena positions in cm are 9-10 bits each (arena is 378x738cm)
	- Blinky angle reports are about 9 bits each (500 samples per rotation)
*/
class robot_sensors_arduino
{
public:
	uint32_t battery:10; // raw A/D reading at top of battery stack (voltage = this*5*2000/384)
	uint32_t bucket:10; // raw A/D value from dump bucket lift encoder
	uint32_t latency:5; // Arduino control loop latency

	uint32_t Mstall:1;
	uint32_t DLstall:1;
	uint32_t DRstall:1;

	uint32_t stop:1; ///< EMERGENCY STOP button engaged
  uint32_t heartbeat:3;

	uint32_t McountL:8; /// Current milliseconds per encoder tick for mining head left motor (255==stopped)
	uint32_t McountR:8; /// Encoder tick count for mining head left motor

	uint32_t DL1count:8; /// Encoder tick count for front left drive wheel
	uint32_t DL2count:8; /// Encoder tick count for back left drive wheel
	uint32_t DR1count:8; /// Encoder tick count for right drive wheel
	uint32_t DR2count:8; /// Encoder tick count for back right drive wheel

	int32_t Rcount:16; /// Encoder tick for bag roll motor

	uint32_t limit_top:8;
	uint32_t limit_bottom:8;

	uint32_t encoder_raw:16;
  uint32_t stall_raw:16;
  uint32_t pad:16; // round up to 32
};

/**
 This class contains a power setting for each of the robot's actuators.

 The signed char run from -100 (full backwards) to 0 (stop) to +100 (full forwards)
*/
class robot_power {
public:
	enum { drive_stop=0 };

	signed char left; // left drive wheels
	signed char right; // right drive wheels
	signed char mine; // mining head dig
	signed char conveyor_raise;
	signed char dump; // storage bucket lift
	signed char roll; //Roll bag
	signed char head_extend; // Extend mining head linear
	
	unsigned char high:1; // High power mode
	unsigned char torqueControl:1; // Drive backwards (for final dump)
	unsigned char mineDump:1; // Run backwards and dump
	unsigned char mineEncoderReset:1; //Get ready to go out and mine again
	
	unsigned char motorControllerReset:1; //Reset BTS motor controller enable pin
	unsigned char mineMode:1; // if true, autonomously run mining head
	unsigned char dumpMode:1; // dock-and-dump mode

	robot_power() { stop(); }
	
	void stop(void) {
		left=right=mine=dump=roll=head_extend=conveyor_raise=drive_stop; // all-stop
		high=dumpMode=mineMode=mineDump=mineEncoderReset=0;
	}
};



enum {
/*
		DN 949 fully up
		DN 330 mining head will drag on bar
		DN 260 mining starts on level ground
		DN 240 conservative mining depth
		DN 180 fully down
*/
		head_mine_stop=0, // stop lowering at this mining height
		head_mine_start=700, // start mining at this height
		head_mine_drive=900, // normal driving height
		head_drive_safe=940, // can safely drive below this height (without tipping over)
		head_mine_dump=940, // dumping height
		head_mine_stow = 850, // stow height, where frame is level

	// These 2 are used to tell whether the box is at max or min height
	box_raise_max = 1000,
	box_raise_min = -500,
	box_raise_limit_high = 1000,
	box_raise_limit_low = -500
	};
/**
  This is a list of possible robot states.
  It's mostly maintained on the backend, but
  can be commanded from the front end.
*/
typedef enum {
	state_STOP=0, ///< EMERGENCY STOP (no motion)
	state_drive, ///< normal manual driving
	state_backend_driver, ///< drive from backend UI

	state_autonomy, ///< full autonomy start state
	state_setup_raise, ///< raise conveyor before driving
	state_setup_extend, ///< extend mining head before driving
	state_setup_lower, ///< lower box before driving
	state_find_camera, ///< turn until camera is visible
	state_scan_obstacles, ///< scan for obstacles

	state_drive_to_mine, ///< autonomous: drive to mining area

	/* Semiauto mine mode entry point: */
	state_mine_lower, ///< mining mode: lowering head, driving forward
	state_mine_stall, ///< mining mode: raising head (after stall)
	state_mine, // actually mine
	state_mine_raise, ///< existing mining mode: raise bucket

	state_drive_to_dump, ///< drive back to bin
	state_dump_align, ///< get lined up

	/* Semiauto dump mode entry point: */
	state_dump_contact, ///< final dock-and-dump mode: drive to contact bin
	state_dump_raise, ///< raise box
	state_dump_pull, ///< pull box up
	state_dump_rattle, ///< rattle mode to empty bucket
	state_dump_push, ///< push box back down

	/* Semiauto dump mode entry point: */
	state_stow, // begin stowing: raise bucket
	state_stow_clean, // clean bucket
	state_stowed, // finished stowing (wait forever)

	state_last ///< end state (repeat from mine_drive)
} robot_state_t;
const char *state_to_string(robot_state_t state);

/// This bitfield convey's the robot's software status.
class robot_status_bits {
public:
	unsigned char stop:1; ///< EMERGENCY STOP engaged
	unsigned char arduino:1; ///< arduino is connected correctly
	unsigned char located:1; ///< robot thinks it knows where it is
	unsigned char autonomy:1; ///< full-autonomy mode is engaged
	unsigned char semiauto:1; ///< semiauto mode is engaged
};

class robot_arduino {
  public:
	robot_power power;
	robot_sensors_arduino sensor;
};


#endif

