/**
  Aurora Robotics keyboard user interface code.

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#ifndef __AURORA_ROBOTICS__UI_H
#define __AURORA_ROBOTICS__UI_H

#include "ogl/event.h" /* for joystick */

#ifdef MSL
	#include "msl/joystick.hpp"
	#include "msl/joystick.cpp"
#endif
#include <iostream>

/**
 Keyboard-based user interface for robot.
*/
class robot_ui {
public:

	float driveLimit=0.1; /* robot driving power (0.0 - 1.0) */
	
	robot_power power; // Last output power commands
	robot_realsense_comms realsense_comms;
	#ifdef MSL
		msl::joystick_t* joystick;
	#endif

	// Current floating-point power values:
	float left, right, front, mine, dump, roll, head_extend, conveyor_raise;

	//UI stall states
	bool Mstall;

	// Human-readable description of current state
	std::string description;

	bool js_button(int button,const std::string& label)
	{
		#ifdef MSL
			if(button==1)
				button=15;
			else if(button==2)
				button=12;
			else if(button==3)
				button=14;
			else if(button==4)
				button=13;
			else if(button==5)
				button=0;
			else if(button==6)
				button=3;
			else
				button=-1;
			if(button>-1&&joystick!=NULL&&joystick->good()&&button<(int)joystick->button_count())
				return joystick->button(button);
		#else
			return oglButton(button,label.c_str());
		#endif
		return false;
	}

	bool js_button_once(const int button,const std::string& label)
	{
		#ifdef MSL
			return js_button(button,label);
		#else
			return oglButtonOnce(button,label.c_str());
		#endif
		return false;
	}

	float js_axis(int axis,const std::string& label)
	{
		#ifdef MSL
			--axis;
			if(axis==2)
				axis=3;
			else if(axis==3)
				axis=2;
			std::cerr<<axis<<"="<<joystick->axis(axis)<<std::endl;
			if(joystick!=NULL&&joystick->good()&&axis<(int)joystick->axis_count())
				return joystick->axis(axis);
		#else
			return oglAxis(axis,label.c_str());
		#endif
		return false;
	}

	void stop(void) {
		left=right=front=mine=dump=roll=head_extend=0.0;
		power.stop();
		description="Sending STOP";
	}

	// Respond to these keystrokes.
	//  The "keys" array is indexed by the character, 0 for up, 1 for down.
	void update(int keys[],const robot_base &robot);

	robot_ui() {
		#ifdef MSL
		joystick=NULL;
		#endif
		stop();
		description="Starting up";
	}

	// Clamp this float within this maximum range
	float limit(float v,float maxPower) const {
		if (v>maxPower) v=maxPower;
		if (v<-maxPower) v=-maxPower;
		return v;
	}

	// Convert a raw float to a motor command, with this maximum range
	byte toMotor(float v,float maxPower) const {
		v=limit(v,maxPower);
		int iv=(int)(v*100);
		if (iv<-100) iv=-100;
		if (iv>100) iv=100;
		return iv;
	}
};

void robot_ui::update(int keys[],const robot_base &robot) {
	#ifdef MSL
		if(joystick==NULL)
		{
			auto joysticks=msl::joystick_t::list();
			if(joysticks.size()>0)
			{
				joystick=new msl::joystick_t(joysticks[0]);
				joystick->open();
			}
		}
		if(joystick!=NULL&&!joystick->good())
		{
			delete joystick;
			joystick=NULL;
		}
	#endif

	static int keys_last[256];
	int keys_once[256]; // key down only *one* time per press
	for (int i=0;i<256;i++) {
		keys_once[i]= (keys[i] && !keys_last[i]);
		keys_last[i]=keys[i];
	}
	description="UI:\n";

// Power limits:
	float mineLimit=0.5;
	float dumpLimit=1.0;
	float rollLimit=0.5;
	float head_extend_limit = 1.0;
	float converyor_raise_limit=0.5;

// Prepare a command:
	if (keys[' ']) { // spacebar--full stop
		stop();
		robotState_requested=state_STOP;
		return; // don't do anything else.  Just stop.
	}
	// else spacebar not down--check other keys for manual control

	/* scale back, so things die away when key is released.
   It'd be better to detect key-up here, and stop, but that's ncurses. */
	left*=0.80;
	right*=0.80;
	front*=0.5;
	mine*=0.5;
	dump*=0.5;
	roll*=0.5;
	conveyor_raise*=0.5;

	head_extend=0.0;


	static bool joyDrive=false;
	bool joyDone=false; // subtle:



/*  Fix: Uses only the left analog stick for driving the robot */
	/* Uses the left analog stick*/
/* TODO: Map dumpMode to joystick*/
	float forward=-js_axis(2,"Go Forward or Reverse"); // left Y
	float turn=js_axis(1,"Turn Left or Right"); //left X, scaled for gentle turns

	/* Oerate the mining head*/
	 float dumpJoy=-js_axis(3,"Operate Dump Buckets");
	 //ToDo: Change this back to mining
	 float conveyor_joy=js_axis(4,"Run Conveyor belt");

	if(forward!=0.0 || turn!=0.0 || dumpJoy!=0 || conveyor_joy!=0)
	{
		joyDrive=true; // using joystick
	}
	else {joyDone=true;}
	if(joyDrive)
	{
		left=driveLimit*(forward+turn);
		right=driveLimit*(forward-turn);
         	dump=dumpLimit*dumpJoy;
		roll+=conveyor_joy;

		description += "  joystick\n";
	}
	joyDrive=!joyDone;
	if(js_button(1,"state_mine"))
	{
		robotState_requested=state_mine;
	}
	if(js_button(2,"state_dump_pull"))
	{
		robotState_requested=state_dump_pull;
	}

	if (js_button(3,"Stop"))
	{
		stop();
		robotState_requested=state_STOP;
	}
	if(js_button(4,"Drive"))
	{
		robotState_requested=state_drive;
	}
	if(js_button(5,"state_dump_pull"))
	{
		robotState_requested=state_dump_pull;
	}
	if((js_button_once(6,"High Power")))
	{
		power.high=!power.high;
	}


	if ((js_button_once(1,"Break"))) { stop(); } // stop without changing state


//Pilot warning messages:

	if(Mstall)
		description+="  MINING HEAD STALLED. RELEASE POWER\n";
	if(robot.sensor.DLstall)
		description+="  LEFT DRIVE STALLED\n";
	if(robot.sensor.DRstall)
		description+="  RIGHT DRIVE STALLED\n";
//Realsense beacon commands
	realsense_comms.command = ' ';
	
	/* Use P + number keys to set drive power, in percent:
	    P-1 = 10%, P-2=20%, etc. */
	if (keys['p']||keys['P']) for (int num=1;num<=9;num++)
	    if (keys['0'+num]) driveLimit=0.1f*num;
	description+="\n  Drive power: "+std::to_string(driveLimit)+"\n";

	float beacon_requested_angle=-1;
	/*
	if(keys['1'])
		beacon_requested_angle=-60;
	if(keys['2'])
		beacon_requested_angle=-45;
	if(keys['3'])
		beacon_requested_angle=-0;
	if(keys['4'])
		beacon_requested_angle=45;
	if(keys['5'])
		beacon_requested_angle=60;
    */

	if (beacon_requested_angle != -1)	
		{
			realsense_comms.requested_angle=(signed char)(beacon_requested_angle);
			realsense_comms.command='P';
			description+= " \n Requested Beacon Angle "+ std::to_string(realsense_comms.requested_angle) + "\n";
		}

// Drive keys:
	float acceleration=.2;
        if(keys['w']||keys['W'])
        {
            left+=acceleration;
            right+=acceleration;
        }
        if(keys['s']||keys['S'])
        {
            left-=acceleration;
            right-=acceleration;
        }
        if(keys['a']||keys['A'])
        {
            left-=acceleration;
            right+=acceleration;
        }
        if(keys['d']||keys['D'])
        {
            left+=acceleration;
            right-=acceleration;
        }
        if(keys_once['t']||keys_once['T'])
        {
			power.torqueControl=!power.torqueControl;
        }

	power.motorControllerReset=keys['r']||keys['R'];

	if(power.motorControllerReset!=0)
		description+="  BTS Motor Reset\n";


	if(power.torqueControl!=0)
		description+="  torque control\n";
	else
		description+="  speed control\n";


// Special features

	if(keys_once['m'])
        {
	   power.mineMode=!power.mineMode;
        }
	if (power.mineMode) {
		mine=mineLimit;
	}

	if(keys['j'])
        {

            mine=-1;
            description+="  mine-\n";
        }
	if(keys_once['p'])
	{
	    power.high=!power.high;
	}

        if(keys[oglSpecialDown])
        {
            dump=-1;
            description+="  depth-\n";
        }
        if(keys[oglSpecialUp])
        {
            dump=+1;
            description+="  depth+\n";
        }
        if(keys[oglSpecialRight])
        {
            mine=+mineLimit;
            description+="  mine+\n";
        }
        if(keys[oglSpecialLeft])
        {
            mine=-mineLimit;
            description+="  mine-\n";
        }
	if(keys['f'])
	{
		roll=+1;
		description+=" roll+\n";
	 }
	if(keys['v'])
	{
		roll=-1;
		description+=" roll-\n";
	}
	if(keys['g'])
	{
		head_extend = +1;
		description += " head_extend+\n";
	}
	if(keys['b'])
	{
		head_extend = -1;
		description += " head_extend-\n";
	}
	if(keys['h'])
	{
		conveyor_raise = +1;
		description += " conveyor_extend+\n";
	}
	if(keys['n'])
	{
		conveyor_raise = -1;
		description += " conveyor_extend-\n";
	}
	if(robot.sensor.Mstall && mine!=0)
		Mstall=true;
	else if(mine==0)
		Mstall=false; // Reset UI stall if no mine command

	left=limit(left,driveLimit);
	right=limit(right,driveLimit);

	if(Mstall)
		mine=0; //Do not allow mining if UI stall is set
	power.left=toMotor(left,driveLimit);
	power.right=toMotor(right,driveLimit);
	power.mine=toMotor(mine,mineLimit);
	power.dump=toMotor(dump,dumpLimit);
	power.roll=toMotor(roll,rollLimit);
	power.head_extend=toMotor(head_extend, head_extend_limit);
	power.conveyor_raise=toMotor(conveyor_raise, converyor_raise_limit);

	robotPrintln("Arduino Heartbeat: %d",robot.sensor.heartbeat);
	robotPrintLines(description);
	printf("Robot raw power: %d %d %d %d %d %d\n",
		power.left, power.right, power.mine, power.dump, power.roll, power.head_extend);
}



#endif

