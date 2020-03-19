/* 
Packets exchanged with the Arduino nanos with motor controllers attached. 
*/
#ifndef __AURORA_NANO_NET_H
#define __AURORA_NANO_NET_H


namespace nano_net
{
  enum { n_motors=4 };
  enum { n_sensors=6 };
  enum { n_nanos=2 };

  // Motor control modes
  enum { 
     motorMode_speed='0', // add this to 0-based sensor number for speed control
     motorMode_speed_end='6', // beyond last valid sensor
     motorMode_torque='T', // raw torque control (speed == PWM value)
  };
  
  // Sensor control modes
  enum {
    sensorMode_motor='0', // add this to motor number to count up/down based on direction
    sensorMode_motor_end='5', // beyond last valid motor
    sensorMode_binary='B', // Only report 0 or 1 (e.g., limit switch)
    sensorMode_constant='C', // Send the constant value 9 (a configuration test)
  };

  // From mega down to nano: (re)configuration / setup
  struct nano_net_setup {
    // The control mode for each motor
    unsigned char motorMode[n_motors];
    // The control mode for each sensor
    unsigned char sensorMode[n_sensors];
  };

  // From mega down to nano: normal commands
  struct nano_net_command
  {
    unsigned char stop:1; // 0: normal mode.  1:stop all motors immediately
    unsigned char torque:1; // 0: normal mode.  1:all motors are in torque control
    unsigned char LED:1; // 0: LED off.  1:LED on.
    unsigned char pad1:5; // future use
    
    signed char speed[n_motors]; //Speed in percent. -100 (full red), 100 (full green)
  };

  // From nano up to mega
  struct nano_net_sensors
  {
    unsigned char stall:4; // For each motor: 0-not stalled.  1-currently stalled.
    unsigned char ok:1; // 0- no commands recently.  1- nano is receiving commands regularly.
    unsigned char nosetup:1; // 1- nano needs setup packet
    unsigned char heartbeat:2; // increments every nano loop
    
    unsigned char raw:6; // 6 raw encoder pins (for debugging, or for level-triggered stuff)
    unsigned char pad2:2; // reserved for future use
    
    unsigned char counts[n_sensors]; // reported encoder counts
  };
  
  
};


#endif
