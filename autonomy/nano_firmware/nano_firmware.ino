/*
 Firmware for robot mining project: 
   "daughter" nano, sent nano_net commands from "mother" mega.
*/
#include "un178_motor.h"
#include "communication_channel.h"
#include "nano_net.h"

#include "encoder.h"
#include "speed_controller.h"

typedef unsigned int milli_t;
milli_t milli;

namespace nano_net
{
  const uint8_t OUTPUT_PINS[12] = {2,3,4,5,6,7,8,9,10,11,12,A0};
  const uint8_t SENSOR_PINS[n_sensors] = { A1,A2,A3,A4,A5,A6 };
  
  un178_motor_single_t motors[n_motors]={
    un178_motor_single_t(11,12,A0),
    un178_motor_single_t(10,9,8),
    un178_motor_single_t(3,2,4),
    un178_motor_single_t(6,5,7),
  };

  encoder_t encoders[n_sensors]={
    encoder_t(A1),
    encoder_t(A2),
    encoder_t(A3),
    encoder_t(A4),
    encoder_t(A5),
    encoder_t(A6),
  };
  
  HardwareSerial &megaport(Serial);
  CommunicationChannel<HardwareSerial> mega(megaport);
  
  // Send error string up to mega (and on to main PC)
  void fatal(const char *why) {
    mega.pkt.write_packet(0xE,strlen(why),why);
  }

  nano_net_setup last_setup= /* Default Nano setup: */ {
    /* Motors: */ { 
    /* motor[0] */ 'T', 
    /* motor[1] */ 'T', 
    /* motor[2] */ 'T', 
    /* motor[3] */ 'T', 
    },
    /* Sensors: */ {
    /* sensor[0] */ 'C',  // uninitialized -> send 99 count
    /* sensor[1] */ 'C', 
    /* sensor[2] */ 'C', 
    /* sensor[3] */ 'C', 
    /* sensor[4] */ 'C', 
    /* sensor[5] */ 'C', 
    },
  };

  typedef speed_controller_t<4,100> speed_controller;
  speed_controller speed_controllers[n_motors];
  
  void got_setup() { // read last_setup and do it
    for (int m=0;m<n_motors;m++)
    {
      unsigned char mode=last_setup.motorMode[m];
      if (mode!='T') {
        int src=mode-'0';
        speed_controllers[m]=speed_controller(&encoders[src]);
      }
    }
  }
  
  nano_net_command last_command;
  
  void got_command() {
    
    // Blink LED via mega command
    digitalWrite(13,last_command.LED);
    
    // Set encoder directions from commanded speeds
    for (int s=0;s<n_sensors;s++) {
      unsigned char mode=last_setup.sensorMode[s];
      if (mode<sensorMode_motor_end) 
      { // sensor direction from motor direction
        int m=mode-'0';
        int dir=0;
        int command=last_command.speed[m];
        if (command<0) dir=-1;
        else if (command>0) dir=+1;
        encoders[s].last_dir=dir;
      }
    }
  }
  
  nano_net_sensors next_sensors;

  /*
  Scale speed from -100 .. +100
  to -254 .. +254 
  (Can't send full 255, the UN178 will shut off)
  */
  int16_t power_percent_to_pwm(int8_t speed)
  {
    if (speed>=100) return 254;
    if (speed<=-100) return -254;
    int16_t pwm = (int16_t(speed)*254)/100; // floor(254*(double(speed)/100));
    return pwm;
  }

  void send_motor_power(const un178_motor_single_t &motor, int8_t speed)
  {
    int16_t pwm = power_percent_to_pwm(speed);
    if(pwm==0)
      motor.stop();
    else if(pwm>0)
      motor.drive_green(pwm);
    else
      motor.drive_red(-pwm);
  }
  
  // Send last command to the motors
  void send_motors() 
  {
    for (int m=0;m<n_motors;m++)
    {
      int8_t speed=0;
      if (mega.is_connected && !last_command.stop) {
        int8_t command=last_command.speed[m];
        if (last_setup.motorMode[m]=='T')
        { // torque control
          speed=command;
        } 
        else 
        { // try speed control
          speed=speed_controllers[m].update(command,last_command.torque);
        }
      }
      send_motor_power(motors[m],speed);
    }
  }

  // Fill out next_sensors from the current encoder values
  void read_encoders()
  {
    next_sensors.ok=mega.is_connected;
    
    int raw=0;
    for (int s=0;s<n_sensors;s++) {
      encoders[s].read();
      if (encoders[s].value)
        raw|=(1<<s); // set that bit of raw
      
      int count=0;
      unsigned char mode=last_setup.sensorMode[s];
      if (mode<sensorMode_motor_end) 
        count=encoders[s].count_dir;
      else if (mode=='B')
        count=encoders[s].value;
      else 
        count=99;
      next_sensors.counts[s]=count;
    }
    next_sensors.raw=raw;

    // DEBUG HACK: report PID debugging as mine counts.
    // next_sensors.counts[1]=speed_controllers[0].debug;
    
    int stall=0;
    for (int m=0;m<n_motors;m++) {
      if (speed_controllers[m].stalled)
        stall|=1<<m; // set that bit of stall
    }
    next_sensors.stall=stall;
  }

  
  void handle_mega_packet(A_packet_formatter<HardwareSerial> &pkt,const A_packet &p)
  {
    if (p.command==0xB) { // boot setup command
      if (!p.get(last_setup)) fatal("n/!0xB");
      
      got_setup();
      next_sensors.nosetup=0;
      
      // Send them sensor data
      pkt.write_packet(0x5,sizeof(next_sensors),&next_sensors);
    }
    else if (p.command==0xC) { // mega command incoming
      if (!p.get(last_command)) fatal("n/!0xC");
      
      got_command();

      // Handle incoming motor powers
      send_motors();
      
      // Send them sensor data
      pkt.write_packet(0x5,sizeof(next_sensors),&next_sensors);
    }
    else if (p.command==0) { // ping request: requesting setup
      pkt.write_packet(0,0,0);
    }
    else fatal("n/cmd");
  }
};
using namespace nano_net;

void setup()
{
  next_sensors.nosetup=1; // need setup packet
  megaport.begin(115200);
  delay(500); // wait for mega to wake up
  // flush old data
  while (megaport.available()) megaport.read();
  mega.pkt.reset(); 
  
  for(int i=0;i<sizeof(OUTPUT_PINS);i++)
    pinMode(OUTPUT_PINS[i],OUTPUT);
  for(int s=0;s<n_sensors;s++)
    pinMode(SENSOR_PINS[s],INPUT);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  
  got_setup();
}

void loop()
{
  delay(4); // limit motor control loop to about 200Hz
  milli=micros()>>10;
  
  nano_net::read_encoders();
  next_sensors.heartbeat++;
  
  A_packet p;
  if (mega.read_packet(p))
     nano_net::handle_mega_packet(mega.pkt,p);
  
  nano_net::send_motors();
}
