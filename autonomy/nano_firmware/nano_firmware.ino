/*
 Firmware for robot mining project: 
   "daughter" nano, sent nano_net commands from "mother" mega.
*/
#include "un178_motor.h"
#include "communication_channel.h"
#include "nano_net.h"

#include "encoder.h"

typedef unsigned int milli_t;
milli_t milli;

namespace nano_net
{
  const uint8_t OUTPUT_PINS[12] = {2,3,4,5,6,7,8,9,10,11,12,A0};
  const uint8_t SENSOR_PINS[n_sensors] = { A1,A2,A3,A4,A5,A6 };

  /*
  un178_motor_single_t left_motor_A(11,12,A0);
  un178_motor_single_t left_motor_B(10,9,8);
  un178_motor_single_t right_motor_A(3,2,4);
  un178_motor_single_t right_motor_B(6,5,7);
  */
  
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

  nano_net_setup last_setup;
  nano_net_command last_command;
  nano_net_sensors next_sensors;

 /*
   Scale speed from -100 .. +100
   to -255 .. +255 
 */
 int16_t power_percent_to_pwm(int8_t speed)
 {
  if (speed>=100) return 255;
  if (speed<=-100) return -255;
  int16_t pwm = (int16_t(speed)*255)/100; // floor(255*(double(speed)/100));
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
 void send_motors() 
 {
  for (int m=0;m<n_motors;m++)
    send_motor_power(motors[m],last_command.speed[m]);
 }

 void read_encoders()
 {
   next_sensors.raw=0;
   for (int s=0;s<n_sensors;s++) {
     encoders[s].read();
     if (encoders[s].value)
       next_sensors.raw|=(1<<s); // set that bit of raw
     next_sensors.counts[s]=encoders[s].count_dir;
   }
 }



  
  void handle_mega_packet(A_packet_formatter<HardwareSerial> &pkt,const A_packet &p)
  {
    if (p.command==0xB) { // boot setup command
      if (!p.get(last_setup)) fatal("n/!0xB");
      
      // Send them back a ping request
      pkt.write_packet(0,0,0);
    }
    else if (p.command==0xC) { // mega command incoming
      if (!p.get(last_command)) fatal("n/!0xC");

      // Handle incoming motor powers
      send_motors();
      
      // Send them sensor data
      pkt.write_packet(0x5,sizeof(next_sensors),&next_sensors);
    }
    else if (p.command==0) { // ping request: requesting data
      pkt.write_packet(0x5,sizeof(next_sensors),&next_sensors);
    }
    else fatal("n/cmd");
  }
};
using namespace nano_net;

void setup()
{
  megaport.begin(115200);
  delay(500); // wait for mega to wake up
  mega.pkt.write_packet(0xB,0,0); // send mega a boot message
  
  for(int i=0;i<sizeof(OUTPUT_PINS);i++)
    pinMode(OUTPUT_PINS[i],OUTPUT);
  for(int s=0;s<n_sensors;s++)
    pinMode(SENSOR_PINS[s],INPUT);
  pinMode(13,OUTPUT);
}

void loop()
{
  milli=micros()>>10;
  
  nano_net::read_encoders();
  
  A_packet p;
  if (mega.read_packet(p))
     nano_net::handle_mega_packet(mega.pkt,p);
  
  if (mega.is_connected) {
    digitalWrite(13,HIGH);
  }
  else {
    digitalWrite(13,LOW);
  }
  
  delay(1);
/*
  nano_net::send_motor_power(nano_net::left_motor_A,20);
  nano_net::send_motor_power(nano_net::left_motor_B,-100);
  nano_net::send_motor_power(nano_net::right_motor_A,50);
  nano_net::send_motor_power(nano_net::right_motor_B,-50);
  for (int s=0;s<nano_net::n_sensors;s++)
    nano_net::Mega.print(nano_net::next_sensors.counts[s]);
  nano_net::Mega.println();
*/
  
}
