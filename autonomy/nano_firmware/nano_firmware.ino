
#include "un178_motor.h"
#include "serial_packet.h"
#include "encoder.h"

typedef unsigned int milli_t;

const uint8_t A_PWM = 11;
const uint8_t A_DIR_1 = 12;
const uint8_t A_DIR_2 = A0;
const uint8_t B_PWM = 10;
const uint8_t B_DIR_1 = 9;
const uint8_t B_DIR_2 = 8;

const uint8_t OUTPUT_PINS[12] = {2,3,4,5,6,7,8,9,10,11,12,A0};
const uint8_t INPUT_PINS[6] = { A1,A2,A3,A4,A5,A6 };

namespace nano_net
{
  un178_motor_single_t left_motor_A (11,12,A0);
  un178_motor_single_t left_motor_B(10,9,8);
  un178_motor_single_t right_motor_A(3,2,4);
  un178_motor_single_t right_motor_B(6,5,7);

  encoder_t encoder_left_A(A1);
  encoder_t encoder_left_B(A2);
  encoder_t encoder_right_A(A3);
  encoder_t encoder_right_B(A4);
  encoder_t encoder_5(A5);
  encoder_t encoder_6(A6);

  typedef struct nano_net_command
  {
    unsigned char torgueMode:4; //1 per motor
    unsigned char stop:1;
    unsigned char pad1:3;
    unsigned char speed[4]; //Speed in percent. -100 (full red), 100 (full green)   
  } command;

  struct nano_net_sensors
  {
    unsigned char stall:4; // For each motor: 0-not stalled.  1-currently stalled.
    unsigned char pad1:4;
  
    unsigned char raw:6; // 6 raw encoder pins (for debugging, or for level-triggered stuff)
    unsigned char ok:1; // 0- no commands recently.  1- nano is receiving commands regularly.
    unsigned char pad2:1; // reserved for future use
  
    unsigned char counts[6]; // up/down counts for each motor encoder, plus monotonic counts for [5] and [6].
  }sensors_data;

 HardwareSerial &Mega = Serial; //serial to Arduino Mega

 int16_t power_percent_to_pwm(int8_t speed)
 {
  int16_t pwm = floor(255*(double(speed)/100));
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

 void read_encoders()
 {
  encoder_left_A.read();
  sensors_data.counts[0]= encoder_left_A.count_dir;
  encoder_left_B.read();
  sensors_data.counts[1]= encoder_left_B.count_dir;
  encoder_right_A.read();
  sensors_data.counts[2]= encoder_left_B.count_dir;
  encoder_right_B.read();
  sensors_data.counts[3]= encoder_right_B.count_dir;
  encoder_5.read();
  encoder_6.read();
    
 }
};

void setup()
{
  nano_net::Mega.begin(115200);
  for(int i=0;i<sizeof(OUTPUT_PINS);i++)
    pinMode(OUTPUT_PINS[i],OUTPUT);
  for(int i=0;i<sizeof(INPUT_PINS);i++)
    pinMode(INPUT_PINS[i],INPUT);
  pinMode(13,OUTPUT);
}

void loop()
{
  nano_net::send_motor_power(nano_net::left_motor_A,20);
  nano_net::send_motor_power(nano_net::left_motor_B,-100);
  digitalWrite(13,HIGH);
  delay(500);
  nano_net::send_motor_power(nano_net::right_motor_A,50);
  nano_net::send_motor_power(nano_net::right_motor_B,-50);
  digitalWrite(13,LOW);
  delay(500);
  nano_net::read_encoders();
  nano_net::Mega.println(nano_net::sensors_data.counts[1]);
  nano_net::Mega.print(nano_net::sensors_data.counts[2]);
  nano_net::Mega.print(nano_net::sensors_data.counts[3]);
  nano_net::Mega.print(nano_net::sensors_data.counts[4]);
  nano_net::Mega.print(nano_net::sensors_data.counts[5]);
  nano_net::Mega.print(nano_net::sensors_data.counts[6]);
  
}
