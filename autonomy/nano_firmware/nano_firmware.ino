
#include "un178_motor.h"

const uint8_t A_PWM = 18;
const uint8_t A_DIR_1 = 19;
const uint8_t A_DIR_2 = 25;
const uint8_t B_PWM = 27;
const uint8_t B_DIR_1 = 31;
const uint8_t B_DIR_2 = 1;

un178_motor_t my_motor(A_PWM, A_DIR_1, A_DIR_2, B_PWM, B_DIR_1, B_DIR_2);

void setup()
{
  pinMode(A_PWM,OUTPUT);
  pinMode(B_PWM,OUTPUT);
  pinMode(A_DIR_1,OUTPUT);
  pinMode(A_DIR_2,OUTPUT);
  pinMode(B_DIR_1,OUTPUT);
  pinMode(B_DIR_2,OUTPUT);
  pinMode(13,OUTPUT);
}

void loop()
{
  my_motor.motor_A.drive_green(254);
  digitalWrite(13,HIGH);
  delay(500);
  my_motor.motor_A.drive_red(254);
  digitalWrite(13,LOW);
  delay(500);
}
