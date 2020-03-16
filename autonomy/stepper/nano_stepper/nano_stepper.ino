#include <AccelStepper.h>
#include "serial_byte.h"


#define STEPS_PER_OREV 600 // output revolution: 200 steps/rev * 3x gear-down

AccelStepper myStep(AccelStepper::DRIVER, 2, 3);

void setup()
{
  Serial.begin(115200);
  Serial.write(BYTE_OK); /* send PC wakeup message at startup */

  pinMode(13,OUTPUT); // debug pin
  digitalWrite(13,LOW); // initially off
  
  pinMode(4,INPUT_PULLUP); // limit switch pin #1
  
  myStep.setMaxSpeed(1000.0); // 2000 doesn't lose steps
  myStep.setAcceleration(10000.0); // 40000 doesn't lose steps
  myStep.moveTo(0);

//  while(true)
//  {
//    myStep.runToNewPosition(600/4); // turn 1/4 rev == 90 degrees
//    delay(500);
//    myStep.runToNewPosition(0);
//    delay(500);
//  }
}

long deg2step(long degs)
{
  long steps = STEPS_PER_OREV * degs / 360;
  return steps;
}

long step2deg(long steps) // rounds down to whole degree
{
  long degs = steps * 360 / STEPS_PER_OREV;
  return degs;
}

long stepperComm(long ang)
{
  long loc = deg2step(ang);
  
  if (loc != myStep.currentPosition())
  {
    myStep.runToNewPosition(loc); // blocking
  }
  
  long current = step2deg(myStep.currentPosition());
  
  return current;
}

// Current stepper angle, in degrees
long curAng=0;

void loop()
{
  if (Serial.available() > 0)
  {
    serial_byte b = Serial.read();

    if (b==BYTE_HOME)
    {
      int seeker = 0;
      digitalWrite(13, HIGH); // light LED while moving
      while(digitalRead(4)) // while limit switch not activated
      {
        //Serial.print(digitalRead(4));
        seeker -= 5;
        curAng = stepperComm(seeker); // go left 5 degs at a time
      }
      myStep.stop(); // stop at fast as possible
      digitalWrite(13, LOW); // LED off again
      Serial.write(deg2byte(0)); // hit switch, must be zero
      Serial.write(BYTE_HOME);
    }
    else if (b<=BYTE_MAX_ANGLE)
    {
      digitalWrite(13, HIGH); // light LED while moving
      long newAng = byte2deg(b); 
      curAng = stepperComm(newAng);
      digitalWrite(13, LOW); // LED off again
      Serial.write(deg2byte(curAng));
    }
    else
    { /* unknown command */
      Serial.write(BYTE_ERROR+0);
    }
  }
}
