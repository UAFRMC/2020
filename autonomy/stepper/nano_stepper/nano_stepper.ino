#include <AccelStepper.h>
#include <MultiStepper.h>

// defaults to AccelStepper::FULL4WIRE
// (4 pins) on 2, 3, 4, 5
AccelStepper myStep;

void setup()
{
  Serial.begin(115200);
  Serial.print("<Arduino Nano Ready>#");
  myStep.setMaxSpeed(300.0);
  myStep.setAcceleration(100.0);
  myStep.runToNewPosition(500);
  delay(500);
}

int deg2step(float deg)
{
  return 0.0;
}

float step2deg(int steps)
{
  return 0;
}

float stepperComm(float ang)
{
  int loc = deg2step(ang);
  
  myStep.runToNewPosition(loc); // blocking
  delay(1000); // needed?
  
  float current = step2deg(myStep.currentPosition());
  delay(500);
  
  return current;
}

void loop()
{
  String str, res;
  float newAng, curAng;
  
  while(Serial.available() > 0)
  {
    str = Serial.readString();

    if(str.length() > 0)
    {
      Serial.print("RECEIVED: New Angle (" + str + ")#");

      newAng = str.toFloat();

      if(newAng != step2deg(myStep.currentPosition()))
      {
        curAng = stepperComm(newAng);
      
        Serial.print("CONFIRM: Current Angle (");
        Serial.print(curAng, 2); Serial.print(")#");
      }
      
      str = "";
    }
  }
}
