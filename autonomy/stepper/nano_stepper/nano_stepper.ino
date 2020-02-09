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
  myStep.moveTo(0);
  delay(500);
}

float stepperComm(float ang)
{
  myStep.runToNewPosition(ang);
  delay(1000);
  float current = myStep.currentPosition();
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
      curAng = stepperComm(newAng);
      
      Serial.print("CONFIRM: Current Angle (");
      Serial.print(curAng, 2); Serial.print(")#");
      
      str = "";
    }
  }
}
