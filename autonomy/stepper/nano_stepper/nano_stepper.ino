#include <AccelStepper.h>
#include <MultiStepper.h>

#define STEPS_PER_MREV 200 // motor revolution; what are the specs?
#define STEPS_PER_OREV 600 // output revolution; what are the specs?

AccelStepper myStep(AccelStepper::DRIVER, 2, 3);

void setup()
{
  Serial.begin(115200);
  Serial.print("<Arduino Nano Ready>#");
  myStep.setMaxSpeed(150.0);
  myStep.setAcceleration(10.0);
  myStep.moveTo(0);
  while(true)
  {
    myStep.runToNewPosition(150);
    delay(1000);
    myStep.runToNewPosition(0);
    delay(1000);
  }
  delay(500);
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
  
  myStep.runToNewPosition(loc); // blocking
  //delay(1000); // needed?

  Serial.print("\nCurrent Position: ");
  Serial.println(myStep.currentPosition());
  long current = step2deg(myStep.currentPosition());
  delay(500);
  
  return current;
}

long newAng = 0.0, curAng;

void loop()
{
  //String str, res;
  //long newAng, curAng;

  newAng += 10.0;
  
  //while(Serial.available() > 0)
  //{
  //  str = Serial.readString();

  //  if(str.length() > 0)
  //  {
  //    Serial.print("RECEIVED: New Angle (" + str + "°)#");

  //    newAng = str.toLong();

  //    if(newAng != step2deg(myStep.currentPosition()))
  //    {
        Serial.print("\nTold stepper to move: ");
        Serial.println(newAng);
        curAng = stepperComm(newAng);
      
  //      Serial.print("CONFIRM: Curr Angle (");
  //      Serial.print(curAng, 2); Serial.print("°)#");
        Serial.print("\nShould have moved: ");
        Serial.println(curAng);
  //    }
  //    
  //    str = "";
  //  }
  //}
}
