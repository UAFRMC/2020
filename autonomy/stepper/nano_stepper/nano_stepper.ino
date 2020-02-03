void setup()
{
  Serial.begin(115200);
  Serial.print("<Arduino Nano Ready>#");
  delay(500);
}

float stepperComm(float ang)
{
  // send ang to stepper
  // wait for stepper to finish
  // receive current angle from stepper
  float current = 1.155;
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
    Serial.print(str + "#");

    if(str.length() > 0)
    {
      Serial.print("RECEIVED: New Angle (" + str + ")#");

      newAng = str.toFloat();
      curAng = stepperComm(newAng);
      
      Serial.print("CONFIRM: Current Angle ("); Serial.print(curAng, 2); Serial.print(")#");
      
      str = "";
    }
  }
}
