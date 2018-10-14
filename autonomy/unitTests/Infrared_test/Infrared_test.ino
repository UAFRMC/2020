/*Sketch to test reflectance using an emiter-detector pair.
Emitter is a 1.2VDc infrared LED 
Written for Aurora Robotics unit tests
05/03/2015
Public Domain
*/

void setup()
{
  pinMode(12, OUTPUT);
  digitalWrite(12,HIGH);
 Serial.begin(  57600);
}

void loop()
{
  int reading1 = analogRead(A0); // Detector on Pin A0
  int reading2 = analogRead(A2); //Detector on Pin A2
  Serial.print(millis());
  String readingA0 = " A0: ";
  readingA0 += reading1;
  String readingA2 = "  A2: ";
  readingA2 += reading2; 
  //Serial.println("A0 : ");
  Serial.print(readingA0);
  //Serial.println("A2 : ");
  Serial.println(readingA2);
  delay(10);
}

