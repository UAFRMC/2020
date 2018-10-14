//analogTest.ino
//Test sensors on pins 34,36,50,52
//Made to test Bryant Klug's custom shield for Aurora Robotics
//Written for Aurora Robotics. (UAF RMC 2017)
//Arsh Chauhan
//02/26/2017
//Public Domain  

const int num_pins=6;
int pins1[num_pins]={42,40,44,38,46,36};
int pins2[num_pins]={48,34,50,32,52,30};

void setup() {
  
  for(int ii=0;ii<num_pins;++ii)
    pinMode(pins1[ii],INPUT);
  for(int ii=0;ii<num_pins;++ii)
    pinMode(pins2[ii],INPUT);

  Serial.begin(9600);

}

void loop() {
for( int ii=0;ii<num_pins;++ii)
    Serial.print(digitalRead(pins1[ii]));
    Serial.print(' ');
for( int ii=0;ii<num_pins;++ii)
    Serial.print(digitalRead(pins2[ii]));
 
 Serial.println();

 delay(50);
 }
