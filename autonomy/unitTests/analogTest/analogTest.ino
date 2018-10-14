//analogTest.ino
//Test sensors on A0-A5 with mode INPUT_PULLUP
//Written for Aurora Robotics. (UAF RMC 2016)
//Arsh Chauhan
//05/01/2016
//Public Domain  




void setup() {
  
  for( int i=A0; i<=A5; i++)
  {
    pinMode(i,INPUT_PULLUP);
  }

  Serial.begin(9600);

}

int values[6];
int oldvalues[6];
int counts[6]={0};

void loop() {
  
 for( int i=0; i<6; i++)
 {
    oldvalues[i] = values[i];
    values[i] = digitalRead(A0 + i);
    if(values[i] != oldvalues[i])
    {
      counts[i]+=1;
    }
    Serial.print(counts[i]);
    Serial.print (" ");
 }
 Serial.println();

 delay(50);
 }
