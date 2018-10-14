void drive(const int16_t speed)
{
  digitalWrite(3,LOW);
  digitalWrite(5,LOW);
  
  if(speed<0)
    analogWrite(3,abs(speed));
  else
    analogWrite(5,abs(speed));
}

void setup()
{
  pinMode(11,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(A1,INPUT);
  
  digitalWrite(11,HIGH);
  digitalWrite(3,LOW);
  digitalWrite(5,LOW);
  
  Serial.begin(57600);
  Serial.println("Running");
}

bool edge=false;
const int threshold=500; //phototransitor light threshold (on/off-ish).
unsigned long last_fall=0; //last falling edge in microseconds.
long fall_target=100000; //microseconds between falling "edges".
unsigned long kickstart_time=fall_target*3;
unsigned long timer=millis()+500;
void loop()
{
  unsigned long new_fall=micros();
  
  int sensor=analogRead(A1);
  if(millis()>=timer)
  {
    Serial.print("s:  ");
    Serial.println(sensor);
    timer=millis()+500;
  }
  
  if(!edge&&sensor>threshold)
  {
    edge=!edge;
  }
  else if(edge&&sensor<threshold)
  {
    edge=!edge;
    unsigned long fall_diff=new_fall-last_fall;
    
    long target_diff=fall_diff-fall_target;
    long motor_speed=128+target_diff*256/fall_target;
    
    if(motor_speed<0)
      motor_speed=0;
    if(motor_speed>255)
      motor_speed=255;
      
    Serial.print("fd td ms  ");
    Serial.print(fall_diff);
    Serial.print("  ");
    Serial.print(target_diff);
    Serial.print("  ");
    Serial.println(motor_speed);

    drive(motor_speed);
    
    last_fall=new_fall;
  }
  
  if(new_fall-last_fall>kickstart_time)
  {
    Serial.println("Kickstart!");
    last_fall=new_fall;
    drive(255);
  }
}
