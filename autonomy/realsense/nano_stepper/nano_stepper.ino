/*
  Arduino Nano stepper motor driver:
    Takes serial commands, sends them to stepper.
*/
const int pin_dir=2; // direction pin
const int pin_step=3; // step mode
const int pin_enable=4; // power up stepper (false) / power down stepper (true)

const int cmd_scale=2; // scale serial byte up to step count

int stepper_current=0; // current position
int stepper_target=0; // target position
int stepper_vel=0; // current velocity (steps of acceleration)
int stepper_idle=10000; // idle counter
int stepper_home=-400; // home position (in steps)

void setup() {
  Serial.begin(57600);
  
  pinMode(pin_dir, OUTPUT);
  pinMode(pin_step, OUTPUT);
  pinMode(pin_enable, OUTPUT);
  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  
  digitalWrite(pin_dir, LOW);
  digitalWrite(pin_enable,HIGH); // power off stepper until we get a command
  digitalWrite(pin_step, LOW);
}

void loop() {
  // Commands: binary unsigned char, absolute stepper target position.
  // Except: 0xff = home left
  // FIXME: make home command block other serial updates
  if (Serial.available()) {
    stepper_target=(int)(unsigned char)Serial.read();
    if (stepper_target==0xff) { // special home command
      stepper_target=2*stepper_home;
    }
    else { // normal move command
      stepper_target*=cmd_scale;
    }
  }
  
  if (stepper_current<stepper_home) 
  { // now at home position
    stepper_target=stepper_current=0; 
  }
  
  // Continually report current position as a binary int.
  if (stepper_current>=0) { // position valid
    int pos=stepper_current/cmd_scale;
    if (pos<0) pos=0;
    if (pos>250) pos=250;
    Serial.write((char)(int)pos);
  }
  
  int del=stepper_target-stepper_current;
  if (del==0) { // we're at the target
    const int stepper_idle_period=20; // ms per idle step
    const int stepper_idle_timeout=600; // in seconds 
    if (stepper_idle/(1000/stepper_idle_period)>stepper_idle_timeout) 
    { // Turn off stepper, to save power
      digitalWrite(pin_enable,true); 
      digitalWrite(13, LOW);
    } else {
      stepper_idle++;
    }
    delay(stepper_idle_period);
    return;
  }
  
  // move required
  stepper_idle=0;
  digitalWrite(13, HIGH);
  digitalWrite(pin_enable,LOW); // power up stepper
  if (del<0) { // negative move
    digitalWrite(pin_dir,HIGH); 
    del=-del;
    stepper_current--;
  }
  else { // positive move
    digitalWrite(pin_dir,LOW); 
    stepper_current++;
  }

  // Calculate acceleration
  const int min_ms=12; // starting ms/step
  const int stepper_vel_cap=6; // maximum speed
  if (stepper_vel>=del) { // slowdown phase
    stepper_vel--;
  } else if (stepper_vel>=stepper_vel_cap) { // coast phase
    
  } else { // speedup phase
    stepper_vel++;
  }
  int denom=stepper_vel;
  if (denom<1) denom=1;
  int wait_ms=(min_ms*stepper_vel_cap)/denom;
  
  digitalWrite(pin_step, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(wait_ms);              // wait for a second
  digitalWrite(pin_step, LOW);    // turn the LED off by making the voltage LOW
  delay(wait_ms);              // wait for a second
}


