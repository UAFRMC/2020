/**
 * Aurora Robotics "Autonomy" firmware - 2015 BTS Version
 * For Arduino Mega
 */
#include "robot_base.h" /* classes shared with PC, and across network */
#include "communication_channel.h" /* CYBER-Alaska packetized serial comms */
#include "nano_net.h"

/* Subtle: I don't need a namespace, but this at least disables the Arduino IDE's
utterly broken prototype generation code in the Arduino IDE. */
namespace aurora {

// Call this function frequently--it's for minimum-latency operations
void low_latency_ops();


// All PC commands go via this (onboard USB) port
HardwareSerial &PCport=Serial; // direct PC
CommunicationChannel<HardwareSerial> PC(PCport);

// Send this debug/error string up to main PC
void send_PC_debug(const char *why) {
  PC.pkt.write_packet(0xE,strlen(why),why);
}

void fatal(const char *why) {
  send_PC_debug(why);
}

CommunicationChannel<HardwareSerial> nanos[nano_net::n_nanos]={
  CommunicationChannel<HardwareSerial>(Serial1),
  //CommunicationChannel<HardwareSerial>(Serial3),
};


// FIXME: match up these with actual hardware.
//   And send commands / read sensors from the right spots.
nano_net::nano_net_setup nano_setup[nano_net::n_nanos] = {
  /* Nano 0: on Serial1, on the back of the robot */ {
    /* Motors: */ {
    /* motor[0] */ '0', // drive right 
    /* motor[1] */ '1', // drive left
    /* motor[2] */ '2', // conveyor belt
    /* motor[3] */ '3', // conveyor raise
    },
    /* Sensors: */ {
    /* sensor[0] */ '0', // drive right
    /* sensor[1] */ '1', // drive left
    /* sensor[2] */ '2', // conveyor
    /* sensor[3] */ 'B',
    /* sensor[4] */ 'B', 
    /* sensor[5] */ 'C',
    },
  },
//  /* Nano 1: on Serial3, left side of robot */ {
//    /* Motors: */ {
//    /* motor[0] */ 'T', // drive left
//    /* motor[1] */ 'T', // roll bag
//    /* motor[2] */ 'T', // UNUSED
//    /* motor[3] */ 'T', // dump up/down
//    },
//    /* Sensors: */ {
//    /* sensor[0] */ 'B', // drivetrain
//    /* sensor[1] */ 'B', // bag roll
//    /* sensor[2] */ 'B', // bag limit low
//    /* sensor[3] */ 'B', // bag limit high
//    /* sensor[4] */ 'B', // back-up left?
//    /* sensor[5] */ 'C',
//    },
//  },
};
nano_net::nano_net_sensors nano_sensors[nano_net::n_nanos];
nano_net::nano_net_command nano_commands[nano_net::n_nanos];





/***************** Robot Control Logic ****************/

long mine_last=0; // millis() at last mining motion check
long stall_last=0; // timer for stall detection average
int sum_stalled=0, sum_stall_count=0;


// Robot's current state:
robot_arduino robot;

// Read all robot sensors into robot.sensor
void read_sensors(void) {

  robot.sensor.battery=0;
  low_latency_ops();

  /*
  robot.sensor.Mstall=encoder_mining_right.stalled; //encoder_mining_left.stalled || encoder_mining_right.stalled;
  robot.sensor.DRstall=encoder_DL1.stalled;
  robot.sensor.DLstall=encoder_DR1.stalled;
  robot.sensor.limit_top=limit_top.count_mono;
  robot.sensor.limit_bottom=limit_bottom.count_mono;
  */
  robot.sensor.DR1count = nano_sensors[0].counts[0];
  robot.sensor.DRstall = nano_sensors[0].stall&(1<<0);

//  robot.sensor.McountL = nano_sensors[0].counts[1];
//  robot.sensor.Mstall = (nano_sensors[0].stall>>1)&1;

  robot.sensor.DL1count = nano_sensors[0].counts[1];
  robot.sensor.DLstall = nano_sensors[0].stall&(1<<1);

  robot.sensor.Rcount = 255-nano_sensors[0].counts[2];

  robot.sensor.limit_bottom = nano_sensors[1].counts[4];
  robot.sensor.limit_top = nano_sensors[1].counts[2];

  robot.sensor.heartbeat=milli;

  robot.sensor.encoder_raw=int(nano_sensors[0].raw) | (int(nano_sensors[1].raw)<<nano_net::n_sensors);
  robot.sensor.stall_raw=int(nano_sensors[0].stall) | (int(nano_sensors[1].stall)<<nano_net::n_sensors);
}

// Scale Sabertooth style 0..64..127 to -100 .. 0 .. +100
signed char scale_from_64(unsigned char speed_64) {
  return (int(speed_64)*100)/64-100;
}


// Send current power values to the motors
void send_motors(void)
{
  for (int n=0;n<nano_net::n_nanos;n++)
  {
    nano_commands[n].stop = !PC.is_connected;
    nano_commands[n].torque = robot.power.torqueControl;
    nano_commands[n].LED = ((milli%1024)<200); // mega tells nanos to blink
  }


  nano_commands[0].speed[0]=+scale_from_64(robot.power.right);
  nano_commands[0].speed[1]=-scale_from_64(robot.power.left);
  nano_commands[0].speed[2]=scale_from_64(robot.power.roll);
  nano_commands[0].speed[3]=scale_from_64(robot.power.conveyor_raise);
    
  //nano_commands[0].speed[2]=-scale_from_64(robot.power.roll);
//  nano_commands[1].speed[2]=0;
//  nano_commands[1].speed[3]=-scale_from_64(robot.power.dump);


  /*
  if(robot.power.motorControllerReset!=0)
    digitalWrite(bts_enable_pin,LOW);
  else
    digitalWrite(bts_enable_pin,HIGH);

  int drivePower=100;
  if(robot.power.high)
  {
    drivePower=255;
  }
  motor_drive_left.max_power=drivePower;
  motor_drive_right.max_power=drivePower;

  int left1=encoder_DL1.update(robot.power.left,robot.power.torqueControl==0);
  send_motor_power(left1,motor_drive_left,encoder_DL1);
  set_direction(left1,encoder_DL2);

  int right1=encoder_DR1.update(robot.power.right,robot.power.torqueControl==0);
  send_motor_power(right1,motor_drive_right,encoder_DR1);
  set_direction(right1,encoder_DR2);


  int roll = encoder_R.update(robot.power.roll,robot.power.torqueControl==0);
  send_motor_power(robot.power.roll,motor_box,encoder_R);

  // ***** Automated mining at fixed rate ***** //
  int mine_target=robot.power.mine;

  // Update left and right speed controllers
  const int mining_rpm_scalar = 100;
  //int mine_target_left = encoder_mining_left.update(mine_target,robot.power.mineMode!=0||robot.power.mineDump!=0);
  int mine_target_left=encoder_mining_left.update(mine_target,robot.power.torqueControl==0,mining_rpm_scalar);
  //int mine_target_right = encoder_mining_right.update(mine_target,robot.power.mineMode!=0||robot.power.mineDump!=0);
  int mine_target_right=encoder_mining_right.update(mine_target,robot.power.torqueControl==0,mining_rpm_scalar);

  // Send lowest speed set by speed controllers
  // TODO: FIX FOR BACKWARDS DRIVE
  int mine_target_final;
  //if(mine_target_left <= mine_target_right) {
  //  mine_target_final = mine_target_left;
  //}
  //else {
    mine_target_final = mine_target_right;
  //}
  send_motor_power(mine_target_final,motor_mining,encoder_mining_right);
  // ********** //

  // Send power to linears
  motor_front_linear.drive(robot.power.head_extend);
  motor_side_linears.drive(robot.power.dump);
  */

}

// Structured communication with PC:
void handle_packet(A_packet_formatter<HardwareSerial> &pkt,const A_packet &p)
{
  if (p.command==0x7) { // motor power commands
    low_latency_ops();
    if (!p.get(robot.power)) { // error
      pkt.write_packet(0xE,0,0);
    }
    else
    { // got power request successfully: read and send sensors
      low_latency_ops(); /* while reading sensors */
      read_sensors();
      low_latency_ops();
      pkt.write_packet(0x3,sizeof(robot.sensor),&robot.sensor);
      robot.sensor.latency=0; // reset latency metric
      low_latency_ops();
    }
  }
  else if (p.command==0) { // ping request
    pkt.write_packet(0,p.length,p.data); // ping reply
  }
  else fatal("PC/cmd?");
}


// Comms to the nano
void handle_nano_packet(A_packet_formatter<HardwareSerial> &pkt,int n,const A_packet &p)
{
  if (p.command==0x5) { // sensor command incoming
    if (!p.get(nano_sensors[n])) { // error (like a packet size mismatch)
      fatal("n-m/!0x5");
    }
    else
    { // got sensor data successfully
      if (nano_sensors[n].nosetup) { // nano needs setup packet
        aurora::send_PC_debug("n/boot");
        pkt.write_packet(0xB,sizeof(nano_setup[n]),&nano_setup[n]);
      }
      read_sensors();
      //pkt.write_packet(0xC,sizeof(nano_commands[n]),&nano_commands[n]);
    }
  }
  else if (p.command==0xE) { // nano hit error
    aurora::send_PC_debug("n/err");
    fatal((const char *)p.data);
  }
  else if (p.command==0) { // ping request
    aurora::send_PC_debug("n/ping");

    // Send a command packet to start reports coming
    pkt.write_packet(0xC,sizeof(aurora::nano_commands[n]),&aurora::nano_commands[n]);
  }
  else fatal("n-m/cmd?");
}

/**** Low latency (sub-millisecond) timing section.
 * We need this for maximum accuracy in our encoder counts.
 */

milli_t last_milli=0;
void low_latency_ops() {
  unsigned long micro=micros();
  milli=micro>>10; // approximately == milliseconds

/*
  //Encoder for mining motor
//  encoder_M.read();
//  // robot.sensor.Mspeed=encoder_M.period;
//  robot.sensor.Mcount=encoder_M.count_mono;

  // Read encoders for left and right mining motors.
  encoder_mining_left.read();
  robot.sensor.McountL = encoder_mining_left.count_mono;
  encoder_mining_right.read();
  robot.sensor.McountR = encoder_mining_right.count_mono;
*/


  // Update latency counter
  unsigned int latency=milli-last_milli;
  if (latency>=1024) latency=1023;
  if (robot.sensor.latency<latency) robot.sensor.latency=latency;
  last_milli=milli;
}


}; // end namespace aurora

void setup()
{
  aurora::PCport.begin(57600); // Control connection to PC via USB

  for (int n=0;n<nano_net::n_nanos;n++)
    aurora::nanos[n].backend.begin(115200);

  aurora::send_PC_debug("mega/boot");

  // Our ONE debug LED!
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);


/*
  // BTS Enable Pin (Controls all pins)
  pinMode(aurora::bts_enable_pin,OUTPUT);
  digitalWrite(aurora::bts_enable_pin,HIGH);
*/
}


milli_t milli;
milli_t next_milli_send=0;
void loop()
{
  aurora::low_latency_ops();

  A_packet p;
  if (aurora::PC.read_packet(p)) aurora::handle_packet(aurora::PC.pkt,p);
  if (!(aurora::PC.is_connected)) aurora::robot.power.stop(); // disconnected?

  for (int n=0;n<nano_net::n_nanos;n++)
    if (aurora::nanos[n].read_packet(p))
      aurora::handle_nano_packet(aurora::nanos[n].pkt,n,p);

  if (milli-next_milli_send>=5)
  { // Send commands to motors (via nanos)
    aurora::send_motors();

    for (int n=0;n<nano_net::n_nanos;n++)
      aurora::nanos[n].pkt.write_packet(0xC,sizeof(aurora::nano_commands[n]),&aurora::nano_commands[n]);

    next_milli_send=milli;
  }
}
