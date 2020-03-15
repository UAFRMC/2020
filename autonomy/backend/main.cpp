/**
  Aurora Robotics Backend Code

  Orion Sky Lawlor, lawlor@alaska.edu, 2014-03-23 (Public Domain)
*/
#define AURORA_IS_BACKEND 1

#include <iostream>
#include <fstream>
#include <cmath>
#include <thread>
#include <mutex>

#include "gridnav/gridnav_RMC.h"

#include "aurora/robot.h"
#include "aurora/robot.cpp"
#include "aurora/display.h"
#include "aurora/network.h"
#include "aurora/ui.h"
#include "aurora/robot_serial.h"

#include <SOIL/SOIL.h>

#include "ogl/event.cpp"
#include "osl/socket.cpp"

#include "osl/porthread.h" /* for threading */
#include "osl/porthread.cpp"

#include "aurora/simulator.h"
#include <iostream>


#include "aurora/lunatic.h"

// Crude global variables for lunatic data exchange
MAKE_exchange_drive_encoders();
MAKE_exchange_stepper_request();
MAKE_exchange_plan_target();
MAKE_exchange_drive_commands();
//Needed for localization
MAKE_exchange_plan_current();
aurora::robot_loc2D currentLocation;

bool show_GUI=true;
bool simulate_only=false; // --sim flag
bool should_plan_paths=true; // --noplan flag
bool driver_test=false; // --driver_test, path planning testing

bool nodrive=false; // --nodrive flag (for testing indoors)

/* Bogus path planning target when we don't want any path planning to happen. */
aurora::robot_navtarget no_idea_loc(0.0f,0.0f,0.0f);

/** X,Y field target location where we drive to, before finally backing up */
aurora::robot_navtarget dump_target_loc(field_x_size/2,field_y_trough_center,0,
    aurora::robot_navtarget::DONTCARE,10.0,90.0); // get back to starting area area
aurora::robot_navtarget dump_align_loc(field_x_trough_edge,dump_target_loc.y,field_angle_trough,
    10.0,30.0,5.0); // final alignment

/** X,Y field target location that we target for mining */
aurora::robot_navtarget mine_target_loc(field_x_size/2,field_y_size-45,90,
    aurora::robot_navtarget::DONTCARE, 30.0,80.0);

/* Convert this unsigned char difference into a float difference */
float fix_wrap256(unsigned char diff) {
  if (diff>128) return diff-256;
  else return diff;
}

int last_Mcount=0;
int speed_Mcount=0;
float smooth_Mcount=0.0;

/**
  This class is used to localize the robot
*/
class robot_locator {
public:
  /** Merged location */
  robot_localization merged;
};


/**
 This class represents everything the back end knows about the robot.
*/
class robot_manager_t
{
public:
  robot_base robot; // overall integrated current state

  robot_locator locator; // localization
  robot_telemetry telemetry; // next-sent telemetry value
  robot_command command; // last-received command
  robot_comms comms; // network link to front end
  robot_ui ui; // keyboard interface
  robot_realsense_comms realsense_comms;

  robot_serial arduino;

  robot_simulator sim;

  robot_manager_t() {
    // HACK: zero out main structures.
    //  Can't do this to objects with internal parts, like comms or sim.
    memset(&robot,0,sizeof(robot));
    memset(&telemetry,0,sizeof(telemetry));
    memset(&command,0,sizeof(command));
    robot.sensor.limit_top=1;
    robot.sensor.limit_bottom=1;

    // Start simulation in random real start location
    sim.loc.y=80.0;
    sim.loc.x= (rand()%10)*20.0+100.0;
    sim.loc.angle=((rand()%8)*8)/360;
    sim.loc.percent=50.0;

  }

  // Do robot work.
  void update(void);
  
  
  void point_camera(int target) {
    if (simulate_only) {
      telemetry.autonomy.markers.beacon=target;
    }
    exchange_stepper_request.write_begin().angle=target;
    exchange_stepper_request.write_end();
  }


private:

  /* Use OpenGL to draw this robot navigation grid object */
  template <class grid_t>
  void gl_draw_grid(grid_t grid)
  {
    glPointSize(4.0f);
    glBegin(GL_POINTS);
    for (int y=0;y<rmc_navigator::GRIDY;y++)
    for (int x=0;x<rmc_navigator::GRIDX;x++)
    {
      int height=grid.at(x,y);
      if (height>0) {
        if (height>50) glColor3f(0.0f,1.0f,1.0f); // cyan trough / walls
        else if (height<15) glColor3f(1.0f,0.5f,1.0f); // purple very short
        else if (height<20) glColor3f(1.0f,0.0f,0.0f); // red short-ish
        else  glColor3f(1.0f,1.0f,1.0f); // white tall
        glVertex2f(
          rmc_navigator::GRIDSIZE*x,
          rmc_navigator::GRIDSIZE*y);
      }
    }
    glEnd();
  }

  // Autonomy support:
  double cur_time; // seconds since start of backend program
  double state_start_time; // cur_time when we entered the current state
  double mine_start_time; // cur_time when we last started mining
  double autonomy_start_time; // cur_time when we started full autonomy
  
  // If true, the mining head has been extended
  bool mining_head_extended=false;
  // If true, the mining head is down in the dirt
  bool mining_head_lowered=true;

  robot_state_t last_state;

  // Enter a new state (semi)autonomously
  void enter_state(robot_state_t new_state)
  {
    // Flush old planned path on state change
    exchange_plan_target.write_begin()=no_idea_loc;
    exchange_plan_target.write_end();

    if (new_state==state_setup_raise) { autonomy_start_time=cur_time; }
    // if(!(robot.autonomous)) { new_state=state_drive; }

    // Log state timings to dedicate state timing file:
    static FILE *timelog=fopen("timing.log","w");
    fprintf(timelog,"%4d spent %6.3f seconds in %s\n",
      (int)(cur_time-autonomy_start_time),
      cur_time-state_start_time, state_to_string(robot.state));
    fflush(timelog);

    // Make state transition
    last_state=robot.state; // stash old state
    robot.state=new_state;
    robotPrintln("Entering new state %s",state_to_string(robot.state));
    state_start_time=cur_time;
  }

  // Advance autonomous state machine
  void autonomous_state(void);

  // Raw robot.power levels for various speeds
  enum {
    power_full_fw=127, // forward
    power_stop=64,
    power_full_bw=1, // backward
  };

  // Dump bucket encoder target a/d values


  // Limit this value to lie in this +- range
  double limit(double v,double range) {
    if (v>range) return range;
    if (v<-range) return -range;
    else return v;
  }

  // Run autonomous mining, if possible
  bool tryMineMode(void) {
    //if (drive_posture()) {    
    robot.power.mine=120; // TUNE THIS mining head rate
    robot.power.dump=64-8; // TUNE THIS lowering rate
    robot.power.mineMode = true; // Start PID based mining
    mining_head_lowered=true;
    
    
    return true;
  }

  // Set the mining head linear and dump linear to natural driving posture
  //  Return true if we're safe to drive
  bool drive_posture() {
    if (!mining_head_extended && (cur_time-state_start_time <10))
      robot.power.head_extend = 127;
    mining_head_extended = true;
    if(mining_head_lowered && cur_time-state_start_time <10)
      robot.power.dump = 127;
    if (sim.bucket>0.9) { // we're back up in driving range
      mining_head_lowered=false;
    }

    return true; // Kept for compatiiblity
  }

  // Autonomous driving rate:
  //  Returns 0-1.0 float power value.
  float drive_speed(float forward,float turn=0.0) {
    return 0.2; // confident but conservative
  }

  // Autonomous drive power from float values:
  //   "drive": forward +1.0, backward -1.0
  //   "turn": left turn +1.0, right turn -1.0 (like angle)
  void set_drive_powers(double forward,double turn=0.0)
  {
    double max_autonomous_drive=1.0; //<- can set a cap for debugging autonomous

    double drive_power=drive_speed(+1.0);
    double t=limit(turn*0.5,drive_power);
    double d=limit(forward*0.5,drive_power);
    double L=d-t;
    double R=d+t;
    robot.power.left=64+63*limit(L,max_autonomous_drive);
    robot.power.right=64+63*limit(R,max_autonomous_drive);
  }

  // Autonomous feeler-based backing up: drive backward slowly until both switches engage.
  //  Return true when we're finally backed up properly.
  bool back_up()
  {
    if(!(drive_posture())) {return false;}
    else {
      set_drive_powers(-0.1);

      // FIXME: back-up sensors?
      return true; // (robot.sensor.backL && robot.sensor.backR);
    }
  }

  //  Returns true once we're basically at the target location.
  bool autonomous_drive(const aurora::robot_navtarget &target) {
    if (!drive_posture()) return false; // don't drive yet
    

    //Read data from the localizer files?
    // if(exchange_plan_current.updated())
    // {
    //   currentLocation = exchange_plan_current.read();
    // }
    // vec2 cur(currentLocation.x, currentLoction.y); // robot location
    // float cur_angle=currentLocation.angle;

     vec2 cur(locator.merged.x,locator.merged.y); // robot location

    // gl_draw_grid(autodriver.navigator.navigator.obstacles);

/*
    if (!simulate_only && fmod(cur_time,3.0)<2.0) {
      return false; // periodic stop (for safety, and for re-localization)
    }
*/

    bool path_planning_OK=false;
    if (should_plan_paths)
    { 
      // Send off request to the path planner
      exchange_plan_target.write_begin()=target;
      exchange_plan_target.write_end();
      
      // Check for a response from the path planner
      static aurora::drive_commands last_drive={0.0f,0.0f};
      static double last_drive_update=0.0;
      const double max_drive_seconds=1.0; // drive this many long on an old plan
      
      if (exchange_drive_commands.updated()) {
        last_drive=exchange_drive_commands.read();
        last_drive_update=cur_time;
      }
      if (cur_time - last_drive_update<max_drive_seconds) 
      {
        if(last_drive.left < 0 && last_drive.right < 0)
        {
          point_camera(180);
        }
        else 
        {
          point_camera(0);
        }
        robot.power.left =64+63*last_drive.left /100.0;
        robot.power.right=64+63*last_drive.right/100.0;
      }
      
      path_planning_OK=true; // FIXME: sanity check the path planner
    }
    if (!path_planning_OK)
    { // Fall back to greedy local autonomous driving: set powers to drive toward this field X,Y location
      double forward=0.0; // forward-backward
      double turn=0.0; // left-right

      double angle=locator.merged.angle; // degrees (!?)
      double arad=angle*M_PI/180.0; // radians
      vec2 orient(cos(arad),sin(arad)); // orientation vector (forward vector of robot)
      vec2 should=normalize(cur-vec2(target.x,target.y)); // we should be facing this way

      turn=orient.x*should.y-orient.y*should.x; // cross product (sin of angle)
      forward=-dot(orient,should); // dot product (like distance)
      printf("Path planning FAILURE: manual greedy mode %.0f,%.0f\n", forward,turn);
      set_drive_powers(forward,turn);
    }

    return target.matches(locator.merged); // we're basically there
  }

  // Force this angle (or angle difference) to be between -180 and +180,
  //   by adding or subtracting 360 degrees.
  void reduce_angle(double &angle) {
    while (angle>=180) angle-=360; // reduce
    while (angle<-180) angle+=360; // reduce
  }

  // Autonomous turning: rotate robot so it's facing this direction.
  //  Returns true once we're basically at the target angle.
  // ToDo: Point camera to an appropriate angle as you turn
  bool autonomous_turn(double angle_target_deg=0.0,bool do_posture=true)
  {
    if (do_posture) { if (!drive_posture()) return false; } // don't drive yet
    // if(exchange_plan_current.updated())
    // {
    //   currentLocation = exchange_plan_current.read();
    // }
    // double angle_err_deg=currentLocation.angle-angle_target_deg;
    // robotPrintln("Autonomous turn to %.0f from %.0f deg\n",
      // angle_target_deg, currentLocation.angle);
    
    double angle_err_deg=locator.merged.angle-angle_target_deg;
    reduce_angle(angle_err_deg);
    robotPrintln("Autonomous turn to %.0f from %.0f deg\n",
      angle_target_deg, locator.merged.angle);
   
    double turn=angle_err_deg*0.1; // proportional control
    double maxturn=drive_speed(0.0,1.0);
    turn=limit(turn,maxturn);
    set_drive_powers(0.0,turn);
    return fabs(angle_err_deg)<5.0; // angle error tolerance
  }

  // Make sure we're still facing the collection bin.  If not, pivot to face it.
  bool check_angle() {
    // if(exchange_plan_current.updated())
    // {
    //   currentLocation = exchange_plan_current.read();
    // } 
    // if (currentLocation.percent<0.2) return true; // we don't know where we are--just keep driving?
    // double target=180.0/M_PI*atan2(currentLocation.y+200.0,currentLocation.x);
    // double err=currentLocation.angle-target;
    // robotPrintln("check_angle: cur %.1f deg, target %.1f deg",currentLocation.angle,target);

    if (locator.merged.percent<20.0) return true; // we don't know where we are--just keep driving?
    double target=180.0/M_PI*atan2(locator.merged.y+200.0,locator.merged.x);
    double err=locator.merged.angle-target;
    robotPrintln("check_angle: cur %.1f deg, target %.1f deg",locator.merged.angle,target);
    
    reduce_angle(err);
    if (fabs(err)<10.0) return true; // keep driving--straight enough
    else return autonomous_turn(target,false); // turn to face target
  }
};


// Return true if the mining head is stalled (according to our sensors
bool is_stalled(const robot_base &robot) {
  return robot.sensor.Mstall;
}


/* Utility function: slow down speed as cur approaches target
  Returns false if already past target.
*/
bool speed_limit(int &howfast,int cur,int target,int dir=+1)
{
  int dist_left=(target-cur)*dir;
  if (dist_left<=0) {
    return false;
  }
  int max_speed=10+dist_left/5;
  if (howfast>max_speed) howfast=max_speed;
  return true;
}


void robot_manager_t::autonomous_state()
{
  robot.power.stop(); // each state starts from scratch

  double time_in_state=cur_time-state_start_time;
  robotPrintln("In state %s for %.1f seconds...\n", state_to_string(robot.state), time_in_state);

  // full autonomy start
  if (robot.state==state_autonomy) {
    robot.autonomous=true;
    enter_state(state_setup_raise);
  }
  // raise: raise the mining head to clear ground for driving
  else if (robot.state==state_setup_raise)
  {
    if(robot.sensor.bucket<head_mine_drive && time_in_state<2.0)// raises until bucket_drive
    {
      robot.power.dump=power_full_fw; // raise bin
    }
    else{
      enter_state(state_setup_extend);
    }
  }
  // state_setup_extend: extend the mining head so it does not get dragged
  else if (robot.state==state_setup_extend)
  {
	  if (time_in_state<7.0)
    {
      robot.power.dump=power_full_fw; // raise bin
      robot.power.head_extend = 127; // 127 for extend, 1 for tuck
	  }
	  else
	  {
		  mining_head_extended = true;
		  enter_state(state_setup_lower);
	  }
  }
  // state_setup_lower: lower the box
  else if (robot.state==state_setup_lower)
  {
	  if (time_in_state<8.0 && robot.sensor.limit_bottom==1)
    {
      robot.power.roll = 30; // lower box somewhat slowly
      robot.power.head_extend = 127;
	  }
	  else
	  {
		  enter_state(state_find_camera);
	  }
  }
  //state_find_camera: line up with centerline
  else if (robot.state==state_find_camera)
  {
    if (!drive_posture()) { /* correct posture first */ }
    else if (locator.merged.percent>=15.0) { // we know where we are!
      sim.loc=locator.merged; // reset simulator to real detected values

    // if(exchange_plan_current.updated())
    // {
    //   currentLocation = exchange_plan_current.read();
    // }
    // else if (currentLocation.percent>=0.1) { // we know where we are!
      // Need some interefacing with the simulation stuff

      sim.loc=locator.merged; // reset simulator to real detected values
      enter_state(state_scan_obstacles);
    }
    else // don't know where we are yet--change pointing
    {
      if (time_in_state<5.0) point_camera(0);
      else if (time_in_state<10.0) point_camera(-10);
      else if (time_in_state<15.0) point_camera(-30);
    }
  }
  //state_scan_obstacles: Scan for obstacles
  else if (robot.state==state_scan_obstacles)
  {
    int scan_angle=0; //Look straight ahead
    if (time_in_state<10.0) { // line up the beacon correctly
      point_camera(scan_angle);
    }
  }
  //state_drive_to_mine: Drive to mining area
  else if (robot.state==state_drive_to_mine)
  {
    if (drive_posture()) {
      double target_Y=field_y_mine_start; // mining area distance (plus buffer)
      double distance=target_Y-locator.merged.y;
      point_camera(0);

      // if(exchange_plan_current.updated())
      // {
      //   currentLocation = exchange_plan_current.read();
      // }
      // double target_Y=field_y_mine_start; // mining area distance (plus buffer)
      // double distance=target_Y-currentLocation.y;

      if (autonomous_drive(mine_target_loc) ||
          distance<0.0)  // we're basically there now
      {
        if (driver_test) enter_state(state_drive_to_dump);
        else enter_state(state_mine_lower); // start mining!
      }
      
    }
  }

  //Enter Semiauto mine modes

  //state_mine_lower: enter mining state
  else if (robot.state==state_mine_lower) {
    tryMineMode();
    //ToDo: point camera until it finds the Aruco markers
    point_camera(-180); //Look back 
    mine_start_time=cur_time; // update mine start time
    enter_state(state_mine);
  }
  else if (robot.state==state_mine)
  {
    if (!tryMineMode()) { // too high to mine (sanity check)
      robot.power.dump=power_full_bw; // lower bucket
      mining_head_lowered=true;
    }

    double mine_time=cur_time-mine_start_time;
    double mine_duration=250.0;
    if(mine_time>mine_duration)
    {
        enter_state(state_mine_raise);
    } // done mining
    
    if (robot.sensor.Mstall) enter_state(state_mine_stall);
  }

  // state_mine_stall: Detect mining head stall. Raise head until cleared
  else if (robot.state==state_mine_stall)
  {
    tryMineMode(); // Start PID based mining
    if(robot.sensor.Mstall && time_in_state<1)
    {
      robot.power.dump=power_full_fw; // raise bucket
    }
    else {enter_state(state_mine);} // not stalled? Then back to mining
  }

  //state_mine_raise: Raise mining conveyor before starting to backup towards Lunarbin
  else if (robot.state==state_mine_raise)
  {
    if(drive_posture())
      enter_state(state_drive_to_dump);
  }

  // Drive back to trough
  else if (robot.state==state_drive_to_dump)
  {
    point_camera(-180);
    if (autonomous_drive(dump_target_loc) 
     || locator.merged.y<dump_target_loc.y+20.0)
    {
      enter_state(state_dump_align);
    }
  }
  else if (robot.state==state_dump_align)
  { 
    // target.y=currentLocation.y; // don't try to turn when this close
    // if (autonomous_drive(target,dump_target_angle)
    //   || (fabs(currentLocation.y-target.y)<30 && fabs(currentLocation.x-field_x_trough_stop)<=10) )

    if (autonomous_drive(dump_align_loc)
      || (fabs(locator.merged.x-field_x_trough_stop)<=10) )
    {
      if (driver_test) {
        // mine_target_loc.x=50+(rand()%250); // retarget in mining area every run
        enter_state(state_drive_to_mine);
      }
      else enter_state(state_dump_contact);
    }
  }


  //Semiauto dump mode entry point: dock and dump mode
  else if (robot.state==state_dump_contact) // final backup to Lunarbin
  {
    back_up();
    if (time_in_state>1.0)
    {
      enter_state(state_dump_raise);
    }
  }

  // raise bucket to dump
  else if (robot.state==state_dump_raise)
  {
      enter_state(state_dump_pull);//2-19: Already in dump position
  }
  // Raise box
  else if(robot.state==state_dump_pull)
  {
    int howfast=32;
    int cur=(signed short)robot.sensor.Rcount;
    int target=box_raise_max;
    if (!speed_limit(howfast,cur,target,+1)  || time_in_state>15.0)
      enter_state(state_dump_rattle);
    else
      robot.power.roll=64+howfast; // forward
  }
  // Give dust time to flow out (maybe gentle rattle?)
  else if (robot.state==state_dump_rattle)
  {
    robot.power.dump=(fmod(time_in_state,0.2)>0.1)?power_full_fw:power_full_bw; // empty out conveyor (and rattle)
    if(time_in_state>2.0) {
      enter_state(state_dump_push);
    }
  }
  // Push box back down after dumping
  else if(robot.state==state_dump_push)
  {
    int howfast=32;
    int cur=(signed short)robot.sensor.Rcount;
    int target=0;
    if (!speed_limit(howfast,cur,target,-1)  || time_in_state>15.0)
      enter_state(state_drive_to_mine);
    else
      robot.power.roll=64-howfast; // backward
  }
  else if (robot.state==state_stow)
  {
    if(mining_head_lowered)
      drive_posture();
    if(time_in_state<20)
      robot.power.dump=127;
    enter_state(state_stowed);

  }
  else if (robot.state==state_stowed)
  {
    /* wait here forever */
  }
  else
  { // what?  unrecognized state?!  manual mode...
    robotPrintln("Autonomy: unrecognized state %s (%d)!?\n",state_to_string(robot.state), robot.state);
    enter_state(state_drive);
  }

  if (nodrive)
  { // do not drive!  (except for state_drive)
    robotPrintln("NODRIVE");
    set_drive_powers(0.0,0.0);
  }
}




robot_manager_t *robot_manager;

unsigned int video_texture_ID=0;

void robot_manager_t::update(void) {
  cur_time=0.001*glutGet(GLUT_ELAPSED_TIME);

#if 1 /* enable for backend UI: dangerous, but useful for autonomy testing w/o frontend */
  // Keyboard control
  ui.update(oglKeyMap,robot);

  // Click to set state:
  if (robotState_requested<state_last) {
    robot.state=robotState_requested;
    robotPrintln("Entering new state %s (%d) by backend UI request",
      state_to_string(robot.state),robot.state);
    robotState_requested=state_last; // clear UI request
  }
#endif

// Show real and simulated robots
//needs to be updated for new data exchange?
  robot_display(locator.merged);

	robot_display_autonomy(telemetry.autonomy);

// Check for a command broadcast (briefly)
  int n;
  while (0!=(n=comms.available(10))) {
    if (n==sizeof(command)) {
      comms.receive(command);
      if (command.command==robot_command::command_STOP)
      { // ESTOP command
        enter_state(state_STOP);
        robot.power.stop();
        robotPrintln("Incoming STOP command");
      }
      else if (command.command==robot_command::command_state)
      {
        if (command.state>=state_STOP && command.state<state_last)
        {
          robot.state=(robot_state_t)command.state;
          telemetry.ack_state=robot.state;
          robotPrintln("Entering new state %s (%d) by frontend request",
            state_to_string(robot.state),robot.state);
        } else {
          robotPrintln("ERROR!  IGNORING INVALID STATE %d!!\n",command.state);
        }
      }
      else if (command.command==robot_command::command_power)
      { // manual driving power command
        robotPrintln("Incoming power command: %d bytes",n);
        if (robot.state==state_drive)
        {
          robot.autonomous=false;
          robot.power=command.power;
        }
        else
        {
          robotPrintln("IGNORING POWER: not in drive state\n");
        }
      }
      if (command.realsense_comms.command=='P')
      {
        point_camera(command.realsense_comms.requested_angle);
      }
    } else {
      robotPrintln("ERROR: COMMAND VERSION MISMATCH!  Expected %d, got %d",
        sizeof(command),n);
    }

  }

// Perform action based on state recieved from FrontEnd
  //E-Stop command
  if(robot.state==state_STOP)
  {// All stop
    robot.power.stop();
    state_start_time=cur_time;
  }
  else if (robot.state==state_drive)
  { // do nothing-- already got power command
    state_start_time=cur_time;
  }
  else if (robot.state==state_backend_driver)
  { // set robot power from backend UI
    robot.power=ui.power;
    printf("Backend driver dump: %d\n",robot.power.dump);
  }
  else if (robot.state>=state_autonomy) { // autonomous mode!
    autonomous_state();
  }

  //Variables to determine if you can raise or lower box
  bool can_raise_up=true;
  bool can_raise_down=true;

  //Detect soft encoder limiters
  // if(robot.sensor.Rcount>=box_raise_max)
  //   can_raise_up=false;
  // if(robot.sensor.Rcount<=box_raise_min)
  //   can_raise_down=false;

  //Detect limit switches and reset encoder offset if needed
  /*if(robot.sensor.limit_top%2!=0)
    can_raise_up=false;
  if(robot.sensor.limit_bottom%2!=0)
    can_raise_down=false;*/

  //Stop raise/lower if limit detected
  if (!robot.power.torqueControl) //Override limit switches in torque control
  {
    if(robot.power.roll>64&&!can_raise_up)
      robot.power.roll=64;
    if(robot.power.roll<64&&!can_raise_down)
      robot.power.roll=64;
  }

  // Send commands to Arduino
  robot_sensors_arduino old_sensor=robot.sensor;
    
  if (simulate_only) { // build fake arduino data
    robot.status.arduino=1; // pretend it's connected
    robot.sensor.McountL=0xff&(int)sim.Mcount;
    robot.sensor.Rcount=0xffff&(int)sim.Rcount;
    robot.sensor.DL1count=0xffff&(int)sim.DLcount;
    robot.sensor.DR1count=0xffff&(int)sim.DRcount;
    robot.sensor.limit_top=0;
    robot.sensor.limit_bottom=0;
  }
  else { // real arduino
    arduino.update(robot);

    //Reset encoder offset if needed
    if(robot.sensor.limit_top==0)
    {
      arduino.Rdiff+=box_raise_limit_high-robot.sensor.Rcount;
      robot.sensor.Rcount=box_raise_limit_high;
    }
    if(robot.sensor.limit_bottom==0)
    {
      arduino.Rdiff+=box_raise_limit_low-robot.sensor.Rcount;
      robot.sensor.Rcount=box_raise_limit_low;
    }
  }
  speed_Mcount=robot.sensor.McountL-last_Mcount;
  float smoothing=0.3;
  smooth_Mcount=speed_Mcount*smoothing + smooth_Mcount*(1.0-smoothing);
  robotPrintln("Mcount smoothed: %.1f, speed %d\n",
     smooth_Mcount, speed_Mcount);
  last_Mcount=robot.sensor.McountL;

  // Fake the bucket sensor from the sim (no hardware sensor for now)
  robot.sensor.bucket=sim.bucket*(950-179)+179;

  // some values for the determining location. needed by the localization.
  // FIXME: tune these for real tracks!
  float fudge=1.06; // fudge factor to make blue printed wheels work mo betta
  float drivecount2cm=fudge*6*5.0/36; // cm of driving per wheel encoder tick == pegs on drive sprockets, space between sprockets, 36 encoder counts per revolution
  float driveL = fix_wrap256(robot.sensor.DL1count-old_sensor.DL1count)*drivecount2cm;
  float driveR = fix_wrap256(robot.sensor.DR1count-old_sensor.DR1count)*drivecount2cm;
  
  // Update drive encoders data exchange
  static aurora::drive_encoders::real_t totalL = 0.0; //<- hacky!  Need to total up distance
  static aurora::drive_encoders::real_t totalR = 0.0;
  totalL -= driveL;
  totalR += driveR;
  aurora::drive_encoders enc;
  enc.left =totalL;
  enc.right=totalR;
  exchange_drive_encoders.write_begin()=enc;
  exchange_drive_encoders.write_end();
  
  locator.merged=exchange_plan_current.read();


// Send out telemetry
  static double last_send=0.0;
  if (cur_time>last_send+0.050)
  {
    last_send=cur_time;
    robotPrintln("Sending telemetry, waiting for command");
    telemetry.count++;
    telemetry.state=robot.state; // copy current values out for send
    telemetry.status=robot.status;
    telemetry.sensor=robot.sensor;
    telemetry.power=robot.power;
    telemetry.loc=locator.merged; 
    locator.merged.percent*=0.999; // slowly lose location fix

    comms.broadcast(telemetry);
  }


  static double last_time=0.0;
  double dt=cur_time-last_time;
  if (dt>0.1) dt=0.1;
  last_time=cur_time;

  if (locator.merged.percent>=10.0)  // make sim track reality
    sim.loc=locator.merged;

  if (simulate_only) // make reality track sim
  {
    float view_robot_angle=0;
    float beacon_FOV=30; // field of view of beacon (markers)
    if (beacon_FOV>fabs(telemetry.autonomy.markers.beacon - view_robot_angle))
      locator.merged.percent+=10.0;
    locator.merged.percent=std::min(100.0,locator.merged.percent*(1.0-dt));
      
  /*
    locator.merged=sim.loc; // blend(locator.merged,sim.loc,0.1);
    if (fabs(sim.loc.angle)<40.0) // camera in view
      locator.merged.percent+=10.0;
    else // camera not in view
      locator.merged.percent*=0.9;
    locator.merged.percent*=0.9;

    if (locator.merged.y>100 && locator.merged.y<500)
    { // simulate angle drift
      if ((rand()%300)==0) {
        sim.loc.angle+=2.0;
        robotPrintln("Injecting angle drift right\n");
      }
      if ((rand()%300)==0) {
        sim.loc.angle-=2.0;
        robotPrintln("Injecting angle drift left\n");
      }
      if ((rand()%300)==0) {
        vec3 delta(0.0);
        int del;
        do { del = (rand()%3)-1; } while (del==0);
        del*=5;
        int axis=rand()%2;
        robotPrintln("Injecting position drift by %d on axis %d\n", del,axis);
        if (axis==0)
          sim.loc.x+=del;
        else
          sim.loc.y+=del;
      }
    }
    */
  }
  sim.simulate(robot.power,dt);
}


void display(void) {
  robot_display_setup(robot_manager->robot);

  robot_manager->update();

  if (video_texture_ID) {
    glTranslatef(field_x_GUI+350.0,100.0,0.0);
    glScalef(300.0,200.0,1.0);
    glBindTexture(GL_TEXTURE_2D,video_texture_ID);
    glEnable(GL_TEXTURE_2D);
    glBegin(GL_QUAD_STRIP);
    glTexCoord2f(0.0,0.0); glVertex2f(0.0,0.0);
    glTexCoord2f(1.0,0.0); glVertex2f(+1.0,0.0);
    glTexCoord2f(0.0,1.0); glVertex2f(0.0,+1.0);
    glTexCoord2f(1.0,1.0); glVertex2f(+1.0,+1.0);
    glEnd();
    glBindTexture(GL_TEXTURE_2D,0);
  }

  glutSwapBuffers();
  glutPostRedisplay();
}

int main(int argc,char *argv[])
{
  // setenv("DISPLAY", ":0",1); // never forward GUI over X
  glutInit(&argc,argv);

  // Set screen size
  int w=1200, h=700;
  for (int argi=1;argi<argc;argi++) {
    if (0==strcmp(argv[argi],"--sim")) {
      simulate_only=true;
      if (argi+1<argc) srand(atoi(argv[++argi])); // optional seed argument
      else srand(1);
    }
    else if (0==strcmp(argv[argi],"--noplan")) {
      should_plan_paths=false;
    }
    else if (0==strcmp(argv[argi],"--driver_test")) {
      simulate_only=true;
      driver_test=true;
    }
    else if (0==strcmp(argv[argi],"--nogui")) { // UNTESTED!
      show_GUI=false;
    }
    else if (0==strcmp(argv[argi],"--nodrive")) {
      nodrive=true;
    }
    else if (2==sscanf(argv[argi],"%dx%d",&w,&h)) {}
    else {
      printf("Unrecognized argument '%s'!\n",argv[argi]);
      exit(1);
    }
  }

  robot_manager=new robot_manager_t;
  robot_manager->locator.merged.y=100;
  if (simulate_only) robot_manager->locator.merged.x=150;

  if (show_GUI) {
    glutInitDisplayMode(GLUT_RGBA + GLUT_DOUBLE);
    glutInitWindowSize(w,h);
    glutCreateWindow("Robot Backend");
    robotMainSetup();

    glutDisplayFunc(display);
    glutMainLoop();
  }
  else
  { // no-GUI version
    while (true) {
      robot_manager->update();
    }
  }
  return 0;
}

