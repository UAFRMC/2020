// Basic skeleton read/write file for the pathplanner, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"
#include "pathplanner.h"


//Supporting Values

bool show_GUI=true;
bool simulate_only=false; // --sim flag
bool should_plan_paths=true; // --noplan flag
bool driver_test=false; // --driver_test, path planning testing
bool nodrive=false; // --nodrive flag (for testing indoors)

/** X,Y field target location where we drive to, before finally backing up */
vec2 dump_target_loc(field_x_size/2,field_y_trough_center); // rough area
vec2 dump_align_loc(field_x_trough_edge,dump_target_loc.y); // final alignment
float dump_target_angle=field_angle_trough;

/** X,Y field target location that we target for mining */
vec2 mine_target_loc(field_x_size/2,field_y_size-60);
float mine_target_angle=90; // along +y




int main() {
    //Make the pathplanning object
    robot_autodriver robotplanner;
    robotplanner.flush();


    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_drive_commands();
    
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_plan_target();
    MAKE_exchange_plan_current();
    MAKE_exchange_field_drivable();

    while (true) {
        aurora::robot_loc2D target = exchange_plan_target.read();
        
        if (target.percent>=1.0) { // we have a valid target
            aurora::robot_loc2D current = exchange_plan_current.read();
            const aurora::field_drivable &currField = exchange_field_drivable.read();
            
            




            //Some logic to determine what are the dive values
            //Currently just creates empty object needs some data?
            aurora::drive_commands newDrive;
            //Set
            //spinning around in place
            newDrive.left = 100.0;
            newDrive.right = 0.0;

            //writing new data to files:
            exchange_drive_commands.write_begin() = newDrive;
            exchange_drive_commands.write_end();
        }

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}



//Helper Functions for Autonomy
bool autonomous_drive(vec2 target,float target_angle) {
  if (!drive_posture()){
      return false; // don't drive yet
  }
  //Current Robot Location 
  vec2 cur(current.x,current.y); 
  float cur_angle=current.angle;
  gl_draw_grid(currField);

  if (!simulate_only && fmod(cur_time,3.0)<2.0) {
      return false; // periodic stop (for safety, and for re-localization)
  }
  //Setting parameters before planning
  bool path_planning_OK=false;
  double forward=0.0; // forward-backward
  double turn=0.0; // left-right

  if (should_plan_paths)
  { //<- fixme: move path planning to dedicated thread, to avoid blocking
  path_planning_OK=robotplanner.autodrive(
      cur,cur_angle,target,target_angle,
      forward,turn, telemetry.autonomy);
  if (path_planning_OK) robotplanner.draw_path();
  }
  if (!path_planning_OK)
  {
      double angle=locator.merged.angle; // degrees (!?)
      double arad=angle*M_PI/180.0; // radians
      vec2 orient(cos(arad),sin(arad)); // orientation vector (forward vector of robot)
      vec2 should=normalize(cur-target); // we should be facing this way
      turn=orient.x*should.y-orient.y*should.x; // cross product (sin of angle)
      forward=-dot(orient,should); // dot product (like distance)
      printf("Path planning FAILURE: manual greedy mode %.0f,%.0f\n", forward,turn);
  }
  set_drive_powers(forward,turn);

  return length(cur-target)<=2*rmc_navigator::GRIDSIZE; // we're basically there
}


bool autonomous_turn(double angle_target_deg=0.0,bool do_posture=true)
  {
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
