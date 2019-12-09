/*
aurora/LUNATIC:  pronounced "lun-AT-ic" (definitely not "lun-A-tic")
Logically UNcoupled Architecture Technology for Intra-robot Communication

This file manages data exchange between all on-robot 
software components, including these programs:

vision manages the realsense camera
    Reads color and depth frames from the camera
    Passes color to aruco to look for computer vision markers
        Any detected markers create position estimates for the localizer
    Passes depth to obstacle detector subsystem to look for rocks
        Any detected obstacles get written to the obstacle grid

localizer integrates the robot position
    Reads drive encoder counts from the backend
    Reads aruco data from the vision subsystem
    Reads camera pointing from the stepper motor
    Publishes coordinates used by all other components

stepper talks to the camera pointing stepper motor
    Reads requested view angle from backend
    Publishes current view angle to the localizat

path planner computes an obstacle-free drive path
    Reads the robot location from the localizer
    Reads the target location from the backend
    Reads obstacle grid from the obstacle detector
    Publishes drive commands to the backend

backend talks to the robot, using two subsystems:
    BAC: Basic Autonomy Control 
        manages the autonomy state machine and pilot comms (over UDP)
    KEND: Keep Electronics Not Dying 
        talks to the Arduino, and restarts it if needed.
*/
#ifndef __AURORA_LUNATIC_H
#define __AURORA_LUNATIC_H

#include <stdio.h>
#include <array>
#include "../aurora/data_exchange.h"
#include "../aurora/coords.h"
#include "../vision/grid.hpp"

namespace aurora {

/* -------------- Drive Command ---------------- 
  Track speed commands, for the left and right tracks.
  Values are speed percent.
    speed=0 means stop.
    speed=+100 means full speed forward.
    speed=-100 means full speed reverse.
  E.g., left=+100; right=0; makes a right pivot around the right track.
*/
struct drive_commands {
    float left;
    float right;
    
    void print(FILE *f=stdout) const {
        fprintf(f,"drive: %5.1f L, %5.1f R\n", left, right);
    }
};

/* This macro declares the variable used to 
send the backend a drive command:
    Written by the path planner
    Read by the backend to send to the motors when in autonomous drive mode
    Read by the localizer to predict future motion
*/
#define MAKE_exchange_drive_commands()   aurora::data_exchange<aurora::drive_commands> exchange_drive_commands("backend.drive")


/* -------------- Drive Encoders --------------
  Wheel encoder ticks: these capture the total forward motion
  taken by the left and right tracks.  
  
  These are total motion since startup, not incremental.
  
  Units are centimeters.
  E.g., at startup, left=right=0.0;
    After driving backwards by 0.5 meters, left=right=-50.0;
    After driving backwards another 0.5 meters, left=right=-100.0;
    After driving forwards by 1.5 meters, left=right=+50.0;
*/
struct drive_encoders {
    typedef double real_t;
    real_t left;
    real_t right;
    
    // Subtract two sets of encoder values (typically current-previous)
    // to get relative motion.
    drive_encoders operator-(const drive_encoders other) const {
        drive_encoders ret;
        ret.left=left - other.left;
        ret.right=right - other.right;
        return ret;
    }
    
    void print(FILE *f=stdout) const {
        fprintf(f,"encoders: %5.2f L, %5.2f R\n", (float)left, (float)right);
    }
};

/* This macro declares the variable used to 
report the current drive track encoders:
    Written by the backend
    Read by the localizer to track current motion
*/
#define MAKE_exchange_drive_encoders()   aurora::data_exchange<aurora::drive_encoders> exchange_drive_encoders("backend.encoders")




/* -------------- Camera Pointing via Stepper Motor --------------
  This same struct is used to request or report the stepper motor position.
*/
struct stepper_pointing {
    float angle; // robot-relative camera pointing angle, in degrees.  0 == robot forward
    int32_t stable; // 0: on break, homing, etc.  1: should be steady.
    
    void print(FILE *f=stdout) const {
        fprintf(f,"stepper: %5.0f angle, %s\n", angle, stable?"stable":"unstable");
    }
};

/* This macro declares the variable used to 
request a new stepper motor position.
    Written by the backend
    Read by the stepper motor controller
*/
#define MAKE_exchange_stepper_request()   aurora::data_exchange<aurora::stepper_pointing> exchange_stepper_request("stepper_request.angle")

/* This macro declares the variable used to 
report the current stepper motor position:
    Written by the stepper motor controller
    Read by the localizer to bake camera coordinate transforms
*/
#define MAKE_exchange_stepper_report()   aurora::data_exchange<aurora::stepper_pointing> exchange_stepper_report("stepper_report.angle")




/* -------------- Robot Localization and Navigation Coordinates --------------
  See aurora/coords.h for robot_loc2D and robot_coord3D.
*/

/* This macro declares the variable used to 
report the robot position for path planning:
    Written by the localizer
    Read by the path planner
Coordinate system: absolute field coordinates
*/
#define MAKE_exchange_plan_current()   aurora::data_exchange<aurora::robot_loc2D> exchange_plan_current("plan_current.loc2D")


/* This macro declares the variable used to 
target a new drive position for path planning:
    Written by the backend when it wants to drive somewhere new
    Read by the path planner
Coordinate system: absolute field coordinates
*/
#define MAKE_exchange_plan_target()   aurora::data_exchange<aurora::robot_loc2D> exchange_plan_target("plan_target.loc2D")



/* This macro declares the variable used to 
project depth camera obstacles into field coordinates:
    Written by the localizer as the robot and camera coordinates change
    Read by the obstacle detection subsystem
Coordinate system: absolute field coordinates
*/
#define MAKE_exchange_obstacle_view()   aurora::data_exchange<aurora::robot_coord3D> exchange_plan_target("plan_target.loc3D")




// This is one report of a computer vision marker currently in-view.
//   Coordinate system: camera-relative coordinates, scaled to unit marker size.
struct vision_marker_report {
    robot_coord3D coords; // camera-relative coordinates of marker
    int32_t markerID; // 0 if invalid, or an Aruco marker ID
    enum {max_count=2};
};
//******This must be defined in the file using the #define Make_exchange*****
// This is an array of reports for all currently visible markers
typedef std::array<vision_marker_report, vision_marker_report::max_count> vision_marker_reports; 

/* This macro declares the variable used to 
report a computer vision derived marker position to the localizer:
    Written by the computer vision system when it sees an aruco marker
    Read by the localizer
Coordinate system: camera-relative coordinates
*/
#define MAKE_exchange_marker_reports()   aurora::data_exchange<vision_marker_reports> exchange_marker_reports("vision_marker.reports")



/* -------------- Robot Obstacle Detection for Navigation --------------
  Stores a rasterized top-down view of the robot field.
    grid_pixel: datatype at each pixel
    GRIDSIZE: cm per pixel (ideally an integer multiple of navigation size)
*/
template <typename grid_pixel, int GRIDSIZE_T>
class field_raster {
public:
  enum {GRIDSIZE=GRIDSIZE_T}; // cm per pixel
  enum {GRIDX=(field_x_size+GRIDSIZE-1)/GRIDSIZE}; // xy pixel dimensions (rounded up)
  enum {GRIDY=(field_y_size+GRIDSIZE-1)/GRIDSIZE};
  enum {GRIDTOTAL=GRIDX*GRIDY}; // total pixel
  
  grid_pixel raster[GRIDTOTAL];
  
  grid_pixel &at(int x,int y) { return raster[y*GRIDX + x]; }
  const grid_pixel &at(int x,int y) const { return raster[y*GRIDX + x]; }
  
  /* Return true if this point has data (in range, and count >0) */
  bool in_bounds(int x,int y) const {
    if (x<0 || x>=GRIDX || y<0 || y>=GRIDY) return false;
    return true;
  }

  /* Assign this value everywhere */
  void clear(const grid_pixel &value) {
    for (size_t i=0;i<GRIDTOTAL;i++) raster[i]=value;
  }  
};

/*
  For each point on the field, store an unsigned char
  indicating what state this part of the field is in:
*/
enum {
    field_unknown=0, // we have no data (uninitialized)
    field_fixed=10, // we expect this part of the field to be occupied (e.g., bin)
    field_mined=20, // we dug deep with our mining head into this area
    field_toolow=30, // looks like a crater here
    field_sloped=50, // slopes seem too high
    field_toohigh=80, // we saw a tall obstacle here
    
    field_driveable = 100, // below this: not driveable.  Above this: driveable

    field_flat=120, // seen and looks flat(enough)
    field_driven=250 // we have driven here before (definitely safe)
};
//*******This must be defined in the file using the #define Make_exchange******
typedef field_raster<unsigned char,4> field_drivable;

/* This macro declares the variable used to 
store the field grid of obstacle / drivable locations:
    Written by the computer vision system
    Read by the path planner
*/
#define MAKE_exchange_field_drivable()   aurora::data_exchange<field_drivable> exchange_field_drivable("field_drivable.grid")










}; // end namespace

#endif



