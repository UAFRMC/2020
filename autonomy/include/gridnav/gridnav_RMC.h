/*
  Use the gridnav library for RMC-style robot.  
  This basically just includes the robot's top-down geometry,
  and the field dimensions.
*/
#ifndef AURURA_GRIDNAV_RMC_H
#define AURURA_GRIDNAV_RMC_H

#include "gridnav.h"
#include "../../firmware/field_geometry.h"

/**
  Build a gridnav navigator to plan paths for our 
  Robot Mining Competition sized robot.
  
  
*/
class rmc_navigator : public gridnav::robot_geometry {
public:
  // RMC-plausible navigation grid dimensions:
  enum {GRIDSIZE=8}; // cm per grid cell
  enum {GRIDX=(field_x_size+GRIDSIZE-1)/GRIDSIZE}; // xy grid cells for field
  enum {GRIDY=(field_y_size+GRIDSIZE-1)/GRIDSIZE};
  enum {GRIDA=72}; // angular slices around 360 degrees
  enum {ROBOTSIZE=(80+GRIDSIZE-1)/GRIDSIZE}; // maximum size measured from middle

  // Create the navigator and planner
  typedef gridnav::gridnavigator<GRIDX, GRIDY, GRIDA, GRIDSIZE, ROBOTSIZE> navigator_t;
  navigator_t navigator;
  
  typedef navigator_t::planner planner;
  typedef navigator_t::fposition fposition;
  typedef navigator_t::searchposition searchposition;
  
  // Discretized version of our geometry
  navigator_t::robot_grid_geometry robot;
  
  // Build the navigator:
  rmc_navigator(bool verbose=false) :robot(*this,verbose)
  {
    navigator.mark_edges(robot);
  }
  
  // Add an obstacle at this (x,y), in centimeters (rounds to navigation grid cell)
  void mark_obstacle(int x,int y,int height) {
     navigator.mark_obstacle((x+GRIDSIZE/2)/GRIDSIZE,(y+GRIDSIZE/2)/GRIDSIZE,height,robot);
  }


 // gridnav::robot_geometry interface:
    //    (0,0) is the robot's origin, at the center of turning.
    //    +x is the direction the robot naturally drives forward
  virtual int clearance_height(float x,float y) const 
  {
    float frontback=robot_x;
    float leftright=robot_y;
    float tracks=robot_track_y; // tracks, frame, motors, etc
  
  // Special dot under mining head
    float dx=x-robot_mine_x, dy=y-0;
    float r=sqrt(dx*dx+dy*dy);
    if (r<robot_mine_radius) return robot_mine_clearance; 
  
  // front-back:
    if (x>frontback || x<-frontback) return gridnav::OPEN;
    if (y<0) y=-y; // apply Y symmetry
    if (y>leftright) return gridnav::OPEN; // beyond the tracks
    if (y>leftright-tracks) return 0; // tracks
    if (y<robot_box_y) return robot_box_clearance;
    return robot_inside_clearance; // clearance area under main body (box, etc)
  }
};




#endif



