/*
  Use the gridnav library for RMC-style robot.  
  This basically just includes the robot's top-down geometry,
  and the field dimensions.
*/
#ifndef AURURA_GRIDNAV_RMC_H
#define AURURA_GRIDNAV_RMC_H

#include "gridnav.h"
#include "../aurora/coords.h"
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
  
  typedef navigator_t::fposition fposition;
  typedef navigator_t::gridposition gridposition;
  typedef navigator_t::searchposition searchposition;
  
  
  // Target is a simple 2D target point
  class planner_target_2D {
    // This is our search target configuration
    gridposition gtarget;
  public:
    planner_target_2D(const fposition &target_)
      :gtarget(gridposition(target_)) {}
    
    
    /* Utility function: return the angular distance */
    float angle_dist(float delta) const {
       delta=navigator_t::fmodplus(delta,GRIDA);
       if (delta>GRIDA/2) delta=GRIDA-delta; // turn the other way
       return delta;
    }
    
    double get_cost_from(const gridposition &from_pos) const
    {
      double drive_dist=length(vec2(from_pos.x,from_pos.y)-vec2(gtarget.x,gtarget.y)); // in grid cells
      double turn_ang=angle_dist(from_pos.a-gtarget.a); // in discrete angle units
      double TURN_AMPLIFY=5.0;
      double estimate = drive_dist + TURN_AMPLIFY*turn_ang*navigator_t::TURN_COST_TO_GRID_COST;
      return estimate;
    }
    
    bool reached_target(const gridposition &grid) const 
    {
      if (true) 
      { // require exact
        return grid==gtarget;
      }
      else 
      { // allow a little position error
        return 
          std::abs(grid.x-gtarget.x)<=1 &&
          std::abs(grid.y-gtarget.y)<=1 &&
          std::abs(grid.a-gtarget.a)<=0;
      }
    }
  };
  
  // Target is a robot_navtarget, with error bars
  class planner_navtarget {
    // This is our search target configuration
    gridposition gtarget;
    gridposition gerror;
    
    bool carex,carey,carea; // true if we care about this axis
  public:
    planner_navtarget(const aurora::robot_navtarget &t)
        :gtarget(gridposition(fposition(t.x,t.y,t.angle))),
         gerror(gridposition(fposition(t.error.x,t.error.y,t.error.angle)))
    {
        carex=(t.error.x!=aurora::robot_navtarget::DONTCARE);
        carey=(t.error.y!=aurora::robot_navtarget::DONTCARE);
        carea=(t.error.angle!=aurora::robot_navtarget::DONTCARE);
    }
    
    /* Utility function: return the angular distance */
    float angle_dist(float delta) const {
       delta=navigator_t::fmodplus(delta,GRIDA);
       if (delta>GRIDA/2) delta=GRIDA-delta; // turn the other way
       return delta;
    }
    
    double get_cost_from(const gridposition &from_pos) const
    {
      double turn_ang=angle_dist(from_pos.a-gtarget.a); // in discrete angle units
      double TURN_AMPLIFY=5.0; // cost of one angular cell per grid drive cell

    // FIXME: these compare cost to the center of the target, maybe edge of allowed region would be better?
      double estimate = 0.0; // in grid cells
      if (carex && carey) estimate += length(vec2(from_pos.x,from_pos.y)-vec2(gtarget.x,gtarget.y)); 
      else if (carex) estimate += std::abs(from_pos.x-gtarget.x);
      else if (carey) estimate += std::abs(from_pos.y-gtarget.y);
      
      if (carea)  estimate += TURN_AMPLIFY*turn_ang*navigator_t::TURN_COST_TO_GRID_COST;
      
      return estimate;
    }
    
    bool reached_target(const gridposition &grid) const 
    {
      // Check our positional error
      return 
          ((!carex) || std::abs(grid.x-gtarget.x)<=gerror.x) &&
          ((!carey) || std::abs(grid.y-gtarget.y)<=gerror.y) &&
          ((!carea) || angle_dist(grid.a-gtarget.a)<=gerror.a);
    }
  };
  
  typedef navigator_t::planner<planner_navtarget> planner;
  
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



