#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include "gridnav/gridnav_RMC.h"
#include "aurora/lunatic.h"
#include <iostream>
#include <fstream>

/**
 This is the autonomous path planning object.
*/
class robot_autodriver
{
public:
  rmc_navigator navigator;
  enum {navigator_res=rmc_navigator::GRIDSIZE};
  typedef rmc_navigator::navigator_t::searchposition planned_path_t;

  bool has_path;
  std::deque<planned_path_t> planned_path;
  rmc_navigator::navigator_t::drive_t last_drive;
  int replan_counter;
  
  aurora::field_drivable last_field;

  robot_autodriver()
  {
    flush_field();
    flush_path();

    compute_proximity();
  }
  
  // Clear all stored obstacles, so we start from zero
  void flush_field() {
    last_field.clear(0);
    
    navigator=rmc_navigator(); // reinitialize the navigator
  }
  
  void update_field(const aurora::field_drivable &f) {
    flush_field(); //<- otherwise it will never forget any obstacle
    
    for (int y=0;y<aurora::field_drivable::GRIDY;y++)
    for (int x=0;x<aurora::field_drivable::GRIDX;x++)
    {
        int fp = f.at(x,y); // field pixel
        int lp = last_field.at(x,y); // last-seen pixel
        if (fp != lp) 
        {
            last_field.at(x,y)=fp; // update last_field
            if (fp>aurora::field_unknown && fp<aurora::field_driveable)
            { // mark the obstacle
                navigator.mark_obstacle(
                    x*aurora::field_drivable::GRIDSIZE,
                    y*aurora::field_drivable::GRIDSIZE,
                    30);
            }
        }
    }
    
    compute_proximity();
  }

  // Mark this field location as an obstacle of this height
  //  (you MUST call compute_proximity after marking obstacles)
  inline void mark_obstacle(int x,int y,int ht) { navigator.mark_obstacle(x,y,ht); }

  // Recompute proximity costs (after marking obstacles)
  void compute_proximity() {
    // Recompute proximity costs after marking obstacles
    const int obstacle_proximity=30/navigator_res; // distance in grid cells to start penalizing paths
    navigator.navigator.compute_proximity(obstacle_proximity);
  }
  
  // Create a disk obstacle of this diameter at this position
  //   (mostly useful for generating test obstacles)
  inline void mark_disk_obstacle(int cx,int cy,int ht,int radius)
  {
    int d=radius+1;
    for (int y=-d;y<=+d;y++)
    for (int x=-d;x<=+d;x++)
    if (x*x+y*y<=radius*radius)
    {
      mark_obstacle(x+cx,y+cy,ht);
    }
  }

  // Dump proximity and debug data to plain text file
  void debug_dump(const char *filename="debug_nav.txt") {
    // Debug dump obstacles, field, etc.
    std::ofstream navdebug(filename);
    navdebug<<"Raw obstacles:\n";
    navigator.navigator.obstacles.print(navdebug,10);

    for (int angle=0;angle<=20;angle+=10) {
      navdebug<<"\n\nAngle "<<angle<<" expanded obstacles:\n";
      navigator.navigator.slice[angle].obstacle.print(navdebug,10);

      navdebug<<"Proximity:\n";
      navigator.navigator.slice[angle].proximity.print(navdebug,1);
    }
  }

  // Flush autonomous drive stored path
  void flush_path() {
    has_path=false;
    replan_counter=0;
    planned_path=std::deque<planned_path_t>();
  }

  // Compute autonomous drive from cur to target
  bool autodrive(const aurora::robot_loc2D &cur, const aurora::robot_loc2D &target,
    aurora::drive_commands &drive, aurora::path_plan &debug)
  {
    const int replan_interval=1; // 1==every frame.  10==every 10 frames.

    const int plan_averaging=2; // steps in new plan to average together
    const int replan_length=2*plan_averaging; // distance of new plan to keep
    static rmc_navigator::navigator_t::drive_t last_drive;

    // refill planned path
    replan_counter=replan_interval;
    flush_path(); // flush old planned path
    debug.plan_len=0;

    // Start position: robot's position
    rmc_navigator::fposition fstart(cur.x,cur.y,cur.angle);
    // End position: at target
    rmc_navigator::fposition ftarget(target.x,target.y,target.angle);
    debug.target=target;

    
    rmc_navigator::planner plan(navigator.navigator,fstart,ftarget,last_drive,false);
    int steps=0;
    for (const rmc_navigator::searchposition &p : plan.path)
    {
        planned_path.push_back(p);
        if (steps<replan_length)
        {
          p.print();
        }
        if (steps<aurora::path_plan::max_path_len)
        {
          debug.plan_len=steps+1;
          
          aurora::robot_loc2D loc;
          loc.x=p.pos.v.x;
          loc.y=p.pos.v.y;
          loc.angle=p.pos.get_degrees();
          loc.percent=80.0-steps;
          debug.path_plan[steps]=loc;
        }

        steps++;
    }

    printf("Planned path from %.0f,%.0f@%.0f to target %.0f,%.0f@%.0f: %d steps\n",
      cur.x,cur.y,cur.angle,
      target.x,target.y,target.angle, steps);
    if (!plan.valid) {
        printf("Path planning FAILED: searched %zd cells\n",plan.searched);
        return false;
    }
      
    int pathslots=plan_averaging;
    rmc_navigator::navigator_t::drive_t next_drive(0.0f,0.0f);
    for (int slot=0;slot<plan_averaging;slot++) {
      if (slot<(int)planned_path.size()) {
        planned_path_t &p=planned_path[slot];
        next_drive += p.drive;
      }
      else { // short plan, use last known data
        next_drive +=last_drive;
      }
    }
    next_drive.forward = next_drive.forward/pathslots;
    next_drive.turn = next_drive.turn/pathslots;

    // Cycle detection
    static int cycle_count=0;
    cycle_count--;
    if (cycle_count<0) cycle_count=0;
    if (next_drive.forward * last_drive.forward <0 || next_drive.turn * last_drive.turn <0)
    {
      cycle_count+=2;
      if (cycle_count>5) {
        printf("Path planning CYCLE DETECTED, counter %d, keeping last drive\n",
          cycle_count);
        cycle_count=0;

        // don't replan, just drive for a bit to clear the cycle
        replan_counter=10;

        //forward=last_drive.forward;
        //turn=last_drive.turn;
      }
    }
    printf("Path planning forward %.1f, turn %.1f\n",next_drive.forward,next_drive.turn);

    // Update last_drive for next time
    // if (planned_path.size()>0) last_drive=planned_path[0].drive;
    last_drive=next_drive; 
    
    const float autonomous_speed=15.0f; // drive speed, as percent power
    drive.left =autonomous_speed*(next_drive.forward-next_drive.turn);
    drive.right=autonomous_speed*(next_drive.forward+next_drive.turn);

    return true;
  }
// TODO find out what parts are needed for this to work
  // Draw a planned path onscreen
//   void draw_path() {
//     glBegin(GL_LINE_STRIP);
//     for (const rmc_navigator::searchposition &p : planned_path) {
//       glColor3f(0.5f+0.5f*p.drive.forward,0.5f+0.5f*p.drive.turn,0.0f);
//       glVertex2fv(p.pos.v);
//     }
//     glColor3f(1.0f,1.0f,1.0f);
//     glEnd();
//   }
};

#endif
