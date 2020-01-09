#ifndef PATHPLANNER_H
#define PATHPLANNER_H



#include "gridnav/gridnav_RMC.h"
#include "aurora/network.h"
#include <iostream>
#include <fstream>


bool show_GUI=true;
bool simulate_only=false; // --sim flag
bool should_plan_paths=true; // --noplan flag
bool driver_test=false; // --driver_test, path planning testing

bool nodrive=false; // --nodrive flag (for testing indoors)


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

  robot_autodriver()
  {
    flush();

    // Add obstacles around the scoring trough
    for (int x=field_x_trough_start;x<=field_x_trough_end;x+=navigator_res)
    for (int y=field_y_trough_start;y<=field_y_trough_end;y+=navigator_res)
      mark_obstacle(x, y, 55);
      
    if (simulate_only && getenv("OBSTACLES")) {
      // seed random number generator with obstacles var
      srand(atoi(getenv("OBSTACLES")));
      for (int obs=0;obs<5;obs++) {
        int x=rand()%(field_x_size-2*30)+30;
        int y=rand()%(field_y_mine_zone-field_y_start_zone)+field_y_start_zone;
        int ht=obs>=3?5:50;  // last few are craters
        mark_disk_obstacle(x,y,ht,25);
      }
    }

    if (simulate_only && false) {
      // Add a few hardcoded obstacles, to show off path planning
      int x=130;
      int y=290;

      //Hard wall
      if (true)
       for (;x<=250;x+=navigator_res) mark_obstacle(x,y,15);

      // Isolated tall obstacle in middle
      navigator.mark_obstacle(130,y,40);
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

  // Flush autonomous drive state
  void flush() {
    has_path=false;
    replan_counter=0;
    planned_path=std::deque<planned_path_t>();
  }

  // Compute autonomous drive
  //   cur and target are (x,y) cm field coords and deg x angles.
  bool autodrive(vec2 cur,float cur_angle,
    vec2 target,float target_angle,
    double &forward,double &turn,
    robot_autonomy_state &debug)
  {
    const int replan_interval=1; // 1==every frame.  10==every 10 frames.

    const int plan_averaging=2; // steps in new plan to average together
    const int replan_length=2*plan_averaging; // distance of new plan to keep
    static rmc_navigator::navigator_t::drive_t last_drive;

    if (planned_path.size()<plan_averaging || (--replan_counter)<=0)
    { // refill planned path
      replan_counter=replan_interval;
      flush(); // flush old planned path
      debug.plan_len=0;
      
      // Start position: robot's position
      rmc_navigator::fposition fstart(cur.x,cur.y,cur_angle);
      // End position: at target
      rmc_navigator::fposition ftarget(target.x,target.y,target_angle);
      debug.target=ftarget;

      rmc_navigator::planner plan(navigator.navigator,fstart,ftarget,last_drive,false);
      int steps=0;
      for (const rmc_navigator::searchposition &p : plan.path)
      {
        planned_path.push_back(p);
        if (steps<replan_length)
        {
          p.print();
        }
        if (steps<robot_autonomy_state::max_path_len)
        {
          debug.plan_len=steps+1;
          debug.path_plan[steps]=p.pos;
        }

        steps++;
      }

      printf("Planned path from %.0f,%.0f@%.0f to target %.0f,%.0f@%.0f: %d steps\n",
          cur.x,cur.y,cur_angle,
          target.x,target.y,target_angle, steps);
      if (!plan.valid) {
        printf("Path planning FAILED: searched %zd cells\n",plan.searched);
        return false;
      }
    }
    int pathslots=plan_averaging;
    for (int slot=0;slot<plan_averaging;slot++) {
      if (slot<(int)planned_path.size()) {
        planned_path_t &p=planned_path[slot];
        forward += p.drive.forward;
        turn += p.drive.turn;
      }
      else { // short plan, use last known data
        forward +=last_drive.forward;
        turn +=last_drive.turn;
      }
    }
    forward = forward/pathslots;
    turn = turn/pathslots;

    // Cycle detection
    static int cycle_count=0;
    cycle_count--;
    if (cycle_count<0) cycle_count=0;
    if (forward * last_drive.forward <0 || turn * last_drive.turn <0)
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
    printf("Path planning forward %.1f, turn %.1f\n",forward,turn);

    // Update last_drive for next time
    if (planned_path.size()>0) last_drive=planned_path[0].drive;

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