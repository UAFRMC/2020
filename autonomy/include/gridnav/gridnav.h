/*
Simple grid-based robot navigation package:

The grid has 3 dimensions:
  2D XY location in a fixed-size field.
  Z axis angular rotation, with wraparound.

The path planner walks through this grid in a lazy fashion
using the A* algorithm to expand the next closest node to the target.

Aurora Robotics 2017
*/
#ifndef AURORA_GRIDNAV_H
#define AURORA_GRIDNAV_H

#include <iostream>
#include <deque> 
#include <queue> // for priority_queue
#include <functional> // for reference_wrapper
#include <math.h>
#include "osl/vec2.h"

namespace gridnav {

  // Clearance height for open parts of robot
  enum {OPEN=1000};
  
  // User-defined class for robot geometric clearances
  class robot_geometry {
  public:
    virtual ~robot_geometry() {}
    
    // Return the clearance height at this (x,y) robot position in centimeters.
    //    (0,0) is the robot's origin, at the center of turning.
    //    +x is the direction the robot naturally drives forward
    //    Height 0 collides with all obstacles (e.g., tracks).
    //    Height 10 collides with obstacles of height >=10.
    //    Height OPEN does not collide with anything.
    virtual int clearance_height(float x,float y) const =0;
  };

/**
 Create a navigation framework for a space with:
    GRIDX by GRIDY 2D XY grid cells, of size GRIDSIZE centimeters each.
    GRIDA angular orientations representing angles from [0,360).
    The robot is approximately ROBOTGRID cells across.
*/
template <int GRIDX,int GRIDY,int GRIDA,int GRIDSIZE,int ROBOTGRID>
class gridnavigator {
public:
  typedef gridnavigator<GRIDX,GRIDY,GRIDA,GRIDSIZE,ROBOTGRID> navigator_t;

  // forward declarations
  class fposition;

  // This is a robot's position in the grid (integer coordinates)
  class gridposition {
  public:
    int x; // 0 .. GRIDX-1
    int y; // 0 .. GRIDY-1
    int a; // 0 .. GRIDA-1
    
    // Return true if this is a valid (in bounds) position.
    bool valid() const {
      if (x<0 || x>=GRIDX) return false;
      if (y<0 || y>=GRIDY) return false;
      if (a<0 || a>=GRIDA) return false;
      return true;
    }
    
    gridposition(int x_,int y_,int a_) :x(x_), y(y_), a(a_) {}
    
    gridposition(const fposition &p) {
      x=(int)floor(p.v.x*(1.0/GRIDSIZE)+0.5);
      y=(int)floor(p.v.y*(1.0/GRIDSIZE)+0.5);
      a=(int)floor(p.a+0.5); if (a==GRIDA) a=0;
    }
    
    bool operator==(const gridposition &p) const { 
      return x==p.x && y==p.y && a==p.a;
    }
  };
  
  // fmod has a bug for negative numbers.
  // This version always returns a value on [0,mod)
  static double fmodplus(double v,double mod) {
    double vm=fmod(v,mod);
    if (vm<0.0) vm+=mod;
    return vm;
  }
  
  // This is a robot's floating-point position in space (float coordinates)
  class fposition {
  public:
    vec2 v; // centimeters
    float a; // angle: 0 .. GRIDA
    
    // Return our angle, in degrees, from 0-360.   
    //    0 degrees is facing in the +X direction
    //    90 degrees is facing in the +Y direction
    float get_degrees(void) const { return a*(360.0/GRIDA); }

    // Set our angle, from degrees.
    void set_degrees(float deg) {
      deg=fmodplus(deg,360.0);  // wrap to 0..360
      a=deg*(GRIDA/360.0);
    }
    
    fposition() :v(-999.9,-999.9), a(-999.9) {}    

    // Library-internal interface: vector and GRIDA angle
    fposition(const vec2 &v_,float a_) :v(v_), a(a_) {  }

    // External interface: x,y, and degrees
    fposition(float x,float y,float deg_) :v(x,y) { set_degrees(deg_); }
    
    friend std::ostream &operator<<(std::ostream &o,const fposition &p) 
    {
      o.precision(3);
      o<<" {xy "<<p.v.x<<" "<<p.v.y<<"  ang "<<p.get_degrees()<<" } ";
      return o;
    }
  };

  // This is a 2D data grid for one slice.
  template <class T>
  class grid2D {
    T data[GRIDY][GRIDX];
  public:
    void clear(const T& clearvalue) {
      for (int y=0;y<GRIDY;y++)
      for (int x=0;x<GRIDX;x++)
        data[y][x]=clearvalue;
    }
    
    // Dump values to the screen
    void print(std::ostream &out,int scale=1) {
      // for (int y=0;y<GRIDY;y++) {
      for (int y=GRIDY-1;y>=0;y-=2) {
        for (int x=0;x<GRIDX;x++) {
          out<<(T)(data[y][x]/scale);
        }
        out<<"\n";
      }
    }
    
    // Accessors, without bounds check
    T &at(int x,int y) { return data[y][x]; }
    const T &at(int x,int y) const { return data[y][x]; }
  };
  
  // This represents everything we know about one slice of our domain
  class gridslice {
  public:
    // In this slice, the robot can drive in this direction
    //   This is a unit vector = vec2(cos(ang_rad),sin(ang_rad))
    vec2 drive;
    
    // Zero = no obstacle here
    // Positive = persistent obstacles
    grid2D<int> obstacle;
    
    // Zero = totally safe driving
    // Higher values = closer to obstacles
    grid2D<int> proximity;
    
    // Nonzero bits here mark that we've visited this cell
    grid2D<unsigned char> visit;
  };
  
  // The whole grid is just a list of slices, indexed by angle.
  gridslice slice[GRIDA];
  
  // Return the drive direction at this grid angle ID, in radians
  static float angle_to_radians(float ia) { return ia*(2.0*M_PI/GRIDA); }
  // Return the drive direction at this grid angle ID, in degrees
  static float angle_to_degrees(float ia) { return ia*(360.0/GRIDA); }
  
  // Set up our slices, assuming no obstacles and a point robot
  gridnavigator() {
    for (int ia=0;ia<GRIDA;ia++) {
      gridslice &s=slice[ia];
      float ang=angle_to_radians(ia);
      s.drive=vec2(cos(ang),sin(ang));
      s.obstacle.clear(0);
      s.proximity.clear(0);
      s.visit.clear(0);
    }
    obstacles.clear(0);
    lastpath.clear(' ');
  }
  
  // After marking obstacles, call this to compute proximity.
  //   This can be re-called if you mark new obstacles.
  void compute_proximity(int cells=3) {
    for (int ia=0;ia<GRIDA;ia++) {
      gridslice &s=slice[ia];
      s.proximity.clear(0);
      for (int y=-1;y<=GRIDY;y++)
      for (int x=-1;x<=GRIDX;x++)
      {
        if (!gridposition(x,y,0).valid() || s.obstacle.at(x,y)!=0) 
        { // mark all nearby cells as near obstacle
          for (int dy=-cells;dy<=cells;dy++)
          for (int dx=-cells;dx<=cells;dx++)
          {
            int cy=y+dy;
            int cx=x+dx;
            if (gridposition(cx,cy,0).valid())
            {
              int dist=cells+1 - std::max(std::abs(dx),std::abs(dy));
              int &prox=s.proximity.at(cx,cy);
              if (prox<dist) prox=dist;
            }
          }
        }
      }
    }
  }
  

/************ Robot Geometry ************************/
  // At a given angle, this lists the discretized grid points relative to the robot origin.
  //   x,y are relative to the robot origin.  a=clearance_height.
  typedef std::vector<gridposition> robot_grid_slice;
  
  // Discretized grid version of robot geometry at each orientation.
  //   Basically a mask for placing around obstacles.
  //   Build this once, at startup.
  class robot_grid_geometry {
  public:
    // For each angle, this gives the corresponding robot clearances
    robot_grid_slice slice[GRIDA];
    
    robot_grid_geometry(const robot_geometry &geo,bool verbose=false) {
      // For each rotation angle:
      for (int ia=0;ia<GRIDA;ia++) {
        if (verbose) std::cout<<"Robot geometry at grid angle "<<angle_to_degrees(ia)<<"\n";
        robot_grid_slice &robotslice=slice[ia];
        float ang=angle_to_radians(ia);
        float delta_ang=angle_to_radians(ia+1)-ang;
        
        enum {CHECK=ROBOTGRID}; // grid cells to search

        // Loop over grid cells relative to robot origin:
        // for (int y=-CHECK;y<=+CHECK;y++)
        for (int y=+CHECK;y>=-CHECK;y--) // <- count down, so ascii art is upright
        for (int x=-CHECK;x<=+CHECK;x++) {

            int height=OPEN;
            
            // Check robot clearance height through the rotations of this grid cell:
            for (double rot=-1.0;rot<=+1.0;rot+=1.0) {
              float c=cos(ang+0.5*delta_ang*rot), s=sin(ang+0.5*delta_ang*rot);
              for (double dy=-0.5;dy<=+0.5;dy+=0.25)
              for (double dx=-0.5;dx<=+0.5;dx+=0.25) {
                vec2 g(GRIDSIZE*(x+dx),GRIDSIZE*(y+dy)); // grid position
                // rotate to robot coordinates
                vec2 r( c*g.x + s*g.y,
                       -s*g.x + c*g.y );
                height=std::min(height,geo.clearance_height(r.x,r.y));
              }
            }
            
            char height_debug=' ';
            if (height<OPEN) {
              robotslice.push_back(gridposition(x,y,height));
              height_debug='0'+height;
            }
            if (verbose && y%2==0) { // skip odd rows to fix aspect ratio
              if (y==0 && x==0) height_debug='+'; // origin
              if (y==2 && x==0) height_debug='y'; // +y direction
              if (y==0 && x==1) height_debug='x'; // +x direction
              std::cout<<height_debug; // ascii art picture
              if (x==+CHECK) std::cout<<"\n";
            }
        }
      }
    }
  };
  
  
  // This grid records all known obstacles on the field.
  //   zero indicates no known obstacles
  //   They're kept here to avoid redundant obstacle updates.
  grid2D<int> obstacles;
  
  // This grid stores the winning path (for display)
  grid2D<char> lastpath;
  
  // Mark this obstacle location, xy in grid coordinates, as impassible for this robot.
  //  This can be called at startup, or at runtime for new obstacles.
  void mark_obstacle(int x,int y,int height, const robot_grid_geometry &robot) {
    if (gridposition(x,y,0).valid()) {
      int &old=obstacles.at(x,y);
      if (old>=height) return; // redundant obstacle
      else old=height;
    }
    
    // Mark where the robot would hit this obstacle in each orientation.
    //   The corresponding robot center points are blocked.
    for (int ia=0;ia<GRIDA;ia++) {
      gridslice &navslice=slice[ia];
      const robot_grid_slice &robotslice=robot.slice[ia];
      for (gridposition g : robotslice) {
        if (g.a<height) { // robot would hit this obstacle
          gridposition hit(x-g.x, y-g.y, ia);
          int &store=navslice.obstacle.at(hit.x,hit.y);
          if (hit.valid() && store<height)
            store=height;
        }
      }
    }
  }
  
  // Mark the edges of the grid as impassible for this robot
  void mark_edges(const robot_grid_geometry &robot) {
    for (int y=-1;y<=GRIDY;y++)
    for (int x=-1;x<=GRIDX;x++)
    {
      if (!gridposition(x,y,0).valid())
        mark_obstacle(x,y,99,robot);
    }
  }
  

/*************** Path Planning *************************/
  // Convert discrete angles to grid cell equivalent.
  //   This scale factor is derived figuring the tip velocity of a turning robot.
  static constexpr double TURN_COST_TO_GRID_COST=((2.0*M_PI/GRIDA)*(ROBOTGRID*0.5));
  
  
  // This class represents a driving command used during the path.
  class drive_t {
  public:
    float forward; // -1 backwards, 0 stopped, 1 forwards
    float turn; // -1 left, 0 straight, +1 right
    drive_t(float f=0.0,float t=0.0) :forward(f), turn(t) {}
    
    void operator+=(const drive_t &d) { forward+=d.forward; turn+=d.turn; }
    bool operator==(const drive_t &d) const { return forward==d.forward && turn==d.turn; }
    
    friend std::ostream &operator<<(std::ostream &o,const drive_t &d) 
    {
      o.precision(3);
      o<<" {forward "<<d.forward<<"  turn "<<d.turn<<" } ";
      return o;
    }
  };

  // This class represents an intermediate position in a path search.
  class searchposition {
  public:
    double cost; // cost to get here
    double estimate; // estimate to reach target
    drive_t drive;
    fposition pos; // position of the robot at this point
    const searchposition *last; // preceding search position, or NULL if none.
    
    searchposition(double cost_,double estimate_, const drive_t &drive_,const fposition &pos_,const searchposition *last_)
      :cost(cost_), estimate(estimate_), drive(drive_), pos(pos_), last(last_)
    {}
    
    // Return the approximate cost to reach the target: 
    //  the cost to get here, plus estimate to reach target
    double total_cost(void) const {
      return cost + estimate;
    }
    
    // Sort so we always grab the best search position first
    bool operator>(const searchposition &p) const {
      return total_cost() > p.total_cost();
    }
    
    void print(void) const {
      printf(" Drive %.1f turn %.1f at %.0f,%.0f@%.0f cost %.3f estimate %.3f\n",
        drive.forward,drive.turn,
        pos.v.x,pos.v.y,pos.get_degrees(),
        cost,estimate);
    }
  };
  
  /* A planner_target template needs these fields:

    // Returns A* heuristic value at this grid cell
    double get_cost_from(const gridposition &from_pos) const 

    // Returns true if we've reached the goal state at this grid cell
    bool reached_target(const gridposition &grid) const
  */
  
  // A planner_target is basically just a cost function.
  class planner_target {
  public:
    /* Return the estimated cost to reach the target from this position.
       If you're at the target, return 0. */
    virtual double get_cost_from(const gridposition &from_pos) const =0;
    
    /* Return true if we've reached the target. */
    virtual bool reached_target(const gridposition &grid) const =0;
    
    
    /* Utility function: return the angular distance */
    float angle_dist(float delta) const {
       delta=fmodplus(delta,GRIDA);
       if (delta>GRIDA/2) delta=GRIDA-delta; // turn the other way
       return delta;
    }

  protected:
    ~planner_target() {} // only subclasses will get deleted, not these parent classes.
  };
  
  // Planner target is a simple 2D target point
  class planner_target_2D : public planner_target {
    // This is our search target configuration
    //fposition target;
    gridposition gtarget;
  public:
    planner_target_2D(const fposition &target_)
      ://target(target_), 
       gtarget(gridposition(target_)) {}
    
    virtual double get_cost_from(const gridposition &from_pos) const
    {
      double drive_dist=length(vec2(from_pos.x,from_pos.y)-vec2(gtarget.x,gtarget.y)); // in grid cells
      double turn_ang=this->angle_dist(from_pos.a-gtarget.a); // in discrete angle units
      double TURN_AMPLIFY=5.0;
      double estimate = drive_dist + TURN_AMPLIFY*turn_ang*TURN_COST_TO_GRID_COST;
      return estimate;
    }
    
    virtual bool reached_target(const gridposition &grid) const 
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
    
    ~planner_target_2D() {}
  };
  
  
  // Build the sequence of steps needed to move the robot from origin to target.
  class planner {
    navigator_t &nav;
    
    // This is the active list of searched positions, sorted by distance to target.
    typedef std::priority_queue<std::reference_wrapper<searchposition>,
      std::vector< std::reference_wrapper<searchposition> >,
      std::greater<searchposition> > search_t;
    search_t search;
    
    // SUBTLE: to allow backtracking, we need to persistently store search positions.
    //  std::deque has the nice property of not invalidating references.
    typedef std::deque<searchposition> pool_t;
    pool_t pool; // safely stores our search positions
    
    // Add this search position to our priority queue.
    //   Returns true if the point was valid and could be added.
    bool add_search(double cost,const drive_t &lastdrive,const drive_t &drive,const fposition &pos,const planner_target &target,const searchposition *last) {
      // make sure we haven't already visited this point
      gridposition g(pos);
      if (!g.valid()) return false; // out of bounds of our grid
      
      gridslice &s=nav.slice[g.a];
      unsigned char &visited=nav.slice[g.a].visit.at(g.x,g.y);
      if (0!=visited) return false; // already visited here
      
      // create data structure to hold this point
      visited=1; // mark as visited
        
      // Check for obstacles in the way:
      const unsigned char &obs=s.obstacle.at(g.x,g.y);
      //if (obs!=0) return false; // has obstacle
      if (obs!=0) cost += 10000.0; // hitting obstacle counts as a 10 meter cost
      
      // Seems legal, so use heuristic to estimate cost to target
      
      if (!(lastdrive == drive)) 
        cost+=20.0; // penalty for swapping drive directions
      
      double estimate=target.get_cost_from(g);
      pool.emplace_back(cost, estimate, drive,pos,last);
      search.push(pool.back());

      return true; // OK point
    }
    
  public:
    // This bool marks that we found a good path.
    //    false == "you can't get there from here"
    bool valid;
    
    // This is the sequence of steps from origin to target
    std::deque<searchposition> path;
    
    // This is how many cells we expanded
    size_t searched;
    
    
    planner(navigator_t &nav_,const fposition &origin,const planner_target &target,drive_t last_drive=drive_t(), bool verbose=false) 
      :nav(nav_)
    {
      valid = plan_path(origin,target,last_drive,verbose);
    } 
    
    planner(navigator_t &nav_,const fposition &origin,const fposition &ftarget, drive_t last_drive=drive_t(), bool verbose=false) 
      :nav(nav_)
    {
      planner_target_2D target(ftarget);
      valid = plan_path(origin,target,last_drive,verbose);
    }
    
    
    bool plan_path(const fposition &origin,const planner_target &target,drive_t last_drive=drive_t(), bool verbose=false)
    {
      // Clear all previous visit marks
      searched=0;
      for (int ia=0;ia<GRIDA;ia++) {
        nav.slice[ia].visit.clear(0);
      }
      nav.lastpath.clear(' ');
      
      // Start search at specified origin
      if (!add_search(0.0,last_drive,last_drive,origin,target,0)) 
      { // Start point no good: can't do this.
        std::cout<<"Starting point is off the grid!?\n";
        return false;
      }
    
      // Repeatedly visit positions with the cheapest net cost
      while (!search.empty()) {
        const searchposition &cur=search.top(); search.pop();
        searched++;
        
        // if (verbose) std::cout<<"At "<<cur.pos<<" cost "<<cur.cost<<": ";
        
        // Find the grid location for this cell
        gridposition gcur(cur.pos);
        if (verbose && gcur.valid()) {
          nav.lastpath.at(gcur.x,gcur.y)='.'; // checked
        }
        if (target.reached_target(gcur)) 
        { // we're done!  Follow the chain of "last" pointers back to the origin.
          if (verbose) std::cout<<"Target reached!  Reverse path:\n";
          for (const searchposition *p=&cur;p!=NULL;p=p->last) {
            if (p->last!=NULL)
              path.push_front(*p);
            if (verbose) {
              std::cout<<p->pos<<"\n";
              gridposition gpath(p->pos);
              if (gpath.valid()) nav.lastpath.at(gpath.x,gpath.y)='#'; // on path
              nav.lastpath.print(std::cout,1);
            }
          }
          
          // Clean out the pool for next time
          search=search_t();
          pool=pool_t();
          
          return true;
        }
        
        // Compute 'obstacle proximity cost' scaling factor
        double turncost=1.0; // extra cost for more turning
        double proxcost=10.0; // extra cost for driving near obstacles
        if (gcur.valid())
          proxcost = 1.0 + 0.5*nav.slice[gcur.a].proximity.at(gcur.x,gcur.y);
        
        // Check all nearby cells
        vec2 drivedir=nav.slice[gcur.a].drive;
        for (float drive=-1.0;drive<=+1.0;drive+=2.0) {
          // Drive at least into the next grid cell:
          double distance=1.01*GRIDSIZE/std::max(fabs(drivedir.x),fabs(drivedir.y)); 
          
          double cost=cur.cost + proxcost * distance;
          fposition next(cur.pos.v+distance*drive*drivedir,cur.pos.a);
          add_search(cost,cur.drive,drive_t(drive,0.0f),next,target,&cur);
        }
        for (float turn=-1.0;turn<=+1.0;turn+=2.0) {
          double cost=cur.cost + proxcost *turncost * GRIDSIZE*TURN_COST_TO_GRID_COST;
          fposition next(cur.pos.v,fmodplus(cur.pos.a+turn,GRIDA)); // angles wrap around
          add_search(cost,cur.drive,drive_t(0.0f,turn),next,target,&cur);
        }
      }
      
      // If we get here, we ran out of options before reaching the target.
      if (verbose) std::cout<<"Out of search options!\n";
      return false;
    }
  }; // end planner class

}; // end templated class gridnavigator

}; // end namespace gridnav


#endif





