/*
  Sensed location estimate for robot
*/
#ifndef __AURORA_pose_H
#define __AURORA_pose_H

#include "vec3.h"

/**
  One sensed 3D robot position and orientation.
  Class is Plain Old Data (POD), suitable for memcpy or network sends.
  
  Coordinate system:
    X axis: along collection bin wall
    Y axis: away from collection bin wall
    Z axis: vertical, 0 at ground level
*/ 
class robot_pose {
public:
  /** Our confidence (0-1) in this position */
  float confidence;
  
  /** Robot center-of-mass position, in centimeters. */
  vec3 pos; 
  
  /** fwd points in the direction the robot would drive.
      It's a unit direction vector. */
  vec3 fwd;
  
  /** rgt points toward the robot's right side. */
  vec3 rgt;
  
  robot_pose() {clear();}
  void clear() {
    confidence=0.0f;
    pos=vec3(0,0,0);
    fwd=vec3(1,0,0);
    rgt=vec3(0,-1,0);
  }
  
  void weighted_update(const robot_pose &ref) {
    float rw=ref.confidence/(ref.confidence+confidence+0.001); // reference weight
    float mw=1.0f-rw; // my weight
    confidence=confidence*mw+ref.confidence*rw;
    pos=pos*mw+ref.pos*rw;
    fwd=normalize(fwd*mw+ref.fwd*rw);
    rgt=normalize(rgt*mw+ref.rgt*rw);
  }
  
  void print(void) const {
    printf("  conf %.2f, pos=(%.0f,%.0f,%.0f), fwd=(%.1f,%.1f,%.1f), rgt=(%.1f,%.1f,%.1f)\n",
      confidence, pos.x,pos.y,pos.z, 
      fwd.x,fwd.y,fwd.z,
      rgt.x,rgt.y,rgt.z);
  }
};

/** 
  A list of all the sensed markers.
*/
class robot_markers_all {
public:

  // Integrated robot position
  robot_pose pose;

  // Raw sensed pose for each marker
  enum {NMARKER=35}; // <- all tag25h9 markers
  robot_pose markers[NMARKER];
  
  // Angle of receiver beacon, in degrees, up from horizontal (+X)
  float beacon;
  
  /// Add another sensed marker:
  ///    across points across the marker horizontally, in plane of marker
  ///    out points out horizontally from plane of marker
  void add(int markerID,  vec3 pos,vec3 across,vec3 out,  vec3 shift,int side) 
  {
    across=normalize(across);
    out=normalize(out);
    
    // Raw data goes into markers array
    robot_pose &m=markers[markerID];
    m.confidence=0.8;
    m.pos=pos;
    m.fwd=across;
    m.rgt=out;
    
    // Full reconstructed robot position updates pose
    robot_pose next;
    next.confidence=m.confidence;
    next.pos=pos+shift.y*out+shift.x*across;
    switch(side) {  // FIXME: check these transforms!
    case 0: next.fwd=+across; next.rgt=+out; break; // left
    case 1: next.fwd=-across; next.rgt=-out; break; // rgt 
    case 2: next.fwd=-out; next.rgt=+across; break; // back
    case 3: next.fwd=+out; next.rgt=-across; break; // fwd
    }
    pose.weighted_update(next); // FIXME: weighted average here
  }
    
};


#endif

