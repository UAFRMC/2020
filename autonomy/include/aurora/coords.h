/* Robot coordinate systems, in 2D and 3D. 


Absolute field coordinates:
  Origin: 0,0,0 is on the ground at the bottom left corner of the arena.
     Units are centimeters.  
     +X is along the short axis of the field.
     +Y is along the long axis of the field.
     +Z is up from the ground.
  Angle: in degrees counterclockwise from the +X axis 
    angle = 0 puts robot forward along the +X direction.
    angle = 90 puts robot forward along the +Y direction.
    angle = 180 puts robot forward along the -X direction.
    angle = 270 puts robot forward along the -Y direction.

Robot coordinates:
  Origin: 0,0,0 is on the ground below the robot's turning center
  Angle=0 points along the robot's forward drive direction
  Angle=90 points left from the robot
  Angle=180 points behind the robot
  Angle=270 points right from the robot


*/
#ifndef __AURORA_COORDS_H
#define __AURORA_COORDS_H

#include <math.h> // for fmod
#include "../aurora/vec3.h"

namespace aurora {

// "vec3" is a 3D vector with overloaded operators.
// It's brought in via aurora/vec3.h, and is modeled after GLSL's vec3.


// Return this degree angle wrapped to 0-359.99999 degrees.
inline float normalize_angle(float angle) {
    angle = fmodf(angle,360.0f);
    if (angle<0.0) angle+=360.0f; //<- fmod is broken for negative inputs
    return angle;
}

struct robot_loc2D;

// Represents the full 3D coordinate system, which includes roll and pitch, 
//   as well as position and orientation.
//   Can represent a robot relative to the field, 
//   or a part of a robot relative to the robot.
struct robot_coord3D {
    vec3 origin; // origin of coordinate system, relative to parent system (or field)
    vec3 X,Y,Z; // unit vector axes of coordinate system: X is along vehicle travel, Y is across (robot's left side), Z is up
    float percent; // Percent confidence.  <=0.0 for "I don't know".  100% for absolute certainty. 
    
    // Create a limited 2D version of this coordinate system.
    //  Returns the current measured angular tilt error, in degrees.
    float get2D(robot_loc2D &loc) const;
    
    // FIXME: add print function
};


// Represents the 2D position and orientation of the robot on the field,
//   or the position and orientation of a sensor on a robot.
// Assumes roll and pitch are zero.
struct robot_loc2D {
    float x; // along short axis of the field
    float y; // along long axis of the field
    float angle; // robot heading: degrees counterclockwise from the +X axis
    float percent; // Percent confidence.   <=0.0 for "I don't know".  100% for absolute certainty.
    
    // Project out to a full 3D coordinate system
    robot_coord3D get3D(float height=0.0) const;
    
    void print(FILE *where=stdout, const char *terminator="\n") 
    {
        fprintf(where, "2Dpos: %6.1f  X, %6.1f  Y   %6.1f deg    %5.1f  %%sure%s",
                x,y,angle, percent, 
                terminator); 
    }
};



};


#endif
