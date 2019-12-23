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
#include "../aurora/vec3.h" // 3D vector
#include "../osl/vec2.h" // 2D vector


#ifndef M_PI
#define M_PI 3.14159265358979323
#endif

#ifndef DEG2RAD /* degrees to radians and back */
#define DEG2RAD (M_PI/180.0)
#define RAD2DEG (180.0/M_PI)
#endif

namespace aurora {

// "vec3" is a 3D vector with overloaded operators.
// It's brought in via aurora/vec3.h, and is modeled after GLSL's vec3.


// Return this degree angle wrapped to 0-359.99999 degrees.
inline float normalize_angle(float angle) {
    angle = fmodf(angle,360.0f);
    if (angle<0.0) angle+=360.0f; //<- fmod is broken for negative inputs
    return angle;
}

// Return a 3D unit vector representing this 2D heading angle, in degrees.
vec3 vec3_from_angle(float angle_deg) {
    float angle_rad=angle_deg*DEG2RAD; // to radians
    return vec3(cos(angle_rad),sin(angle_rad),0.0f);
}
// Rotate this vector 90 degrees right-handed around the Z axis
vec3 rotate_90_Z(const vec3 &v) {
    return vec3(-v.y,v.x,v.z);
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
    
    // Project this 3D local point into a 3D world coordinates position
    vec3 world_from_local(const vec3 &robot) const {
        return origin+X*robot.x+Y*robot.y+Z*robot.z;
    }
    // Project this 3D local direction into a 3D world coordinates direction
    vec3 world_from_local_dir(const vec3 &robot) const {
        return X*robot.x+Y*robot.y+Z*robot.z;
    }
    
    // Project this world-coordinates point into robot coordinates
    vec3 local_from_world(const vec3 &world) const {
        vec3 rel=world-origin;
        return vec3(rel.dot(X),rel.dot(Y),rel.dot(Z));
    }
    
    robot_coord3D() { reset(); }
    
    // Reset to origin at (0,0,0), no rotations on XYZ
    void reset() {
        origin=vec3(0,0,0);
        X=vec3(1,0,0); Y=vec3(0,1,0); Z=vec3(0,0,1);
        percent=0;
    }
    
    // Compose these coordinate systems, where we are the parent,
    //   and the child coordinates are relative to our coordinates, 
    //   and the composition goes directly from child to world coordinates.
    robot_coord3D compose(const robot_coord3D &child) const {
        robot_coord3D out;
        out.origin=world_from_local(child.origin);
        out.X=world_from_local_dir(child.X);
        out.Y=world_from_local_dir(child.Y);
        out.Z=world_from_local_dir(child.Z);
        out.percent=percent*child.percent/100.0;
        return out;
    }
    
    // Create a limited 2D version of this coordinate system.
    //  Returns the current measured angular tilt error, in degrees.
    float get2D(robot_loc2D &loc) const;
    
    void print(FILE *where=stdout, const char *terminator="\n") const
    {
        fprintf(where, 
            "origin: %6.1f %6.1f %6.1f / X: %5.2f %5.2f %5.2f  Y:  %5.2f %5.2f %5.2f Z:  %5.2f %5.2f %5.2f / %5.1f %%sure%s",
            origin.x,origin.y,origin.z,
            X.x,X.y,X.z,
            Y.x,Y.y,Y.z,
            Z.x,Z.y,Z.z,
            percent, 
            terminator); 
    }
};


// Represents the 2D position and orientation of the robot on the field,
//   or the position and orientation of a sensor on a robot.
// Assumes roll and pitch are zero.
struct robot_loc2D {
    float x; // along short axis of the field
    float y; // along long axis of the field
    float angle; // robot heading: degrees counterclockwise from the +X axis
    float percent; // Percent confidence.   <=0.0 for "I don't know".  100% for absolute certainty.
    

// Utility functions
	/*
	  Return a world coordinates unit 2D direction vector
	  for this robot-relative angle.
	  Angle==0 is facing along the robot's forward axis, in the direction of motion.
	  Angle==90 is facing to the robot's left (counterclockwise, normal math angles).
	*/
	vec2 dir_from_deg(float ang_deg=0.0) const {
		float ang=(this->angle+ang_deg)*DEG2RAD;
		return vec2(cos(ang),sin(ang)); 
	}
	// Return a robot coordinates angle from this direction vector.
	//  angles range from -180 to +180
	float deg_from_dir(const vec2 &dir) const {
		float world_deg=RAD2DEG*atan2(dir.y,dir.x); 
		float deg=world_deg-this->angle;
		if (deg<-180.0) deg+=360.0;
		if (deg>+180.0) deg-=360.0;
		return deg;
	}

	vec2 center(void) const { return vec2(x,y); }
	vec2 forward(void) const { return dir_from_deg(0.0); }
	vec2 right(void) const { return dir_from_deg(-90.0); }
    
    
    // Project us out to a full 3D coordinate system
    robot_coord3D get3D(float height=0.0) const {
        robot_coord3D loc;
        loc.origin=vec3(x,y,height);
        loc.X=vec3_from_angle(angle);
        loc.Y=rotate_90_Z(loc.X);
        loc.Z=vec3(0,0,1);
        loc.percent=percent;
        return loc;
    }
    
    void print(FILE *where=stdout, const char *terminator="\n") const
    {
        fprintf(where, "2Dpos: %6.1f  X, %6.1f  Y   %6.1f deg    %5.1f  %%sure%s",
                x,y,angle, percent, 
                terminator); 
    }
};



};

// Old name for robot_loc2D: robot_localization
typedef aurora::robot_loc2D robot_localization;


#endif
