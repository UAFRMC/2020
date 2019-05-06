
// Store info about how the markers are scaled, positioned and oriented
struct marker_info_t {
	int id; // marker's ID, from 0-31
	float true_size; // side length, in cm, of black part of pattern
	
	int side; // side of the robot (0==right side, 1==left side, 2==back side, 3==front side)
	vec3 shift; // marker-relative cm translation to robot origin, in meters from center of pattern	
};
#define robot_frontback 33  // half the X front-back length of robot
#define robot_side2side 66  // half the Y side-side width of robot
#define robot_side_vec(x) vec3(x,robot_side2side,0)

const static marker_info_t marker_info[]={
	{-1,14.5}, // fallback default case
	
	// true heading
	{28, 1.6, -1, vec3(0,0,-45)},
	{33, 1.6, -1, vec3(0,0,0)}, 
	{2, 1.6, -1, vec3(0,0,+45)}, 
	
	
/*
           Robot FRONT
  Robot LEFT       Robot RIGHT
     26 (18cm)        27 (18cm)
17(4cm)
 
 14 (14.5cm)                16 (14.5cm)
 13 (25cm)                  25 (25cm)

     6 (18cm)           7 (18cm)
           Robot BACK
*/

#if 1 // NEW single markers
   	{14, 14.5, 1, -robot_side_vec(0) }, // cat
    {13, 25.0, 1, -robot_side_vec(23) }, // creeper (on 30cm full-size test plate)
   	
	{17, 4.0, 2, -robot_side_vec(0) }, // bird (on 20cm half-size test plate)

	{16, 14.5, 0, +robot_side_vec(10) }, // elipsis (on tiny 5cm test plate)
	{25, 25.0, 0, +robot_side_vec(0) }, // bleachers 
	
	{ 6, 18.0, 2, -vec3(-robot_frontback,robot_side2side-11,10) }, // back left plate
	{ 7, 18.0, 2, -vec3(-robot_frontback,-robot_side2side+11,10) }, // back right
  
	{26, 18.0, 3, -vec3(robot_frontback,robot_side2side-11,10) }, // front left plate
	{27, 18.0, 3, -vec3(robot_frontback,-robot_side2side+11,10) }, // front right
    
#else // OLD left side only marker	
	// Left sideplate
	{13, 25.0, 1, -robot_side_vec(17) }, // creeper (on 30cm full-size test plate)
	{17, 14.5, 1, -robot_side_vec(0) }, // bird (on 20cm half-size test plate)
	{14, 14.5, 1, -robot_side_vec(-6) }, // cat
	{25, 10.0, 1, -robot_side_vec(-20) }, // bleachers 
	{16,  4.0, 1, -robot_side_vec(-4) }, // elipsis (on tiny 5cm test plate)
	
	{ 6, 18.0, 2, +vec3(robot_side2side-2-9,-robot_frontback,10) }, // back left plate
	{ 7,  5.0, 2, +vec3(robot_side2side-2-18-5,-robot_frontback,4.5) }, 
  
	{26, 18.0, 3, -vec3(robot_side2side-2-9,robot_frontback,10) }, // front left plate
	{27,  5.0, 3, -vec3(robot_side2side-2-18-5,robot_frontback,4.5) }, 
#endif
	
	
};

// Look up the calibration parameters for this marker
const marker_info_t &get_marker_info(int id) {
	for (int i=1;i<sizeof(marker_info)/sizeof(marker_info_t);i++) {
		if (marker_info[i].id==id) 
			return marker_info[i];
	}
	return marker_info[0];
}

