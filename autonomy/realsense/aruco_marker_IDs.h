
// Store info about how the markers are scaled, positioned and oriented
struct marker_info_t {
	int id; // marker's ID, from 0-31
	float true_size; // side length, in cm, of black part of pattern
	
	int side; // side of the robot (0==right side, 1==left side, 2==back side, 3==front side)
	vec3 shift; // marker-relative cm translation to robot origin, in meters from center of pattern	
};

#define robot_side_vec(x) vec3(x,66,0)

const static marker_info_t marker_info[]={
	{-1,14.5}, // fallback default case
	
	{13, 25.0, 0, -robot_side_vec(23) }, // creeper (on 30cm full-size test plate)
	{17, 14.5, 0, -robot_side_vec(0) }, // bird (on 20cm half-size test plate)
	{14, 14.5, 0, -robot_side_vec(0) }, // cat
	{25, 10.0, 0, -robot_side_vec(-15) }, // bleachers 
	{16,  4.0, 0, -robot_side_vec(-4) }, // elipsis (on tiny 5cm test plate)
};

// Look up the calibration parameters for this marker
const marker_info_t &get_marker_info(int id) {
	for (int i=1;i<sizeof(marker_info)/sizeof(marker_info_t);i++) {
		if (marker_info[i].id==id) 
			return marker_info[i];
	}
	return marker_info[0];
}

