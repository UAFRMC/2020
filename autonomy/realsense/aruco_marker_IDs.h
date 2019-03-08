
// Store info about how the markers are scaled, positioned and oriented
struct marker_info_t {
	int id; // marker's ID, from 0-31
	float true_size; // side length, in meters, of black part of pattern
	
	int side; // side of the robot (0==right side, 1==left side, 2==back side, 3==front side)
	vec3 shift; // marker-relative translation to robot origin, in meters from center of pattern	
};

const static marker_info_t marker_info[]={
	{-1,0.145}, // fallback default case
	
	{13, 0.250}, // creeper (on 30cm full-size test plate)
	{17, 0.145}, // bird (on 20cm half-size test plate)
	{14, 0.145}, // cat
	{25, 0.100}, // bleachers 
	{16, 0.040}, // elipsis (on tiny 5cm test plate)
};

// Look up the calibration parameters for this marker
const marker_info_t &get_marker_info(int id) {
	for (int i=1;i<sizeof(marker_info)/sizeof(marker_info_t);i++) {
		if (marker_info[i].id==id) 
			return marker_info[i];
	}
	return marker_info[0];
}

