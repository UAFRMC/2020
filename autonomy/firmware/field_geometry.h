/*
 Geometric dimensions, in cm, of the mining field.
*/
#ifndef __AURORA_FIELD_GEOM_H
#define __AURORA_FIELD_GEOM_H

/** These are the robot's dimensions, in centimeters. */
enum {
  robot_x=70/2, // Center to front-back of main robot, in cm
  robot_y=132/2, // Center to left-right of main body, in cm
  robot_track_y=20, // width of tracks, frame, motors, etc
  robot_inside_clearance=25, // Z between box and tracks
  
  robot_box_y=30, // half of box width
  robot_box_clearance=25, // Z clearance under storage box
  
  robot_mine_x=robot_x+19, // center of robot's mining head
  robot_mine_y=0, // mining head center
  robot_mine_radius=15, // radius around robot mining head
  robot_mine_clearance=13, // Z under mining head
};


/**
  These are the field dimensions, in centimeters.
  Y runs up the field, with the scoring trough on the Y==0 side.
  X runs across the field, from left to right side.  X==0 is the left side.
  Both coordinates are always positive.
*/
enum {
	field_y_size=738, // Y-length of field, in centimeters
//	field_y_size=500, // Y-length of field, in centimeters for the test arena
	field_y_start_zone=183, // y end of start area, in centimeters
	field_y_mine_zone=field_y_start_zone+294, // y where mining area starts
	field_y_mine_start=field_y_mine_zone+30, // y where it's safe to start mining
	
  field_x_size=378, // X-width of field, in cm

#define TROUGH_LEFT 1
#if TROUGH_LEFT
  field_x_trough_edge=55, // Dump edge of scoring trough (left-side dump)
  field_x_trough_stop=field_x_trough_edge+(robot_x-10), // stop here
  
  field_x_trough_align=field_x_trough_edge+20, // drive target
  field_x_trough_start=0,
  field_x_trough_end=field_x_trough_edge,

  field_angle_trough=0, // Robot's heading angle when dumping at trough (left)
  
#else // Trough is right
  field_x_trough_edge=field_x_size-55, // Edge of scoring trough (right-side dump)
  field_x_trough_stop=field_x_trough_edge-(robot_x-10),
  
  field_x_trough_align=field_x_trough_edge-20,
  field_x_trough_start=field_x_trough_edge,
  field_x_trough_end=field_x_size,

  field_angle_trough=180, // Robot's heading angle when dumping at trough (right)
#endif
  
  field_y_trough_center=105, // Center of scoring trough, minus space for beacon
  field_x_trough_center=37, // X
  
  field_y_trough_size=165, // length of trough in Y axis
  field_y_trough_hsize=field_y_trough_size/2, // length of trough in Y axis
  field_y_trough_start=field_y_trough_center-field_y_trough_hsize,
  field_y_trough_end=field_y_trough_center+field_y_trough_hsize,
  
  field_y_beacon=field_y_trough_end-10, // center point of beacon
  field_x_beacon=field_x_trough_edge,
  
  
	field_x_GUI=field_x_size+20, // start X for GUI display
};



#endif

