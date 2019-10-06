/**
  Holds a RealSense sensor, Raspberry Pi processor,
  and 28BYJ geared stepper motor to make a computer-controlled
  depth camera station.
  
  Dr. Orion Lawlor, lawlor@alaska.edu, 2018-10 (Public Domain)
*/

// Smoothness of rounded parts
$fs=0.2; $fa=10; // fast
$fs=0.1; $fa=5; // smooth



// Model units are in mm
inch=25.4;

// Horizontal field of view for realsense
realsense_hfov=87;
// Vertical field of view for realsense
realsense_vfov=60;

// Body of sensor (as a cube)
realsense_body=[90,25,25];
// Location of depth reference relative to center of body
//   See realsense functional specification PDF, around page 61
realsense_reference_point=[-17.5,0,+4.2];
// Printed orientation of realsense casing
realsense_reference_orient=[30,0,0];

// Space to leave for ventilation around realsense
realsense_ventilation=2;
// Round off the corners this far
realsense_rounding=10;

// USB-C cable, at RealSense end
usb_C=[150,14,8];

// Round this 2D profile like a realsense
module realsense_outside_2D_rounding(more_rounding=0) {
	r=realsense_rounding+more_rounding;
	offset(r=+r+realsense_ventilation) offset(r=-r) 
		children();
}

// Rounded outside profile of realsense sensor
module realsense_outside_2D() {
	realsense_outside_2D_rounding()
		square([realsense_body[0],realsense_body[1]],center=true);
}

// Far-plane field of view
module realsense_view_2D(viewscale=1,farplane) {
	translate([0,0,farplane])
	minkowski() {
		smear_left=12*viewscale;
		translate([-smear_left/2,0]) 
			square([
				smear_left + 2*tan(realsense_hfov/2)*farplane,
				2*tan(realsense_vfov/2)*farplane
				],center=true);
		realsense_outside_2D();
	}
}

// Solidified field of view
module realsense_view_3D(fatten=0,viewscale=1) {
	translate([0,0,-3])
	hull() {
		linear_extrude(height=1,convexity=2)
			offset(r=fatten) realsense_outside_2D();
		
		farplane=20*viewscale-fatten;
		translate([0,0,farplane])
		linear_extrude(height=1,convexity=2)
			offset(r=fatten) realsense_view_2D(viewscale,farplane);
	}
}

// Realsense container, generic version
module realsense_holder_3D(fatten=0,viewscale=1) {
	realsense_view_3D(fatten,viewscale);
	
	translate([0,0,-realsense_body[2]-fatten+0.1])
	linear_extrude(height=realsense_body[2]+2*fatten,convexity=2)
		offset(r=fatten) realsense_outside_2D();
}

// Back bolt mounts
realsense_backbolt_length=6; // M3 x this length
module realsense_backbolt_mounts() {
	// Back mounting holes are for M3 bolts
	for (LR=[-1,+1]) translate([LR*45/2,0,-realsense_body[2]+2.2-realsense_backbolt_length])
		children();
}

// Realsense outside surfaces
module realsense_plus(wall_thickness=2.0) {
	rotate(realsense_reference_orient)
	translate(realsense_reference_point)
	union() {
		realsense_holder_3D(wall_thickness,1.0);
		
		realsense_backbolt_mounts() 
			cylinder(d1=7,d2=15,h=realsense_backbolt_length);
	}
}

// Realsense inside holes
module realsense_minus(wall_thickness=2.0) {
	rotate(realsense_reference_orient)
	translate(realsense_reference_point)
		union() {
			
			bottom_bolt_Z=-15;
			
			// Space for body and view
			difference() {
				realsense_holder_3D(0.0,2.0);
				
			}
			
			realsense_backbolt_mounts() 
			union() {
				// Thru hole
				translate([0,0,-0.1]) cylinder(d=3.1,h=20,center=true);
				// Socket head insert path
				scale([1,1,-1]) cylinder(d=7,h=20);
			}
			
			
			// USB-C plug
			translate([-realsense_body[0]/2,0,-realsense_body[2]+0.5*usb_C[2]])
				cube(usb_C,center=true);
				
			
		}
}

// Thickness of all support walls
wall=1.2;

// Top and bottom pivot planes
pivot_top_Y=25;
pivot_bottom_Y=-25;

box_left_X=-93;
box_right_X=46;
box_top_Y=pivot_top_Y-12;
box_bottom_Y=pivot_bottom_Y;
box_back_Z=-27;
box_top_Z=30;

raincube_extra_Y=7;


// Bottom side mount bolt locations
module bottom_mountbolts(inside=0) {
	for (x=[-50,0]) translate([x,-8,box_back_Z])
		if (inside)
			cylinder(d=2.6,h=10);
		else // outside
			cylinder(d1=14,d2=4,h=8);
}


module electronics_box(shrink=0) {
	translate([0,0,box_back_Z+shrink])
	{
		// rain cube
		translate([-80+shrink,box_bottom_Y+shrink,0])
			cube([110-2*shrink,box_top_Y-box_bottom_Y+raincube_extra_Y-2*shrink,25-2*shrink]);
		
		linear_extrude(height=box_top_Z-box_back_Z) 
		offset(r=+realsense_rounding) offset(r=-realsense_rounding) 
		offset(r=-shrink)
		{
			translate([box_left_X,box_bottom_Y])
			square([box_right_X-box_left_X,
				box_top_Y-box_bottom_Y]);
		}
	}
}


module realsense_mount_box_complete() {
	difference() {
		union() {
			realsense_plus(wall);
			
			difference() {
				electronics_box(0.0);
				electronics_box(wall);
			}
			
			// Supports underneath realsense (avoid spaghetti)
			translate([-12.5,0,0])
			for (slice=[-1.5:1.0:+1.5])
				translate([slice*25-wall/2,box_bottom_Y,box_back_Z])
					cube([wall,box_top_Y-box_bottom_Y+raincube_extra_Y,25]);
			
			
			// Reinforcing around top pivot bolt
			translate([0,pivot_top_Y,0]) 
				rotate([90,0,0])
					cylinder(d2=30,d1=5,h=15);
			
			bottom_mountbolts(0);
		}
		
			bottom_mountbolts(1);
			translate([0,pivot_top_Y,0]) 
				rotate([90,0,0])
					cylinder(d=2.8,h=12,center=true);
		
		realsense_minus();
		
		// USB-C cable re-enters box: not needed, can re-use thru hole
		//translate([box_left_X,-25,box_back_Z+10])
		//	cube(usb_C,center=true);
		
		// Cutaway
		//translate([1000-2,0,0]) cube([2000,2000,2000],center=true);
		//translate([0,-1000-20,0]) cube([2000,2000,2000],center=true);
		
	}
}



// Printable versions:
realsense_mount_box_complete();
