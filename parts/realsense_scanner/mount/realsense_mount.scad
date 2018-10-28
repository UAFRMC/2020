

// Smoothness of rounded parts
$fs=0.2; $fa=10; // fast
$fs=0.1; $fa=5; // smooth



// Model units are in mm
inch=25.4;

// Horizontal field of view for realsense
realsense_hfov=90; // 86;
// Vertical field of view for realsense
realsense_vfov=60; // 57;
// Tilt downward for realsense
realsense_tilt=realsense_vfov/2;

// Body of sensor (as a cube)
realsense_body=[90,25,25];
// Location of depth reference relative to center of body
//   See realsense functional specification PDF, around page 61
realsense_reference_point=[-17.5,0,+4.2];
// Printed orientation of realsense casing
realsense_reference_orient=[20,0,0];

// Space to leave for ventilation around realsense
realsense_ventilation=2;
// Round off the corners this far
realsense_rounding=10;

// USB-C cable, at RealSense end
usb_C=[100,14,8];

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
module realsense_view_2D(more_rounding=0,farplane) {
	translate([0,0,farplane])
	minkowski() {
		realsense_outside_2D_rounding(more_rounding)
			square([
				2*tan(realsense_hfov/2)*farplane,
				2*tan(realsense_vfov/2)*farplane
				],center=true);
		realsense_outside_2D();
	}
}

// Solidified field of view
module realsense_view_3D(fatten=0,extra_view=0) {
	translate([0,0,-3])
	hull() {
		linear_extrude(height=1,convexity=2)
			offset(r=fatten) realsense_outside_2D();
		
		farplane=50+extra_view-fatten;
		translate([0,0,farplane])
		linear_extrude(height=1,convexity=2)
			offset(r=fatten) realsense_view_2D(extra_view*0.5,farplane);
	}
}

// Realsense container, generic version
module realsense_holder_3D(fatten=0,extra_view=0) {
	realsense_view_3D(fatten,extra_view);
	
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
		realsense_holder_3D(wall_thickness,0.0);
		
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
				realsense_holder_3D(0.0,25.0);
				
				// Inset for bottom mounting bolt
				translate([0,-0.5*realsense_body[1]-1,bottom_bolt_Z]) rotate([90,0,0]) 
					cylinder(d1=0.5*inch,d2=0.5*inch+20,h=10);
				
			}
			
			realsense_backbolt_mounts() 
			union() {
				// Thru hole
				translate([0,0,-0.1]) cylinder(d=3.1,h=20,center=true);
				// Socket head insert path
				scale([1,1,-1]) cylinder(d=7,h=20);
			}
			
			// Bottom mounting hole is for a 1/4 inch bolt
			translate([0,0,bottom_bolt_Z]) rotate([90,0,0]) cylinder(d=0.25*inch+1,h=20);
			
			// USB-C plug
			translate([-realsense_body[0]/2,0,-realsense_body[2]+0.5*usb_C[2]])
				cube(usb_C,center=true);
				
			
		}
}

// Thickness of all support walls
wall=1.2;

// Top and bottom pivot planes
pivot_top_Y=25;
pivot_bottom_Y=-175;

box_left_X=-105;
box_right_X=70;
box_top_Y=pivot_top_Y-5;
box_bottom_Y=pivot_bottom_Y;
box_back_Z=-26;
box_top_Z=51;


module electronics_box(shrink=0) {
	translate([0,0,box_back_Z+shrink])
	linear_extrude(height=box_top_Z-box_back_Z) 
	offset(r=+realsense_rounding) offset(r=-realsense_rounding) 
	offset(r=-shrink)
	{
		translate([box_left_X,box_bottom_Y])
		square([box_right_X-box_left_X,
			box_top_Y-box_bottom_Y]);
	}
}

// Generic module to create 4 mounting bolts
module mounting_bolts_4x(width,height,startx,starty) {
	for (dx=[0,width])
		for (dy=[0,height])
			translate([startx+dx,starty+dy,0])
				children();
}


// Emergency stop / on-off switch box
estop_hole_dia=22.5;
estop_box_size=[45,37,30];
estop_box_corner=[box_right_X-2*wall,-46,box_back_Z+2*wall+0.5*estop_box_size[2]];

module estop() {
	translate(estop_box_corner) color([1,0,0]) union() {
		translate([-estop_box_size[0],-estop_box_size[1]/2,-estop_box_size[2]/2]) 
			cube(estop_box_size);
		rotate([0,90,0]) {
			cylinder(d=estop_hole_dia,h=20);
			translate([0,0,10]) cylinder(d=60,h=2);
			translate([0,0,22]) cylinder(d=40,h=5);
			
			
		}
	}
}

// Cooling fan (optional)
fan_size=[40,40,11];
fan_corner=[-30,-70,box_back_Z+wall];
module fan_box() {
	translate(fan_corner) cube(fan_size);
}
module fan_bolts()
{
	translate(fan_corner) mounting_bolts_4x(32,32,4,4) children();
}

module fan_clearance()
{
	step=10;
	fanhole=step-wall;
	start=10;
	translate(fan_corner)
	for (dy=[start:step:fan_size[1]-start])
	for (dx=[start:step:fan_size[0]-start])
			translate([dx,dy,0])
					cube([fanhole,fanhole,15],center=true);
		
}

// Raspberry pi computer
pi_box_size=[61,90,27]; // includes vilros case
pi_corner=[box_left_X+3+wall,box_bottom_Y+6+wall,box_back_Z+wall+3];

module pi_box() {
	translate(pi_corner) color([0,1,0]) cube(pi_box_size);
}

// Put children at each of the pi mounting bolt hole locations
module pi_mounting_bolts() {
	translate(pi_corner) mounting_bolts_4x(49,58,4,4) children();
}



clearance=0.3;

stepper_shaft_dia=5.0;
stepper_mast_dia=9+clearance;
stepper_mast_center=[0,8.0,0];

// Main body
stepper_body_dia=28.1+clearance;
stepper_body_z=19.3+clearance;
stepper_backwire_x=18.1+clearance;
stepper_backwire_y=stepper_body_dia/2+6+clearance; // extra Y for wire protrusion

// Mounting "ears":
stepper_ear_dx=17.5;
stepper_ear_z=1;
stepper_ear_dia=7+clearance;

// Steel mounting rod
mount_x=10.2+clearance;
mount_y=25.4;
mount_z=stepper_body_z; // actual height: 22.2+clearance;
mount_wall=2;



// Tiny geared stepper motor 28BYJ
// Body clearance for mounting.  
//  [0,0,0] is top center of the output shaft mast
module stepper_body(fatten=0,outside=0) {
    z=5+outside;
	
	// Output shaft mast
	translate([0,0,-0.01-outside])
	cylinder(d=stepper_mast_dia+2*fatten,h=1.5+2*outside);

	// Output shaft itself
	if (outside>0)
		cylinder(d=stepper_shaft_dia,h=10);
	
	translate(-1*stepper_mast_center)
	{
    // main body
    translate([0,0,-z])
        cylinder(d=stepper_body_dia+2*fatten,h=z);

    // wiring protrusion in back
    translate([-stepper_backwire_x/2-fatten,-stepper_backwire_y-outside,-z])
            cube([stepper_backwire_x+2*fatten,stepper_body_dia/2+fatten+outside,z]);

    // mounting ears
    translate([0,0,-stepper_ear_z-outside])
        linear_extrude(height=stepper_ear_z+outside)
            hull() 
			for (side=[-1:+1])
				translate([side*stepper_ear_dx,0,0])
                    circle(d=stepper_ear_dia+2*fatten);
	}
}


// Puts the stepper output shaft poking out the bottom of the box
module stepper_orient() {
	translate([0,box_bottom_Y+wall,0]) rotate([90,0,0]) rotate([0,0,180])
		children();
}

stepper_driver_box=[32,35,15];
stepper_driver_origin=[box_right_X-wall-3-stepper_driver_box[0],
	box_bottom_Y+wall+12, box_back_Z+wall+3];
module stepper_driver_bolts() {
	translate(stepper_driver_origin) 
		mounting_bolts_4x(26,30.2,2.5,2.5)
			children();
}

module m3_support() {
	scale([1,1,-1]) cylinder(d2=10,d1=6,h=pi_corner[2]-box_back_Z);
}


module m3_tap_hole() {
	cylinder(d=2.5,h=40,center=true);
}



/* Switch "class":
        [body size XYZ, wiring size XYZ]
*/
switch_micro=[ [6, 13, 8], [3,12,8] ];

function switch_body(switch) = switch[0];
function switch_wiring(switch) = switch[1];

limit_switch=switch_micro;


limit_switch_radius=30;

module limit_switch_centers() {
	box=switch_body(limit_switch);
	translate([0,box_bottom_Y+box[2]/2-0.1,box[1]/2]) 
	for (side=[-1,+1])
	translate([side*limit_switch_radius,0,0]) 
		rotate([90,0,0]) rotate([0,180,0]) children();
}

module fatcube(size,fatten,fattenZ) {
	cube([size[0]+2*fatten,
		size[1]+2*fatten,
		size[2]+2*fattenZ],center=true);
}
module limit_switch_space(fatten=0.0,fattenZ=0.0) {
	limit_switch_centers() {
		box=switch_body(limit_switch);
		fatcube(box,0.4+fatten,fattenZ);
		if (fattenZ)
			translate([0,-box[1]/2,-box[2]/2])
				cylinder(d=9,h=5,center=true);
	}
}


module all_bolts() {
	pi_mounting_bolts() children();
	stepper_driver_bolts() children();
	fan_bolts() children();
}


module realsense_mount_box_complete() {
	difference() {
		union() {
			realsense_plus(wall);
			
			difference() {
				electronics_box(0.0);
				electronics_box(wall);
			}
			stepper_orient() stepper_body(wall);
			limit_switch_space(wall,0.0);
			
			// Supports underneath realsense (avoid spaghetti)
			translate([realsense_reference_point[0],0,0])
			for (slice=[-1:0.5:+1])
				translate([slice*45-wall/2,-10,box_back_Z])
					cube([wall,30,30]);
			
			
			// Reinforcing around top pivot bolt
			translate([0,pivot_top_Y,0]) 
				rotate([90,0,0])
					cylinder(d2=30,d1=5,h=15);
			
			intersection() {
				union() all_bolts() m3_support();
				translate([0,0,1000+box_back_Z])
					cube([2000,2000,2000],center=true);
			}
		}
		
		realsense_minus();
		
		limit_switch_space(0.0,10.0);
		
		stepper_orient() stepper_body(0.0,5.0);
		
		// Estop hole
		estop();
		
		// Top pivot bolt (M3 threads down through bearing)
		rotate([-90,0,0])
			cylinder(d=2.3,h=50);
		
		// USB-C cable re-enters box: not needed, can re-use thru hole
		//translate([box_left_X,-25,box_back_Z+10])
		//	cube(usb_C,center=true);
		
		pi_box();
		all_bolts() m3_tap_hole();
		
		fan_clearance();
		
		// Cutaway
		//cube([100,100,100]);
		
	}
}


module interior_parts() 
{
	estop();

	pi_box();
	
	translate(stepper_driver_origin) cube(stepper_driver_box);
	
	fan_box();
}

module electronics_box_travels() {
	for (angles=[-60,+60])
		rotate([0,angles,0]) {
			electronics_box(0.0);
			estop();
		}
}

// The pivot arm supports the box as it rotates
pivot_height=100;
pivot_width=20;
pivot_thick=10;
module realsense_pivot_complete() {
	intersection() {
		translate([-1000+pivot_width/2,0,0])
			cube([2000,2000,2000],center=true);
		difference() {
			union() {
		
				// Top reinforcing
				translate([0,pivot_top_Y,0])
					rotate([-90,0,0])
						cylinder(d=pivot_width,h=pivot_thick);
				
				// Bottom reinforcing
				translate([0,pivot_bottom_Y-pivot_thick,0])
					rotate([-90,0,0])
					{
						// Extra meat around stepper shaft
						cylinder(d1=pivot_width+20,d2=pivot_width,h=pivot_thick);
						// Anti-tipover plate
						cylinder(d=120,h=3);
					}
				
				// Arms out to pivots
				translate([0,0,-pivot_height])
				{
					translate([-pivot_width/2,pivot_bottom_Y-pivot_thick,0])
						cube([pivot_width,pivot_top_Y-pivot_bottom_Y+2*pivot_thick,pivot_thick]);
					linear_extrude(height=pivot_height) {
						// Bottom upright:
						translate([0,pivot_bottom_Y-pivot_thick/2])
							square([pivot_width,pivot_thick],center=true);
						// Top upright:
						translate([0,pivot_top_Y+pivot_thick/2])
							square([pivot_width,pivot_thick],center=true);
					}
				}
				
				// Diagonal reinforcing
				diag=30;
				translate([pivot_width/2,0,-pivot_height]) {
					rotate([0,-90,0]) linear_extrude(height=3) {
						translate([0,pivot_bottom_Y,0])
							polygon([[0,0],[diag,0],[0,diag]]);
						translate([0,pivot_top_Y,0])
							polygon([[0,0],[diag,0],[0,-diag]]);
					}
				}
				
				// Endstop
				rotate([0,30,0]) scale([1,1,-1])
					linear_extrude(height=limit_switch_radius+10)
					hull() {
						translate([-pivot_thick,pivot_bottom_Y-pivot_thick])
							square([pivot_width,1]);
						translate([0,pivot_bottom_Y-1])
							square([pivot_width-pivot_thick,1]);
					}
			}
			
			// Bottom pivot hole for servo
			translate([0,pivot_bottom_Y-pivot_thick-0.1,0])
			{
				// rotate([-90,0,0]) cylinder(r=limit_switch_radius,h=20,center=true);
				
				rotate([-90,0,0])
					cylinder(d=stepper_shaft_dia+0.2,h=pivot_thick+0.2);
				// M3 bolts to retain flat on stepper
				flats=3;
				M3_wide=2.7;
				M3_length=18;
				for (side=[-1,+1])
					translate([-side*(-flats/2-M3_wide/2),pivot_thick/2,10-M3_length+4*side])
					{
						cylinder(d=2.5,h=M3_length);
						translate([0,0,M3_length-0.1])
							cylinder(d=7,h=M3_length);
					}
			}
			
			// Top hole for M3 bearing
			translate([0,pivot_top_Y-0.1,0])
				rotate([-90,0,0])
				{
					// Bearing itself
					cylinder(d=10,h=4);
					
					// M3 bolt clearance
					cylinder(d=7,h=pivot_thick+2);
				}
			
		}
	}
}

// intersection() { translate([-90,-40,-40]) cube([1000,1000,1000]); // box demo section
realsense_mount_box_complete();


// interior_parts();
//#electronics_box_travels();

//rotate([0,90,0])
//realsense_pivot_complete();



