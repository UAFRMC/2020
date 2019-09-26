/**
  Large direct-drive wheels for NASA Robotic Mining Competition

Constraints: minimum mass (points, but also print time)
maximum strength (both static loading and impact resistance, rocks hit at any angle)

This version is covered in lumps, like the PowerWheels wheels, since those compound
curves are self-reinforcing.  
*/
include <drive_param.scad>
coarse=0; $fs=0.2; $fa=10; // fine version (smoother)
//coarse=1; $fs=0.5; $fa=15; // coarse version (faster)

inch=25.4; // units here are mm

sector=0; // 1: draw quarter circle only.  0: full circular wheel

wheel_overall_OD=280; // tip-to-tip size

grouser_count=12; // number of grousers around wheel perimeter
grouser_height=17; // grousers stick this high above smooth surface
grouser_fillet=12; // grousers are rounded into smooth surface
grouser_thick=2.5; // thickness of each grouser

wheel_OD=wheel_overall_OD-2*grouser_height; // smooth inside surface (top of grousers is higher)
wheel_Z=80; // full end-to-end width of wheel
motor_Z=60; // Z height to motor drive area

wheel_taper_Z=15; // height of Z steps during tapering
wheel_taper_R=16; // total radius loss due to tapering
wheel_ID=wheel_OD-2*wheel_taper_R;
echo("Wheel ID = ",wheel_ID);

wheel_OD_wall=2.0; // Thickness of smooth inside wall of wheel
wheel_rib_thick=2.0;
wheel_floor=1.3;

// Degrees of rotation per mm of Z height
helix=360/1000;


wiggle=0.2;
barbie_gearbox_height=16.5; // Z size of motor drive cross
spoke_OD=3.8; // diameter of holes for basalt fiber 'spokes'
spoke_Z=motor_Z-barbie_gearbox_height-spoke_OD;

// Bearing that fits on axle
module axle_bearing(fatten=0.0) {
        cylinder(d=bearing_outer_diam+wiggle+2*fatten, h=bearing_width+fatten+wiggle+1);
}

// Holes to fit all the projections associated with a Barbie jeep gearbox:
module barbie_gearbox(fatten=0.0)
{
    radius=16;
    cube_width=13;
    cube_diameter=50;
	round=1.5;
    // Main body
        translate([0,0,motor_Z-barbie_gearbox_height-fatten])
                linear_extrude(height=barbie_gearbox_height+fatten,convexity=4)
                        offset(r=+round)  offset(r=-round)
                        offset(r=-round)  offset(r=+round)
                        offset(r=2*wiggle+fatten)
                        {
                                circle(r=radius,center=true);
                                square([cube_width,cube_diameter],center=true);
                                square([cube_diameter,cube_width],center=true);
                        }

        // Inside bearing hole
        translate([0, 0, motor_Z-barbie_gearbox_height-bearing_width-fatten]){
                axle_bearing(fatten);
        }
		axle_bearing(fatten);

        // Thru axle hole
        translate([0, 0, 0]){
                cylinder(d=12+2*(wiggle+fatten), h=motor_Z);
        }
}


// Wheel "lumps" on driving surface add compound curves, for strength
sz=25;// size of wheel lumps
tread=2.8; // height of wheel lumps above smooth surface
module wheel_lump(inset=0.0) {
	// cube([sz-2*inset,sz-2*inset,sz-2*inset],center=true); // <- faster

	fa=45/3;
	scale([1,1.3,1.5]) // stretched spheres
	rotate([0,0,fa/2]) 
		sphere(d=sz-2*inset,$fa=coarse?fa*2:fa); // <- stronger
}

// Circular row of wheel lumps
module wheel_lump_row(inset,z=0.0,ashift=0.0,tilt=0.0,inshift=0.0) {
	end_angle=sector?100:359;
	delta_angle=30/3;
	translate([0,0,z*sz*1.3])
	for (angle=[delta_angle/2:delta_angle:end_angle]) rotate([0,0,angle+delta_angle*ashift])
	translate([wheel_OD/2-sz/2+tread-inshift,0,0])
	rotate([0,-tilt,0])
	{
		wheel_lump(inset);
	}
}

module wheel_lumps(inset) {
	translate([0,0,wheel_Z/2]) {
		wheel_lump_row(inset,0,0,0,-0.5);
		for (flip=[+1,-1]) scale([1,1,flip]) {
			//wheel_lump_row(inset,0.5,0.5);
			//wheel_lump_row(inset,1.0);
			//wheel_lump_row(inset,0.5,0.5,15,1);
			wheel_lump_row(inset,0.67,0.5,30,5);
		}
	}
}
//wheel_lumps(0.0);


// Main smooth wheel profile with grousers
module wheel_2D(inset,with_grousers,insides) {
	offset(r=-inset)
	offset(r=-grouser_fillet) offset(r=+grouser_fillet) // round corners
	union() {
		difference() {
			circle(d=wheel_OD);
			//circle(d=wheel_OD-2*wheel_OD_wall);
		}
		if (with_grousers)
		for (grouser=[0:grouser_count-1])
			rotate([0,0,360/grouser_count*grouser])
				translate([wheel_OD/2,-grouser_thick/2,0])
					square([grouser_height,grouser_thick]);
	}
}

module wheel_2D_lighter(inset,with_grousers, insides) {
	if (insides) { // hollow tubes behind each grouser
		smooth=3;
		offset(r=+smooth) offset(r=-smooth)
		//offset(r=-0.9*grouser_thick)
		difference() {
			wheel_2D(inset,with_grousers,insides);
			// circle(d=wheel_OD-2*wheel_OD_wall); // grouser_thick);
		}
	}
	else { // normal wheel
		wheel_2D(inset,with_grousers,insides);
	}
}

// Extrude 2D section up to 3D, starting at this height, and extending this far
module wheel_extrude(start_Z,len_Z,scale=1.0,flip=1) {
	translate([0,0,start_Z])
		rotate([0,0,-helix*start_Z*flip])
			linear_extrude(height=len_Z,twist=helix*len_Z*flip,scale=scale,
				convexity=6,slices=1)
				children();
}

module wheel_3D(inset, with_grousers, insides, with_lumps=1) {
	maxstep=2;
	maxshrink=(2*wheel_taper_R)/wheel_OD; // shrink factor
	scales=[1.0-maxshrink,1.0-0.25*maxshrink,1.0];
	
	// Unscaled middle
	wheel_extrude(maxstep*wheel_taper_Z, wheel_Z-2*maxstep*wheel_taper_Z,1.0,1.0)
		wheel_2D_lighter(inset,with_grousers, insides);
	
	// Scaled/shifted steps
	for (bottom=[0,1]) 
	for (step=[1:maxstep])
		scale([scales[step],scales[step],(bottom?-1:+1)])
		wheel_extrude(
			bottom?-step*wheel_taper_Z:
			wheel_Z-step*wheel_taper_Z,
			wheel_taper_Z,
			scales[step-1]/scales[step],
			bottom?-1:+1)
			wheel_2D_lighter(inset,with_grousers, insides);
	
	// Lumps increase impact strength and stiffness
	if (with_lumps) 
	union() {
		wheel_lumps(inset);
	}
}


// Helical reinforcing ribs
module wheel_ribs(step=1,thickscale=1) {
	helix_Z=spoke_Z;
	for (angle=[19:60/3*step:359]) rotate([0,0,angle])
		// Angle to make cool helix shape
		translate([0,0,helix_Z]) rotate([0,35,0]) translate([0,0,-helix_Z]) 
			cube([wheel_rib_thick*thickscale,1.2*wheel_OD,2.5*wheel_Z],center=true);
}

// M3 screws on drive mount points
drive_mount_bolt_R=40;
drive_mount_bolt_OD=2.6; // tap for M3
module drive_mount_bolts() {
	for (angle=[0:120:360]) rotate([0,0,90+angle])
		translate([drive_mount_bolt_R,0,motor_Z])
			scale([1,1,-1])
				children();
}

// Outer rim of wheel: lumps and grousers on outside; lumps on inside 
module wheel_rim() {
	difference() {
		wheel_3D(0.0,1,0); // outside of full wheel
		wheel_3D(wheel_OD_wall,1,1); // interior of grousers
	}
}


module wheel_spoke_centerlines() {
	for (angle=[30:120:360-1]) rotate([0,0,angle])
		translate([14,0,0])
			children();
}
module wheel_spokes(fatten=0.0) {
	wheel_spoke_centerlines() 
		translate([0,0,spoke_Z])
			rotate([90,0,0])
				rotate([5,0,0]) // slightly tilt spokes, so they miss each other in 3D
				cylinder(d=spoke_OD+2*fatten,h=wheel_overall_OD,center=true);
}

module wheel_interior_parts(fatten=0.0) {
	barbie_gearbox(fatten); // surrounds motor drive
	drive_mount_bolts() cylinder(d=drive_mount_bolt_OD+2*fatten,h=24);
	wheel_spokes(fatten);
}

wheel_extra=5;
module wheel_floor_2D() {
	round=20;
	offset(r=-round) offset(r=+round)
	offset(r=+wheel_extra)
	union() {
		drive_mount_bolts() rotate([0,0,60])
			translate([-wheel_rib_thick/2,0,0])
				square([wheel_rib_thick,drive_mount_bolt_R*2]);
		wheel_spoke_centerlines() square([wheel_rib_thick,1000],center=true);
		difference() {
			circle(d=1000);
			circle(d=wheel_ID);
		}
	}
}

module simpler_wheel() {
	difference() {
		union() {
			wheel_rim(); // <- comment this for speedup
			intersection() {
				difference() {
					union() {
						wheel_interior_parts(3.4);
						
						// Support the basalt with walls
						wheel_spoke_centerlines() 
							rotate([5,0,0])
							cube([wheel_rib_thick,1000,2*spoke_Z],center=true);
						
						//cylinder(d=wheel_OD,h=wheel_floor); // floorplate
						
						// Support behind each mount bolt
						drive_mount_bolts() 
						for (sweep=[60:30:120]) rotate([0,0,sweep])
						//for (angle=[0:120:360-1]) rotate([0,0,angle])
							translate([-wheel_rib_thick/2,0,0])
							cube([wheel_rib_thick,drive_mount_bolt_R,motor_Z]);
						
						linear_extrude(height=wheel_floor,convexity=4) 
							wheel_floor_2D();
						
						
						
						difference() {
							union() {
								wheel_ribs(1,1);
								// Top rim
								translate([0,0,wheel_Z-wheel_floor])
									cylinder(d=wheel_OD,h=wheel_floor);
							}
							// Remove ribs in center
							cylinder(d=wheel_ID-2*wheel_extra,h=wheel_Z+1);
						}
					}
					wheel_interior_parts(0.0);
				}
				wheel_3D(0.0,1,0,coarse?0:1); // trim to outside of full wheel
			}
		}
		
		// Allow spokes to actually get inserted
		wheel_spokes(0.0);
		
		// Cutaway
		// cube([1000,1000,1000]);
	}
}
simpler_wheel();

