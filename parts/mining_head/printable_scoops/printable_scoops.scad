/**
3D printable mining head scoops.

*/
// $fs=0.2; $fa=10; // fast preview
$fs=0.1; $fa=3; // smooth final

// Thickness of back and side walls
scoop_wall=2.0;

// Thickness of front wall (dirt contact)
scoop_front=4;

// Thickness of bottom
scoop_floor=1.5;

// Side-to-side size of scoops
scoop_width=150;

// Front-back size of scoops: distance between mining head idler and cutting plane
scoop_stickout=75;

// Top-bottom size of back side of scoops
scoop_deep=45;

// Circumference of mining idler
idler_circumference=50*10;

// Thickness of polypropylene strap
strap_thick=1.3;
strap_width=30;
strap_to_strap=75;

// Thickness of back wall (idler contact)
scoop_back=max(4,scoop_wall+strap_thick);

pi=3.141592;

// Effective diameter of mining idler
idler_dia=idler_circumference/pi+2*strap_thick;
// Effective diameter of outside of cutting plane
cutting_dia=idler_dia+2*scoop_stickout;

// Angle to retreat back from cutting face.
//   This seems key to getting material ejected reliably.
relief_angle=35;

// Angle to slope entrance to cutting face
//   This is to enhance strength by putting everything into tension
rake_angle=-15;

// Angle to slope up floor.  This is mostly to avoid relief overhang
floor_angle=relief_angle*0.2;

// Vertical location of drive peg.
//   This is angled to allow access to the screw head
peg_angle=2;

// In radius-vertical plane:
module scoop_sideview_2D() {
	difference() {
		
		translate([cutting_dia/2,0,0]) rotate([0,0,-relief_angle]) translate([-cutting_dia/2,0,0]) 
		circle(d=cutting_dia);
		
		circle(d=idler_dia);
		
		// Trim off left, top, and bottom surfaces
		translate([cutting_dia/2,0,0]) rotate([0,0,rake_angle])
		{
			translate([0,1000]) square([2000,2000],center=true);
			translate([0,-1000-scoop_deep]) square([2000,2000],center=true);
		}
		translate([-1000,0]) square([2000,2000],center=true);
		
	}
}

module scoop_peg_transform() {
	rotate([0,-peg_angle,0]) 
	translate([idler_dia/2,0,0])
	rotate([0,90,0])
	for (side=[-1,+1]) translate([0,side*strap_to_strap/2,0])
	children();
}

module scoop_planning() {
	scoop_sideview_2D();
	scoop_peg_transform()
		cylinder(d=6,h=20,center=true);
}

scoop_fillet=15;

module scoop_rounding(extra_r=0) {
	$fs=0.1; $fa=8; 
	r=extra_r+scoop_fillet;
	offset(r=+r) offset(r=-r)
	offset(r=-r) offset(r=+r)
		children();
}

scoop_bay_width=scoop_width  - 2*scoop_wall;
scoop_radius=(cutting_dia-idler_dia)/2+scoop_front; // front-back scoop box thickness

// Body of scoop itself, in radius-horizontal plane:
module scoop_body_2D(has_straps=1) {
	// Body of scoop
	scoop_rounding(scoop_wall)
	difference() {
		translate([idler_dia/2,-scoop_width/2])
			square([scoop_radius,scoop_width]);
		
		// Clear spaces for straps
		if (has_straps)
		for (side=[-1,+1])
			translate([idler_dia/2-1,side*strap_to_strap/2-strap_width/2])
				square([1+strap_thick,strap_width]);
				
	}
}

// Back wall, against idler, in radius-horizontal plane:
module scoop_backwall_2D() 
{
	difference() {
		scoop_body_2D();

		// Clear interior bay backsides
		//for (side=[-1,+1])
			scoop_rounding()
				translate([idler_dia/2+scoop_back+1000,0])
					square([2000,scoop_bay_width],center=true);
	}
}

// Front wall, against cutting face, in radius-horizontal plane:
module scoop_frontwall_2D() {
	difference() {
		scoop_body_2D(0);

		// Clear interior bay frontsides
		//for (side=[-1,+1])
			scoop_rounding()
				translate([cutting_dia/2-1000,0])
					square([2000,scoop_bay_width],center=true);
	}
}


// Everything is slanted due to the rake angle, relative to the cutting face
module scoop_rake_rotate() {
	translate([cutting_dia/2,0,0]) rotate([0,-rake_angle,0]) translate([-cutting_dia/2,0,0]) 
		children();
}

// The trimbox is used to paste the front and back together
scoop_switchover_x=idler_dia/2+1.7*scoop_fillet;
module scoop_trimbox_3D(xshift=0) {
	scoop_rake_rotate()
		translate([scoop_switchover_x+xshift,-scoop_width/2-10,-1000])
		cube([100,scoop_width+20,1000]);
}


// Big lathe operations use these parameters
module scoop_lathe() {
	rotate([90,0,0]) rotate_extrude(convexity=4) 
		children(0);
}

// Lathe out the back surface of the scoop, pushing against the idler
module scoop_backlathe() {
	scoop_lathe()
		children();
}

// Lathe out the front surface of the scoop:
//   This is a little weird due to the tilt in relief_angle
module scoop_frontlathe() {
	translate([cutting_dia/2,0,0]) rotate([0,relief_angle,0]) translate([-cutting_dia/2,0,0]) 
		scoop_lathe()
			children();
}

// Tilt to the floor orientation
module scoop_floor_tilt() {
	floor_tilt_point=[idler_dia/2,0,-scoop_deep];
	translate(floor_tilt_point) rotate([0,floor_angle,0]) translate(-floor_tilt_point)
		children();
}


module scoop_full() {
	// Trim everything by rotation surfaces:
	intersection() {
		scoop_backlathe() scoop_body_2D();
		scoop_backlathe() scoop_body_2D();
		scoop_frontlathe() scoop_body_2D(0);
		
		// Trim off front cutting surface
		scoop_lathe() scoop_rounding(scoop_wall) 
			translate([idler_dia/2,-scoop_width/2])
				square([(cutting_dia-idler_dia)/2,scoop_width]);
		
		// Adds:
		union() {
			// Back half:
			intersection() {
				scoop_backlathe() scoop_backwall_2D();
				scoop_trimbox_3D(-100);
			}
			
			// Front half:
			intersection() {
				scoop_frontlathe() scoop_frontwall_2D();
				scoop_trimbox_3D(0);
			}
			
			// Floor:
			scoop_rake_rotate() 
			scoop_floor_tilt() {
				translate([0,-scoop_width/2-10,-scoop_deep])
					cube([1000,scoop_width+20,scoop_floor]);
			}
			
			// Reinforcing around screw holes
			scoop_peg_transform() {
				reinforce=5.5;
				cylinder(d1=32,d2=18,h=reinforce);
				
				// Reinforcing rays around screw
				ray_dia=12;
				rotate([0,6,0]) // thin down bottom ones
				for (angle=[-90:45:+90])
					if (angle!=0)
					rotate([0,1,angle]) // everybody tapers away
						translate([0,0,reinforce-ray_dia/2])
							rotate([0,90,0])
								cylinder(d=ray_dia,h=scoop_width/2);
			}
			
			// Bevels on top of leading edge walls
			scoop_rake_rotate()
			for (side=[-1,0,+1])
				translate([cutting_dia/2,side*(scoop_width/2-scoop_wall/2),0])
					rotate([45,0,0])
						cube([10+cutting_dia-idler_dia,0.7*scoop_wall,0.7*scoop_wall],center=true);
			
			// Central stiffener (supports cutting edge)
			stiff_rounding=10;
			scoop_rake_rotate() 
			difference() {
				scale([1,1,-1]) // extrude downward
					linear_extrude(height=30)
						offset(r=-stiff_rounding) offset(r=+stiff_rounding)
						union() {
							// Stringer
							translate([(idler_dia/2+cutting_dia/2)/2,0]) square([200,scoop_wall],center=true);
							
							// Flat plates at both ends:
							backside_fudgefactor=4.8;
							translate([idler_dia/2+scoop_back-backside_fudgefactor-10,0]) square([20,2000],center=true);
							translate([cutting_dia/2+10,0]) square([20,2000],center=true);
						}
				
				// Trim off bottom flat in printed Z, so it will bridge correctly
				scoop_floor_tilt() {
					translate([0,0,-18-1000]) cube([2000,2000,2000],center=true);
				}
			}
			
		}
		
		// Trims are intersection with a difference operation
		difference() {
			// Big cube provides body and subtracts floor
			scoop_rake_rotate()  
			scoop_floor_tilt() 
				translate([0,0,1000-scoop_deep])
					cube([2000,2000,2000],center=true);
			
			// Thru hole for bolts
			scoop_peg_transform() {
				cylinder(d=6,h=20,center=true);
			}
			
			scoop_rake_rotate() {
			// Serate leading edge with little shapes
			sharpie=12;
			sharpie_range=scoop_width/2-scoop_fillet-sharpie/2;
			for (flip=[-1,+1]) scale([1,flip,1])
			for (sharp=[scoop_wall/2+sharpie/2:sharpie*0.8:scoop_width/2-0.8*scoop_fillet])
				translate([cutting_dia/2,sharp,sharpie*0.5])
					rotate([0,45,0])
					rotate([0,0,45])
					cylinder(d=sharpie,h=70,center=true,$fa=20);
			
			// Sharpen corners
			corner=8;
			for (flip=[-1,+1]) scale([1,flip,-1])
				translate([cutting_dia/2-scoop_fillet,scoop_width/2-scoop_fillet-0.5,-corner-0.1])
				cylinder(r1=scoop_fillet+corner,r2=scoop_fillet-corner,h=2*corner);
			
			// Trim back top corner
				translate([idler_dia/2-5,0,15])
					rotate([0,10,0])
						cube([10,scoop_width,100],center=true);
			
			// Trim back bottom corner
				translate([idler_dia/2+1.5,0,-scoop_deep])
					rotate([0,-20,0])
						cube([10,scoop_width,100],center=true);
			}
		}
	}
}


rotate([0,rake_angle-floor_angle,0]) // put bottom flat on printer bed
scoop_full();



