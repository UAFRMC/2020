/**
3D printable mining head scoops:
  New 2018-11 flatter face version.

*/
$fs=0.2; $fa=10; // fast preview
// $fs=0.1; $fa=3; // smooth final

// Thickness of back and side walls
scoop_wall=1.5;

// Thickness of front wall (dirt contact)
scoop_front=2.0;

// Thickness of bottom
scoop_floor=1.3;

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
relief_angle=60;

// Angle to slope entrance to cutting face
//   This is to enhance strength by putting everything into tension
rake_angle=-15;

// Angle to slope up rounded floor.  This is mostly to avoid relief overhang
floor_angle=11.3;
// Angle to trim off to floor
floor_trim_angle=25;
floor_trim_Z=-6;

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

scoop_fillet=16;

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

// Cut middle walls through scoop bays
module scoop_middlewalls_2D() {
	round_midwall=6;
	
	offset(r=+round_midwall) offset(r=-round_midwall) 
	difference() {
		children(); // bay
		
		// Middle wall
		square([2000,scoop_wall],center=true);
	}	
}

// Back wall, against idler, in radius-horizontal plane:
module scoop_backwall_2D() 
{
	difference() {
		scoop_body_2D();

		// Clear interior bay backsides
		scoop_middlewalls_2D() 
			scoop_rounding()
				translate([idler_dia/2+scoop_back+1000,0])
					square([2000,scoop_bay_width],center=true);
	}
}

// Front wall, against cutting face, in radius-horizontal plane:
module scoop_frontwall_2D(extra_front=0) {
	difference() {
			scoop_body_2D(0);

		// Clear interior bay frontsides
		translate([-extra_front,0,0])
		scoop_middlewalls_2D() 
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

// Trim off stuff to above rounded bottom of floor
bottomR=16;
module scoop_floor_trimbox() {
	rotate([0,floor_angle,0]) 
	translate([idler_dia/2+bottomR,0,floor_trim_Z])
	rotate([0,floor_trim_angle,0])
		translate([0,0,1000])
		cube([2000,2000,2000],center=true);
}	

module scoop_roundbottom(fatten=0.0) {
	hull() rotate([0,floor_angle,0]) {
		for (side=[-1,+1]) translate([idler_dia/2+bottomR,side*(scoop_width/2-bottomR),0])
			sphere(r=bottomR+fatten,$fa=20);
	}
}

// The trimbox is used to paste the front and back together
scoop_switchover_x=idler_dia/2+1.35*scoop_fillet;
module scoop_trimbox_3D(xshift=0,withfloor=1) {
	intersection() {
		scoop_rake_rotate()
			translate([scoop_switchover_x+xshift,-scoop_width/2-10,-1000])
			cube([100,scoop_width+20,1000]);
		
		if (withfloor) scoop_floor_trimbox();
	}
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
		linear_extrude(height=200,convexity=4,center=true)
			children();
}

module scoop_full() {
	extrameat=2.5; // total width of front cutting face
	
	// Trim everything by rotation surfaces:
	intersection() {
		union() {
			intersection() {
				scoop_backlathe() scoop_body_2D();
				scoop_frontlathe() scoop_body_2D(0);
				scoop_trimbox_3D(-100);
			}
			intersection() {
				scoop_frontlathe() scoop_body_2D(0);
				scoop_trimbox_3D(0);
			}
			intersection() {
				scoop_backlathe() scoop_body_2D();
				scoop_frontlathe() scoop_body_2D(0);
				scoop_roundbottom(0.0);
			}
		}
		
		// Trim off front cutting surface
		scoop_lathe() scoop_rounding(scoop_wall) 
			translate([idler_dia/2,-scoop_width/2])
				square([(cutting_dia+1-idler_dia)/2,scoop_width]);
		
		
		// Adds:
		union() {
			
			// Back half:
			intersection() {
				scoop_backlathe() scoop_backwall_2D();
				scoop_trimbox_3D(-100,0);
			}
			
			// Front half:
			intersection() {
				union() {
					scoop_frontlathe() scoop_frontwall_2D();
					
					// Extra meat right by cutting face
					if (0) translate([cutting_dia/2,0,0]) rotate([0,relief_angle*0.7,0]) translate([-cutting_dia/2,0,0]) 
						scoop_lathe() scoop_frontwall_2D(extrameat);
				}
				scoop_trimbox_3D(0,0);
			}
			
			// Rounded bottom
			difference() {
				difference() {
					scoop_roundbottom(0.0);
					scoop_floor_trimbox();
				}
				scoop_roundbottom(-scoop_wall);
			}
			
			// Reinforcing around screw holes
			scoop_peg_transform() {
				reinforce=5.5;
				cylinder(d1=64,d2=25,h=reinforce);
				
			}
			
			nailspots() {
				
			}
			
		}
		
		// Trims are intersection with a difference operation
		difference() {
			// Big cube provides body and subtracts floor
			scoop_rake_rotate()  
				translate([0,0,1000-2*scoop_deep])
					cube([2000,2000,2000],center=true);
			
			
			// Thru hole for bolts
			scoop_peg_transform() {
				cylinder(d=6,h=20,center=true);
			}
			
			nailspots() {
				
			}
			
		}
	}
}


rotate([0,180+rake_angle,0]) // put top surface flat on printer bed
	scoop_full();



