/*
 Brackets for holding stuff onto robot tubes.

*/

$fs=0.1;
$fa=5;


inch=25.4;

// Gap between bolt and nut
bolt_gap=0.5*inch;

// Bolt: #10-32, 3/4" long
bolt_OD=5;
bolt_head=12;

// Nut: #10-32 stainless nylock
nut_OD=10.8+0.2; // across the points (not flats), plus a little clearance
nut_Z=7;

// "Strap": geometry radially out from the tube
strap_thick=2.3;

// "Wide": geometry along the tube
strap_wide=14;


// Tube geometry for 1" round tubes
tube_1_d=1.0*inch+0.1;
module tube_1_round() {
	circle(d=tube_1_d);
}

// Tube geometry for 5/8" square tube
tube_58_d=5/8*inch+0.1;
module tube_58_square() {
	rounding=1.5;
	offset(r=+rounding) offset(r=-rounding)
		square([tube_58_d,tube_58_d],center=true);
}


// Super simple leveling bracket: sit between flat space and round tube.
//   Bolt threads into tapped hole in tube.
module simple_flat(d) {
	difference() {
		linear_extrude(height=strap_wide,convexity=4,center=true) {
			strap_round_2D() 
			difference() {
				hull() {
					circle(d=10);
					translate([d/2+strap_thick/2,0])
					square([strap_thick,d+2*strap_thick],center=true);
				}
				children();
			}
		}
			
		rotate([0,90,0]) {
			cylinder(d=bolt_OD,h=100,center=true);
		}
	}
}


// Gap to slide onto tube(s)
slide_on_gap=[100,11];
slide_on_start=[tube_1_d/4,0];
module slide_on_gap_2D() {
	translate(slide_on_start) square(slide_on_gap);
}
module strap_round_2D() {
	rounding=strap_thick/2-0.1;
	offset(r=-rounding) offset(r=+rounding) // round insides
	offset(r=+rounding) offset(r=-rounding) // round outsides
		children();
}


// Strap on an electronics box.  
//  The #10-32 bolt goes through the box's brackets too.
module bracket_electronics(d) {
	bolt_center_x=(d+nut_OD)/2+0.5;
	nut_center_y=-3;
	bracket_xMax=bolt_center_x+nut_OD/2+3;
	bracket_yMax=d/2+strap_thick;
	bracket_yMin=nut_center_y-3;
	difference() {
		linear_extrude(height=strap_wide,convexity=4,center=true) {
			strap_round_2D() 
			difference() {
				union() {
					// Strap
					offset(r=+strap_thick) children();
					// Block to support electronics (& make space for gap)
					translate([0,bracket_yMin,0])
					square([bracket_xMax,bracket_yMax-bracket_yMin]);
				}
				
				// Hole for tube
				children();
				
				// Gap to slide onto tube
				slide_on_gap_2D();
			}
		}
		
		// Space for bolt & nut
		translate([bolt_center_x,nut_center_y]) 
		rotate([90,0,0]) {
			cylinder(d=bolt_OD,h=100,center=true);
			cylinder(d=nut_OD,h=nut_Z,$fn=6);
		}
	}
}

// Strap on an emergency stop
module bracket_estop(d) {
	estop_mount=44/2;  // distance from estop's corner to its mounting bolt
	estop_angle=-20;
	estop_start=[0.45*d,-0.5*d];
	
	bolt_center_x=(d+nut_OD)/2+0.5;
	nut_center_y=-3;
	bracket_xMax=bolt_center_x+nut_OD/2+2;
	bracket_yMax=d/2+strap_thick;
	bracket_yMin=nut_center_y-3;
	difference() {
		linear_extrude(height=strap_wide,convexity=4,center=true) {
			strap_round_2D() 
			difference() {
				hull() {
					// Strap
					offset(r=+strap_thick) children();
					// Block to support electronics (& make space for gap)
					translate([0,bracket_yMin,0])
					square([bracket_xMax,bracket_yMax-bracket_yMin]);
					
					// Support for estop
					translate(estop_start) rotate([0,0,estop_angle])
						square([estop_mount+8,3]);
				}
				
				// Space for actual estop
				#translate(estop_start) rotate([0,0,estop_angle])
					scale([1,-1,1])
						square([1000,1000]);
				
				// Hole for tube
				children();
				
				// Gap to slide onto tube
				slide_on_gap_2D();
			}
		}
		
		// Space for mounting bolt & nut
		translate([bolt_center_x,nut_center_y]) 
		rotate([90,0,0]) {
			cylinder(d=bolt_OD,h=100,center=true);
			cylinder(d=nut_OD,h=nut_Z+100,$fn=6);
		}
		
		// Space for estop fastener
		#translate(estop_start) rotate([0,0,estop_angle])
		translate([estop_mount,4]) rotate([-90,0,0])
		{
			cylinder(d=5,h=100,center=true);
			flare=17;
			cylinder(d1=10, d2=10+3*flare, h=flare);
		}
	}
}


// simple_flat(tube_1_d) tube_1_round();
bracket_electronics(tube_1_d) tube_1_round();
// bracket_estop(tube_1_d) tube_1_round();

