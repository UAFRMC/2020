
// $fa=5; $fs=0.1; // coarse preview
$fa=5; $fs=0.3; // fine resolution

use <../barbie_gearbox.scad>;

peg_dia=2*14; // diameter of peg hole
pegH=30;

wiggle=0.1;
epsilon=0.1; // avoid roundoff
gearbox_fatten=0.1;
gearbox_walls=2.5;
gearboxDriveH=16;  // height of hole for gearbox drive
gearboxH=18+10; // height of total gearbox drive area
teethH=3*25; // vertical space between teeth
overbite=25; // space above and below teeth

rebar_OD=4.0-0.3; // carbon fiber reinforcing rods

overallH=overbite+teethH+overbite+gearboxH;

teeth=4;
teeth_spacing=50; // along-track spacing between teeth
OD=teeth*teeth_spacing/PI;

center_hole_diam = 3/8*25.4; // 9.525;// 3/8 inch
bearing_outer_diam = 7/8*25.4; // 22.225; // 7/8 inch
bearing_width = 9/32*25.4; // 7.14375; // 9/32 inch
thru_diam=0.8*bearing_outer_diam;


module make_teeth(OD,nteeth) {
	// Rows of teeth holes
	for (toothRow=[0,1])	
		translate([0,0,overbite+toothRow*teethH])
		for (tooth=[0:nteeth-1])
			rotate([0,0,tooth*360/nteeth])
			translate([0,-(OD+epsilon)/2,0])
				children();
}

module make_bearing_holes(topZ) {
	for (bearingZ=[-epsilon,topZ-bearing_width+epsilon])
		translate([0,0,bearingZ])
			cylinder(d=bearing_outer_diam-0.1,h=bearing_width+wiggle);
	
	cylinder(d=thru_diam,h=topZ+epsilon);
}

module make_sprocket(include_gearbox=1.0)
{
	H=overbite+teethH+overbite+include_gearbox*gearboxH;
	difference() {
		union() {
			cylinder(d=OD,h=H);
		}
		
		// Outside of gearbox output shaft
		if (include_gearbox)
		translate([0,0,H+epsilon])
		scale([1,1,-1])
		barbie_gearbox_drive(height=gearboxDriveH,fatten=gearbox_fatten,wiggle=0.0);
		
		// Bearings
		make_bearing_holes(H-include_gearbox*gearboxDriveH);
		
		make_teeth(OD,teeth) rotate([-90,0,0])
			cylinder(d=peg_dia,h=pegH);
		
		// Rebar
		for (rebar=[0:teeth-1])
			rotate([0,0,(rebar+0.5)*360/teeth])
				translate([OD/2-7,0,0])
					cylinder(d=rebar_OD,h=H+epsilon);
		
		// Big plastic-saving bubble in middle
		translate([0,0,overbite+teethH/2])
			scale([1,1,1.7])
				sphere(d=OD-2*12);
		
		// Cutaway cube (for debugging)
		//translate([0,0,-epsilon]) rotate([0,0,45]) cube([1000,1000,1000]);
	}
}


module make_pinholes(OD,nteeth) {
	for (tooth=[0:2:nteeth-1])
		rotate([0,0,(tooth+0.5)*360/nteeth])
			translate([0,-OD/2+rebar_OD/2+2,0])
				children();
}

module make_idler_tooth(pegH,fatten,extralong) {
	D=2*fatten+peg_dia;
	cylinder(d=D,h=pegH+extralong);
	bevel=5+fatten;
	cylinder(d1=D+2*bevel,d2=D,h=bevel);
}

module make_big_idler(nteeth) {
	wall=1.2;
	outerwall=1.8;
	pegH=16;
	rim_inset=pegH*1.8-wall;
	H=overbite+teethH+overbite;
	OD=nteeth*teeth_spacing/PI;
	ODcore=2*wall+bearing_outer_diam;
	difference() {
		union() {
			// central core
			cylinder(d=ODcore,d2=thru_diam+2*wall,h=H/2);
			
			// Outside surface rim:
			difference() {
				cylinder(d=OD,h=H/2,$fa=2);
				translate([0,0,-epsilon])
				cylinder(d=OD-2*outerwall,h=H+2*epsilon);
			}
			
			// Trimmed reinforced working area
			intersection() {
				cylinder(d=OD-0.1,h=H/2);
				union() {
					// rebar around peg holes
					make_teeth(OD,nteeth) rotate([-90,0,0])
						make_idler_tooth(pegH,outerwall,0.0);
					
					// walls
					difference() {
						ribinsideZ=4*bearing_width;
						riboutsideZ=H/2; // overbite+26;
						linear_extrude(height=riboutsideZ,convexity=12) {
							fillet=wall;
							offset(r=-fillet) offset(r=+fillet)
							union() {
								// Outer ring
								difference() { 
									circle(d=OD-0.2); circle(d=OD-2*outerwall+0.1); 
								}
								// Inside ring
								difference() { 
									d=OD-rim_inset;
									circle(d=d); circle(d=d-2*wall); 
								}
								
								// Inside-to-outside support ribs:
								make_teeth(OD,nteeth) 
								for (outside=[-1,0,+1]) // for (inside=[-1,+1])
									hull() { 
										// start on outside wall:
										translate([outside*(2*wall+peg_dia)/2,0,0]) circle(d=wall);
										// move to inside wall
										translate([outside*(20)/2,outside==0?6:OD/2,0]) circle(d=wall);
									}
								
								// Pins hold top and bottom halves together
								make_pinholes(OD,nteeth) circle(d=2*outerwall+rebar_OD);
								
							}
						}
						
						// Bevel the top surface
						translate([0,0,ribinsideZ])
						difference() {
							insidebearing=12; // tapered area
							d_in=thru_diam;
							d_middle=d_in+2*insidebearing;
							
							// Main tapered cylinder
							cylinder(d1=d_middle+2*insidebearing,d2=OD,h=riboutsideZ+epsilon-ribinsideZ);
							
							// More meat around inner bearing
							cylinder(d1=d_middle,d2=d_in,h=insidebearing);
						}
					}
					
				}
			}
			
			// Add rebar around pin holes
			make_pinholes(OD,nteeth) cylinder(d=2*outerwall+rebar_OD,h=H/2);
			
			// Cover bottom with solid plate
			linear_extrude(height=wall, convexity=4) 
				difference() {
					circle(d=OD);
					circle(d=bearing_outer_diam);
					
					for (angle=[0:2*360/nteeth:359])
						rotate([0,0,angle-90])
							translate([(OD-rim_inset-25)/2,0,0])
								circle(d=25);
				}
		}
		
		// Holes for the sprockets:
		make_teeth(OD,nteeth) rotate([-90,0,0])
			make_idler_tooth(pegH,0.0,wall);
		
		// Bearings
		make_bearing_holes(H);

		// Pins hold top and bottom halves together
		translate([0,0,-epsilon])
		make_pinholes(OD,nteeth) cylinder(d=rebar_OD,h=H+epsilon);
		
		// Cross section cube (for inspection only)
		// rotate([0,0,3]) cube([1000,1000,1000]);
	}
	
}

make_big_idler(8); // big idler (8 holes, 5 inch OD version)
//make_big_idler(10); // big idler (10 holes)
// make_sprocket(0.0); // idler
// make_sprocket(1.0); // drive
