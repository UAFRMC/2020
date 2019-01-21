/*
 Sheet metal stamping tools.
 The basic tools are imported from Fusion 360 STL files.

 This adds a thick collar around the outside, and some alignment pegs.
*/
module outline(size) {
	difference() {
		offset(r=size) children();
		children();
	}
}

module alignment_pegs() {
	for (side=[+1,-1]) scale([side,1,1])
		translate([150/2,0,0])
			cube([10,10,6]);
}

intersection() {
	union() {
		translate([0,0,1]) import("StampBase.stl",convexity=6);
		
		// Add collar to keep the bottom part from splitting around the edge
		stampSize=[150,45];
		linear_extrude(height=4)
		outline(4)
		offset(r=3)
		translate([0,-stampSize[1]/2])
			square(stampSize,center=true);
		
		// Add pegs to align both the steel and the top stamp
		alignment_pegs();

		
		// This is the top stamp, minus alignment pegs
		difference() {
			import("StampTop.stl",convexity=6);
			translate([0,-114.5,0]) scale([1,-1,1])
				alignment_pegs();
		}
	}
	
	// Half (cross section)
	//translate([1000,0,0]) cube([2000,2000,2000],center=true);
}

