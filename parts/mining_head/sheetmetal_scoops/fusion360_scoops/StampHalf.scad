
intersection() {
	union() {
		translate([0,0,1]) import("StampBase.stl",convexity=6);
		import("StampTop.stl",convexity=6);
	}
	
	translate([1000,0,0]) cube([2000,2000,2000],center=true);
}

