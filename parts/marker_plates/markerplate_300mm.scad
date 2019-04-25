/* 

Computer vision marker plate:

*/
$fs=0.1;
dimensions=[300,300];

wall=0.8;
floor=0.8;

perimeter=15;
interior=10;
sub=5;

module marker_outside_2D() {
	fillet=10;
	offset(r=-fillet) offset(r=+fillet)
	square(dimensions);
}

module marker_walls_2D() {
	difference() {
		marker_outside_2D();
		offset(r=-wall) marker_outside_2D();
	}
}

module isogrid_2D(spacing=25,rounding=1,thickness=wall,phase=0) {
	intersection() {
		marker_outside_2D();
		difference() {
			// Crossing lines
			offset(r=-rounding) offset(r=+rounding)
			union() {
				for (angle=[0,+60,+120])
				{
					translate([0,0])
					rotate([0,0,angle])
					for (slice=[phase*spacing:spacing:2*dimensions[1]])
					{
						translate([0,slice])
							square([4*dimensions[0],thickness],center=true);
						translate([0,-slice])
							square([4*dimensions[0],thickness],center=true);
					}
				}
			}
			
			// Holes at each intersection
			dx=spacing*2/sqrt(3);
			diameter=sqrt(3)*rounding+thickness;
			if (rounding>0) 
			union()
			for (rows=[0:2:dimensions[1]/spacing])
			for (evenodd=[0,1])
			for (x=[dx*(1-evenodd):dx:dimensions[0]])
				translate([x+evenodd*dx/2,(rows+evenodd)*spacing])
					circle(d=diameter);
		}
	}
}

module isogrid(height=15,spacing=25,rounding=1,thickness=wall,phase=0) {
	linear_extrude(height=height,convexity=4)
	offset(r=-rounding) offset(r=+rounding) // blend wall joints
	union() {
			marker_walls_2D();
			isogrid_2D(spacing,rounding,thickness,phase);
	
	}
}


module marker_3D() {
	union() {
		// floor
		linear_extrude(height=floor) marker_outside_2D();
		
		// walls
		linear_extrude(height=perimeter,convexity=4) 
			marker_walls_2D();
		
		// Reinforcing ribs:
		sz=dimensions[0]/4;
		isogrid(interior,sz,2,wall);
		
		linear_extrude(height=sub,convexity=4)
			isogrid_2D(sz,0,0.5*wall,0.5);
		
		if (0)
		linear_extrude(height=sub/2,convexity=4) 
		union() {
			isogrid_2D(sz,0,0.5*wall,0.75);
			isogrid_2D(sz,0,0.5*wall,0.25);
		}
	}
}

marker_3D();
