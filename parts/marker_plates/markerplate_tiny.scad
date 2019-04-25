/* 

Tiny computer vision marker plate, to stick on an arrow and hang off the
front of the Realsense, so the realsense knows its exact angle relative to the field.

Small, to block less of the field.

*/
$fs=0.1;
dimensions=[40,40];

wall=1.0;
floor=1.2;

arrow=7.5; // mounting hole diameter
arrowcenter=[dimensions[0]/2,-arrow/2];

perimeter=15;
interior=10;
sub=5;

module marker_outside_2D() {
	fillet=10;
	difference() {
		offset(r=-fillet) offset(r=+fillet)
		union() {
			square(dimensions);
			translate(arrowcenter)
				circle(d=arrow+3*wall);
		}
		translate(arrowcenter)
			circle(d=arrow);
	}
}

module marker_walls_2D() {
	round=3;
	difference() {
		marker_outside_2D();
		offset(r=-wall) 
		offset(r=+round) offset(r=-round) marker_outside_2D();
	}
}

module isogrid_2D(spacing=25,rounding=1,thickness=wall,phase=0) {
	intersection() {
		marker_outside_2D();
		translate([-3,0])
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

module isogrid(height=15,spacing=25,rounding=0,thickness=wall,phase=0) {
	linear_extrude(height=height,convexity=4)
	//offset(r=-rounding) offset(r=+rounding) // blend wall joints
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
		sz=dimensions[0]/2;
		isogrid(interior,sz,2,wall);
		
		linear_extrude(height=sub,convexity=4)
			isogrid_2D(sz,0,0.5*wall,0.5);
		
		linear_extrude(height=sub/2,convexity=4) 
		union() {
			isogrid_2D(sz,0,0.5*wall,0.75);
			isogrid_2D(sz,0,0.5*wall,0.25);
		}
	}
}

marker_3D();
