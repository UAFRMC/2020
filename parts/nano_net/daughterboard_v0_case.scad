
dxf_name="daughterboard_v0_case_xport.dxf";

// Load a DXF file layer
module dxf(layer_name) {
	s=90/96; // fix cursed Inkscape DPI switch
	scale([s,s,1])
	import(dxf_name,layer=layer_name);
}

floor=1.3;
outerwall=1.5;
outerwall_z=floor+25.5; // FIXME

innerwalls=1.5;

circuit_low_z=floor+2.2; // base of UN178 board
circuit_high_z=circuit_low_z+13.3; // base of driver board



// Round off an outline
module round(fillet) {
	offset(r=-fillet) offset(r=+fillet)
	offset(r=+fillet) offset(r=-fillet)
		children();
}

// Outer walls
module outline_2D(thickness=outerwall) {
	offset(r=+thickness)
	offset(r=-3) // <- prepared with 3mm wall in mind 
	dxf("outline");
}

// Sensor plug insertion area
module sensors_2D() {
	union() {
		for (shift=[+3,-3])
			translate([0,0.5*shift])
				dxf("sensors");
	}
}

// Inner electronics bays
module bays_2D(extra=0) {
	dxf("circuit");
	sensors_2D();
	
	// Stiffening rib for electronics
	if (extra)
	translate([-14.0,-20]) square([2,23]);
}


module dice() {
	difference() {
		children();
		
		union() {
		translate([50,0,0])
		for (dx=[-100:5:+100]) 
		for (plusminus=[+1,-1])
			rotate([0,0,plusminus*45]) translate([dx,0,0]) square([1.0,200],center=true);
		}
	}
}

// Add mounting points for M3 hold-down screws
module screw_mounts(height,r) {
	difference() {
		linear_extrude(height=height)
			offset(r=r)
				children();
	}
}

// Add "ears" for securing wires with zipties
ear_size=4.5;
module ear_mounts() {
	perimeter=ear_size;
	linear_extrude(height=circuit_low_z,convexity=8)
	difference() {
		offset(r=+perimeter)
			children();
		children();
	}
}

// 2D profile of velcro mounting slot, facing in +Y
velcro_sz=[4,18];
module velcro() {
	round=1.2;
	offset(r=+round) offset(r=-round)
		square(velcro_sz);
	
}


// Main frame, with no add-ons or holes
module frame_3D() {
	// Outer walls
	linear_extrude(height=outerwall_z,convexity=4) 
	difference() {
		outline_2D();
		offset(r=-outerwall) outline_2D();
	}
	
	// Floor
	linear_extrude(height=floor)
	difference() {
		outline_2D();
		dice() 
			dxf("vent");
	}
	
	// Mini walls around electronics bays
	linear_extrude(height=circuit_high_z-3,convexity=6)
	difference() {
		offset(r=+innerwalls) bays_2D(1);
		bays_2D(1);
	}
}

// Add "ears" for securing wires with velcro zipties
module velcro_and_bolts_2D() {
	top=87; bot=-50;
	left=-6-outerwall; right=+86+outerwall;
	
	for (side=[0,1]) // 0: left side.  1:right side
	translate([side?right:left,0,0]) {
		translate([0,bot,0]) scale([side?1:-1,1]) velcro();
		translate([0,top,0]) scale([side?1:-1,-1]) velcro();
		if (side) translate([0,(bot+top-velcro_sz[1])/2,0]) velcro();
	}
	
	// Bolt holes, with legs
	boltx=70;
	translate([boltx,0,0])
	for (updown=[0,1]) {
		translate([0,updown?top:bot,0]) scale([1,updown?+1:-1]) 
			translate([0,12,0]) {
				bolt=5.2;
				circle(d=bolt); // #10 mounting bolt (like tracks)
				
				// Legs
				for (del=[-bolt/2,+bolt/2])
				for (angle=[-45:45:+45])
					rotate([0,0,180+angle])
						translate([del,0,0])
						square([0.01,15]);
			}
		
	}
}

module whole_frame() {	
	difference() {
		union() {
			difference() {
				frame_3D();
				
			}
			
			// Tapered wall intersection
			intersection() {
				for (taper=[0:0.5:2])
				linear_extrude(height=circuit_low_z-taper)
				round(4)
				{
					offset(5.0+taper)
						dxf("holes");
					offset(ear_size+taper)
						velcro_and_bolts_2D();
					difference() {
						square([1000,1000],center=true);
						outline_2D(-taper);
					}
				}
				linear_extrude(height=outerwall_z)
					outline_2D();
			}
			
			// Back side walls
			difference() {
				union() {
					mount=4.0; // wall around circuit board screw mounts
					screw_mounts(circuit_high_z,mount)
						dxf("circuitholes");
					for (taper=[0:0.5:2])
					screw_mounts(circuit_low_z-taper,mount+taper)
						dxf("circuitholes");
				}
				linear_extrude(height=outerwall_z)
				{
					offset(r=0.5) dxf("un178");
					bays_2D();
				}
			}
			
			// Add the ear mounts
			linear_extrude(height=circuit_low_z,convexity=8)
			difference() {
				round(4) 
				union() {
					offset(r=+ear_size)
						velcro_and_bolts_2D();
					outline_2D();
				}
				velcro_and_bolts_2D();
				outline_2D(0);
			}
			
				
		}
		
		// Openings for electronics
		translate([0,0,circuit_low_z])
		linear_extrude(height=outerwall_z)
			dxf("insert");
		
		// Punch holes for mounting screws all the way through everything
		translate([0,0,-0.1])
			linear_extrude(height=2*circuit_high_z)
			union() {
				dxf("holes");
				dxf("circuitholes");
			}
		
				
		// Thru holes for bottom-insert stuff
		translate([0,0,-0.1])
		linear_extrude(height=outerwall_z)
		union() {
			sensors_2D();
			dxf("rj45");
		}
	}
}

whole_frame();



// Drop raw DXF on top of everything
// color([1,0,0]) translate([0,0,3]) import(dxf_name);

