$fs = 0.01;

dxf_name="daughterboard_v0_case_xport.dxf";

// Load a DXF file layer
module dxf(layer_name) {
	s=90/96; // fix cursed Inkscape DPI switch
	scale([s,s,1])
	import(dxf_name,layer=layer_name);
}

floor=0.9;
outerwall=1.5;
outerwall_z=15; // height of holes in frame
outerwall_z_full=floor+30-7; // complete top-bottom cover size

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
	offset(r=-0.9) // <- prepared with fitting over the case in mind
	dxf("outline");
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
	linear_extrude(height=outerwall_z_full,convexity=4) 
	difference() {
		outline_2D();
		offset(r=-outerwall) outline_2D();
	}
}

// Add "ears" for securing wires with velcro zipties
module velcro_2D() {
	top=87; bot=-50;
	left=-6-outerwall; right=+86+outerwall;
	
	for (side=[0,1]) // 0: left side.  1:right side
	translate([side?right:left,0,0]) {
		if (side) translate([0,(bot+top-velcro_sz[1])/2,0]) velcro();
		else {
			translate([0,bot,0]) scale([side?1:-1,1]) velcro();
			translate([0,top,0]) scale([side?1:-1,-1]) velcro();
		}
	}
}

ear_size=4.5;
module whole_frame() {    
	difference() {
		union() {
            frame_3D();
			
			// Add the ear mounts
			linear_extrude(height=circuit_low_z,convexity=8)
			difference() {
				round(4) 
                union() {
					offset(r=+ear_size)
						velcro_2D();
					outline_2D();
				}
				velcro_2D();
				outline_2D(0);
			}
		}
        
		// Openings for electronics
		intersection() {
			translate([-100,-100,0]) cube([200,200,200],center=true);
			translate([-3,0,circuit_low_z-4]) scale([0.9,1,1])
			linear_extrude(height=outerwall_z)
				dxf("insert");
		}
		
        
		// Openings for big wires to enter at base
		holesize=2.25;
        translate([75,-28-8,circuit_low_z-5]) scale([5,holesize,1])
        linear_extrude(height=outerwall_z+1)
        square(velcro_sz);
        
        translate([75,32,circuit_low_z-5]) scale([5,holesize,1])
        linear_extrude(height=outerwall_z+1)
        square(velcro_sz);
		
		
	}
}

// 2D supports for the velcro ears
module supports_2D() {
    for (loc=[0:7:14]) { // 0: left, 1: middle, 2: right
        translate([0,loc,0]) scale([2.75,1,1]) circle(1);
    }
    translate([-1,18,0]) rotate([0,0,45]) scale([2.75,1,1]) circle(1);
    translate([-1,-4,0]) rotate([0,0,-45]) scale([2.75,1,1]) circle(1);
}

// 3Dsupports for the velcro ears
module supports_3D() {
    for (pos=[59.5]) { // 0: left, 1: middle, 2: right
        translate([93.75,pos-48,0]) supports_2D();
    }
    
    difference() {
        for (pos=[0,119]) { // 0: left, 1: right
            mirror([1,0,0]) translate([13.75,pos-48,0]) supports_2D();
        }
        translate([-11.5,-30,0]) rotate([0,0,-45]) scale([3,1.5,1.5]) circle(1);
        translate([-11.5,67,0]) rotate([0,0,45]) scale([3,1.5,1.5]) circle(1);
        translate([-14,71,0]) scale([3,1.5,1.5]) circle(1);
    }
}

module cover() {
	linear_extrude(height=floor) outline_2D();
    
    translate([0,0,outerwall_z_full+1]) mirror([0,0,1]) whole_frame();
    
    linear_extrude(height=outerwall_z_full-2.7)
    supports_3D();
}

// Made to match, based largely on the case
mirror([0,1,0]) cover();
