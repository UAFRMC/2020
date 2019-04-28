
$fs=0.1;
$fa=5;

height=10;

nema_plate=3;

module NEMA17_bolts() {
	d=31.0/2;
	for (dx=[-d,+d]) for (dy=[-d,+d])
		translate([dx,dy])
			children();
}
module NEMA17_2D() {
	difference() {
		// Outside body
		intersection() {
			circle(d=54);
			square([42,42],center=true);
		}
		
		// Center circle boss
		circle(d=22.2);
		
		// M3 bolts
		NEMA17_bolts()
				circle(d=3.2);
	}
}


clearance=0.0;

stepper_shaft_dia=5.0;
stepper_mast_dia=9+clearance;
stepper_mast_center=[0,8.0,0];

// Main body
stepper_body_dia=28.1+clearance;
stepper_body_z=19.3+clearance;
stepper_backwire_x=18.1+clearance;
stepper_backwire_y=stepper_body_dia/2+3; // extra Y for wire protrusion

// Mounting "ears":
stepper_ear_dx=17.5;
stepper_ear_z=1;
stepper_ear_dia=7+clearance;


stepper_z=8;

module stepper_body() {
	// main body
	fatten=0.0;
	outside=0.0;
	z=0.0;
	difference() {
		translate(-stepper_mast_center)
		union() {
			translate([0,0,0])
				cylinder(d=stepper_body_dia,h=stepper_z);

			// wiring protrusion in back
			translate([-stepper_backwire_x/2,-stepper_backwire_y,0])
					cube([stepper_backwire_x,stepper_body_dia/2,stepper_z]);
			
			// mounting ears
			translate([0,0,0])
				linear_extrude(height=stepper_z)
					hull() 
					for (side=[-1:+1])
						translate([side*stepper_ear_dx,0,0])
							circle(d=stepper_ear_dia+2*fatten);
		}
	}
}



difference() {
	union() {
		linear_extrude(height=nema_plate,convexity=6)
			NEMA17_2D();
		
		translate([0,0,height-stepper_z]) stepper_body();
		translate([0,0,nema_plate-1]) {
			cylinder(d1=32,d2=12,h=3);
			// Bevel around stepper mast circle
			translate(-stepper_mast_center)
				cylinder(d1=stepper_body_dia+5,d2=stepper_body_dia,h=2.5);
		}
	}
	
	NEMA17_bolts() translate([0,0,height-8]) cylinder(d=8,h=100);
	
	// Thru hole
	cylinder(d=7,h=100,center=true);
}

