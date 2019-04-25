/*
	Guard for encoder board

*/
$fs=0.1;
$fa=5;

inch=25.4;

wall=2;

clearance_lo=0.2; // around clean precise things
clearance_hi=0.4; // around lumpy things like solder or board edges

// Circuit board stub
circuit_thick=1.6+clearance_lo;
circuit_wide=8.2+clearance_hi;
circuit_deep=6.8+clearance_hi;

// Actual 3-pin plug
wire_wide=8+clearance_hi;
wire_thick=2.8+clearance_hi;
wire_ystart=3.0;
wire_deep=9;

// 3-pin male header soldered to circuit board
soldertop_thick=4.6+clearance_lo;
soldertop_deep=5;
solderbot_thick=1.5;
solderbot_deep=circuit_deep-0.4;

// Mounted into a 3/8" hole in the motor housing
thruhole_dia=0.375*inch-clearance_lo;
thruhole_tilt=0;
thruhole_insert=3;
thruhole_deep=6;  // start point of thruhole, relative to end of wires

motormount_dia=45; // motor mount on left side

epsilon=0.01;


module thruhole_orient() {
	translate([0,-circuit_thick,thruhole_deep])
	rotate([0,thruhole_tilt,0])
		children();
}

module thruhole_bevelarea() {
	/* // Round version: 
	f=2;
	scale([1,1,-1])
		cylinder(d1=thruhole_dia+2*f,d2=thruhole_dia,h=f);
	*/
	
	// Squared off version
	translate([0,2,0])
	scale([1,1,0.5])
		rotate([0,45,0])
			cube([8,15,8],center=true);
	
}

module guard_bottom_2D() {
	offset(r=wall)
	union() {
		//scale([0.7,1.0])
		//	circle(d=thruhole_dia);
		
		translate([-circuit_wide/2,-circuit_thick])
			square([circuit_wide,circuit_thick+wire_ystart+wire_thick]);
	}
	
}

module circuitboard() {
	color([1,1,0.2])
	translate([-circuit_wide/2,-circuit_thick,-circuit_deep])
		cube([circuit_wide,circuit_thick,1.5*inch]);
}

module guard_holes() {
	// Actual PCB
	circuitboard();
	
	// Space for hot glue and 3-pin header topside pins
	color([0.1,0.1,0.1])
	translate([-wire_wide/2-epsilon,-epsilon,-epsilon])
		cube([wire_wide+2*epsilon,soldertop_thick+2*epsilon,soldertop_deep+10]);
	
	// Space for solder bumps underneath
	color([0.5,0.5,0.5])
	translate([-wire_wide/2,-circuit_thick-solderbot_thick+epsilon,0])
		cube([wire_wide,solderbot_thick,solderbot_deep]);
	
	// Space for 3-pin wire end to plug in
	color([1.0,0.1,0.1])
	translate([-wire_wide/2,wire_ystart,-wire_deep-epsilon])
		cube([wire_wide,wire_thick,wire_deep+2*epsilon]);
}

module guard_plus() {
	// Bottom body of guard
	translate([0,0,-wire_deep+0.01])
		linear_extrude(height=wire_deep)
			guard_bottom_2D();
	
	// Guard to thruhole transition
	hull() {
		for (z=[0,-wire_deep])
			translate([0,0,z])
				linear_extrude(height=1)
					guard_bottom_2D();
		thruhole_orient() thruhole_bevelarea();
	}	
	
	
	// Sloped plug for thruhole
	thruhole_orient()
	union() {
		cylinder(d=thruhole_dia,h=thruhole_insert);
		thruhole_bevelarea();
	}
	
}

difference() {
	guard_plus();
	guard_holes();
	
	translate([motormount_dia/2+circuit_wide/2+0.1,0,0])
		rotate([90,0,0])
			cylinder(d=motormount_dia,h=100,center=true,$fs=0.1,$fa=5);
	
	// Cutaway:
	// translate([1000,0,0]) cube([2000,2000,2000],center=true);
}
