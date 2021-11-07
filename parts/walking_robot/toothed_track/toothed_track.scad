/*
Drive a pin along a track using a geared tooth profile along the track.

Model "Tongan" RS550 gearbox to drive along the track.
*/
inch=25.4; // file units are mm

include <../../gear_library.scad>;
$fs=0.1;
$fa=5;
wall=3.6;
clearance=0.2;

drive_OD=61; // output gear outside diameter
drive_OD_clear=66; // total space around drive gear

bearing_OD=7/8*inch+clearance; // 608 style bearing
bearing_Z=7+2*clearance;
bearing_wallZ=1.5; // material retaining bearing
bearing_wallR=2; // material retaining bearing
bearing_slideOD=bearing_OD+0.2; // clearance for bearing to slide around.

motorout_ht=12; // height of output gear (plus a little wiggle room)
gearout_ht=8; // height of drive gear
axle_OD=3/8*inch+2*clearance;
axlecap_OD=9/16*inch/cos(30)+2*clearance; // hex head on axle
axlecap_Z=7.5-bearing_wallZ; // clearance for head travel


// Gear profile for drive and rack
gear_pitch = 10; // mm per gear tooth, along pressure plane
geartype_rack = [ gear_pitch, 12.0, 20, 0.35, 0.35 ]; 

travel_updown=50;
travel_leftright=80;

gear_motor = gear_create(geartype_rack,8);
motorR=gear_R(gear_motor);
ringteeth = round((travel_updown+2*motorR)/geartype_Dpitch(geartype_rack)/4)*4;
gear_ring = gear_create(geartype_rack,ringteeth);
ringR = gear_R(gear_ring); // distance from center to pressure plane
gearbetween=round(travel_leftright/geartype_Cpitch(geartype_rack))*geartype_Cpitch(geartype_rack); // distance between the centers of the gear rings
axlecenterR=-gear_R(gear_motor); // from outer rack to axle

module gearbetween_both() {
    children();
    translate([gearbetween,0,0]) children();
}


module drivegear_3D(extra_ht) 
{
    linear_extrude(height=gearout_ht+extra_ht,convexity=6)
    difference() {
        gear_2D(gear_motor);
        circle(d=axle_OD);
    }
}


gearwall=8; // pressure plane to back of support

// Simple 2D hull around gear area.
//   pressureR is distance beyond the gear pressure surface.
module gear_outline(pressureR=gearwall) {
    hull() gearbetween_both() circle(r=ringR+pressureR);
}

// Circular gear rack
module gear_racks()
{
    clearance=0.1;
    offset(r=-clearance)
    difference() {
        gear_outline();
        
        gear_outline(-geartype_sub(geartype_rack));
        
        rackteeth=gearbetween/geartype_Cpitch(geartype_rack)+1;
        for (side=[-1,+1]) scale([1,side,1])
            translate([0,ringR,0])
                scale([-1,1,1])
                rotate([0,0,90])
                    gear_rack_2D(geartype_rack,gearwall,rackteeth);
        
        gearbetween_both() gear_2D(gear_ring);
    }
}
//gear_racks();
//color([1,0,0]) gear_outline(-geartype_sub(geartype_rack));

// The slot down which the axle bearing travels
module bearing_slot_2D(inset=0) {
    difference() {
        gear_outline(axlecenterR+bearing_slideOD/2-inset);
        gear_outline(axlecenterR-bearing_slideOD/2+inset);
    }
}

// The slot needed for the hex cap on the axle to not hit any supports
module axlecap_slot_2D() {
    difference() {
        gear_outline(axlecenterR+axlecap_OD/2);
        gear_outline(axlecenterR-axlecap_OD/2);
    }
}
//axlecap_slot_2D();

zstart=-20;
module track_3D(illustrate=0) {
    difference() {
        union() {
            linear_extrude(height=gearout_ht,convexity=6)
                gear_racks();
            hull() {
                for (z=[0,1])
                    translate([0,0,z*zstart])
                    linear_extrude(height=0.01,convexity=6)
                        gear_outline(z?-2:gearwall);
            }
        }
        
        // space for bearing
        translate([0,0,-bearing_wallZ-bearing_Z])
            linear_extrude(height=bearing_Z,convexity=4)
            difference()
            {
                bearing_slot_2D();
                // Leave a thin wall to support the bearing lip
                if (!illustrate)
                    bearing_slot_2D(bearing_wallR-0.3);
            }
        
        // Tiny wall holds bearing in place
        translate([0,0,-bearing_wallZ-bearing_Z-bearing_wallZ])
            linear_extrude(height=bearing_wallZ+bearing_Z+bearing_wallZ+0.2,convexity=6)
                bearing_slot_2D(bearing_wallR);
        
        // Circular cut to insert bearing
        translate([gearbetween/2,-(ringR+axlecenterR),-bearing_wallZ-0.01])
            cylinder(d=bearing_slideOD+1,h=bearing_wallZ+0.1);
        
        // Slot for axle hex cap
        translate([0,0,-bearing_wallZ-bearing_Z-bearing_wallZ-axlecap_Z])
            linear_extrude(height=axlecap_Z+0.2,convexity=6)
                axlecap_slot_2D();
                
        // Lighten the center
        round=2;
        translate([0,0,zstart+1.5])
        linear_extrude(height=-zstart,convexity=4)
        offset(r=+round) offset(r=-round)
        difference() {
            gear_outline(axlecenterR-bearing_slideOD/2-bearing_wallR);
            
            // Ribs:
            translate([gearbetween/2,0,0])
                for (r=[-90,-45,0,+45]) rotate([0,0,r])
                    square([2,1000],center=true);
        }
        
        // cutaway
        if (illustrate) 
            translate([0,0,-500]) cube([1000,1000,1000]);
    }
}
track_3D();

//cylinder(d=bearing_OD,h=bearing_Z); // bearing in slot
//translate([0,ringR-motorR,0]) drivegear_3D();
//translate([0,0,zstart]) drivegear_3D(10);

/*********** Motor gearbox hookup *********/
// Make children at the center point of each of the six drive gear pips
module tongan_pip_centers() 
{
    for (ball=[0:360/6:360-1]) rotate([0,0,ball])
        translate([drive_OD/2,0,0])
            children();
}


/* 2D outline of output gear outside surface */
module tongan_output_OD_2D()
{
    difference() {
        circle(d=drive_OD);
        tongan_pip_centers() circle(d=11);
    }
}

/* 2D outline of carved channel for output drive gear */
module tongan_output_drive_2D() {
    difference() {
        offset(r=+clearance)
            tongan_output_OD_2D();
        
        gearout=2.0; // includes a bit of clearance
        difference() {
            offset(r=-gearout-clearance)
                tongan_output_OD_2D();
        }
    }
}

/* 2D outline of device wrapping around output gear */
module tongan_output_wall_2D()
{
    round=5;
    // outer wall:
    difference() {
        circle(d=min(drive_OD_clear,drive_OD+2*wall));
        
        offset(r=+clearance)
            tongan_output_OD_2D();
    }
    // inner wall:
    gearout=2.0; // includes a bit of clearance
    difference() {
        offset(r=-gearout-clearance)
            tongan_output_OD_2D();
        //offset(r=+round-wall) offset(r=-round-gearout)
        //    tongan_output_OD_2D();
        
        circle(d=bearing_OD);
    }
}

module tongan_drivegear_3D()
{
    linear_extrude(height=motorout_ht,convexity=6) {
        tongan_output_wall_2D();
    }
}

module tongan_output_printable() {
    difference() {
        tongan_drivegear_3D();
    }
}

//tongan_output_printable();



