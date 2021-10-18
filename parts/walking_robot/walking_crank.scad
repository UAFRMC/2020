/*
Model "Tongan" RS550 gearbox output peripheral:
    "Crank" to hold an eccentric rotating foot, to allow walking.
*/
inch=25.4; // file units are mm

$fs=0.1;
$fa=5;
wall=3.6;
clearance=0.2;

drive_OD=61; // output gear outside diameter
drive_OD_clear=66; // total space around drive gear

bearing_OD=7/8*inch+clearance;

out_ht=12; // height of output gear (plus a little wiggle room)

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
        offset(r=+round-wall) offset(r=-round-gearout)
            tongan_output_OD_2D();
    }
}

module torque_bar(big=0,extra_long=0) {
    size=[inch,61+2*wall-5,inch]+[2*big,extra_long-0.01*big,-0.01*big];
    translate([-size[0]/2,-size[1]/2,1.5])
        cube(size);
}

// This is the offset crank distance:
holes=[60,40,20];
hole_OD=1/4*inch; // tap for 5/16" bolt / bearing

module crank_holes() {
    for (h=holes) translate([0,h,0]) circle(d=hole_OD);
}

module tongan_crank_2D() {
    difference() {
        hull() {
            circle(d=drive_OD_clear);
            translate([0,holes[0],0]) circle(d=hole_OD+2*wall);
        }
        
        circle(d=bearing_OD);
        
        crank_holes();
    }
}

// Create walls on the outline of this shape
module wallify()
{
    round=3.0;
    difference() {
        children();
        offset(r=+round) offset(r=-round)
        offset(r=-wall) 
            children();
    }
}

module tongan_output_3D() {
    difference() {
        union() {
            // drive gear mating section
            linear_extrude(height=out_ht,convexity=8) 
                wallify()
                difference() {
                    circle(d=drive_OD_clear);
                    tongan_output_drive_2D();
                    crank_holes();
                    circle(d=bearing_OD);
                }
            
            // Plate tying everything together
            scale([1,1,-1]) // below zero elevation
            linear_extrude(height=2,convexity=4)
                tongan_crank_2D();
            
            // Reinforcing around bearing
            difference() {
                cylinder(d=bearing_OD+2*wall,h=out_ht);
                translate([0,0,-0.01]) {
                    cylinder(d=bearing_OD,h=out_ht-wall);
                    cylinder(d=3/8*inch+2*clearance,h=2*out_ht);
                }
            }
            
            // more material around crank holes
            linear_extrude(height=out_ht-5) {
                wallify() difference() {
                    tongan_crank_2D();
                    tongan_output_drive_2D();
                }
            }
        }
        
        // re-carve drive channel
        linear_extrude(height=out_ht+1,convexity=4)
            tongan_output_drive_2D();
        
        // For M3 bolts to thread into print (hold layers together)
        tongan_pip_centers() {
            cylinder(d=2.7,h=100,center=true);
            //translate([0,0,17])
            //    cylinder(d=6.5,h=100);
        }
    }
}

module tongan_output_printable() {
    difference() {
        tongan_output_3D();
    }
}

tongan_output_printable();


