/*
Sweep a robot foot through a circular path, using a spinning circular offset crank.

Model "Tongan" RS550 gearbox output peripheral:
    "Crank" to hold an eccentric rotating foot, to allow walking.
*/
inch=25.4; // file units are mm

include <../gear_library.scad>;
$fs=0.2;
$fa=5;

spinR=50; // radius of circle that foot sweeps out (== the walking ground clearance?)

wall=1.7;
clearance=0.2;

drive_OD=61; // output gear outside diameter
drive_OD_clear=66; // total space around drive gear
drivebearing_OD=7/8*inch; // goes on motor shaft
drivebearingZ=7; // depth of hole for bearing
outerbearingSpin=[0,0,-30]; // location of outside bearing



bearingBallOD=6.1; //<- airsoft BB, plus clearance
bearingBallR=bearingBallOD/2;
bearingFlangeX=bearingBallR+1.5*wall; //<- thickness of flange supporting bearing
bearingFlangeY=2*wall+bearingBallOD; // along bearing Y / world Z direction
bearingWall=3; // thickness of support for bearings to frame
bearingSpace=1.0; // distance between moving halves of bearing (on each side of centerline)

bearingZlo = -10; // Z location of bearing centerline, bottom

bearingCenterR=drive_OD_clear/2+spinR; // bearing radial centerline
bearingSpin=[spinR,0,bearingZlo]; // point around which bearing spins

bearingFillSlotX=12;
bearingFillSlotY=25;
bearingFillSlot=bearingSpin+[bearingCenterR-bearingFillSlotX/2,0,bearingFlangeY/2-bearingBallR];

// synchronization gear connects several spinners
gearZ=8; // Z height of gears
gearZloc=-20; // Z location of gear centerline
gearSpin=[bearingSpin[0],bearingSpin[1],gearZloc];
geartype_drive = [ 7.0, 10.0, 20, 0.32, 0.4 ];
gearteeth = round((bearingCenterR+geartype_sub(geartype_drive)/2)*2/geartype_Dpitch(geartype_drive));
gear_drive=gear_create(geartype_drive,gearteeth);

out_ht=12; // height of output gear (plus a little wiggle room)

// Make children at the center point of each of the six drive gear pips
module tongan_pip_centers(skip=-1) 
{
    for (ball=[0:360/6:360-1]) if (ball!=skip) rotate([0,0,ball])
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
        
        circle(d=drivebearing_OD);
        
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
                    circle(d=drivebearing_OD);
                }
            
            // Plate tying everything together
            scale([1,1,-1]) // below zero elevation
            linear_extrude(height=2,convexity=4)
                tongan_crank_2D();
            
            // Reinforcing around bearing
            difference() {
                cylinder(d=drivebearing_OD+2*wall,h=out_ht);
                translate([0,0,-0.01]) {
                    cylinder(d=drivebearing_OD,h=out_ht-wall);
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

// Spaces needed for tongan drive:
module tongan_subtract_2D() {
    tongan_output_drive_2D();
    circle(d=drivebearing_OD-2*wall);
}

// Drive parts for tongan gearbox:
module tongan_simple_add_2D(d=drive_OD_clear) {
    difference() {
        circle(d=d);
        tongan_subtract_2D();
    }
}

// Lightening holes
module tongan_lightening_holes_2D() {
    round=5.5;
    offset(r=+round) offset(r=-round)
    offset(r=-wall) 
    difference() {
        tongan_simple_add_2D(d=drive_OD);
        circle(d=drivebearing_OD);
    }
}

// Subtract, including lightening holes
module tongan_lighter_subtract_2D() {
    tongan_subtract_2D();
    tongan_lightening_holes_2D();
}

// Heavy drive spline for Tongan gearbox, including bearing hole
module tongan_add_2D() {
    difference() {
        circle(d=drive_OD_clear);
        tongan_lighter_subtract_2D();
    }
}

// Mark these surfaces as very round (e.g., bearing or outer surfaces)
module veryRound() {
    $fs=0.1; $fa=2; // this surface needs to be round
    children();
}


// Bearing contact surface
module bearing_surface_2D(extraHeight=1) 
{
    round=0.7;
    offset(r=-round) offset(r=+round)
    translate([bearingCenterR,0,0]) {
        circle(d=bearingBallOD);
        square([2*bearingSpace,bearingFlangeY+extraHeight],center=true);
    }
}

module bearing_surface_3D(extraHeight=1)
{
    veryRound() translate(bearingSpin) rotate_extrude()     
        bearing_surface_2D(extraHeight);
}


// Space for an M3 bolt, with cap, threaded into the plastic here
module M3_threaded() {
    cylinder(d=6.5,h=10); // M3 cap
    cylinder(d=2.5,h=100,center=true); // M3 shaft
}

module bearingFillSlotHoles() {
    translate(bearingFillSlot) 
    for (updown=[-1,+1]) 
        translate([-bearingFillSlotX*0.2,updown*bearingFillSlotY*0.35,0])
            children();
}
module bearingFillSlot2D() 
{
    translate(bearingFillSlot) 
        square([bearingFillSlotX,bearingFillSlotY],center=true);
}

/************ Spinner *************
The spinner is the central part, connecting to the motor, bearings, and gear.
*/

// 3D printed spinner is tilted like this (more space for bearing, and better layer lines tying across Z)
spinner_print_tilt=[0,3,0];

//  Basic shape of overall spinner
module spinner_hull() {
    hull() {
        cylinder(h=out_ht,d=drive_OD_clear); // taper up to drive bearing
        translate(bearingSpin) 
            cylinder(r=bearingCenterR-bearingSpace,h=bearingFlangeY,center=true);
        translate(gearSpin)
            cylinder(r=gear_IR(gear_drive)-wall,h=gearZ,center=true);
        translate(outerbearingSpin) {
            cylinder(d=2*wall+drivebearing_OD,h=drivebearingZ);
            
            // Support basically the whole bottom of the print
            rotate(-spinner_print_tilt)
            translate([bearingSpin[0],bearingSpin[1],0])
            linear_extrude(height=10)
            difference() { 
                circle(r=gear_IR(gear_drive)-wall);
            }
        }
    }
}

// Slots cut in spinner
module spinner_slots_2D()
{
    round=2;
    rib=wall;
    ribStart=drive_OD/2-rib/2+2;
    
    offset(r=+round) offset(r=-round)
    difference() {
        translate(bearingSpin) circle(r=bearingCenterR-bearingFlangeX);
        circle(d=drive_OD_clear); 
        
        offset(r=0.75*bearingFlangeX) bearingFillSlot2D();
        
        ribs=9;
        angleDel=360/ribs;
        for (ribAngle=[0:angleDel:360-1])
            //if (abs(ribAngle-180)>40) // omit near side (lumpy)
            rotate([0,0,ribAngle])
            translate([ribStart,0])
                square([rib,4*bearingCenterR],center=true);
        
        rebarCircle=66; // connect empty spaces in grid (for triangles)
        rotate([0,0,360/ribs*0.5])
        difference() {
            circle(r=rebarCircle,$fn=ribs);
            circle(r=rebarCircle-wall,$fn=ribs);
        }
    }
}

// Spinner with lightening holes and such
module spinner_full() 
{
    difference() {
        union() {
            intersection() {
                spinner_hull();
                // do full lightening holes here
            }
            
        }
        
        // Main slots/ribs
        linear_extrude(height=400,center=true,convexity=6)
            spinner_slots_2D();
        
        // Space for motor drive
        linear_extrude(height=out_ht+1,convexity=8) tongan_lighter_subtract_2D();
        
        // M3 bolts to hold layers together
        tongan_pip_centers(180) translate([-1,0,out_ht-4]) 
            M3_threaded();
        
        
        // Lighten around motor
        linear_extrude(height=500,center=true,convexity=6) 
            tongan_lightening_holes_2D();
        
        // Clearance around motor
        motor_round=12;
        rotate_extrude()
            translate([drive_OD_clear/2,bearingZlo+bearingFlangeY/2])
            scale([3,1])
            offset(r=+motor_round) offset(r=-motor_round)
            square([1000,1000]);
        
        // Space for main axle
        cylinder(d=drivebearing_OD-2*wall,h=300,center=true);
        
        // Space for drivebearings
        translate([0,0,out_ht-drivebearingZ])
            cylinder(d=drivebearing_OD,h=2*drivebearingZ);
        translate(outerbearingSpin) 
            cylinder(d=drivebearing_OD,h=2*drivebearingZ,center=true);
        
        // Bearing fill slot
        bearingFillSlotHoles() M3_threaded();
        translate([0,0,bearingFillSlot[2]])
        linear_extrude(height=bearingFlangeY+clearance,center=true)
                bearingFillSlot2D();
        
        // Smooth surface of large bearing
        bearing_surface_3D();
    }
    
    // Add gear teeth
    translate(gearSpin) linear_extrude(height=gearZ,convexity=4,center=true)
    difference() {
        gear_2D(gear_drive);
        circle(d=gear_ID(gear_drive)-2*wall);
    }
}

// 3D printable orientation for spinner
module spinner_printable() {
    difference() {
        translate(-outerbearingSpin) 
        rotate(spinner_print_tilt)
            spinner_full();
        translate([0,0,-1000]) cube([2000,2000,2000],center=true);
    }
}

// 3D printable fill slot (holds balls in the spinner)
module fillSlot() {
    difference() {
        translate([0,0,bearingFillSlot[2]])
        linear_extrude(height=bearingFlangeY,center=true)
                bearingFillSlot2D();
        bearingFillSlotHoles() {
            M3_threaded();
            cylinder(d=3.1,h=100,center=true); // <- let shaft through
        }
        bearing_surface_3D(5);
    }
}

/************* Frame ****************/
// The frame holds the outer bearing race, and mounts down to the foot
frameBaseY=-bearingCenterR-15;

gear_idler=gear_create(geartype_drive,10);
center_to_idlerR=gear_R(gear_drive)+gear_R(gear_idler);
idlerSpin=[center_to_idlerR,0,0];
idlerAxleOD=1/4*inch;
echo("Center to idler distance: ",center_to_idlerR);

// Basic 2D outline of frame
module frame_2D() {
    difference() {
        r=bearingCenterR+bearingFlangeX;
        hull() { // outside frame
            translate([-r,frameBaseY])
                square([2*r,1]);
            circle(r=r); 
            translate(idlerSpin) circle(d=idlerAxleOD+2*wall);
        }
        translate(idlerSpin) circle(d=idlerAxleOD);
        
        circle(r=bearingCenterR+bearingSpace); // center hole for spinner
    }
}

// Ring supporting outside of bearing area
module frame_bearing_reinforce() 
{
    rotate_extrude() translate([bearingCenterR,0]) 
        circle(r=bearingBallR+2.5*wall);
}

// Degrees to rotate frame during printing, for nice layer line directions
frame_print_tilt=[-3,0,0];
frame_base_lip=50; // distance the bottom of the frame should stick out

// Trim inside face of frame surface, so it can be printed flat
module frame_trim() {
    // Trim off top surface, for printability
    translate([0,bearingCenterR,bearingFlangeY/2])
        rotate(frame_print_tilt)
            translate([0,0,1000])
                cube([2000,2000,2000],center=true);
}

module frame_base(offset=0,round=0) {
    difference() {
        union() {
            translate([0,0,-frame_base_lip])
                linear_extrude(height=100,convexity=4)
                {
                    offset(r=+round) offset(r=-round)
                    difference() {
                        offset(r=-offset)
                            frame_2D();
                        children();
                    }
                }
        }
   }
}


module frame_full() {
    difference() {
        union() {
            frame_base(0);
            frame_bearing_reinforce();
        }
        
        // Trim base flat
        frame_trim();
        
        // Spot for the spinner inside
        cylinder(r=bearingCenterR,h=100,center=true);
        
        // Hollow out the lower pockets
        difference() {
            translate([0,0,-2]) //<- leave a flat floor
            difference() {
                frame_base(wall,4) { // cut in ribs
                    for (hrib=[-1,-0.2,+0.2,+1])
                        translate([bearingCenterR*hrib*0.6,0,0])
                            square([wall,1000],center=true);
                    for (rrib=[-90,-60,-45,+45,+60]) 
                        rotate([0,0,rrib])
                            square([wall,1000],center=true);
                    for (rrib=[0]) 
                        translate(idlerSpin) rotate([0,0,rrib])
                            square([wall,50],center=true);
                }
                frame_trim();
            }
            // leave meat around the bearings
            frame_bearing_reinforce();
        }
        
        // Leave space for the main gear
        translate(-bearingSpin+gearSpin) 
            translate([0,0,gearZ/2+1])
            scale([1,1,-1])
                cylinder(d=gear_OD(gear_drive)+8,h=100);
        
        // Trim back front face, to leave clearance for gear to mate horizontally
        clear_round=frame_base_lip-5;
        translate([0,frameBaseY+2*wall,-bearingFlangeY/2])
        rotate([0,90,0])
        linear_extrude(height=3*bearingCenterR,center=true)
            scale([1,1.5,1])
            offset(r=+clear_round) offset(r=-clear_round)
                square([3*bearingCenterR,3*bearingCenterR]);
        
        // Cut in the bearing surface
        translate(-bearingSpin) bearing_surface_3D(50);
        
        // Cut in mounting bolt holes
        for (x=[-0.75,-0.4,+0.4,+0.75])
            translate([x*bearingCenterR,frameBaseY,-((abs(x)<0.5)?0.6:0.0)*frame_base_lip])
                rotate([90,0,0]) cylinder(d=6,h=30,center=true);
    }
}

module frame_printable() {
    rotate(-frame_print_tilt)
        rotate([180,0,0])
            frame_full();
}

module idler_gear() {
    translate(idlerSpin) 
    difference() {
        union() {
            linear_extrude(height=gearZ,convexity=6) gear_2D(gear_idler);
            cylinder(d=idlerAxleOD+2*wall,h=gearZ+1);
        }
        cylinder(d=idlerAxleOD,h=100,center=true);
    }
}

module assembled() {
    translate(-bearingSpin) spinner_full(); 
    color([0.3,0.5,1.0]) frame_full();
    translate(-bearingSpin+gearSpin+[0,0,-4]) idler_gear();
}

difference() { assembled(); //rotate([0,0,45]) translate([0,0,-100]) cube([1000,1000,1000]); 
}
//spinner_printable();
//frame_printable();
//fillSlot(); 
//idler_gear();
