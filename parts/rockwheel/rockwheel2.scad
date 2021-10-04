/*
  Micro-size rockwheel, to break up:
    - packed gravel for NASA Robotic Mining Contest
    - lunar permafrost for NASA Break The Ice contest
  This version has internal space for an RS-550 size motor.
  
*/
include <gear_library.scad>;


//$fs=0.1; $fa=3; // fine resolution
$fs=0.3; $fa=10; // coarse resolution
inch=25.4; // file units are mm

rockwheelZ=120; // length of round cylinder area

rockwheelOD=100; // diameter of round cylinder area
rockwheelWall=3; // plastic thickness in main rockwheel cylinder
rockwheelID=rockwheelOD-2*rockwheelWall;


axleOD=3/8*inch;

toothOD=4.7; // steel pins

// Rebar pins hold the outer ring together, add inertia
rebarOD=1/8*inch;
nrebar=3;
rebarR=rockwheelOD/2-1.0; // centerline of rebar (produces bump-out in surface)
rebarCover=1.5; // plastic around the rebar pins

module rebarCenters() {
    rotate([0,0,3]) //<- phase rebar to miss teeth
    for (r=[0:nrebar-0.1]) rotate([0,0,r*360/nrebar])
        translate([rebarR,0,0])
            children();
}
module rebarPins() {
    rebarCenters() cylinder(d=rebarOD,h=rockwheelZ);
}



// Clear lexan coverplates over both ends (non-rotating?)
coverplateZ=2.0; // thickness of top/bottom plates
coverplateOD=rockwheelOD-4; // thickness of top/bottom plates
coverplateClearance=0.3; // vertical clearance for coverplates (per side)

motorZ=rockwheelZ-22; // face of motor
gearZ=motorZ+8; // start of gears
gearThick = rockwheelZ-gearZ-2;

coreOD=39; // diameter of center shaft

// Mark these surfaces as very round (e.g., bearing or outer surfaces)
module veryRound() {
    $fs=0.15; $fa=3; // this surface needs to be round
    children();
}



/************** Bearings ******************/

// Bearings are filled with airsoft BBs
// Diameter of spherical bearings, 6mm airsoft
//   (in the long run, tapered roller bearings are probably better)
bearingBallOD=6.1; //<- airsoft BB, plus clearance
bearingBallR=bearingBallOD/2;
bearingFlangeX=12.0; //<- thickness of flange supporting bearing
bearingFlangeY=10.0; // along bearing Y / world Z direction
bearingWall=3; // thickness of support for bearings to frame
bearingSpace=1.0; // distance between moving parts

bearingZlo = 16; // Z height of bearing centerline, bottom
bearingZhi = gearZ-bearingZlo;

bearingCenterR=35; // dividing line between bearing halves


module bearing2D() {
    intersection() {
        union() 
        for (z=[bearingZlo,bearingZhi]) translate([0,z])
        {
            translate([bearingCenterR,0])
                difference() {
                    round=6; // big chamfers on supports
                    offset(r=-round) offset(r=+round)
                    union() {
                        // Meat around actual bearing
                        square([bearingFlangeX,bearingFlangeY],center=true);
                        // Diagonal supports to spread the load
                        translate([-bearingCenterR,0])
                        for (side=[-1,+1])
                            hull() {
                                translate([coreOD/2+2,side*14]) circle(d=bearingWall);
                                translate([rockwheelOD/2-2,-side*12]) circle(d=bearingWall);
                            }
                    }
                    // Hole for actual bearing balls
                    circle(d=bearingBallOD);
                    // Space between bearing halves
                    square([bearingSpace,bearingFlangeY+1],center=true);
                }
        }
        
        // Trim down to correct space
        translate([coreOD/2,0]) square([rockwheelOD/2-coreOD/2,gearZ]);
    }
}

// This is the path the bearings travel


// Select the inside of the bearing
module bearingInsideCut2D() {
    square([bearingCenterR,rockwheelZ]);
}
module bearingFinalRound() {    
    round=0.7; // tiny chamfer enhances printability of channel
    offset(r=+round) offset(r=-round)
        children();
}
module bearing3Dinside() {
    veryRound() rotate_extrude(convexity=8) 
    bearingFinalRound() intersection() {
        bearing2D();
        bearingInsideCut2D();
    }
}
module bearing3Doutside() {
    veryRound() rotate_extrude(convexity=8) 
    bearingFinalRound() difference() {
        bearing2D();
        bearingInsideCut2D();
    }
}

// This little cube allows bearings to be fed into the races
bearingPlugSize=[10,7,14];

// Slot to allow bearing balls to be inserted.
//   Screws into moving face after insertion.
module bearingPlug(fatten=0.0)
{
    translate([-bearingPlugSize[0]/2-fatten,bearingCenterR+0.01*fatten,-bearingBallR-fatten])
        cube(bearingPlugSize+2*[fatten,fatten,0]);
}

// M3x16 bolt hole, to secure the assembly slot
module bearingAssemblyScrew(diameter=3.1)
{
    translate([0,bearingCenterR+bearingBallOD/2+2,0])
        cylinder(d=diameter,h=rockwheelZ);
}

// Put children at the locations of each bearing assembly slot
module bearingPlugs() {    
    for (top=[0,1]) translate([0,0,top?bearingZhi:bearingZlo])
        scale([1,1,top?+1:-1])
            children();
}

// Add material in preparation for cutting Plugs
module bearing3DoutsideAddPlugs()
{
    union() {
        bearing3Doutside();
        // Add material behind and around the slot
        bearingPlugs()
            translate([0,bearingBallR])
            bearingPlug(2.0);
    }
}

module bearing3DoutsideWithPlugs() {    
    difference() {
        bearing3DoutsideAddPlugs();
        
        // Space for the slot
        bearingPlugs()
            bearingPlug(0.2);
        
        // Screw to retain slot
        bearingAssemblyScrew(2.5); // tap the plastic
    }
}

module bearingPlugsPrintable() {
    rotate([-90,0,0])
    intersection() {
        bearing3DoutsideAddPlugs();
        difference() {
            bearingPlugs()
                bearingPlug(0.0);
            bearingAssemblyScrew(3.1); // clear an M3
        }
    }
}

/********************* Gears ********************/
// Geared drive:
geartype_motor=geartype_550; // RS-550 type 10-tooth gear
gear_motor=gear_create(geartype_motor,10); //<- tiny input
gear_ring=gear_create(geartype_motor,114); //<- big output
gear_planet=gear_create(geartype_motor,
    (gear_nteeth(gear_ring)-gear_nteeth(gear_motor))/2);
gearClearance=0.1;

nplanet=2;
planet_radius=gear_R(gear_motor)+gear_R(gear_planet);
planetAxleOD=1/4*inch;
planetAxleTap=0.191*inch;

echo("Planet radius=",planet_radius);
// Create children at each planet gear
module makePlanets() {
    for (p=[0:nplanet-0.1]) rotate([0,0,p*360/nplanet])
        translate([planet_radius,0,0])
            children();
}


// 550 size motor face
motor_OD=38.4; // motor body (plus assembly clearance)
motor_boss=13; // boss on front side
motor_gear_height=10.0;
motor_bolt_sep=25; // distance between mounting bolt holes
motor_bolt_dia=3.1; // clearance for M3 socket cap screws
motor_bolt_cap=7; // socket caps
motor_bolt_len=5-2; // use M3x5, with 2mm thread engagement

// Holes for mounting motor, relative to motor face
module motorMount()
{
		// Thru hole for motor boss and drive gear
		cylinder(d=motor_boss+ring_clearance,h=100,center=true);
		
		// Space to insert M3 cap screw to hold motor to drive.
		for (side=[-1,+1])
		translate([side*motor_bolt_sep/2,0,motor_bolt_len]) {
			caplen=0.1+motor_gear_height-motor_bolt_len;
			cylinder(d=motor_bolt_cap,h=caplen);
			//translate([side*-3,0,caplen-3]) cylinder(d=motor_bolt_cap,h=caplen);
			scale([1,1,-1]) translate([0,0,-0.01])
				cylinder(d=motor_bolt_dia,h=10);
		}
}

// The core holds the mounting pins and motor
module core2Dholes() {
    circle(d=motor_OD); 
    makePlanets() circle(d=planetAxleTap); // planets (threaded part)
}

coreWall=3;
module core2D() {
    round=12;
    difference() {
        offset(r=-round) offset(r=+round)
        offset(r=+coreWall) core2Dholes();
        core2Dholes();
    }
}
module core3D() {
    difference() {
        union() {
            // Main core
            linear_extrude(height=gearZ,convexity=6) core2D();
            
            // Bearing supports
            bearing3Dinside();
            
            // Meat on top of motor for mounting
            translate([0,0,motorZ]) 
                cylinder(d=motor_OD+coreWall,h=gearZ-motorZ);

            // Stand up planets (and reinforce sidewalls)
            makePlanets() 
                    cylinder(d=planetAxleOD+2*coreWall,h=gearZ+1);
            
            // Taper support for outside
            translate([0,0,gearZ-0.01])
            difference() {
                cylinder(d1=motor_OD+2*coreWall,
                    d2=2*planet_radius+planetAxleOD,
                    h=rockwheelZ-gearZ);
                
                makePlanets()
                    cylinder(d=gear_OD(gear_planet)+2,h=50);
            }
        }
        
        // Motor mount holes
        translate([0,0,motorZ-0.01])
        {
            motorMount();
            // Front vent holes
            for (side=[-1,+1]) translate([0,16*side,0])
            {
                scale([1,1.2,1]) cylinder(d1=10,d2=12,h=25); // to outside vent
                translate([0,-2*side,0])
                scale([1.5,1,1]) cylinder(d=10,h=5); // spread around motor vent
            }
                    
            // Motor back vent holes
            translate([0,0,-50])
                rotate([90,0,0])
                    cylinder(d=20,h=100,center=true);
        }
        
        // Smooth areas where bolts enter
        planetAxleSmoothLen=28; // un-threaded area of 2" 1/4" bolt
        // Top (gear side)
        translate([0,0,rockwheelZ-planetAxleSmoothLen])
            makePlanets()
                cylinder(d=planetAxleOD,h=planetAxleSmoothLen);
        // Bottom (wire side)
        translate([0,0,-0.01])
            makePlanets()
                cylinder(d=planetAxleOD,h=planetAxleSmoothLen);
        // Thru
        makePlanets()
            cylinder(d=planetAxleTap,h=rockwheelZ);
    }
}

// Outside ring gear, drives the outer wheel
module ringGear2D() {    
    difference() {
        circle(d=rockwheelOD);
        gear_2D(gear_ring);
        rebarCenters() circle(d=rebarOD);
    }
}
module ringGear3D() {
    translate([0,0,gearZ])
        linear_extrude(height=rockwheelZ-gearZ,convexity=4) 
            ringGear2D();
}

// One planet gear
module planetGear2D() {
    difference() {
        offset(r=-gearClearance) // all clearance in easily replaceable planets
            gear_2D(gear_planet);
        circle(d=planetAxleOD);
    }
}
module planetGear3D() {
    linear_extrude(height=gearThick,convexity=4) planetGear2D();
}

// Illustrate all the active gears
module showGears2D() {
    ringGear2D();
    makePlanets() planetGear2D();
    rotate([0,0,0.5*360/gear_nteeth(gear_motor)])
        gear_2D(gear_motor);
}
module showGears3D() {
    ringGear3D();
    translate([0,0,gearZ+1]) {
        makePlanets() planetGear3D();
        rotate([0,0,0.5*360/gear_nteeth(gear_motor)])
            linear_extrude(height=10) gear_2D(gear_motor);
    }
}


/***************** Final Rockwheel assembly *************/

// Outer cylinder, plus bump-outs for rebar pins
module rockwheel2D() {
    round=8;
    difference() {
        offset(r=-round) offset(r=+round) 
        union() {
            veryRound() difference() {
                circle(d=rockwheelOD);
                circle(d=rockwheelID);
            }
            rebarCenters() circle(d=rebarOD+2*rebarCover);
        }
        rebarCenters() circle(d=rebarOD);
    }
}


//cylinder(d=rockwheelOD,h=rockwheelZ);
/*
module rockwheel2D()
{
    difference() {
        circle(d=rockwheelOD);
        circle(d=axleOD);
    }
}
*/

// Cuts interior of rockwheel
module rockwheelCut2D() 
{
    circle(d=rockwheelOD-2*rockwheelWall);
    /*
    round=5;
    
    offset(r=+round) offset(r=-round)
    difference() {
        offset(r=-rockwheelOuterWall) rockwheel2D();
        
        for (spoke=[0:360/4:360-1])
            rotate([0,0,spoke])
        scale([-1,1,1])
            translate([-axleOD*0.5,axleOD*0.5])
                square([rockwheelOD,3]);
    }*/
}

// Create children at the centers of each rock tooth
module rockwheelTeethCenters() 
{
    nteeth=30; // total tooth count
    step=360/nteeth; // degrees between teeth
    for (rot=[0:step:360-1]) rotate([0,0,rot])
    {
        phase=2*(((rot/step*0.376)%1)-0.5); // between -1 (bottom edge) and +1 (top edge)
        z=phase*(rockwheelZ/2-toothOD+2.5)+5.5+rockwheelZ/2;
        translate([rockwheelOD/2-rockwheelWall+1,0,z])
            rotate([50,90,0])
            rotate([0,-20*phase,0])
                children();
    }
}

module teethPins() {
    taper=toothOD*0.7-0.1;
    len=18;
    rockwheelTeethCenters() {
        cylinder(d=toothOD,h=len-taper);
        translate([0,0,len-taper-0.01]) 
            cylinder(d1=toothOD,d2=0.01,h=taper);
    }
}

// Bosses around each tooth
module teethBosses() {
    intersection() {
        boss=16;
        rockwheelTeethCenters()
        {
            // Tapered tooth holder
            cylinder(d1=boss,d2=toothOD+2,h=10);
            // Support back side with straight cylinder
            scale([1,1,-1]) cylinder(d=boss,h=10);
            // Fillet back to body
            translate([0,0,9])
            rotate([-45,0,0]) scale([1,1,-1])
                cylinder(d1=toothOD,d2=boss,h=8);
        }
        
        // Don't let bosses reach the interior of the wheel
        difference() {
            cylinder(d=2*rockwheelOD,h=rockwheelZ+10);
            cylinder(d=rockwheelOD-rockwheelWall,h=3*rockwheelZ,center=true);
        }
    }
}

// Ribs inside the main shell, to reduce deflection and transmit torsion forces
module shellRibs() {
    rib=2.5;
    ribID=rockwheelID-2*rib; // gear_D(gear_ring);
    intersection() {
        union()
        {
            translate([0,0,gearZ/2]) 
                for (angle=[0:30:360-1]) rotate([0,0,angle])
                    rotate([30,0,0])
                        cube([rockwheelOD*2,rib,rockwheelZ*2],center=true);
            cube([rockwheelOD*2,rockwheelOD*2,rib],center=true);
        }
        
        // Constrain extent of ribs
        difference() {
            cylinder(d=rockwheelOD,h=gearZ);
            cylinder(d=ribID,h=3*rockwheelZ,center=true);
        }
    }
}

// Final finished outside shell/rim
module shell3D(holes=1) {
    difference() {
        union() {
            // Main body
            translate([0,0,-coverplateZ])
            linear_extrude(height=rockwheelZ+2*coverplateZ,convexity=6) rockwheel2D();
            
            // Drive gear
            ringGear3D();

            taper=8; // bevel up to the ring gear (avoid stress riser)
            translate([0,0,gearZ-taper])
            difference() {
                cylinder(d=rockwheelOD-rockwheelWall,h=taper);
                translate([0,0,-0.01])
                cylinder(d1=rockwheelOD-2*rockwheelWall,
                    d2=gear_D(gear_ring),
                    h=taper+0.02);
            }
            
            // Bearings
            bearing3DoutsideWithPlugs();
            
            // Interior ribs
            shellRibs();
            
            teethBosses();
        }
        
        // Hex head for a bolt mount
        cylinder(d=0.1+3/8*inch,h=rockwheelZ);
        translate([0,0,rockwheelZ-10])
            cylinder($fn=6,d=0.2+9/16*inch/cos(30),h=20);
        
        // Clearance for non-rotating coverplates
        for (top=[0,1]) translate([0,0,top?rockwheelZ-coverplateClearance+0.01:-coverplateZ-0.01])
            veryRound() 
            cylinder(d=coverplateOD,h=coverplateZ+coverplateClearance);
        
        // Holes for the steel teeth pins
        if (holes) {
            teethPins();
            rebarPins();
        }
    }
    if (holes==0) teethPins();
}

module cutaway() {
    difference() {
        children();
        cube([300,300,300]);
    }
}

module fullyAssembled() {
    cutaway() {
        rockwheel3D(); // printable version

        core3D();
    }
    #showGears3D();
}

// Make sure the rebar and teeth don't intersect
module clearanceCheck() {
    teethPins();
    rebarPins();
}

part=0;
if (part==0) shell3D();
else if (part==1) core3D();
else if (part==2) bearingPlugsPrintable();
else if (part==3) makePlanets() planetGear3D();
else if (part==4) clearanceCheck();
else fullyAssembled();
