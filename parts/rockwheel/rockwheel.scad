/*
  Micro-size rockwheel, to break up packed gravel for NASA Robotic Mining Contest
*/
$fs=0.1; $fa=3;
inch=25.4; // file units are mm

rockwheelOD=100;
rockwheelZ=50;

axleOD=3/8*inch;

toothOD=4.7; // steel pins

rockwheelOuterWall=4;

//cylinder(d=rockwheelOD,h=rockwheelZ);

module rockwheel2D()
{
    difference() {
        circle(d=rockwheelOD);
        circle(d=axleOD);
    }
}

// Cuts interior of rockwheel
module rockwheelCut2D() 
{
    round=5;
    
    offset(r=+round) offset(r=-round)
    difference() {
        offset(r=-rockwheelOuterWall) rockwheel2D();
        
        for (spoke=[0:360/4:360-1])
            rotate([0,0,spoke])
        scale([-1,1,1])
            translate([-axleOD*0.5,axleOD*0.5])
                square([rockwheelOD,3]);
    }
}

// Rockwheel minus the cut
module rockwheelSpokes2D() 
{
    difference() {
        rockwheel2D();
        rockwheelCut2D();
    }
}

// Create children at the centers of each rock tooth
module rockwheelTeethCenters() 
{
    step=30; // degrees between teeth
    for (rot=[0:step:360-1]) rotate([0,0,rot])
    {
        phase=2*(((rot/step*0.62)%1)-0.5); // between -1 (bottom edge) and +1 (top edge)
        z=phase*(rockwheelZ/2-toothOD)+1.5+rockwheelZ/2;
        translate([rockwheelOD/2-rockwheelOuterWall+1,0,z])
            rotate([45,90,0])
            rotate([0,-20*phase,0])
                children();
    }
}

// Final finished model
module rockwheel3D() {
    difference() {
        union() {
            union() {
                linear_extrude(height=rockwheelZ,convexity=6) rockwheelSpokes2D();
                // Boss around hex head
                translate([0,0,rockwheelZ-12]) cylinder(d=22,h=12);
            }
            
            // Bosses around each tooth
            intersection() {
                boss=16;
                rockwheelTeethCenters()
                {
                    cylinder(d1=boss,d2=toothOD+2,h=10);
                    scale([1,1,-1]) cylinder(d=boss,h=10);
                }
                
                // Don't let bosses reach the interior of the wheel
                difference() {
                    cylinder(d=2*rockwheelOD,h=rockwheelZ+10);
                    cylinder(d=rockwheelOD-2*rockwheelOuterWall+0.1,h=3*rockwheelZ,center=true);
                }
            }
        }
        
        // Hex head for a bolt mount
        cylinder(d=0.1+3/8*inch,h=rockwheelZ);
        translate([0,0,rockwheelZ-10])
            cylinder($fn=6,d=0.2+9/16*inch/cos(30),h=20);
        
        // Holes for the steel teeth pins
        #rockwheelTeethCenters()
            cylinder(d=toothOD,h=3/4*inch);
    }
}

rockwheel3D();
