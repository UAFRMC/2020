/**
  Demonstrate a way of creating rounded 3D shapes with walls,
  starting from a simple 2D outline.
  
  Dr. Orion Lawlor, lawlor@alaska.edu, 2016-02-01 (public domain)
*/

dz=0.3; // size of 3D printer's layers (smaller == slower!)

 // Round off this 2D shape
 module round_2D(f=5.0) {
    offset(r=f) offset(r=-f) // rounds outside corners
    offset(r=-f) offset(r=f) // rounds inside corners
     children();
 }
 
 // Create a rounded 3D profile of this 2D shape.
 module round_3D_floor(h,f,wall) {
     fi=f-wall; // inside radius
     fo=f; // outside radius
     
     // Fillets and floor: stepwise for 3D printer layers (lame hack!)
     for (z=[dz:dz:fo+dz]) {
         translate([0,0,z-dz])
         linear_extrude(height=dz,convexity=10) 
         difference() {
             // Outside of floor
             offset(r=-fo+fo*sqrt(1-pow(1-z/fo,2))) 
                children();
             // Inside of floor
             offset(r=-fi+fi*sqrt(1-pow(1-(z-wall)/fi,2))-wall) 
                children();
         }
     }
 }

// Create a 2D extrusion inside the child shape
module round_2D_wall_inside(wall) {
        difference() { 
            children();  // outside of walls
            offset(r=-wall) children();  // inside of walls
        }
}

// Create a 2D extrusion outside the child shape
module round_2D_wall_outside(wall) {
        difference() { 
            offset(r=+wall) children();  // outside of walls
            children();  // inside of walls
        }
}
 
 // Create a rounded 3D extrusion of the outside of this shape
 module round_3D_wall(h,f,wall) {
     linear_extrude(height=h,convexity=10) 
		round_2D_wall_inside(wall) children();
 }
 
 // Extrude 2D child up into walled shape of this height
 module round_3D_closed(h,f,wall) {
     
     // Walls:
     translate([0,0,f])
        round_3D_wall(h-2*f,f,wall) 
            children();
     
     // Floor:
     round_3D_floor(h,f,wall) children();
     
     // Ceiling:
     translate([0,0,h])
        scale([1,1,-1]) // flip Z
            round_3D_floor(h,f,wall) children();
}

/*
// Simple filleted 2D outline (demo shape)
module shapey_2D() {
    difference() {
        union() 
         {
            square([150,20]);
            square([50,50]);
         }
         circle(r=10);
     }
 }
 
// Use these tools on a random shape:
difference() {
f=4;
round_3D_closed(50,f,2) 
    round_2D(f) 
        shapey_2D();

translate([-1,-1,-1])
    cube([25,80,25]);
}
*/

