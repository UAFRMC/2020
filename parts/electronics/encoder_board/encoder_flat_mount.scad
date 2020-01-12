/* 
 Mounting plate to hold magnetic encoder
*/

$fs=0.1;
$fa=5;

wall=2.5;
floor=1.0;

inch=25.4;

left_x=-9; // board stub past pins
right_x=34; // to encoder
encoder_x=29;
solder_x=8;
height=9;
pin_y=12;
pin_x=0.11*inch;

board=2.2;
components=4.2;


module walls_2D(fatten=0) {
	offset(r=+1) offset(r=-1)
	{
	
		// Supports length along board
		w=fatten+board+fatten;
		translate([left_x-fatten,-w/2])
		square([right_x-left_x+2*fatten,w]);
		
		// Supports the pin
		x=fatten+pin_x+fatten;
		translate([-x/2,-pin_y])
		square([x,pin_y]);
	}
}

module encoder_mount() {
	translate([0,0,-floor])
	linear_extrude(height=floor)
	hull() walls_2D(wall);
	
	linear_extrude(height=height)
	difference() {
		offset(r=+0.5) offset(r=-0.5) offset(r=-1) offset(r=+1)
		difference() {
			walls_2D(wall);
			offset(r=0.01) walls_2D(0);
		}
		
		// Gap for pin solder and hot glue
		hollow=wall*0.75;
		scale([1.6,1.1,1]) translate([0,-0.5])
			circle(r=board/2+hollow);
		
		// Gap for actual encoder and solder
		translate([encoder_x,+0.5])
			scale([1.7,0.8,1])
			circle(r=board/2+hollow);
			
	}
}

encoder_mount();
translate([0,10]) scale([1,-1,1]) encoder_mount();
