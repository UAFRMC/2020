use <roundlib.scad>;


// Holes to fit the 4-sided drive lug of a Barbie jeep gearbox:

module barbie_gearbox_drive(height=20.0,fatten=0.0,wiggle=0.3)
{
linear_extrude(convexity=4,height=height) {
    radius=16;
    cube_width=13;
    cube_diameter=50;
	round_2D(1.5)
	offset(r=wiggle+fatten)
	{
		circle(r=radius,center=true);
		square([cube_width,cube_diameter],center=true);
		square([cube_diameter,cube_width],center=true);
	}
}
}

