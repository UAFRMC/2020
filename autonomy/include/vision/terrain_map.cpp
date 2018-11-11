//Calculates an optimal path around obstacles
//Written by Bradley Morton, Jim Samson, George Meier, and Ian Ferguson
//Started 11/10/18 (public domain)



struct x_y_coord
{
x_y_coord() {x=y=0}
int x;
int y;
};;

void terrainMap(vector<grid_square> & terrain)
{

	//This part of the code marks parts of the terrain that
	//do not have measurements. This is referred to as shadows
	//because they are in the same places that shadows would 
	//be cast if the sensor was acting as a light source. 
	//The basic idea of our terrain tracking system is that 
	//the obstacles we care about, rocks and craters, will each
	//have shadows in the measurements. First, our goal is to
	//mark these grid_squares. 

	int minForShadow=0; //To potentially be changed upon experimentation
	for(int i=0; i<terrain.size(); ++i)
	{
		if (terrain[i].getCount()<=minForShadow)
		{
			terrain[i].inShadow=true;
		}
	}



	//Next, our goal is to determine if the shadow is being cast 
	//by a rock or a crater. Grid_squares have a mean height
	//characteristic that will be used to make the determination.
	//First, groupings of grid_squares with the shadow flags 
	//set to true are generated. Next, a number of height 
	//observations will be taken 

	for(int i=0; i<terrain.size(); ++i)
	{
		//try horizontal
		//square to right is flagged and unincluded
		//try vertical
		//square below is flagged and unincluded
	}





	//Next, now that we have an idea of groupings of where shadows
	//are, the goal is to look above and below the 
}




