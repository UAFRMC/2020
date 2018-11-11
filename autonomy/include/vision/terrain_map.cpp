//Calculates an optimal path around obstacles
//Written by Bradley Morton, Jim Samson, George Meier, and Ian Ferguson
//Started 11/10/18 (public domain)



void terrainMap(vector<grid_square> & terrain)
{

	//This part of the code marks parts of the terrain that
	//do not have measurements. This is referred to as shadows
	//because they are in the same places that shadows would 
	//be cast if the sensor was acting as a light source. 
	//The basic idea of our terrain tracking system is that 
	//the obstacles we care about, rocks and craters, will each
	//have shadows in the measurements. First, our goal is to
	//mark all of them as flags. 
	int minForShadow;
	for(int i=0; i<terrain.size(); ++i)
	{
		if (terrain[i].getCount()<=minForShadow)
		{
			terrain[i].inShadow=true;
		}
	}



	//Next, our goal is to determine if the shadow is being cast 
	//by a rock or a crater. Grid_squares have a mean height
	//characteristic that 
}




