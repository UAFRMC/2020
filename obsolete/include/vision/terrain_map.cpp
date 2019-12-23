//Calculates an optimal path around obstacles
//Written by Bradley Morton, Jim Samson, George Meier, and Ian Ferguson
//Started 11/10/18 (public domain)

#include <vector>
#include "grid.hpp"
#include "terrain_map.hpp"

x_y_coord::x_y_coord()
{
	_x=_y=0;
}
x_y_coord::x_y_coord(int x, int y)
{
	_x=x;
	_y=y;
}


// bitwise grid flags we'll be using in this file
enum {
	atShadow = 1<<0,
	beenChecked = 1<<1,
	impassible = 1<<2,
};




void terrainMap(std::vector<grid_square> & terrain)
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
			terrain[i].setFlag(atShadow);
		}
	}



	//Next, our goal is to determine if the shadow is being cast 
	//by a rock or a crater. Grid_squares have a mean height
	//characteristic that will be used to make the determination.
	//First, groupings of grid_squares with the shadow flags 
	//set to true are generated. Next, a number of height 
	//observations will be taken 

	//"region growing"
	std::vector<std::vector<x_y_coord>> obstacles;
	for(int i=0; i<terrain.size(); ++i)
	{
		if(!terrain[i].getFlag(beenChecked)&&terrain[i].getFlag(atShadow))
		{	
			obstacles.push_back(std::vector<x_y_coord>());
			findGroupings(x_y_coord(i/429, i%429), terrain,  obstacles[obstacles.size()-1]);
		}
		terrain[i].setFlag(beenChecked);
	}





	//Next, now that we have an idea of groupings of where shadows
	//are, the goal is to look below and to the sides of the groupings
	//to find if the height of the areas differ. If the areas below 
	//have a higher height, the obstacle is a rock. If the height is 
	//the same, or close to the same, it is a hole. 
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	// for(int i=0; i<obstacles.size(); ++i)
	// {
	// 	for(int i=0; i<obstacles.size(); ++i)
	// 	{
	// 		for(int j=0; j<obstacles[i].size; ++j)
	// 		{
	// 			terrain[obstacles[i][j]].setFlag(impassable);
	// 		}
	// 	}
	// }





}



//Recursive backtracking function to find groupings
//Given a starting x, y coordinate this function finds 
//adjacent shadowed cells to create a grouping.

void findGroupings(x_y_coord start, std::vector<grid_square> & terrain, std::vector<x_y_coord> & grouping)
{
	if(terrain[getPos(start._x, start._y)].getFlag(beenChecked))
	{
		return;
	}
	terrain[getPos(start._x, start._y)].setFlag(beenChecked);
	if (terrain[getPos(start._x+1, start._y)].getFlag(atShadow))
	{
		grouping.push_back(x_y_coord(start._x+1, start._y));
		findGroupings(x_y_coord(start._x+1, start._y), terrain, grouping);
	}

	if (terrain[getPos(start._x, start._y-1)].getFlag(atShadow))
	{
		grouping.push_back(x_y_coord(start._x, start._y-1));
		findGroupings(x_y_coord(start._x, start._y-1), terrain, grouping);
	}

	if (terrain[getPos(start._x-1, start._y)].getFlag(atShadow))
	{
		grouping.push_back(x_y_coord(start._x-1, start._y));
		findGroupings(x_y_coord(start._x-1, start._y), terrain, grouping);
	}

	if (terrain[getPos(start._x, start._y+1)].getFlag(atShadow))
	{
		grouping.push_back(x_y_coord(start._x, start._y+1));
		findGroupings(x_y_coord(start._x, start._y+1), terrain, grouping);
	}
}


int getPos(int x, int y)
{
	return x+y*429;
}




