#include "grid.hpp"

#include <cmath>

grid_square::grid_square()
{
  clear();
}
void grid_square::clear() 
{
	count=0;
	max=-1000;
	min=1000;
	sum=0;
	sumSquares=0;
}

void grid_square::addPoint(float z)
{
	++count;
	sum+=z;
	sumSquares+=z*z;
	if(z<min)
	{
		min=z;
	}
	if(max<z)
	{
		max=z;
	}
}
float grid_square::getMean()
{
	float total=count;
	return sum/count;
}
float grid_square::getVariance()
{
	float total=count;
	return sumSquares/count-getMean()*getMean();
}
float grid_square::getMax()
{
	return max;
}
float grid_square::getMin()
{
	return min;
}

int compare(grid_square a, grid_square b)
{
	if(std::abs(a.getMean() - b.getMean())>5)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}




#include <vector>


