#include "grid.hpp"

#include <cmath>

grid_square::grid_square()
{
	count=0;
	max=0;
	min=0;
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
		min=z
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
	if(std::abs(a.getMean() - b.getMean())>3)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}




#include <vector>


