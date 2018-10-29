#ifndef GRIDHPP
#define GRIDHPP

class grid_square
{
private:
float max;
float min;
float sum;
float sumSquares;
int count;
public:
grid_square();
void addPoint(float z);
float getMean();
float getVariance();
float getMax();
float getMin();
};

int compare(grid_square a, grid_square b);




#endif
