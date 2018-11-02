/**
 Stores depth samples and computes their statistics, for navigation purposes.
 
 Units are centimeters.
 
 Bradley Morton & Dr. Orion Lawlor, 2018-11-01 (public domain)
*/
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
  void clear();
  void addPoint(float z);
  int getCount() { return count; }
  float getMean();
  float getVariance();
  float getMax();
  float getMin();
};

int compare(grid_square a, grid_square b);




#endif
