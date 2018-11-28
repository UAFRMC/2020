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
  int flags;
public:
  grid_square();

  void clear();

  void addPoint(float z);

  int getCount() const { return count; }
  float getMean() const;
  float getTrimmedMean() const;
  float getVariance() const;
  float getMax() const { return max; }
  float getMin() const { return min; }

  void setFlag(int the_flag) { flags |= the_flag; }
  bool getFlag(int the_flag) const { return the_flag & flags; }
};

int compare(grid_square a, grid_square b);




#endif
