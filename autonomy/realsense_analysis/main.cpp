/* 
  Do data analysis on realsense dataset. 
*/
#include <opencv2/opencv.hpp>  
#include "vision/grid.hpp"
#include "vision/grid.cpp"

int main(int argc,char *argv[]) {

  // Figure out which grid filename to read
  std::string name="input";
  if (argc>1) name=argv[1];
  
  // Read the grid
  obstacle_grid obstacles;
  obstacles.read(name);
  cv::Mat debug=obstacles.get_debug_2D(1);

  // Loop over all points in the grid
  int nbor=2;
  for (int y = nbor; y < obstacle_grid::GRIDY-nbor; y++)
  for (int x = nbor; x < obstacle_grid::GRIDX-nbor; x++)
  {
    grid_square &me=obstacles.at(x,y);
    cv::Vec3b debug_color(50+me.getTrimmedMean(),me.getCount()/10,0);
    
    // Compute statistics of neighbors
    grid_square neighborhood;
    for (int ny=-nbor;ny<=+nbor;ny++)
    for (int nx=-nbor;nx<=+nbor;nx++)
    if (nx!=0 || ny!=0) // this isn't me
    {
      const grid_square &they=obstacles.at(x+nx,y+ny);
      if (they.getCount()>=4 && they.getTrimmedMean()<40) 
      {
        neighborhood.addPoint(they.getTrimmedMean());
      }
    }
    
    if (neighborhood.getCount()>=(2*nbor)*(2*nbor)) 
    { // I have a reasonable number of neighbors
      float nearby = neighborhood.getMin(); //< HACK!
      float height=me.getTrimmedMean()-nearby;
      if (height>=6.0 || height<=-8.0) {
        printf("Obstacle at (%d,%d) cm, height %.0f cm\n",
          x*obstacle_grid::GRIDSIZE,
          y*obstacle_grid::GRIDSIZE,
          height);
        debug_color=cv::Vec3b(0,0,255);
      }
    }
    
    debug.at<cv::Vec3b>(obstacle_grid::GRIDY-1-y,x)=debug_color;
  }
  imwrite(name+"_debug.png",debug);
  
  return 0;
}

