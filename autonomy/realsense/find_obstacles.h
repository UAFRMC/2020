#include <opencv2/opencv.hpp>  
#include "vision/grid.hpp"
#include "aurora/beacon_commands.h"

/*
  Turn this grid of depth data into a discrete list of obstacle locations.
*/
void find_obstacles(const obstacle_grid & obstacles,
  std::vector<aurora_detected_obstacle> &obstacle_list)
{
  cv::Mat debug=obstacles.get_debug_2D(1);

  // Loop over all points in the grid
  int nbor=2;
  for (int y = nbor; y < obstacle_grid::GRIDY-nbor; y++)
  for (int x = nbor; x < obstacle_grid::GRIDX-nbor; x++)
  {
    const grid_square &me=obstacles.at(x,y);
    cv::Vec3b debug_color(50+me.getTrimmedMean(),me.getCount()/100,0);
    
    // Compute statistics of neighbors
    grid_square neighborhood;
    for (int ny=-nbor;ny<=+nbor;ny++)
    for (int nx=-nbor;nx<=+nbor;nx++)
    if (nx!=0 || ny!=0) // this isn't me
    {
      const grid_square &they=obstacles.at(x+nx,y+ny);
      if (they.getCount()>=4 && they.getTrimmedMean()<60) 
      {
        neighborhood.addPoint(they.getTrimmedMean());
      }
    }
    
    if (neighborhood.getCount()>=(2*nbor)*(2*nbor)) 
    { // I have a reasonable number of neighbors
      float nearby = 0.5*neighborhood.getTrimmedMean()+0.5*neighborhood.getMin(); //< HACK!
      float height=me.getTrimmedMean()-nearby;
      if (height>=8.0 ) {
        aurora_detected_obstacle det;
        det.x=x*obstacle_grid::GRIDSIZE;
        det.y=y*obstacle_grid::GRIDSIZE;
        det.height=height;
        obstacle_list.push_back(det);
        
        printf("Obstacle at (%d,%d) cm, height %d cm\n",
          (int)det.x,(int)det.y,(int)height);
        debug_color=cv::Vec3b(0,0,255);
      }
    }
    
    debug.at<cv::Vec3b>(obstacle_grid::GRIDY-1-y,x)=debug_color;
  }
  const char *name="obstacles_detected.png";
  imwrite(name,debug);
  obstacles.write("obstacles_debug");
  printf("Wrote obstacle list debug image to '%s'.  Found \n",name);
}



