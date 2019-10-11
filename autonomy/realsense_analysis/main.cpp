/* 
  Do data analysis on realsense dataset. 
*/
#include <opencv2/opencv.hpp>  
#include "vision/grid.hpp"
#include "vision/grid.cpp"
#include "find_obstacles.h"

int main(int argc,char *argv[]) {

  // Figure out which grid filename to read
  std::string name="obstacles_debug.bin";
  if (argc>1) name=argv[1];
  printf("Grid size: %d x %d = %d cells\n",
    (int)obstacle_grid::GRIDX,
    (int)obstacle_grid::GRIDY,
    (int)obstacle_grid::GRIDTOTAL);
  
  // Read the grid
  obstacle_grid obstacles;
  obstacles.read(name);
  
  std::vector<aurora_detected_obstacle> list;
  find_obstacles(obstacles,list);
  
  return 0;
}

