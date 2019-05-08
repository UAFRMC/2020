/**
 Stores depth samples and computes their statistics, for navigation purposes.
 
 Units are centimeters.
 
 Bradley Morton & Dr. Orion Lawlor, 2018-11-01 (public domain)
*/
#ifndef GRIDHPP
#define GRIDHPP
#include <fstream>
#include <vector>
#include "../aurora/vec3.h"
#include "../../firmware/field_geometry.h"

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




/// Make it easy to swap between float (fast for big arrays) and double
typedef float real_t;

/// Keeps track of location of obstacles
class obstacle_grid {
public:
  enum {GRIDSIZE=4}; // cm per grid cell
  enum {GRIDX=(30+field_x_size+GRIDSIZE-1)/GRIDSIZE}; // xy grid cells for field
  enum {GRIDY=(30+field_y_size+GRIDSIZE-1)/GRIDSIZE};
  enum {GRIDTOTAL=GRIDX*GRIDY}; // total grid cells

  /* Raster pattern of GRIDX * GRIDY cells,
     where we accumulate depth data. */
  std::vector<grid_square> grid;
  grid_square &at(int x,int y) { return grid[y*obstacle_grid::GRIDX + x]; }
  const grid_square &at(int x,int y) const { return grid[y*obstacle_grid::GRIDX + x]; }
  
  obstacle_grid() 
    :grid(obstacle_grid::GRIDTOTAL)
  {
  }
  
  /* Return true if this point has data (in range, and count >0) */
  bool in_bounds(int x,int y) const {
    if (x<0 || x>=GRIDX || y<0 || y>=GRIDY) return false;
    return true;
  }

  /* Flush all stored points */
  void clear(void) {
    for (size_t i=0;i<grid.size();i++) grid[i]=grid_square();
  }

  /* Add this point to our grid */ 
  void add(vec3 world) {
    unsigned int x=world.x*(1.0/obstacle_grid::GRIDSIZE);
    unsigned int y=world.y*(1.0/obstacle_grid::GRIDSIZE);
    if (x<obstacle_grid::GRIDX && y<obstacle_grid::GRIDY)
    {
      at(x,y).addPoint(world.z);
    }
  }
  
#ifdef OPENCV_CORE_HPP
  /* Get a top-down debug image.
     Scale the image up by depthscale */
  cv::Mat get_debug_2D(int depthscale) const
  {
    int nw=obstacle_grid::GRIDX*depthscale;
    int nh=obstacle_grid::GRIDY*depthscale;
    cv::Mat world_depth(cv::Size(nw,nh),
      CV_8UC3, cv::Scalar(0,0,0));
    for (int h = 0; h < obstacle_grid::GRIDY; h++)
    for (int w = 0; w < obstacle_grid::GRIDX; w++)
    {
      const grid_square &g=at(w,h);
      for (int dy=0; dy<depthscale;dy++)
      for (int dx=0; dx<depthscale;dx++)
      {
        int x=w*depthscale+dx;
        int y=h*depthscale+dy;
        if (g.getCount()>0) {
          cv::Vec3b color(50+g.getMin(), 50+g.getTrimmedMean(), 50+g.getMax());
          
          world_depth.at<cv::Vec3b>(nh-1-y,x)=color;
        }
      }
    }

    return world_depth;
  }
#endif

  /* Write this obstacle grid to this base filename */
  void write(std::string filename) const
  {

#ifdef OPENCV_CORE_HPP
    imwrite((filename+".png").c_str(),get_debug_2D(1));
#endif
    
    FILE *f=fopen((filename+".bin").c_str(),"wb");
    fwrite(&grid[0],obstacle_grid::GRIDY*obstacle_grid::GRIDX,sizeof(grid[0]),f);
    fclose(f);
  }


  /* Read this obstacle grid from this base filename */
  void read(std::string filename)
  {
    FILE *f=fopen((filename+".bin").c_str(),"rb");
    if (!fread(&grid[0],obstacle_grid::GRIDY*obstacle_grid::GRIDX,sizeof(grid[0]),f))
      printf("Error doing read from %s\n",filename.c_str());
    fclose(f);
  }
};


#endif
