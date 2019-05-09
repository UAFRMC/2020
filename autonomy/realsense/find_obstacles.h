/**
  Use positive and negative deviations in point count 
  ("humps" and "shadows") to estimate obstacle locations.
*/
#include <opencv2/opencv.hpp>  
#include "vision/grid.hpp"
#include "aurora/beacon_commands.h"

struct point {
public:
  int x,y;
  point(int x_=0,int y_=0) :x(x_), y(y_) {}
};

cv::Mat hits;
void mark_hit(const point &p,   int r,int g,int b) 
{
  hits.at<cv::Vec3b>(obstacle_grid::GRIDY-1-p.y,p.x)=cv::Vec3b(b,g,r);
}

class polar_dir {
public:
  float dir_x,dir_y;
  polar_dir(float angle) {
    dir_x=cos(angle), dir_y=sin(angle);
  }
  
  point at(float range) const {
    return point(
        (int)((field_x_beacon+range*dir_x)*(1.0/obstacle_grid::GRIDSIZE)),
        (int)((field_y_beacon+range*dir_y)*(1.0/obstacle_grid::GRIDSIZE))
      );
  }
};

bool is_empty(const obstacle_grid & obstacles,const point &p,int empty_threshold=0) {
  if (!obstacles.in_bounds(p.x,p.y)) return false;
  return obstacles.at(p.x,p.y).getCount()<=empty_threshold;
}

/*
  Turn this grid of depth data into a discrete list of obstacle locations.
*/
void find_obstacles(const obstacle_grid & obstacles,
  std::vector<aurora_detected_obstacle> &obstacle_list)
{
  int w=obstacle_grid::GRIDX;
  int h=obstacle_grid::GRIDY;
  hits=obstacles.get_debug_2D(1);
  imwrite("00_topdown.png",hits);
  for (int y = 0; y < h; y++)
  for (int x = 0; x < w; x++)
  {
     //const grid_square &me=obstacles.at(x,y);
     mark_hit(point(x,y), 0,0,0); // me.getCount()/32,0);
  }
  
  // You need this many counts to be valid
  int valid_min=10;
  
  // Accumulate getCounts for nearby pixels
  std::vector<grid_square> range_counts; // nearby getCounts per image pixel
  range_counts.resize(w*h);
  std::vector<grid_square> range_heights; // nearby heights per image pixel
  range_heights.resize(w*h);
  int nbors=10;
  for (int y = 0; y < h; y++)
  for (int x = 0; x < w; x++) {
    grid_square heights;
    grid_square counts;
    int nbors_donut=nbors/2; // 'donut hole' in middle of kernel
    for (int dy=-nbors;dy<=nbors;dy++)
    for (int dx=-nbors;dx<=nbors;dx++)
    if (obstacles.in_bounds(x+dx,y+dy)) 
    {
      int r2=dx*dx+dy*dy; // radius squared: inside to outside
      if (r2<nbors*nbors && r2>nbors_donut*nbors_donut)
      {
        const grid_square &here=obstacles.at(x+dx,y+dy);
        int count=here.getCount();
        if (count>=valid_min)
        {
          counts.addPoint(count);
          heights.addPoint(here.getTrimmedMean()); 
        }
      }
    }
    range_counts[y*w+x]=counts;
    range_heights[y*w+x]=heights;
  }
  
  // Find pixels that are substantial deviations from their neighorhood counts
  for (int y = 0; y < h; y++)
  for (int x = 0; x < w; x++) {
    int my=obstacles.at(x,y).getCount();
    if (my>=valid_min) {
      point p(x,y);
      const grid_square &us=range_counts[y*w+x];
      float stdev=sqrt(us.getVariance());
      float diff = (my-us.getTrimmedMean())/stdev;
      float thresh=0.3;
      float scaleRed=200, scaleBlue=300; // standard deviations to 0-255 data numbers
      if (diff>thresh) 
      { // we have an above-average count
        mark_hit(p, std::min(255,(int)((diff-thresh)*scaleRed)),0,0);
      }
      if (diff<-thresh) 
      { // we have a below-average count
        mark_hit(p, 0,0,std::min(255,(int)((-diff-thresh)*scaleBlue)));
      }
    }
  }
  
  imwrite("01_hits.png",hits);
  
  /*
  cv::Mat hits_eroded;
  cv::erode(hits,hits_eroded,cv::getStructuringElement(cv::MORPH_CROSS,cv::Size(3,3)));
  imwrite("02_hits_dilate.png",hits_eroded);
  */
  
  cv::Mat hits_blurred;
  for (int rep=0;rep<2;rep++) {
    cv::blur(hits,hits_blurred,cv::Size(3,3));
    std::swap(hits,hits_blurred);
  }
  imwrite("03_hits_blur.png",hits);
  
  int thresh=45; // brightness in each blurred channel
  int nbor_frac=nbors*nbors*2; // required extant neighboring pixels
  int min_height=3;
  
  for (int y = 0; y < h; y++)
  for (int x = 0; x < w; x++)
  {
    bool is_obstacle=false;
    auto &me=obstacles.at(x,y);
    float my_height=me.getTrimmedMean();
    float neighbor_heights=range_heights[y*w+x].getTrimmedMean();
    float height=my_height-neighbor_heights;

    cv::Vec3b pixel=hits.at<cv::Vec3b>(obstacle_grid::GRIDY-1-y,x);
    if (pixel[0]>thresh && pixel[2]>thresh 
       && range_counts[y*w+x].getCount()>nbor_frac
     ) 
    { // a shadow-detected obstacle
      if (height>=min_height) {
	is_obstacle=true;
        mark_hit(point(x,y), 0,255,height);        
      }
    }
    if (!is_obstacle && me.getCount()>100 && height>=10 && height<=60) 
    { // a height-detected obstacle
      mark_hit(point(x,y), 255,255,height);        
      is_obstacle=true;
    }
    if (is_obstacle) {
        aurora_detected_obstacle det;
        det.x=x*obstacle_grid::GRIDSIZE;
        det.y=y*obstacle_grid::GRIDSIZE;
        det.height=height;
        obstacle_list.push_back(det);
        
        printf("Obstacle at (%d,%d) cm, height %d cm\n",
          (int)det.x,(int)det.y,(int)height);
    }
  }


  imwrite("04_marked.png",hits);
  
  
  printf("Wrote debug image to hits.png");
}



