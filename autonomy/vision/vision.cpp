/*
This is the computer vision system, reading color and depth data
from a realsense camera, and writing data used by the localizer.

From the color images, we extract aruco marker locations.

From the depth images, we extract drivable / non-drivable areas.
*/
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>   
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

#include "vision/realsense_camera.hpp"
#include "vision/realsense_camera.cpp"

#include "vision/aruco_detector.hpp"
#include "vision/aruco_detector.cpp"

// Obstacle detection
#include "vision/grid.hpp"
#include "vision/grid.cpp"


/* Fill out computer vision marker reports, based on observations from aruco */
class vision_marker_watcher {
public:
    aurora::vision_marker_reports reports;
    int index;
    
    // Return the number of valid markers seen
    int found_markers() { return index; }
    
    vision_marker_watcher() :index(0)
    {
    }
    
    void found_marker(const cv::Mat &matrix4x4,const aruco::Marker &marker,int ID)
    {
        printf("  MARKER %d: ",ID);
        
        double size=18.0; // physical size of marker, in centimeters
        if (ID==14) {  // small cat marker (debugging)
            size=14.5; 
        }
        else if (ID==13) {  // large mullet
            size=25; 
        } 
        else if (ID==6) {  // goblin
            size=18; 
        }
        else if (ID==7) {  // mask
            size=5; 
        }
        else {
            printf("Ignoring--unknown number %d\n",ID);
            return;
        }
        
        if (index>=aurora::vision_marker_report::max_count) {
            printf("Ignoring--too many reports already\n");
            return;
        }
        aurora::vision_marker_report &report=reports[index];
        index++;
        
        report.markerID=ID;
        report.coords.origin=size*extract_row(matrix4x4,3);
        
        // Swap Aruco Y-out Z-down, to camera coords Y-down Z-out
        report.coords.X=extract_row(matrix4x4,0);
        report.coords.Y=-extract_row(matrix4x4,2); 
        report.coords.Z=extract_row(matrix4x4,1);
        
        float min_area=120.0*120.0; // pixel area onscreen to reach 0% confidence
        report.coords.percent=90.0*(1.0-min_area/(min_area+marker.getArea()));
        
        report.coords.print();
    }
private:
    // Extract a row of this OpenCV 4x4 matrix, as a 3D vector
    vec3 extract_row(const cv::Mat &matrix4x4,int row) {
        return vec3(
            matrix4x4.at<float>(0,row),
            matrix4x4.at<float>(1,row),
            matrix4x4.at<float>(2,row)
        );
    }
};

/* Erode depth data: increase black space around missing data, for reliability*/
void erode_depth(realsense_camera_capture &cap,int erode_depth) {
    // erode the depth image here, to trim back depth sparkles
    cv::Mat depth_eroded(cv::Size(cap.depth_w, cap.depth_h), CV_16U);
    cv::Mat element=getStructuringElement(cv::MORPH_ELLIPSE,
        cv::Size(2*erode_depth+1, 2*erode_depth+1),
        cv::Point(erode_depth,erode_depth));
    cv::erode(cap.depth_image,depth_eroded,element);

    //depth_raw=depth_eroded; // swap back (easy, but systematic low-depth bias)

    // Zero out depths for all pixels that were eroded
    for (int y = 0; y < cap.depth_h; y++)
    for (int x = 0; x < cap.depth_w; x++) {
        if (0==depth_eroded.at<realsense_camera_capture::depth_t>(y,x))
            cap.depth_image.at<realsense_camera_capture::depth_t>(y,x)=0;
    }
}

/* Project current depth data onto 2D map */
void project_depth_to_2D(const realsense_camera_capture &cap,
    const aurora::robot_coord3D &view3D,
    obstacle_grid &map2D)
{
    printf("Camera view: "); view3D.print();
    const float depth_calibration_scale_factor=1.0f; // fudge factor to match real distances
    const float sanity_distance_min = 100.0; // mostly parts of robot if they're too close
    const float sanity_distance_max = 550.0; // depth gets ratty if it's too far out
    const float sanity_Z_max = 200.0; // ignore ceiling (with wide error band for tilt)
    const float sanity_Z_min = -100.0; // ignore invalid too-low
    const int realsense_left_start=30; // invalid data left of here
    for (int y = 0; y < cap.depth_h; y++)
    for (int x = realsense_left_start; x < cap.depth_w; x++)
    {
        float depth=cap.get_depth(x,y);
        if (depth<=sanity_distance_min || depth>sanity_distance_max) 
            continue; // out of range value
        
        depth *= depth_calibration_scale_factor;
        
        vec3 cam = cap.project_3D(depth,x,y);
        vec3 world = view3D.world_from_local(cam);
        
        if (world.z<sanity_Z_max && world.z>sanity_Z_min)
        {
            map2D.add(world);
        }
    }   
}

/* Mark grid cells as driveable or non-driveable */
void mark_obstacles(const obstacle_grid &map2D,aurora::field_drivable &field)
{ 
  cv::Vec3b slope(255,0,255); // big difference between max and min
  cv::Vec3b high(0,255,0); // too high to be drivable 
  cv::Vec3b low(255,0,0); // too low to be drivable 
  
  const int slope_thresh=8;
  const int high_thresh=40.0;
  const int low_thresh=-30.0;
  
  // Loop over all cells in the grid
  for (int y = 0; y < obstacle_grid::GRIDY; y++)
  for (int x = 0; x < obstacle_grid::GRIDX; x++)
  {
    const grid_square &me=map2D.at(x,y);
    if (me.getCount()<3) continue; // skip cells where we don't have data (common case)
    
    unsigned char mark=aurora::field_flat; // assume it's OK until proven bad
    if (me.getTrimmedMean() > high_thresh) {
      mark=aurora::field_toohigh;
    }
    if (me.getTrimmedMean() < low_thresh) {
      mark=aurora::field_toolow;
    }
    if (me.getMax() > slope_thresh+me.getMin()) {
      mark=aurora::field_sloped;
    }
    
    // Write this directly to the field
    field.at(x,y) = mark;
  }
}


int main(int argc,const char *argv[]) {
    int show_GUI=0;
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_marker_reports();
    MAKE_exchange_field_raw();
    MAKE_exchange_obstacle_view();

    // res=720; fps=30; // <- 200% of gaming laptop CPU
    // res=540; fps=60; // <- 220% of gaming laptop CPU
    // res=540; fps=30; // <- 100% of gaming laptop CPU
    
    int res=540; // camera's requested vertical resolution
    int fps=30; // camera's frames per second 
    bool aruco=true; // look for computer vision markers in RGB data
    bool obstacle=true; // look for obstacles/driveable areas in depth data
    int erode=3; // image erosion passes
    for (int argi=1;argi<argc;argi++) {
      std::string arg=argv[argi];
      if (arg=="--gui") show_GUI++;
      else if (arg=="--res") res=atoi(argv[++argi]); // manual resolution
      else if (arg=="--fps") fps=atoi(argv[++argi]); // manual framerate
      else if (arg=="--no-aruco") aruco=false; 
      else if (arg=="--no-obstacle") obstacle=false; 
      else if (arg=="--erode") erode=atoi(argv[++argi]);
      else if (arg=="--clear") 
      {
        unsigned char mark=aurora::field_unknown;
        aurora::field_drivable clearField;
        clearField.clear(mark);
        exchange_field_raw.write_begin() = clearField;
        exchange_field_raw.write_end();
      }
      else {
        std::cerr<<"Unknown argument '"<<arg<<"'.  Exiting.\n";
        return 1;
      }
    }

    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    
    printf("Connecting to realsense camera...\n");
    realsense_camera cam(res,fps);
    printf("Connected.\n");
    
    aruco_detector *detector=0;
    if (aruco) {
        detector = new aruco_detector();
    }

    while (true) {
        // Grab data from realsense
        realsense_camera_capture cap(cam);
        // If the two captures dont have the same data do not draw the obsticles.
        // Maybe solution is to iterate over the two realsense scene.
        // Helper script maybe define an way to compare in realsense.h
        // ex: cap.blend(last);

        
        // Run aruco marker detection on color image
        if (aruco)
        {
            vision_marker_watcher watcher;
            detector->find_markers(cap.color_image,watcher,show_GUI);
            if (watcher.found_markers()>0) { // only write if we actually saw something.
                exchange_marker_reports.write_begin()=watcher.reports;
                exchange_marker_reports.write_end();
            }
        }
        
        // Run obstacle detection on depth image
        if (obstacle) {
            if (erode) erode_depth(cap,erode);
            
            // Project to 2D map
            obstacle_grid map2D;
            aurora::robot_coord3D view3D = exchange_obstacle_view.read();
            project_depth_to_2D(cap,view3D,map2D);
            
            // Mark out the obstacles on the map
            aurora::field_drivable &newField = exchange_field_raw.write_begin();
            mark_obstacles(map2D,newField);
            exchange_field_raw.write_end();
            
            if (show_GUI) {
                cv::Mat debug=map2D.get_debug_2D(6);
                imshow("2D Map",debug);
            }
        }
        
        // Show debug GUI
        if (show_GUI) {
            imshow("Color image",cap.color_image);
            imshow("Depth image",8.0f*cap.depth_image); // scale up brightness
        }

        
        if (show_GUI) {
            int key = cv::waitKey(1);  
            if (key == 27 || key=='q')  
                break;  
        }
        // Store The previous copy of the data before grabbing new
        realsense_camera_capture last(cap);
    }
    return 0;
}
