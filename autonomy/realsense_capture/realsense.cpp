/**
  Compute 3D points from RealSense data.
  
  This needs librealsense2, from https://github.com/IntelRealSense/librealsense
  
  Dr. Orion Lawlor, lawlor@alaska.edu (public domain)
  Modified from tiny example by BjarneG at https://communities.intel.com/thread/121826
*/
#include <librealsense2/rs.hpp>  
#include <opencv2/opencv.hpp>  

#include "cvmat_binio.h"

#include "vision/grid.hpp"
#include "vision/grid.cpp"
#include "vision/terrain_map.cpp"
#include "../firmware/field_geometry.h"

using namespace std;  
using namespace cv;  

int show_GUI=0; // show debug window onscreen: 0 none; 1 basic; 2 lots

int sys_error=0;

/// Rotate coordinates using right hand rule
class coord_rotator {
public:
  const real_t angle; // rotation angle in radians
  const real_t c,s; // cosine and sine of rotation angle
  coord_rotator(real_t angle_degs=0.0) 
    :angle(angle_degs*M_PI/180.0), c(cos(angle)), s(sin(angle)) 
  { }
  
  inline void rotate(real_t &x,real_t &y) const {
    real_t new_x = x*c - y*s;
    real_t new_y = x*s + y*c;
    x=new_x; y=new_y;
  }
};

/// Transforms 3D points from depth camera coords to world coords,
///  by rotating and translating
class camera_transform {
public:  
  vec3 camera; // world-coordinates camera origin position (cm)
  coord_rotator camera_tilt; // tilt down
  coord_rotator Z_rotation; // camera panning
  
  camera_transform(real_t camera_Z_angle=0.0)
    :
    camera(170,30,70.0), 
    // camera(field_x_beacon,field_y_beacon,70.0),  // camera position
     camera_tilt(-20), // X axis rotation (camera mounting tilt)
     Z_rotation(camera_Z_angle-90) // Z axis rotation
  {
  }
  
  // Project this camera-relative 3D point into world coordinates.
  vec3 world_from_camera(vec3 point) const {
    // Point: Z is away from camera, Y is up
    real_t x=point.x, y=point.z, z=-point.y;
    // Y is now away from camera, Z is up
    
    camera_tilt.rotate(y,z); // tilt up, so camera is level
    Z_rotation.rotate(x,y); // rotate, to align with field
    x+=camera.x;
    y+=camera.y;
    z+=camera.z; 
    return vec3(x,y,z);
  }
};

/* Transforms raw realsense 2D + depth pixels into 3D:
  Camera X is along sensor's long axis, facing right from sensor point of view
  Camera Y is facing down
  Camera Z is positive into the frame
*/
class realsense_projector {
public:
  // Camera calibration
  rs2_intrinsics intrinsics;
  
  // Cached per-pixel direction vectors: scale by the depth to get to 3D
  std::vector<float> xdir;
  std::vector<float> ydir;
  
  realsense_projector(const rs2::depth_frame &frame)
    :xdir(frame.get_width()*frame.get_height()),
     ydir(frame.get_width()*frame.get_height())
  {
    auto stream_profile = frame.get_profile();
    auto video = stream_profile.as<rs2::video_stream_profile>();
    intrinsics = video.get_intrinsics();
    
    // Precompute per-pixel direction vectors (with distortion)
    for (int h = 0; h < intrinsics.height; ++h)
    for (int w = 0; w < intrinsics.width; ++w)
    {
      const float pixel[] = { (float)w, (float)h };

      float x = (pixel[0] - intrinsics.ppx) / intrinsics.fx;
      float y = (pixel[1] - intrinsics.ppy) / intrinsics.fy;

      if (intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
      {
          float r2 = x * x + y * y;
          float f = 1 + intrinsics.coeffs[0] * r2 + intrinsics.coeffs[1] * r2*r2 + intrinsics.coeffs[4] * r2*r2*r2;
          float ux = x * f + 2 * intrinsics.coeffs[2] * x*y + intrinsics.coeffs[3] * (r2 + 2 * x*x);
          float uy = y * f + 2 * intrinsics.coeffs[3] * x*y + intrinsics.coeffs[2] * (r2 + 2 * y*y);
          x = ux;
          y = uy;
      }

      xdir[h*intrinsics.width + w] = x;
      ydir[h*intrinsics.width + w] = y;
    }
  }
  
  // Project this depth at this pixel into 3D camera coordinates
  vec3 lookup(float depth,int x,int y) 
  {
    int i=y*intrinsics.width + x;
    return vec3(xdir[i]*depth, ydir[i]*depth, depth);
  }
};



int main(int argc,const char *argv[])  
{  
    rs2::pipeline pipe;  
    rs2::config cfg;  
  
    bool bigmode=true; // high res 720p input
    bool do_depth=true; // auto-read depth frames, parse into grid
    bool do_color=true; // read color frames, look for vision markers
    int fps=6; // framerate (USB 2.0 compatible by default)
    
    for (int argi=1;argi<argc;argi++) {
      std::string arg=argv[argi];
      if (arg=="--gui") show_GUI++;
      else if (arg=="--depth") do_depth=true;
      else if (arg=="--nodepth") do_depth=false;
      else if (arg=="--nocolor") do_color=false;
      else if (arg=="--coarse") bigmode=false; // lowres mode
      else if (arg=="--fast") fps=30; // USB-3 only
      else {
        std::cerr<<"Unknown argument '"<<arg<<"'.  Exiting.\n";
        return 1;
      }
    }
    
    int depth_w=1280, depth_h=720; // high res mode: definitely more detail visible
    int color_w=1280, color_h=720; 
    if (!bigmode) { // low res
      if (fps<10) fps=15;
      depth_w=480; depth_h=270;
      color_w=640; color_h=480; //color_w=424; color_h=240;
    }

    printf("Connecting to RealSense device...\n");
    cfg.enable_stream(RS2_STREAM_DEPTH, depth_w,depth_h, RS2_FORMAT_Z16, fps);  
    cfg.enable_stream(RS2_STREAM_COLOR, color_w,color_h, RS2_FORMAT_BGR8, fps);  
  
    rs2::pipeline_profile selection = pipe.start(cfg);  

    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    float scale =  sensor.get_depth_scale();
    printf("RealSense connected.  Depth scale: %.3f\n",scale);
    double depth2cm = scale * 100.0; 
    double depth2screen=255.0*scale/4.5;
    
    int framecount=0;
    int writecount=0;
    int nextwrite=1;

    int obstacle_scan=0; // frames remaining for obstacle scan
    int obstacle_scan_target=-999; // angle at which we want to do the scan
    
    // Debug color image of depth data
    Mat depth_color(Size(depth_w, depth_h), CV_8UC3);
    
    rs2::frameset frames;  
    while (true)  
    {  
        camera_transform camera_TF(90);

        // Wait for a coherent pair of frames: depth and color
        frames = pipe.wait_for_frames();  
        rs2::video_frame color_frame = frames.get_color_frame();  
        rs2::depth_frame depth_frame = frames.get_depth_frame();  
        if ((depth_w != depth_frame.get_width()) ||
            (depth_h != depth_frame.get_height()) || 
            (color_w != color_frame.get_width()) ||
            (color_h != color_frame.get_height()))
        {
          std::cerr<<"Realsense capture size mismatch!\n";
          exit(1);
        }
        
        framecount++;
  
        void *color_data = (void*)color_frame.get_data();  
          
        // Make OpenCV version of raw pixels (no copy, so this is cheap)
        Mat color_image(Size(color_w, color_h), CV_8UC3, color_data, Mat::AUTO_STEP);  
        if (do_color) 
        {
          if (show_GUI) 
            imshow("Color Image",color_image);
        }
        
        typedef unsigned short depth_t;
        depth_t *depth_data = (depth_t*)depth_frame.get_data();
        Mat depth_raw(Size(depth_w, depth_h), CV_16U, depth_data, Mat::AUTO_STEP); 
        obstacle_grid obstacles;
        if (do_depth) 
        {
          // Display raw data onscreen
          
          if (show_GUI>=1) { 
            for (int y = 0; y < depth_h; y++)
            for (int x = 0; x < depth_w; x++) {
              int i=y*depth_w + x;
              int depth=depth_data[i]*depth2cm; // depth, in cm
              cv::Vec3b debug_color=cv::Vec3b((depth%32)*(255/31),(depth/32)*(256/20),depth%256);
              depth_color.at<cv::Vec3b>(y,x)=debug_color;
            }
            imshow("Depth", depth_color);
          }
          
          // Set up *static* 3D so we don't need to recompute xdir and ydir every frame.
          static realsense_projector depth_to_3D(depth_frame);

          // if (obstacle_scan>=16) obstacles.clear(); // clear the grid
          
          const int realsense_left_start=50; // invalid data left of here
          for (int y = 0; y < depth_h; y++)
          for (int x = realsense_left_start; x < depth_w; x++)
          {
            int i=y*depth_w + x;
            float depth=depth_data[i]*depth2cm; // depth, in cm
            
            int depth_color=depth*(255.0/400.0);
            cv::Vec3b debug_color;
            if (depth_color<=255) debug_color=cv::Vec3b(depth_color,depth_color,0);
            
            if (depth>0) {
              vec3 cam = depth_to_3D.lookup(depth,x,y);
              vec3 world = camera_TF.world_from_camera(cam);
              
              if (world.z<200.0 && world.z>-100.0)
              {
                obstacles.add(world);
              }

            }
          }   
          
          if (show_GUI) {
            cv::Mat world_depth=obstacles.get_debug_2D(6);
            imshow("2D World",world_depth);    
          }
        }    
        
        int k = waitKey(10);  
  if ((framecount>=30) || k == 'i') // image dump 
  {
    framecount=0;
    char filename[500];
    if (0!=system("mkdir -p vidcaps")) { 
      printf("Can't make output directory\n"); 
    }
    else {
      if (do_depth) {
        sprintf(filename,"vidcaps/%03d_depth.raw",(int)writecount);
        serializeMatbin(depth_raw,filename);
        
        sprintf(filename,"vidcaps/%03d_depth.jpg",(int)writecount);
        imwrite(filename,depth_color);
        
        sprintf(filename,"vidcaps/%03d_obstacles",(int)writecount);
        obstacles.write(filename);
        printf("Stored image to file %s\n",filename);
        obstacles.clear();
      }
      if (do_color) {
        imwrite("vidcaps/latest.jpg",color_image);
        sprintf(filename,"cp vidcaps/latest.jpg vidcaps/%03d_view.jpg",writecount);
        sys_error=system(filename);
      }
    }
    
    writecount++;
  }
        if (k == 27 || k=='q')  
            break;  
    }  
  
  
    return 0;  
}  
