/**
  Read Realsense depth and color frames.
*/
#ifndef __VISION_REALSENSE_H
#define __VISION_REALSENSE_H

#include <librealsense2/rs.hpp>  
#include <opencv2/opencv.hpp>  
#include "../aurora/coords.h"  // for vec3 and robot_coord3D


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
  vec3 lookup(float depth,int x,int y) const
  {
    int i=y*intrinsics.width + x;
    return vec3(xdir[i]*depth, ydir[i]*depth, depth);
  }
};


/**
 Manages communication with realsense camera.
*/
class realsense_camera  {
public:
    realsense_camera(
        int res=720, // resolution, 720p is high res, 480p is medium res, 240p low res
        int fps=30 // framerate, 6fps is USB 2.0 compatible
    );
    ~realsense_camera();
    
private:
    realsense_camera(const realsense_camera &no_copies);
    void operator=(const realsense_camera &no_copies);
    
    // These are set up in the constructor
    rs2::pipeline pipe;  
    rs2::config cfg;  
    
    // These are cached during a capture
    realsense_projector *depth_projector;
    float depth2cm;
    friend class realsense_camera_capture;
};


/* One moment's data extracted from the camera */
class realsense_camera_capture {
public:
    // Calling this constructor captures one frame of image data from the camera.
    realsense_camera_capture(realsense_camera &cam);
    
private:
    // This is the librealsense handle to the frame's allocated memory.
    rs2::frameset frames;  
    rs2::video_frame color_frame;
    rs2::depth_frame depth_frame;
    
public: // Actual captured data (read/write)
    
    // Dimensions of the images, in pixels
    int color_w,color_h;
    int depth_w,depth_h;
    
    // OpenCV copies of the image data.  These are writeable.
    cv::Mat color_image;
    
    typedef unsigned short depth_t;
    depth_t *depth_data;
    cv::Mat depth_image;

    // This is cached inside the camera, to avoid reallocation
    const realsense_projector *depth_projector;
    float depth2cm;
    
    // Return the last-grabbed depth at this (x,y) pixel
    //   The value may be zero, indicating invalid depth data there.
    float get_depth(int x,int y) const {
        return depth2cm*depth_data[x+y*depth_w];
    }
    
    // Extract camera-coordinates (Y down, Z out) 3D location of this depth value
    vec3 project_3D(float d,int x,int y) const {
        return depth_projector->lookup(d,x,y);
    }
};




#endif




