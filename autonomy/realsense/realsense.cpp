/**
  Compute 3D points from RealSense data.
  
  This needs librealsense2, from https://github.com/IntelRealSense/librealsense
  
  Dr. Orion Lawlor, lawlor@alaska.edu (public domain)
  Modified from tiny example by BjarneG at https://communities.intel.com/thread/121826
*/
#include <librealsense2/rs.hpp>  
#include <opencv2/opencv.hpp>  
#include "vision/grid.hpp"
#include "vision/grid.cpp"

  
using namespace std;  
using namespace cv;  

/// Make it easy to swap between float (fast for big arrays) and double
typedef float real_t;

/// Simple 3D vector
class vec3 {
public:
  real_t x;
  real_t y;
  real_t z;
  
  vec3(real_t X=0.0, real_t Y=0.0, real_t Z=0.0) {
    x=X; y=Y; z=Z;
  }
};

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
    :camera(0.0,0.0,75.0),  // camera position
     camera_tilt(-20), // X axis rotation (camera mounting tilt)
     Z_rotation(camera_Z_angle) // Z axis rotation
  {
  }
  
  // Project this camera-relative 3D point into world coordinates
  vec3 world_from_camera(vec3 point) const {
    real_t x=point.x, y=point.z, z=-point.y;
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

  
/// Keeps track of location
class obstacle_grid {
public:
  enum {GRIDSIZE=4}; // cm per grid cell
  enum {GRIDX=(50+807+GRIDSIZE-1)/GRIDSIZE}; // xy grid cells for field
  enum {GRIDY=(50+369+GRIDSIZE-1)/GRIDSIZE};
  enum {GRIDTOTAL=GRIDX*GRIDY}; // total grid cells
  
  obstacle_grid() {
    
  }

  // Detected obstacles (mostly for display as a GUI)
  cv::Mat obstacles;
};


int main()  
{  
    rs2::pipeline pipe;  
    rs2::config cfg;  
  
    bool bigmode=true;

    int fps=6;
    // fps=30; // USB 3.0 only
    int depth_w=1280, depth_h=720; // high res mode: definitely more detail visible
    int color_w=1280, color_h=720; 
    if (!bigmode) { // low res
      fps=15;
      depth_w=480; depth_h=270;
      color_w=424; color_h=240;
    }

    cfg.enable_stream(RS2_STREAM_DEPTH, depth_w,depth_h, RS2_FORMAT_Z16, fps);  
    cfg.enable_stream(RS2_STREAM_COLOR, color_w,color_h, RS2_FORMAT_BGR8, fps);  
  
    rs2::pipeline_profile selection = pipe.start(cfg);  

    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    float scale =  sensor.get_depth_scale();
    printf("Depth scale: %.3f\n",scale);
    double depth2cm = scale * 100.0; 
    double depth2screen=255.0*scale/4.5;
    
    int framecount=0;
    int nextwrite=1;

    camera_transform camera_TF;
    std::vector<grid_square> grid(obstacle_grid::GRIDTOTAL);
    
    rs2::frameset frames;  
    while (true)  
    {  
        // Wait for a coherent pair of frames: depth and color  
        frames = pipe.wait_for_frames();  
        rs2::depth_frame depth_frame = frames.get_depth_frame();  
        rs2::video_frame color_frame = frames.get_color_frame();  
        framecount++;
  
        if ((depth_w != depth_frame.get_width()) ||
          (depth_h != depth_frame.get_height()) || 
          (color_w != color_frame.get_width()) ||
          (color_h != color_frame.get_height()))
        {
          std::cerr<<"Realsense capture size mismatch!\n";
          exit(1);
        }
  
        typedef unsigned short depth_t;
        depth_t *depth_data = (depth_t*)depth_frame.get_data();  
        void *color_data = (void*)color_frame.get_data();  
        
        // Make OpenCV versions of raw pixels:
        //Mat depth_raw(Size(depth_w, depth_h), CV_16U, depth_data, Mat::AUTO_STEP);  
        //Mat color(Size(color_w, color_h), CV_8UC3, color_data, Mat::AUTO_STEP);  
        
        // Display raw data onscreen
        //imshow("Depth", depth_raw);
        //imshow("RGB", color);
        
        // Set up *static* 3D so we don't need to recompute xdir and ydir every frame.
        static realsense_projector depth_to_3D(depth_frame);
        
        Mat debug_image(Size(depth_w, depth_h), CV_8UC3, cv::Scalar(0));
        
        for (size_t i=0;i<grid.size();i++) grid[i]=grid_square();
        
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
            
            if (world.y>0.0 && world.y<200.0) { // green Y stripe
              const cv::Vec3b green(0,255,0);
              debug_color=green;
            }
            if (world.x>0.0 && world.x<20.0) { // red X stripe
              const cv::Vec3b red(0,0,255);
              debug_color=red;
            }
            
            if (world.z<5.0 && world.z>-30.0) { // blue Z stripe
              const cv::Vec3b blue(255,0,0);
              debug_color=blue;
            }
            
            if (world.z<150.0 && world.z>-50.0)
            {
              unsigned int x=world.x*(1.0/obstacle_grid::GRIDSIZE)+obstacle_grid::GRIDX/2;
              unsigned int y=world.y*(1.0/obstacle_grid::GRIDSIZE);
              if (x<obstacle_grid::GRIDX && y<obstacle_grid::GRIDY)
              {
                grid[y*obstacle_grid::GRIDX + x].addPoint(world.z);
              }
            }
            
          }
          debug_image.at<cv::Vec3b>(y,x)=debug_color;
        }   
        //imshow("Depth image",debug_image);
        
        enum {depthscale=6};
        cv::Mat world_depth(cv::Size(obstacle_grid::GRIDX*depthscale,obstacle_grid::GRIDY*depthscale), 
          CV_8UC3, cv::Scalar(0,0,0));
        for (int h = 0; h < obstacle_grid::GRIDY; h++)
        for (int w = 0; w < obstacle_grid::GRIDX; w++)
        {
          grid_square &g=grid[h*obstacle_grid::GRIDX + w];
          for (int dy=0; dy<depthscale;dy++)
          for (int dx=0; dx<depthscale;dx++)
          {
            int x=w*depthscale+dx;
            int y=h*depthscale+dy;
            if (g.getCount()>0) {
              cv::Vec3b color(50+g.getMin(), 50+g.getMean(), 50+g.getMax());
              
              world_depth.at<cv::Vec3b>(y,x)=color;
            }
          }
        }
        imshow("2D World",world_depth);
        
        
        int k = waitKey(10);  
        if (k == 27 || k=='q')  
            break;  
    }  
  
  
    return 0;  
}  
