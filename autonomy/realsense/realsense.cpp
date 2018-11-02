/**
  Compute drivable / non-driveable regions from RealSense data.

  Modified from tiny example by BjarneG at https://communities.intel.com/thread/121826
*/
#include <librealsense2/rs.hpp>  
#include <opencv2/opencv.hpp>  
#include "vision/grid.hpp"
#include "vision/grid.cpp"

  
using namespace std;  
using namespace cv;  

typedef float real_t;
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
  
  inline void rotate(real_t &x,real_t &y) {
    real_t new_x = x*c - y*s;
    real_t new_y = x*s + y*c;
    x=new_x; y=new_y;
  }
};

/// Transforms 3D points from depth camera coords to world coords
class camera_transform {
public:  
  vec3 camera; // field-coordinates camera origin position (cm)
  coord_rotator camera_tilt; // tilt down
  coord_rotator Z_rotation; // camera panning
  
  camera_transform()
    :camera(250.0,50.0,70.0),  // camera position
     camera_tilt(-20), // X axis rotation (camera mounting tilt)
     Z_rotation(0) // Z axis rotation
  {
  }
  
  // Project this camera-relative 3D point into world coordinates
  vec3 world_from_camera(vec3 point) {
    real_t x=point.x, y=point.z, z=-point.y;
    camera_tilt.rotate(y,z); // tilt up, so camera is level
    Z_rotation.rotate(x,y); // rotate, to align with field
    x+=camera.x;
    y+=camera.y;
    z+=camera.z; 
    return vec3(x,y,z);
  }
};

/// Keeps track of location
class obstacle_grid {
public:
  enum {GRIDSIZE=10}; // cm per grid cell
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
    
    /*
    rs2_error* e = nullptr;
    rs2_intrinsics intr;
    rs2_get_video_stream_intrinsics(selection, &intr, &e);
*/

    
    
    Mat depth_sum(Size(depth_w,depth_h), CV_32S, cv::Scalar(0.0));  
    Mat depth_ssq(Size(depth_w,depth_h), CV_64F, cv::Scalar(0.0));  
    Mat depth_count(Size(depth_w,depth_h), CV_32S, cv::Scalar(0.0));  
    Mat depth_max(Size(depth_w,depth_h), CV_8U, cv::Scalar(0.0));  
    Mat depth_min(Size(depth_w,depth_h), CV_8U, cv::Scalar(255.0));  
    int framecount=0;
    int nextwrite=1;

    bool has_depth_intrin=false;
    rs2_intrinsics depth_intrinsics;
    rs2_intrinsics color_intrinsics;

    std::vector<float> depth_xdir(depth_w*depth_h);
    std::vector<float> depth_ydir(depth_w*depth_h);
    std::vector<grid_square> grid(obstacle_grid::GRIDTOTAL);
    camera_transform camera_TF;
    
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
        
        if (!has_depth_intrin)
        { // Extract camera intrinsic calibrations
          has_depth_intrin=true;
          
          auto stream_profile = depth_frame.get_profile();
          auto video = stream_profile.as<rs2::video_stream_profile>();
          depth_intrinsics = video.get_intrinsics();
          
          stream_profile = color_frame.get_profile();
          video = stream_profile.as<rs2::video_stream_profile>();
          color_intrinsics = video.get_intrinsics();
          
          // Precompute per-pixel direction vectors (with distortion)
          for (int h = 0; h < depth_intrinsics.height; ++h)
          for (int w = 0; w < depth_intrinsics.width; ++w)
          {
            const float pixel[] = { (float)w, (float)h };

            float x = (pixel[0] - depth_intrinsics.ppx) / depth_intrinsics.fx;
            float y = (pixel[1] - depth_intrinsics.ppy) / depth_intrinsics.fy;

            if (depth_intrinsics.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY)
            {
                float r2 = x * x + y * y;
                float f = 1 + depth_intrinsics.coeffs[0] * r2 + depth_intrinsics.coeffs[1] * r2*r2 + depth_intrinsics.coeffs[4] * r2*r2*r2;
                float ux = x * f + 2 * depth_intrinsics.coeffs[2] * x*y + depth_intrinsics.coeffs[3] * (r2 + 2 * x*x);
                float uy = y * f + 2 * depth_intrinsics.coeffs[3] * x*y + depth_intrinsics.coeffs[2] * (r2 + 2 * y*y);
                x = ux;
                y = uy;
            }

            depth_xdir[h*depth_intrinsics.width + w] = x;
            depth_ydir[h*depth_intrinsics.width + w] = y;
          }
        }
  
        typedef unsigned short depth_t;
        depth_t *depth_data = (depth_t*)depth_frame.get_data();  
        void *color_data = (void*)color_frame.get_data();  
        
        for (size_t i=0;i<grid.size();i++) grid[i]=grid_square();
        
        for (int h = 0; h < depth_intrinsics.height; h++)
        for (int w = 50; w < depth_intrinsics.width; w++)
        {
          int i=h*depth_intrinsics.width + w;
          float depth=depth_data[i]*depth2cm;
          if (depth>0) {
            vec3 cam(depth_xdir[i]*depth, depth_ydir[i]*depth, depth);
            vec3 world=camera_TF.world_from_camera(cam);
            if (world.z<120.0) {
              // printf("  %.0f  %.0f   %.0f   \n",world.x,world.y,world.z);
              unsigned int x=world.x*(1.0/obstacle_grid::GRIDSIZE);
              unsigned int y=world.y*(1.0/obstacle_grid::GRIDSIZE);
              if (x<obstacle_grid::GRIDX && y<obstacle_grid::GRIDY)
              {
                grid[y*obstacle_grid::GRIDX + x].addPoint(world.z);
                // world_depth.at<unsigned char>(y,x)=50+world.z;
              }
            }
          }
        }   
        
        enum {depthscale=10};
        cv::Mat world_depth(cv::Size(obstacle_grid::GRIDX*depthscale,obstacle_grid::GRIDY*depthscale), CV_8U, cv::Scalar(0,0,0));
        for (int h = 0; h < obstacle_grid::GRIDY; h++)
        for (int w = 0; w < obstacle_grid::GRIDX; w++)
        {
          grid_square &g=grid[h*obstacle_grid::GRIDX + w];
          for (int dy=0; dy<depthscale;dy++)
          for (int dx=0; dx<depthscale;dx++)
          {
            int x=w*depthscale+dx;
            int y=h*depthscale+dy;
            if (g.getCount()>0)
              world_depth.at<unsigned char>(y,x)=50+g.getMean();
          }
        }
        imshow("Depth",world_depth);
        
  
        // Create OpenCV Matrices  
        Mat depth_raw(Size(depth_w, depth_h), CV_16U, depth_data, Mat::AUTO_STEP);  
        Mat color(Size(color_w, color_h), CV_8UC3, color_data, Mat::AUTO_STEP);  
        Mat filtered_0(Size(depth_w, depth_h), CV_8U, cv::Scalar(0));
	
	      depth_raw.convertTo(filtered_0,CV_8U,depth2screen);
	      cv::max(depth_max,filtered_0,depth_max);
	      // cv::min(depth_min,filtered_0,depth_min);

	      if (false && bigmode) {
	        // Shrink images (for small screens)
	        cv::resize(filtered_0,filtered_0, Size(), 0.5,0.5, CV_INTER_AREA);
	        cv::resize(color,color, Size(), 0.5,0.5, CV_INTER_AREA);
	      }
  
        // Display  
        //imshow("Image", color);
        imshow("Filtered 0", filtered_0);

	      // add rolling per-pixel depth averages
	      for (int r=0;r<depth_raw.rows;r++)
	      for (int c=0;c<depth_raw.cols;c++) {
		      uint16_t d=depth_raw.at<uint16_t>(r,c);
		      if (d>0) {
			      depth_sum.at<int32_t>(r,c)+=d;
			      depth_ssq.at<double>(r,c)+=d*d;
			      uint8_t &dm=depth_min.at<uint8_t>(r,c);
			      dm=std::min(dm,(uint8_t)(d*depth2screen));
			      depth_count.at<int32_t>(r,c)++;
		      }
	      }
        
        int k = waitKey(10);  
	      if (k=='m') { /* 'm' key dumps so-far min and max as images */
		      imwrite("depth_min.png",depth_min);
		      imwrite("depth_max.png",depth_max);
	              Mat filtered_1(Size(depth_w, depth_h), CV_8U, cv::Scalar(0));  

	        for (int r=0;r<depth_raw.rows;r++)
	        for (int c=0;c<depth_raw.cols;c++) {
		        double sum=depth_sum.at<int32_t>(r,c);
		        double ssq=depth_ssq.at<double>(r,c);
		        int count=depth_count.at<int32_t>(r,c);
		        filtered_0.at<uint8_t>(r,c)= depth2screen * 
			        sum / count;
		        double st=sqrt((ssq-sum*sum/count) / (count-1));
		        double stmax=250; // mm maximum visible std deviation
		        if (st>stmax) st=stmax;
		        filtered_1.at<uint8_t>(r,c)= 255.0 * st / stmax;
		        if (r==100 && c==100) printf("sum %f, ssq %f, count %f, st %f\n",
			        sum,ssq,(double)count,st);
	        }
		      imwrite("depth_stdev.png",filtered_1);
		      imwrite("depth_mean.png",filtered_0);
	      }
        if (k== 'w' || (framecount>=nextwrite)) {
          char name[1024];

          sprintf(name,"depth_%04d.png",framecount);
          imwrite(name,filtered_0);

          for (int r=0;r<depth_raw.rows;r++)
          for (int c=0;c<depth_raw.cols;c++) {
	          filtered_0.at<uint8_t>(r,c)= depth2screen * 
		          depth_sum.at<int32_t>(r,c) / 
		          depth_count.at<int32_t>(r,c);
          }

          sprintf(name,"sum_%04d.png",framecount);
          imwrite(name,filtered_0);
          nextwrite=nextwrite*2;
          printf("Wrote frame images to '%s'\n",name);
          // if (nextwrite>=1024) break;
        }
        if (k == 27 || k=='q')  
            break;  
    }  
  
  
    return 0;  
}  
