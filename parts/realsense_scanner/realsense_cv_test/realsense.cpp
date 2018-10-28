/**

  Modified from tiny example by BjarneG at https://communities.intel.com/thread/121826
*/
#include <librealsense2/rs.hpp>  
#include <opencv2/opencv.hpp>  
  
using namespace std;  
using namespace cv;  
  
int main()  
{  
    rs2::pipeline pipe;  
    rs2::config cfg;  
  
    bool bigmode=false;
    int fps=15;
    if (bigmode) { // high res
      cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 6);  
      cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 6);  
    }
    else
    { // low res
      cfg.enable_stream(RS2_STREAM_COLOR, 424, 240, RS2_FORMAT_BGR8, fps);  
      cfg.enable_stream(RS2_STREAM_DEPTH, 480, 270, RS2_FORMAT_Z16, fps);  
    }
  
    rs2::pipeline_profile selection = pipe.start(cfg);  

    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    float scale =  sensor.get_depth_scale();
    printf("Depth scale: %.3f\n",scale);
  
    Mat depth_sum(Size(480, 270), CV_32S, cv::Scalar(0.0));  
    int framecount=0;
    int nextwrite=1;

    rs2::frameset frames;  
    while (true)  
    {  
        // Wait for a coherent pair of frames: depth and color  
        frames = pipe.wait_for_frames();  
        rs2::depth_frame depth_frame = frames.get_depth_frame();  
        rs2::video_frame color_frame = frames.get_color_frame();  
  
        unsigned int d_w = depth_frame.get_width();  
        unsigned int d_h = depth_frame.get_height();  
        unsigned int c_w = color_frame.get_width();  
        unsigned int c_h = color_frame.get_height();  
  
        void *depth_data = (void*)depth_frame.get_data();  
        void *color_data = (void*)color_frame.get_data();  
  
        // Creating OpenCV Matrices  
        Mat depth_raw(Size(d_w, d_h), CV_16U, depth_data, Mat::AUTO_STEP);  
        cv::add(depth_sum, depth_raw, depth_sum, noArray(), CV_32S);
        Mat color(Size(c_w, c_h), CV_8UC3, color_data, Mat::AUTO_STEP);  
  
        Mat filtered_0(Size(d_w, d_h), CV_8U, cv::Scalar(0));  
        // filtered_0.setTo(255, depth_raw == 0);  
	
	depth_raw.convertTo(filtered_0,CV_8U,255.0*scale/4.5);

	if (bigmode) {
	  // Shrink images (for small screens)
	  cv::resize(filtered_0,filtered_0, Size(), 0.5,0.5, CV_INTER_AREA);
	  cv::resize(color,color, Size(), 0.5,0.5, CV_INTER_AREA);
	}
  
        // Display  
        imshow("Image", color);  
        imshow("Filtered 0", filtered_0);  
  
        int k = waitKey(10);  
        if (k== 'w' || (framecount>=nextwrite)) {
            char name[1024];

            sprintf(name,"depth_%d.png",framecount);
            imwrite(name,filtered_0);

            depth_sum.convertTo(filtered_0,CV_8U,255.0*scale/4.5/(1+framecount));
            sprintf(name,"sum_%d.png",framecount);
            imwrite(name,filtered_0);
            nextwrite=nextwrite*8;
            printf("Wrote frame images to '%s'\n",name);
        }
        if (k == 27)  
            break;  
        framecount++;
    }  
  
  
    return 0;  
}  
