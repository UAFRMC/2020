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
  
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_BGR8, 6);  
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 6);  
  
    pipe.start(cfg);  
  
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
        Mat color(Size(c_w, c_h), CV_8UC3, color_data, Mat::AUTO_STEP);  
  
        Mat filtered_0(Size(d_w, d_h), CV_8U, cv::Scalar(0));  
        Mat filtered_65535(Size(d_w, d_h), CV_8U, cv::Scalar(0));  
        filtered_0.setTo(255, depth_raw == 0);  
        filtered_65535.setTo(255, depth_raw == 65535);  
  
        // Display  
        imshow("Image", color);  
        imshow("Filtered 0", filtered_0);  
        imshow("Filtered 65535", filtered_65535);  
  
        int k = waitKey(10);  
        if (k == 27)  
            break;  
    }  
  
  
    return 0;  
}  
