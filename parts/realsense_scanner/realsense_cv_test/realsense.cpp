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
  
    bool bigmode=true;

    int fps=6;
    int depth_w=1280, depth_h=720; // high res mode
    int color_w=1280, color_h=720; 
    if (!bigmode) { // low res
      fps=15;
      depth_w=424; depth_h=240;
      color_w=480; color_h=270;
    }

    cfg.enable_stream(RS2_STREAM_DEPTH, depth_w,depth_h, RS2_FORMAT_Z16, fps);  
    cfg.enable_stream(RS2_STREAM_COLOR, color_w,color_h, RS2_FORMAT_BGR8, fps);  
  
    rs2::pipeline_profile selection = pipe.start(cfg);  

    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    float scale =  sensor.get_depth_scale();
    printf("Depth scale: %.3f\n",scale);
    double depth2screen=255.0*scale/4.5;
  
    Mat depth_sum(Size(depth_w,depth_h), CV_32S, cv::Scalar(0.0));  
    Mat depth_count(Size(depth_w,depth_h), CV_32S, cv::Scalar(0.0));  
    int framecount=0;
    int nextwrite=1;

    rs2::frameset frames;  
    while (true)  
    {  
        // Wait for a coherent pair of frames: depth and color  
        frames = pipe.wait_for_frames();  
        rs2::depth_frame depth_frame = frames.get_depth_frame();  
        rs2::video_frame color_frame = frames.get_color_frame();  
        framecount++;
  
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
	
	depth_raw.convertTo(filtered_0,CV_8U,depth2screen);

	if (false && bigmode) {
	  // Shrink images (for small screens)
	  cv::resize(filtered_0,filtered_0, Size(), 0.5,0.5, CV_INTER_AREA);
	  cv::resize(color,color, Size(), 0.5,0.5, CV_INTER_AREA);
	}
  
        // Display  
        imshow("Image", color);  
        imshow("Filtered 0", filtered_0);  


	// add rolling per-pixel depth averages
	for (int r=0;r<depth_raw.rows;r++)
	for (int c=0;c<depth_raw.cols;c++) {
		int d=depth_raw.at<uint16_t>(r,c);
		if (d>0) {
			depth_sum.at<int32_t>(r,c)+=d;
			depth_count.at<int32_t>(r,c)++;
		}
	}
  
        int k = waitKey(10);  
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
        }
        if (k == 27)  
            break;  
    }  
  
  
    return 0;  
}  
