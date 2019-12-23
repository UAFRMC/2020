/* 
 Implementation file for realsense camera.
*/
#include "realsense_camera.hpp"


realsense_camera::realsense_camera(int res,int fps) 
    :depth_projector(0)
{ 
    // You get an exception at runtime if your resolution / framerate is not matched.
    int depth_w=1280, depth_h=720; // high res, 30fps: definitely more detail visible
    int color_w=1280, color_h=720; 
    if (res==720) { // default case above (USB 2.0 -> 6fps limit)
    } 
    else if (res==540) { // medium res, 60fps, widescreen (USB 3.0 only)
        depth_w=848; depth_h=480;
        color_w=960; color_h=540; 
    } 
    else if (res==480) { // non-widescreen (USB 3.0 only)
        depth_w=640; depth_h=480;
        color_w=640; color_h=480; 
    } 
    else if (res==240) { // low res, works everywhere
        depth_w=480; depth_h=270;
        color_w=424; color_h=240;
    }
    else throw std::runtime_error("Unknown resolution passed to realsense_camera");

    cfg.enable_stream(RS2_STREAM_DEPTH, depth_w,depth_h, RS2_FORMAT_Z16, fps);  
    cfg.enable_stream(RS2_STREAM_COLOR, color_w,color_h, RS2_FORMAT_BGR8, fps);  
  
    rs2::pipeline_profile selection = pipe.start(cfg);  

    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    float scale =  sensor.get_depth_scale();
    depth2cm = scale * 100.0; 
}
realsense_camera::~realsense_camera() {
    delete depth_projector;
}



// Calling this constructor captures image data from the camera:
realsense_camera_capture::realsense_camera_capture(realsense_camera &cam)
    :frames(cam.pipe.wait_for_frames()),
     color_frame(frames.get_color_frame()),
     depth_frame(frames.get_depth_frame()),
     color_w(color_frame.get_width()), color_h(color_frame.get_height()),
     depth_w(depth_frame.get_width()), depth_h(depth_frame.get_height()),
     color_image(cv::Size(color_w, color_h), CV_8UC3, 
        (void*)color_frame.get_data(), cv::Mat::AUTO_STEP),
     depth_data((depth_t*)depth_frame.get_data()),
     depth_image(cv::Size(depth_w, depth_h), CV_16U, 
        depth_data, cv::Mat::AUTO_STEP)
{
    if (!cam.depth_projector) { // <- only make once, at startup
        cam.depth_projector=new realsense_projector(depth_frame);
    } 
    depth_projector=cam.depth_projector;
    depth2cm = cam.depth2cm;
}


