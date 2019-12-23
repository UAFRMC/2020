/**
High level interface to the aruco-3 computer vision marker 
detection library.

*/
#ifndef __AURORA_ARUCO_DETECTOR_H
#define __AURORA_ARUCO_DETECTOR_H

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

class aruco_detector {
public:
    aruco_detector(float min_size=0.1, const char *cam_parameters="camera.xml");
    
    /**
      Find markers in this image.
      
      The marker_watcher class has one required method, where it passes the markers it detects:
         void found_marker(cv::Mat &matrix4x4,const aruco::Marker &marker,int ID);
      matrix4x4 is the marker's 4x4 perspective transform matrix, in camera coords.
      ID is the Aruco marker number observed.
    */
    template <class marker_watcher>
    void find_markers(cv::Mat &image,marker_watcher &watcher, bool draw_debug=false);
    
private:
    /* Extract location data from this valid, detected marker. */
    template <class marker_watcher>
    void extract_location(const aruco::Marker &marker,marker_watcher &watcher);

    aruco::MarkerDetector MDetector;
    std::vector<aruco::Marker> TheMarkers;
    aruco::MarkerDetector::Params params;
    aruco::CameraParameters cam_param;
    bool cam_param_resized;
};

#endif





