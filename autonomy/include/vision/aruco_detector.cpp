/**
 Implementation of aruco_detector header calls.
*/
#include "aruco_detector.hpp"

aruco_detector::aruco_detector(float min_size, const char *cam_parameters) 
    :MDetector("TAG25h9"),
    cam_param_resized(false)
{
  //	if (ThePyrDownLevel>0)
  //		params.pyrDown(ThePyrDownLevel);
  //	params.setCornerRefinementMethod(MarkerDetector::CORNER_SUBPIX); // more accurate?
  params.setCornerRefinementMethod(aruco::CORNER_LINES); // more reliable?
  params.setDetectionMode(aruco::DM_FAST,min_size); // for distant/small markers (smaller values == smaller markers, but slower too)
  
  cam_param.readFromXMLFile(cam_parameters);
}

/**
  The marker_watcher class has one required method:
     void found_marker(cv::Mat &matrix4x4,const aruco::Marker &marker,int ID);
  matrix4x4 is the marker's 4x4 perspective transform matrix, in camera coords.
  ID is the Aruco marker number observed.
*/
template <class marker_watcher>
void aruco_detector::find_markers(cv::Mat &color_image,marker_watcher &watcher, bool draw_debug) {
	if (!cam_param_resized) {
        cam_param_resized=true;
        cam_param.resize(color_image.size());
    }
	
	// Detect all the markers
	MDetector.detect(color_image,TheMarkers,cam_param,1.0,true);
	
	// Extract locations from different markers:
	for (int i=0; i<(int)TheMarkers.size(); i++) {
		aruco::Marker &marker=TheMarkers[i];
		extract_location(marker,watcher);
	}
	
	if (draw_debug) // draw debug info onto the image
	{
	  // Draw 3D box around all detected markers
	  for (int i=0; i<(int)TheMarkers.size(); i++) {
		  aruco::Marker &marker=TheMarkers[i];
		  // cout<<marker<<endl;
		  
		  marker.draw(color_image,cv::Scalar(0,0,255),1);

		  //draw a 3d cube on each marker if there is 3d info
		  if (cam_param.isValid()) {
			  aruco::CvDrawingUtils::draw3dCube(color_image,marker,cam_param,1,true);
			  aruco::CvDrawingUtils::draw3dAxis(color_image,marker,cam_param);
		  }
	  }
    /*
		//print other rectangles/scanlines that contains invalid markers
		for (int i=0; i<(int)MDetector.getCandidates().size(); i++) {
			aruco::Marker m( MDetector.getCandidates()[i],999);
			m.draw(color_image,cv::Scalar(255,0,0));
		}
    */
	}
}



/* Extract location data from this valid, detected marker. 
   Does not modify the location for an invalid marker.
*/
template <class marker_watcher>
void aruco_detector::extract_location(const aruco::Marker &marker,marker_watcher &watcher)
{
	// Extract 3x3 rotation matrix
	cv::Mat Rot(3,3,CV_32FC1);
	cv::Rodrigues(marker.Rvec, Rot); // euler angles to rotation matrix

	// Full 4x4 output matrix:
	cv::Mat full(4,4,CV_32FC1);

	// Copy rotation 3x3
	for (int i=0; i<3; i++)
		for (int j=0; j<3; j++)
			full.at<float>(i,j)=Rot.at<float>(i,j);
	
	// Copy translation vector
	full.at<float>(0,3)=marker.Tvec.at<float>(0,0);
	full.at<float>(1,3)=marker.Tvec.at<float>(1,0);
	full.at<float>(2,3)=marker.Tvec.at<float>(2,0);

	// Final row is identity (nothing happening on W axis)
	full.at<float>(3,0)=0.0;
	full.at<float>(3,1)=0.0;
	full.at<float>(3,2)=0.0;
	full.at<float>(3,3)=1.0;

	// Invert, to convert marker-from-camera into camera-from-marker
	//cv::Mat back=full.inv(); // marker on scoring trough, camera on robot
	cv::Mat back=full; // marker on robot, camera on scoring trough

    if (false) {
        // Splat marker's inverse matrix to console, for debugging
        for (int i=0; i<4; i++) {
            for (int j=0; j<4; j++)
                printf("%.2f	",back.at<float>(i,j));
            printf("\n");
        }
    }
	
	watcher.found_marker(back,marker,marker.id);
}

