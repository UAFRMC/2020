/*
  Library-style version of OpenCV Aruco marker detection code,
  used for robot localization.
*/
#include <stdio.h>
#include <errno.h>
#include <vector>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"

// Store info about how the markers are scaled, positioned and oriented
struct marker_info_t {
	int id; // marker's ID, from 0-31
	float true_size; // side length, in meters, of black part of pattern
	
	float x_shift; // marker-relative translation to robot origin, in meters from center of pattern
	float y_shift; 
	float z_shift; 
	
	float rotate; // Z axis rotation angle of marker relative to robot
};

const static marker_info_t marker_info[]={
	{-1,0.145}, // fallback default case
	
	{13, 0.250}, // creeper (on 30cm full-size test plate)
	{17, 0.145}, // bird (on 20cm half-size test plate)
	{16, 0.040}, // elipsis (on tiny 5cm test plate)
};

// Look up the calibration parameters for this marker
const marker_info_t &get_marker_info(int id) {
	for (int i=1;i<sizeof(marker_info)/sizeof(marker_info_t);i++) {
		if (marker_info[i].id==id) 
			return marker_info[i];
	}
	return marker_info[0];
}


#include "aurora/location_binary.h"


class aruco_localizer {
public:
  aruco::MarkerDetector MDetector;
	unsigned int framecount;
	uint32_t vidcap_count;
	std::vector<aruco::Marker> TheMarkers;
	aruco::MarkerDetector::Params params;
	aruco::CameraParameters cam_param;
	bool cam_param_resized;
	int skipCount; // only process frames ==0 mod this
	int skipPhase;

aruco_localizer() 
  :MDetector("TAG25h9"),
  framecount(0),
  vidcap_count(0),
  cam_param_resized(false),
  skipCount(1),
  skipPhase(0)
{
  //	if (ThePyrDownLevel>0)
  //		params.pyrDown(ThePyrDownLevel);
  //	params.setCornerRefinementMethod(MarkerDetector::CORNER_SUBPIX); // more accurate
  params.setCornerRefinementMethod(aruco::CORNER_LINES); // more reliable?
  params.setDetectionMode(aruco::DM_FAST,0.1); // for distant/small markers (smaller values == smaller markers, but slower too)
  
  cam_param.readFromXMLFile("camera.xml");
  
}

bool find_markers(cv::Mat &color_image) {
	skipPhase=(skipPhase+1)%skipCount;
	if (skipPhase!=0) return false; // skip this frame
	
	if (!cam_param_resized) {
	  cam_param_resized=true;
	  cam_param.resize(color_image.size());
  }
	
	// Detect all the markers
	MDetector.detect(color_image,TheMarkers,cam_param,1.0,true);
	
	// Extract locations from different markers:
	enum {n_locs=8};
	location_binary locs[n_locs];
	
	for (unsigned int i=0; i<std::min((int)TheMarkers.size(),(int)n_locs); i++) {
		aruco::Marker &marker=TheMarkers[i];
		extract_location(locs[i],marker);
	}
	
	if (true) // draw debug info
	{
	  // Draw 3D box around all detected markers
	  for (unsigned int i=0; i<TheMarkers.size(); i++) {
		  aruco::Marker &marker=TheMarkers[i];
		  // cout<<marker<<endl;
		  
		  marker.draw(color_image,cv::Scalar(0,0,255),1);

		  //draw a 3d cube on each marker if there is 3d info
		  if (cam_param.isValid()) {
			  aruco::CvDrawingUtils::draw3dCube(color_image,marker,cam_param,1,true);
			  aruco::CvDrawingUtils::draw3dAxis(color_image,marker,cam_param);
		  }
	  }

		//print other rectangles that contains invalid markers
		for (unsigned int i=0; i<MDetector.getCandidates().size(); i++) {
			aruco::Marker m( MDetector.getCandidates()[i],999);
			m.draw(color_image,cv::Scalar(255,0,0));
		}
	}
	
	return true;
}



/* Extract location data from this valid, detected marker. 
   Does not modify the location for an invalid marker.
*/
void extract_location(location_binary &bin,const aruco::Marker &marker)
{
	const marker_info_t &mi=get_marker_info(marker.id);

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
	
	if (mi.rotate==90) {
		for (int i=0; i<3; i++) {
			std::swap(full.at<float>(i,0),full.at<float>(i,2)); // swap X and Z
			full.at<float>(i,0)*=-1; // invert (new) X
		}
	}


	// Invert, to convert marker-from-camera into camera-from-marker
	//cv::Mat back=full.inv(); // marker on scoring trough, camera on robot
	cv::Mat back=full; // marker on robot, camera on scoring trough

  if (false) {
	  // Splat marker's inverse matrix to screen, for debugging
	  for (int i=0; i<4; i++) {
		  for (int j=0; j<4; j++)
			  printf("%.2f	",back.at<float>(i,j));
		  printf("\n");
	  }
  }
	
	bin.valid=1;
	double scale=mi.true_size;
	bin.x=back.at<float>(0,3)*scale+mi.x_shift;
	bin.y=back.at<float>(2,3)*scale+mi.y_shift;
	bin.z=(-back.at<float>(1,3)*scale)+mi.z_shift;
	bin.angle=180.0/M_PI*atan2(back.at<float>(2,0),back.at<float>(0,0))+mi.rotate;
	bin.marker_ID=marker.id;

	// Print grep-friendly output
	printf("Marker %d: Camera %.3f %.3f %.3f meters, heading %.1f degrees\n",
	       marker.id, bin.x,bin.y,bin.z,bin.angle
	      );
	fflush(stdout);
}


};




