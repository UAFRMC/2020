// This is the computer vision system data exchange system, the cvs.cpp will 
// write data to be used by localization two main jobs. 1)Writing logic for
// determining if a position is driveable. 2) Used for defining Auruco markers data 
// 
//

#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

typedef std::array<aurora::vision_marker_report, aurora::vision_marker_report::max_count> vision_marker_reports; 
typedef aurora::field_raster<unsigned char,4> field_drivable;

int main() {
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_marker_reports();
    MAKE_exchange_field_drivable();
    MAKE_exchange_obstacle_view();

    while (true) {
        
        aurora::vision_marker_reports currMarkerReports = exchange_marker_reports.read();
        aurora::robot_coord3D currentRobotPos = exchange_plan_target.read();
        //with curent position and realsense data able to update the feild ?
        

        aurora::field_drivable newField; 
        // newField.at(currentRobotPos.X, currentRobotPos.y) = ?
        
        exchange_field_drivable.write_begin() = newField;
        exchange_field_drivable.write_end();

        //getting auruco data here to build the new markerreport.
        //build out the markerreport before adding to the array that exsists.
        aurora::vision_marker_report newMarkerReport;
        //handles data from auruco to set the new 3d coords as well as markerID
        // newMarkerReport.coords
        // newMarkerReport.markerID = 0;
        // newMarkerReport.max_count = 3;

        exchange_marker_reports.write_begin() = currMarkerReports;
        exchange_marker_reports.write_end();


        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}
