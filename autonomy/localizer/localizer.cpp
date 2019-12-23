// Basic skeleton read/write file for the localizer, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"


int main() {
    float wheelbase=40; // cm between track centerlines
    float drivecount2cm=6*5.0/36; // cm of driving per wheel encoder tick == 8 pegs on drive sprockets, 5 cm between sprockets, 36 encoder counts per revolution
    
    //Data sources need to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_drive_commands();
    MAKE_exchange_drive_encoders();
    MAKE_exchange_stepper_report();
    MAKE_exchange_marker_reports();

    //Data source needed to write too, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_plan_current();
    MAKE_exchange_obstacle_view();
    aurora::drive_encoders lastencoder={0.0,0.0};
    
    aurora::robot_loc2D pos;
    pos.x = field_x_size/2;
    pos.y = 100.0; // start location
    pos.angle=0.0f;
    pos.percent=0.0f;
    
    int printcount=0;

    while (true) {
        bool print=false;
        if ((printcount++%50)==0) print=true;
        
        aurora::drive_encoders currentencode = exchange_drive_encoders.read();
        aurora::drive_commands currentdrive = exchange_drive_commands.read();
        aurora::stepper_pointing currentstepper = exchange_stepper_report.read();
        aurora::vision_marker_reports currentvision = exchange_marker_reports.read();
        
        //Some logic to determine what our next plan of movement is?
        //Currently just creates empty objects needs some data?
        aurora::robot_loc2D new2D=pos;
        
        // Extract position and orientation from absolute location
        //Interesting issue, the pos used in the new iteration is not 3d cords. the vec3 is a a 3d cord stuff?
        vec3 P=vec3(pos.x, pos.y,0.0); // position of robot (center of wheels)
        double ang_rads=pos.angle*M_PI/180.0; // 2D rotation of robot

    // Reconstruct coordinate system and wheel locations 
        vec3 FW=vec3(cos(ang_rads),sin(ang_rads),0.0); // forward vector
        vec3 UP=vec3(0,0,1); // up vector
        vec3 LR=FW.cross(UP); // left-to-right vector
        vec3 wheel[2];
        wheel[0]=P-0.5*wheelbase*LR;
        wheel[1]=P+0.5*wheelbase*LR;

    //How does wheels vs tracks work?
    // Move wheels forward by specified amounts
        aurora::drive_encoders encoderchange = currentencode - lastencoder;
        float maxjump=50.0f; // < avoids huge jumps due to startup
        if (fabs(encoderchange.left<maxjump) && fabs(encoderchange.right<maxjump)) 
        {
            wheel[0]+=FW*encoderchange.left;
            wheel[1]+=FW*encoderchange.right;
        }
        
        lastencoder = currentencode;

    // Extract new robot position and orientation
        P=(wheel[0]+wheel[1])*0.5;
        LR=normalize(wheel[1]-wheel[0]);
        FW=UP.cross(LR);
        ang_rads=atan2(FW.y,FW.x);

    // Put back into merged absolute location
        new2D.angle=180.0/M_PI*ang_rads;
        new2D.x=P.x; new2D.y=P.y;

    //method to determine confidence?
    // float view_robot_angle=get_beacon_angle(locator.merged.x,locator.merged.y);
    // float beacon_FOV=30; // field of view of beacon (markers)
    // if (beacon_FOV>fabs(telemetry.autonomy.markers.beacon - view_robot_angle))
    //   locator.merged.confidence+=0.1;
    // locator.merged.confidence=std::min(1.0,locator.merged.confidence*(1.0-dt));
        
        exchange_plan_current.write_begin() = new2D;
        exchange_plan_current.write_end();
        pos=new2D;
        if (print) { printf("Robot: "); pos.print(); }
        
        aurora::robot_coord3D robot3D=new2D.get3D();
        
        // Start with camera coordinates relative to robot coordinates
        aurora::robot_coord3D camera3D;
        camera3D.origin=vec3(0,23.0,60.0); // centimeters relative to robot turning center
        // camera_heading == 0 -> facing
        float camera_heading=exchange_stepper_report.read().angle; 
        camera3D.X=aurora::vec3_from_angle(camera_heading);
        camera3D.Y=aurora::rotate_90_Z(camera3D.X);
        camera3D.Z=vec3(0,0,1);
        
        // Add the camera's inherent coordinate system and mounting angle
        aurora::robot_coord3D camera_downtilt;
        vec3 tilt=aurora::vec3_from_angle(30.0);
        float c=tilt.x, s=tilt.y;
        camera_downtilt.X=vec3( 0,-1, 0); // camera X is robot -Y
        camera_downtilt.Y=vec3(-s, 0,-c); // camera Y is mostly down
        camera_downtilt.Z=vec3( c, 0,-s); // camera Z is mostly robot +X (forward)
        
        aurora::robot_coord3D camera_total = robot3D.compose(camera3D.compose(camera_downtilt));
        if (print) { printf("Camera: "); camera_total.print(); }
        exchange_obstacle_view.write_begin() = camera_total;
        exchange_obstacle_view.write_end();
        

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}
