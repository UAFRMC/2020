// Basic skeleton read/write file for the localizer, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

const aurora::vision_marker_reports knownMarkers {
    aurora::vision_marker_report(field_x_trough_center, 00.0, 180.0, 13), //mullet thing
    aurora::vision_marker_report(field_x_trough_center, 30.0, 180.0, 14), //cat
    };

void marker_update_robot_pos(aurora::robot_loc2D & currentPos, const aurora::robot_coord3D & currentReportCoord,const int32_t markerID){
    for(aurora::vision_marker_report known : knownMarkers){
         if ( markerID == known.markerID){
            vec3 diff = known.coords.origin - currentReportCoord.origin;
            float diffwt = 0.1;
            currentPos.x += diff.x*diffwt;
            currentPos.y+= diff.y*diffwt;

            
            float anglediff = aurora::angle_signed_diff(known.coords.extract_angle(), currentReportCoord.extract_angle());
            float anglediffwt = 0.1;
            currentPos.angle += anglediff*anglediffwt;
            std::cout << anglediff << "= angle diff \n";
            std::cout << known.coords.extract_angle() << " = known angle, " <<  currentReportCoord.extract_angle() << " = reported angle \n";
        
        }
    }
   
}


// Move the robot based on wheel encoder ticks
aurora::robot_loc2D move_robot_encoder(const aurora::robot_loc2D &pos,const aurora::drive_encoders &encoderchange)
{
    float wheelbase=40; // cm between track centerlines
    
    // Don't move if the encoders are stopped (save CPU and confidence loss)
    if (encoderchange.left == 0 && encoderchange.right==0) return pos;
    
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
    float maxjump=30.0f; // < avoids huge jumps due to startup
    if (fabs(encoderchange.left<maxjump) && fabs(encoderchange.right<maxjump)) 
    {
        wheel[0]+=FW*encoderchange.left;
        wheel[1]+=FW*encoderchange.right;
    }

// Extract new robot position and orientation
    P=(wheel[0]+wheel[1])*0.5;
    LR=normalize(wheel[1]-wheel[0]);
    FW=UP.cross(LR);
    ang_rads=atan2(FW.y,FW.x);

// Put back into merged absolute location
    new2D.angle=180.0/M_PI*ang_rads;
    new2D.x=P.x; new2D.y=P.y;
    
    // Lose a little confidence due to error accumulating, especially on turns
    float turn=fabs(encoderchange.left-encoderchange.right);
    new2D.percent=pos.percent*(1.0-0.0001f-0.001f*turn);
    
    return new2D;
}


// Initialize the obstacle view field for a clean startup
void reinitialize_field(aurora::field_drivable &field) {
    // Zero out most of the field
    field.clear(aurora::field_unknown);
    
    const int scale=aurora::field_drivable::GRIDSIZE;
    // Fill in the known obstacles, like the scoring trough
    for (int y=field_y_trough_start/scale;y<field_y_trough_end/scale;y++)
    for (int x=field_x_trough_start/scale;x<field_x_trough_end/scale;x++)
        if (field.in_bounds(x,y))
            field.at(x,y)=aurora::field_fixed;
}

int main() {
    
    //Data sources need to read from, these are defined in lunatic.h
    MAKE_exchange_drive_commands();
    MAKE_exchange_drive_encoders();
    MAKE_exchange_stepper_report();
    MAKE_exchange_marker_reports();
    MAKE_exchange_T265();

    //Data source needed to write to, these are defined in lunatic.h
    MAKE_exchange_plan_current();
    MAKE_exchange_obstacle_view();
    
    // Zero out all markers
    exchange_marker_reports.write_begin()=aurora::vision_marker_reports();
    exchange_marker_reports.write_end();
    
    // Zero out detected obstacles and such (we have no localization, so they're old)
    MAKE_exchange_field_drivable();
    reinitialize_field(exchange_field_drivable.write_begin());
    exchange_field_drivable.write_end();
    //zero put encoder
    aurora::drive_encoders blank = {0,0};
    exchange_drive_encoders.write_begin()=blank;
    exchange_drive_encoders.write_end();  
    
    // Define our start configuration
    aurora::robot_loc2D pos;
    pos.x = field_x_size/2;
    pos.y = 100.0; // start location
    pos.angle=179.0f;
    pos.percent=5.0f; //<- placeholder, so we can see it change
    
    aurora::drive_encoders lastencoder={0.0,0.0};
    int printcount=0; // <- moderate printing pace, for easier debugging
    bool loc_changed=true;
    while (true) {
        bool print=true;
        if ((printcount++%50)==0) print=true;
        
        aurora::drive_commands currentdrive = exchange_drive_commands.read();
     
    // Update position based on new encoder values
        aurora::drive_encoders currentencoder = exchange_drive_encoders.read();
        aurora::drive_encoders encoder_change = currentencoder - lastencoder;
        if (encoder_change.left!=0 || encoder_change.right!=0) {
            loc_changed=true;
        }
        aurora::robot_loc2D new2D=move_robot_encoder(pos,encoder_change);
        lastencoder = currentencoder;
        if (loc_changed) {
            exchange_plan_current.write_begin() = new2D;
            exchange_plan_current.write_end();
            loc_changed=false;
        }
        pos=new2D;
        if (print) { printf("Robot: "); pos.print(); }
        
        // FIXME: incorporate gyro data here?
        
     // Create camera coordinate transform
        aurora::robot_coord3D robot3D=pos.get3D();
        
        // Start with camera coordinates relative to robot coordinates
        aurora::robot_coord3D camera3D;
        camera3D.origin=vec3(0,23.0,87.0); // centimeters relative to robot turning center
        
        // We define camera_heading == 0 -> camera is facing forward on robot

        auto camera_heading=exchange_stepper_report.read(); 
        std::cout << "current stability is:" << camera_heading.stable << "\n"; 
        if (camera_heading.stable == 0)
        {
            camera3D.percent = 0.0;
        }
        else 
        {
            camera3D.percent = 100.0;
        }
        camera3D.X=aurora::vec3_from_angle(camera_heading.angle);
            camera3D.Y=aurora::rotate_90_Z(camera3D.X);
            camera3D.Z=vec3(0,0,1);
            
            // Add the camera's inherent coordinate system and mounting angle
            aurora::robot_coord3D camera_downtilt;
            vec3 tilt=aurora::vec3_from_angle(20.0);
            float c=tilt.x, s=tilt.y;
            camera_downtilt.X=vec3( 0,-1, 0); // camera X is robot -Y
            camera_downtilt.Y=vec3(-s, 0,-c); // camera Y is mostly down
            camera_downtilt.Z=vec3( c, 0,-s); // camera Z is mostly robot +X (forward)
            camera_downtilt.percent = 100.0;
            aurora::robot_coord3D camera_total = robot3D.compose(camera3D.compose(camera_downtilt));
            std::cout << "camera_total:" << camera_total.percent << "\n";
            exchange_obstacle_view.write_begin() = camera_total;
            exchange_obstacle_view.write_end();
            if (print) { printf("Camera: "); camera_total.print(); }

        
        
     // If you see a newly updated aruco marker, incorporate it into your likely position
            if (exchange_marker_reports.updated()) {
                const aurora::vision_marker_reports &currentvision = exchange_marker_reports.read();
                for (aurora::vision_marker_report report : currentvision)
                    if (report.is_valid())
                    {
                        // Put the marker in world coordinates
                        aurora::robot_coord3D marker_coords=camera_total.compose(report.coords);
                        // fixme: estimate robot position from marker position
                        marker_update_robot_pos(pos,marker_coords,report.markerID);
                        if (print) { printf("Marker%d: ",report.markerID); marker_coords.print(); }
                        loc_changed=true;
                    }
    
                if (print) { printf("\n"); }
        // Limit our cycle rate to 100Hz maximum (to save CPU)
            }
        aurora::data_exchange_sleep(10);

    }
    return 0;
}
