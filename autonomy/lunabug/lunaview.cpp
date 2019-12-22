/* 
  Do data analysis on realsense dataset. 
*/
#include <opencv2/opencv.hpp>  
#include <opencv2/imgproc.hpp>  
#include "aurora/lunatic.h"

/*
  Debugging image for a robotic mining field area
*/
class field_debug_image {
  float pixels_per_cm; // convert cm distance to onscreen distance
  int ht; // Y pixels
  int wid; // X pixels
  cv::Mat image;
  
  // Current drawing state:
  int line_thickness;
  cv::Scalar draw_color;
  
  // Project this field-coordinates 3D point (in cm)
  //  to a 2D cv::Point (in pixels)
  cv::Point project(const vec3 &v) const {
    cv::Point ret(
       v.x*pixels_per_cm,
       ht - v.y*pixels_per_cm // flip so Y==0 is at top onscreen
    );
    return ret;
  }
  // Convert xyz=rgb vec3 into a cv::Scalar
  cv::Scalar colorize(const vec3 &v) const {
    return cv::Scalar(255*v.z,255*v.y,255*v.x); // <- stupid BGR colors
  }
  
public:
  field_debug_image(int _ht=1000) 
    :pixels_per_cm(_ht*1.0f/field_y_size),
     ht(_ht), wid((int)(field_x_size*pixels_per_cm)),
     image(cv::Size(wid,ht), CV_8UC3, cv::Scalar(0,0,0)),
     line_thickness(5),
     draw_color(colorize(vec3(1,0,0)))
  {
  }
  
  // Set the line width used by subsequent line draw calls
  void set_thickness(float cm) { 
    line_thickness=1+(int)(cm*pixels_per_cm);
  }
  
  // Set the color used by subsequent line draw calls
  void set_color(const vec3 &color) {
    draw_color=colorize(color);
  }
  
  // Draw a line between these two points
  void line(const vec3 &start,const vec3 &end)
  {
    cv::line(image,project(start),project(end),draw_color,line_thickness);
  }
  
  void decay(void) {
    image=image*0.8f; // slowly dim the image, CRT style
  }
    
  void show(const char *name) {
    imshow(name,image);
  }
};

// Like fmod, but always returns a positive mod
double fmodpos(double x,double mod) {
    double ret=fmod(x,mod);
    if (ret<0) return mod+ret;
    else return ret;
}


// Draw robot tracks to this debug image
void draw_robot(const aurora::robot_coord3D &loc,
    const aurora::drive_encoders &enc,
    field_debug_image &img)
{

    // Tracks
    float track_thick=8.0f;
    float track_x_min=-30; // ends of tracks
    float track_x_max=+30;
    float track_y=robot_y-track_thick/2-2; // centerline of tracks
    for (int leftright=-1;leftright<=+1;leftright+=2) {
        vec3 track_color(0.5f-0.4f*leftright,0.5f+0.4f*leftright,0.2f);
        
        // Main track path
        img.set_color(0.3f*track_color);
        img.set_thickness(track_thick);
        img.line(
            loc.world_from_robot(vec3(track_x_min,leftright*track_y,0.0f)),
            loc.world_from_robot(vec3(track_x_max,leftright*track_y,0.0f)));
        
        // Grousers
        float per_grouser=10.0;
        img.set_color(1.0f*track_color);
        img.set_thickness(1.0f);
        float start=per_grouser-fmodpos((leftright>0?enc.left:enc.right),per_grouser);
        for (float x=track_x_min-per_grouser/2+start;
                   x<track_x_max+per_grouser/2;
                   x+=per_grouser) 
        {
            img.line(
                loc.world_from_robot(vec3(x,leftright*track_y-track_thick/2,0.0f)),
                loc.world_from_robot(vec3(x,leftright*track_y+track_thick/2,0.0f)));
        }
    }
    
    // Robot perimeter box
    const int ncorners=5;
    const static vec3 corners[ncorners]={
        vec3(-robot_x,-robot_y,0),
        vec3(-robot_x,+robot_y,0),
        vec3(+robot_x,+robot_y,0),
        vec3(+robot_x+robot_mine_x,0,0),
        vec3(+robot_x,-robot_y,0),
    };
    img.set_thickness(0.5f);
    img.set_color(0.6f*vec3(1,1,1));
    for (int c=0;c<ncorners;c++) {
        img.line(
            loc.world_from_robot(corners[c]),
            loc.world_from_robot(corners[(c+1)%ncorners]));
    }
    
    // camera mast
    vec3 cam(0,+robot_y,50.0f);
    img.set_color(vec3(1,1,1));
    img.set_thickness(0.5f);
    float camera_facing=0.0f;
    for (int fov=-1;fov<=+1;fov+=2) {
        img.line(
            loc.world_from_robot(cam),
            loc.world_from_robot(cam+
                250.0f*aurora::vec3_from_angle(camera_facing+fov*30)
            ));
    }
}


int main(int argc,char *argv[]) {
    // Figure out which grid filename to read
    std::string name="obstacles_debug.bin";
    if (argc>1) name=argv[1];
    int ht=1000;

    MAKE_exchange_drive_encoders();
    MAKE_exchange_plan_current();

    field_debug_image img(ht);

    while (true) {
        aurora::robot_coord3D loc3D=exchange_plan_current.read().get3D();
        draw_robot(loc3D,exchange_drive_encoders.read(),img);

        img.show("LunaView");
        img.decay();

        int key=cv::waitKey(30);
        if (key == 27 || key=='q') break; 
    }

    return 0;
}

