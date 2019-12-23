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
  // Fill a rectangle between these two corners
  void rectangle(const vec3 &start,const vec3 &end)
  {
    cv::rectangle(image,project(start),project(end),draw_color,-1);
  }
  
  void decay(float decay_rate=0.15f) {
    image=image*(1.0f-decay_rate); // slowly dim the image, CRT style
  }
    
  void show(const char *name) {
    imshow(name,image);
  }
};


// Draw a set of horizontal and vertical lines
void draw_grid(float step,field_debug_image &img) {
    for (int d=0;d<field_y_size;d+=step) {
        img.line(vec3(d,0,0),vec3(d,field_y_size,0));
        img.line(vec3(0,d,0),vec3(field_x_size,d,0));
    }
}

// Debug a 3D coordinate system, by drawing its coordinate axes
void debug_coords(const aurora::robot_coord3D &coord,std::string what,field_debug_image &img) 
{
    float size=10.0f;
    img.set_thickness(0.5f);
    
    // printf("  %s: ",what.c_str()); coord.print();
    
    for (int axis=0;axis<3;axis++) {
        vec3 dir(0.0f); dir[axis]=1.0f;
        img.set_color(dir); // X == R, Y==G, Z==B
        img.line(coord.origin,coord.world_from_local(size*dir));
    }
}


// Draw the areas of this field that are drivable
void draw_drivable(const aurora::field_drivable &field,field_debug_image &img)
{
    float del=aurora::field_drivable::GRIDSIZE;
    for (int y=0;y<aurora::field_drivable::GRIDY;y++)
    for (int x=0;x<aurora::field_drivable::GRIDX;x++) {
        unsigned char f=field.at(x,y);
        if (f==aurora::field_unknown) continue; // <- skip common empty case
        vec3 color=vec3(1,0,0); // bright red unknown
        if (f==aurora::field_flat) color=vec3(0.5,0.5,0.5); // gray flat
        else if (f==aurora::field_driven) color=vec3(0.8,0.8,0.8); // bright gray history
        else if (f==aurora::field_sloped) color=vec3(0.7,0.0,0.0); // dim red slope
        else if (f==aurora::field_toohigh) color=vec3(0.8,0.3,0.3); // light red high
        else if (f==aurora::field_toolow) color=vec3(0.8,0.0,0.8); // purple low
        else if (f==aurora::field_fixed) color=vec3(0.8,0.8,0.4); // yellow paydirt
        else if (f==aurora::field_mined) color=vec3(0,1.0,1.0); // cyan mined
        img.set_color(0.5f*color);
        img.rectangle(del*vec3(x,y,0),del*vec3(x+1,y+1,0));
    }
}
        

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
        vec3 track_color(0.6f-0.4f*leftright,0.5f+0.4f*leftright,0.2f);
        
        // Main track path
        img.set_color(0.3f*track_color);
        img.set_thickness(track_thick);
        img.line(
            loc.world_from_local(vec3(track_x_min,leftright*track_y,0.0f)),
            loc.world_from_local(vec3(track_x_max,leftright*track_y,0.0f)));
        
        // Grousers
        float per_grouser=20.0; //<- more visual than actual
        img.set_color(1.0f*track_color);
        img.set_thickness(1.0f);
        float start=per_grouser-fmodpos((leftright>0?enc.left:enc.right),per_grouser);
        for (float x=track_x_min-track_thick/2+start;
                   x<track_x_max+track_thick/2;
                   x+=per_grouser) 
        {
            float w=track_thick*0.6f; // visual width of grousers
            img.line(
                loc.world_from_local(vec3(x,leftright*track_y-w,0.0f)),
                loc.world_from_local(vec3(x,leftright*track_y+w,0.0f)));
        }
    }
    
    // Robot perimeter box
    const int ncorners=5;
    const static vec3 corners[ncorners]={
        vec3(-robot_x,-robot_y,0),
        vec3(-robot_x,+robot_y,0),
        vec3(+robot_x,+robot_y,0),
        vec3(+robot_x+20,0,0),
        vec3(+robot_x,-robot_y,0),
    };
    img.set_thickness(0.5f);
    img.set_color(0.6f*vec3(1,1,1));
    for (int c=0;c<ncorners;c++) {
        img.line(
            loc.world_from_local(corners[c]),
            loc.world_from_local(corners[(c+1)%ncorners]));
    }
}

// Draw camera's 3D view frustum
void draw_cameraview(const aurora::robot_coord3D &camera3D,
    field_debug_image &img)
{
    // camera mast
    vec3 cam(0,0,0.0f);
    img.set_color(0.8f*vec3(1,1,1));
    img.set_thickness(0.5f);
    for (int fovX=-1;fovX<=+1;fovX+=2) 
    for (int fovY=-1;fovY<=+1;fovY+=2) {
        float view_distance=250.0f;
        vec3 view=view_distance*vec3(fovX*0.6,fovY*0.3,1.0);
        img.line(
            camera3D.world_from_local(cam),
            camera3D.world_from_local(cam+view)
        );
    }
}

// Draw all visible computer vision markers
void draw_markers(const aurora::robot_coord3D &camera3D,const aurora::vision_marker_reports &reports,field_debug_image &img)
{
        for (aurora::vision_marker_report report : reports) 
            if (report.is_valid())
            {
                aurora::robot_coord3D marker_coords=camera3D.compose(report.coords);
                img.set_color(vec3(0.9f));
                img.set_thickness(2.0f);
                float markersize=15.0;
                img.line(
                    marker_coords.world_from_local(vec3(-markersize,0,0)),
                    marker_coords.world_from_local(vec3(+markersize,0,0)));
                
                debug_coords(marker_coords,
                    "marker"+std::to_string(report.markerID),img);
            }
}


int main(int argc,char *argv[]) {
    // Figure out which grid filename to read
    std::string name="obstacles_debug.bin";
    int ht=1000;
    for (int argi=1;argi<argc;argi++) {
      std::string arg=argv[argi];
      if (arg=="--ht") ht=atoi(argv[++argi]); // set size of window
      else {
        std::cerr<<"Unknown argument '"<<arg<<"'.  Exiting.\n";
        return 1;
      }
    }
    if (argc>1) name=argv[1];

    MAKE_exchange_drive_encoders();
    MAKE_exchange_plan_current();
    MAKE_exchange_obstacle_view();
    MAKE_exchange_field_drivable();
    MAKE_exchange_marker_reports();

    field_debug_image img(ht);

    while (true) {
        // Draw the field
        draw_drivable(exchange_field_drivable.read(),img);
        img.set_color(0.5f*vec3(1,1,1));
        img.set_thickness(0.2f);
        draw_grid(100.0,img);
        
        // Draw the robot
        aurora::robot_coord3D loc3D=exchange_plan_current.read().get3D();
        draw_robot(loc3D,exchange_drive_encoders.read(),img);
        debug_coords(loc3D,"robot",img);
        
        // Draw the camera
        aurora::robot_coord3D camera3D=exchange_obstacle_view.read();
        draw_cameraview(camera3D,img);
        debug_coords(camera3D,"camera",img);
        
        // Draw all detected computer vision markers
        if (exchange_marker_reports.updated())
            draw_markers(camera3D,exchange_marker_reports.read(),img);

        img.show("LunaView");
        img.decay();

        int key=cv::waitKey(30);
        if (key == 27 || key=='q') break; 
    }

    return 0;
}

