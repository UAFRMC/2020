// Simulates computer vision data

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include "gridnav/gridnav.h"
#include "gridnav/gridnav_RMC.h"
#include "aurora/coords.h"
#include "../firmware/field_geometry.h"
#include "aurora/lunatic.h"
#include "vision/grid.hpp"
#include "vision/grid.cpp"


class field_simulator {
public:
    enum { GRIDX=obstacle_grid::GRIDX };
    enum { GRIDY=obstacle_grid::GRIDY };
    char field[GRIDY][GRIDX];
    
    size_t count_depthrange;
    size_t count_flat; // grid squares that are driveable
    size_t count_obstacle; // grid squares that have obstacles
    size_t count_samples; // total samples
    
    void clear_counts() {
        count_depthrange=0;
        count_flat=0;
        count_obstacle=0;
        count_samples=0;
    }
    void print_counts() {
        printf("Generated frame: %d flat, %d obstacle cells (%d samples)\n",
            (int)count_flat,(int)count_obstacle, (int)count_samples);
    }

    void read_field(const char *field_filename) {
        // Read the ASCII art field
        std::ifstream field_file("field.txt");
        for (int y=0;y<GRIDY;y++) {
            std::string line="";
            std::getline(field_file,line); //<- may fail, that's OK
            
            // Add spaces to end of short lines
            while (line.size()<GRIDX) line+=" "; 
            
            // Write into field bottom-up
            strncpy(&field[GRIDY-1-y][0],&line[0],(int)GRIDX);
        }
    }
    
    /* Figure out what obstacles are visible from this camera view. 
       HACKY: works backwards, should make real rendered depth image. */
    obstacle_grid get_map(const aurora::robot_coord3D &view3D) 
    {
        obstacle_grid map2d;
        map2d.clear();
        for (int oy=0;oy<obstacle_grid::GRIDY;oy++)
        for (int ox=0;ox<obstacle_grid::GRIDX;ox++)
        {
            vec3 world(ox*obstacle_grid::GRIDSIZE,oy*obstacle_grid::GRIDSIZE,70.0);
            vec3 camera=view3D.local_from_world(world);
            if (camera.z<100.0 || camera.z>220.0) continue; // beyond depth range (cm)
            
            float angle=camera.x/camera.z;
            if (fabs(angle)>0.8) continue; // beyond camera horizontal FOV
            
            if (ox>=GRIDX || oy>=GRIDY) continue; // not in ascii art range
            float z=0.0;
            
            char c=field[oy][ox];
            if (c==' ') z=0.0; 
            else z=50.0; // <- fire high limit
            
            if (z!=0.0) count_obstacle++; else count_flat++;
            
            // This grid square receives a certain number of camera depth pixels
            int nsamples=3*100/camera.z+(rand()%3);
            for (int s=0;s<nsamples;s++) {
                // add depth noise to each sample
                float noise=0.1*(rand()%64);
                
                map2d.at(ox,oy).addPoint(z+noise);
                count_samples++;
            }
        }
        return map2d;
    }
};

int main(int argc, char * argv[])
{
    int fps=6; // depth camera framerate
    field_simulator sim;
    
    
    // Read from localizer to figure out robot's viewpoint
    MAKE_exchange_obstacle_view();
    
    // View of field exported out to cartographer
    MAKE_exchange_field_raw();
    
    printf("Field size should be %d x %d chars\n",obstacle_grid::GRIDX,obstacle_grid::GRIDY);
    sim.read_field("field.txt");
    
    while (true)
    {
        aurora::robot_coord3D view3D = exchange_obstacle_view.read();
        if (view3D.percent > 0) {
            sim.clear_counts();
        
            obstacle_grid map2D=sim.get_map(view3D);
            exchange_field_raw.write_begin() = map2D;
            exchange_field_raw.write_end();
            
            sim.print_counts();
        }

        aurora::data_exchange_sleep(1000/fps);
    }

    return 0;
}
