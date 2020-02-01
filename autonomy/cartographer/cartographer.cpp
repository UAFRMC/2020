#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

// // Obstacle detection
// #include "vision/grid.hpp"
// #include "vision/grid.cpp"
// #include <opencv2/opencv.hpp> 


// /* Mark grid cells as driveable or non-driveable */
// void mark_obstacles(const obstacle_grid &map2D,aurora::field_drivable &field)
// { 
//   cv::Vec3b slope(255,0,255); // big difference between max and min
//   cv::Vec3b high(0,255,0); // too high to be drivable 
//   cv::Vec3b low(255,0,0); // too low to be drivable 
  
//   const int slope_thresh=8;
//   const int high_thresh=40.0;
//   const int low_thresh=-30.0;
  
//   // Loop over all cells in the grid
//   for (int y = 0; y < obstacle_grid::GRIDY; y++)
//   for (int x = 0; x < obstacle_grid::GRIDX; x++)
//   {
//     const grid_square &me=map2D.at(x,y);
//     if (me.getCount()<3) continue; // skip cells where we don't have data (common case)
    
//     unsigned char mark=aurora::field_flat; // assume it's OK until proven bad
//     if (me.getTrimmedMean() > high_thresh) {
//       mark=aurora::field_toohigh;
//     }
//     if (me.getTrimmedMean() < low_thresh) {
//       mark=aurora::field_toolow;
//     }
//     if (me.getMax() > slope_thresh+me.getMin()) {
//       mark=aurora::field_sloped;
//     }
    
//     // Write this directly to the field
//     field.at(x,y) = mark;
//   }
// }
// looks at north east south and west neighbors, if they jump off a bridge so should you.
// Think of some mathy way to describe how the obsticles should look. 
// void basicFilter(aurora::field_drivable & currField){
//     for(auto x = 0; x < currField.GRIDX; x++){
//         for(auto y = 0; y < currField.GRIDY; y++){
//             auto home = currField.at(x,y);
            
//             auto east = currField.at(x+1,y);
//             auto west = currField.at(x-1,y);
//             auto north = currField.at(x,y+1);
//             auto south = currField.at(x,y-1);
//            //checking if three or more of your neighbors match then you are messed up. 
//             if(home != east && east == north && east == south){
//                 home = east;
//             }
//             else if(home != east && east == west && east == north){
//                 home = east;
//             }
//             else if(home != east && east == west && east == south){
//                 home = east;
//             }
//             else if(home != west && west == north && west == south){
//                 home = west;
//             }
//         }
//     }
// }

int main(int argc,const char *argv[]){
    bool obstacle=true; // look for obstacles/driveable areas in depth data
    MAKE_exchange_field_drivable();
    MAKE_exchange_field_raw();
    MAKE_exchange_obstacle_view();
    aurora::robot_coord3D view3D = exchange_obstacle_view.read();

    for (int argi=1;argi<argc;argi++) {
        std::string arg=argv[argi];
        if (arg=="--clear") 
        {
            unsigned char mark=aurora::field_unknown;
            aurora::field_drivable clearField;
            clearField.clear(mark);
            exchange_field_raw.write_begin() = clearField;
            exchange_field_raw.write_end();
        }
        else if (arg=="--no-obstacle") obstacle=false; 
    }

    aurora::field_drivable newField;
    while(true){
        if(exchange_field_raw.updated()){
            newField = exchange_field_raw.read();
        }
        // mark_obstacles(map2D,newField);
        // basicFilter(newField);
        exchange_field_drivable.write_begin() = newField;
        exchange_field_drivable.write_end();
    }
    return 0;
}