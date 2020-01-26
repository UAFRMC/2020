/*
  The path planner takes a target, and plans a safe path to drive to it.
*/
#include "pathplanner.h"
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"


int main(int argc,char *argv[]) {
    int delaytime=500; // <- delay, in ms, between planning runs.  Higher: less CPU, less jittery
    for (int argi=1;argi<argc;argi++) {
      std::string arg=argv[argi];
      if (arg=="--lag") delaytime=atoi(argv[++argi]); 
      else {
        std::cerr<<"Unknown argument '"<<arg<<"'.  Exiting.\n";
        return 1;
      }
    }

    //Make the pathplanning object
    robot_autodriver autodriver;

    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_drive_commands();
    MAKE_exchange_path_plan();
    
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_plan_target();
    MAKE_exchange_plan_current();
    MAKE_exchange_field_drivable();

    while (true) {
        bool try_plan=exchange_plan_current.updated() || exchange_plan_target.updated();
        aurora::robot_navtarget target = exchange_plan_target.read();
        if (target.valid()) { // we have a valid target
            aurora::robot_loc2D current = exchange_plan_current.read();
            
            if (exchange_field_drivable.updated()) 
            { // New field obstacles detected: update driver
                autodriver.update_field(exchange_field_drivable.read());
                try_plan=true;
            }
            
            if (try_plan)
            { // Try to plan a new path
                aurora::drive_commands newDrive;
                newDrive.left = newDrive.right = 0.0;
                
                aurora::path_plan debug; 
                debug.plan_len=0;
                
                autodriver.debug_dump();
                if (autodriver.autodrive(current, target, newDrive,debug))
                {
                    exchange_drive_commands.write_begin() = newDrive;
                    exchange_drive_commands.write_end();
                    
                    exchange_path_plan.write_begin()=debug;
                    exchange_path_plan.write_end();
                }
            }
        }

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(delaytime);
    }
    return 0;
}
