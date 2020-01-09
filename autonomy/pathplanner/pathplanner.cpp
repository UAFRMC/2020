// Basic skeleton read/write file for the pathplanner, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"
#include "pathplanner.h"


int main() {
    //Make the pathplanning object
    robot_autodriver robotplanner;

    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_drive_commands();
    
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_plan_target();
    MAKE_exchange_plan_current();
    MAKE_exchange_field_drivable();

    while (true) {
        aurora::robot_loc2D target = exchange_plan_target.read();
        
        if (target.percent>=1.0) { // we have a valid target
            aurora::robot_loc2D current = exchange_plan_current.read();
            const aurora::field_drivable &currField = exchange_field_drivable.read();
            
            //Some logic to determine what are the dive values
            //Currently just creates empty object needs some data?
            aurora::drive_commands newDrive;
            //Set
            //spinning around in place
            newDrive.left = 100.0;
            newDrive.right = 0.0;

            //writing new data to files:
            exchange_drive_commands.write_begin() = newDrive;
            exchange_drive_commands.write_end();
        }

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}
