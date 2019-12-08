// Basic skeleton read/write file for the pathplanner, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

int main() {
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_drive_commands();
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_plan_target();
    MAKE_exchange_plan_current();
    MAKE_exchange_field_drivable();

    while (true) {
        aurora::robot_loc2D old2dCoord = exchange_plan_current.read();
        aurora::robot_loc2D old3dCoord = exchange_plan_target.read();
        aurora::field_drivable currField = exchange_field_drivable.read();
        //Some logic to determine what are the dive values
        //Currently just creates empty object needs some data?
        aurora::drive_commands newDrive;
        //Set


        //writing new data to files:
        exchange_drive_commands.write_begin() = newDrive;
        exchange_drive_commands.write_end();
       

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}
