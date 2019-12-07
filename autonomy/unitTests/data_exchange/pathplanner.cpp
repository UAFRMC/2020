// Basic skeleton read/write file for the pathplanner, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

int main() {
    //Data sources need to write to, these are defined by lunatic.h for what files we will be talk on
    aurora::data_exchange<aurora::drive_commands> exchange_drive("backend.drive");

    //Data source needed to read from, these are defined by lunatic.h for what files we will be talk on
    aurora::data_exchange<aurora::robot_loc2D> exchange_plan_current("plan_current.loc2D");
    aurora::data_exchange<aurora::robot_coord3D> exchange_plan_target("plan_target.loc2D");

    while (true) {
        aurora::robot_loc2D old2dCoord = exchange_plan_current.read();
        aurora::robot_coord3D old3dCoord = exchange_plan_target.read();

        //Some logic to determine what are the dive values
        //Currently just creates empty object needs some data?
        aurora::drive_commands newDrive;



        //writing new data to files:
        exchange_drive.write_begin() = newDrive;
        exchange_drive.write_end();
       

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}
