// Basic skeleton read/write files for the andretti, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

//TODO: Write logic for how andretti will drive us!?

int main()
{
    MAKE_exchange_drive_commands();
    MAKE_exchange_plan_current();
    MAKE_exchange_path_plan();

    while (true) {
        aurora::robot_loc2D current = exchange_plan_current.read();
        if(exchange_path_plan.updated()){
            aurora::path_plan currentPlan = exchange_path_plan.read();
        }
        aurora::drive_commands newDrive();
        //Logic here for this stuff to make a new drive command and 
        exchange_drive_commands.write_begin()=newDrive;
        exchange_drive_commands.write_end();

    }

    return 0;
}
