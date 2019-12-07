// Basic skeleton read/write file for the localizer, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

int main() {
    typedef std::array<aurora::vision_marker_report, aurora::vision_marker_report::max_count> vision_marker_reports;
    //Data sources need to read from, these are defined by lunatic.h for what files we will be talk on
    MAKE_exchange_drive_commands();
    MAKE_exchange_drive_encoders();
    MAKE_exchange_stepper_report();
    MAKE_exchange_marker_reports();

    //Data source needed to write too, these are defined by lunatic.h for what files we will be talk on
    MAKE_exchange_obstacle_view();
    MAKE_exchange_plan_current();

    while (true) {
        aurora::drive_commands currentdrive = exchange_drive_commands.read();
        aurora::drive_encoders currentencode = exchange_drive_encoders.read();
        aurora::stepper_pointing currentstepper = exchange_stepper_report.read();
        aurora::vision_marker_reports currentvision = exchange_marker_reports.read();


        //Some logic to determine what our next plan of movement is?
        //Currently just creates empty objects needs some data?
        aurora::robot_loc2D new2dcords;
        aurora::robot_coord3D new3dcords;



        //writing new data to files:
        exchange_plan_current.write_begin() = new2dcords;
        exchange_plan_current.write_end();

        exchange_plan_target.write_begin() = new3dcords;
        exchange_plan_target.write_end();

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}
