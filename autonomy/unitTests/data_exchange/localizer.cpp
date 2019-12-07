// Basic skeleton read/write file for the localizer, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

int main() {
    //Data sources need to read from, these are defined by lunatic.h for what files we will be talk on
    aurora::data_exchange<aurora::drive_commands> exchange_drive("backend.drive");
    aurora::data_exchange<aurora::drive_encoders> exchange_encode("backend.encoders");
    aurora::data_exchange<aurora::stepper_pointing> exchange_stepper("stepper_report.angle");
    aurora::data_exchange<aurora::vision_marker_report> exchange_vision("vision_marker.reports");

    //Data source needed to write too, these are defined by lunatic.h for what files we will be talk on
    aurora::data_exchange<aurora::robot_loc2D> exchange_plan_current("plan_current.loc2D");
    aurora::data_exchange<aurora::robot_coord3D> exchange_plan_target("plan_target.loc2D");

    while (true) {
        aurora::drive_commands currentdrive = exchange_drive.read();
        aurora::drive_encoders currentencode = exchange_encode.read();
        aurora::stepper_pointing currentstepper = exchange_stepper.read();
        aurora::vision_marker_report currentvision = exchange_vision.read();


        //Some logic to determine what our next plan of movement is?
        //Currently just creates empty objects needs some data?
        aurora::robot_loc2D new2dcords;
        aurora::robot_coord3D new3dcords;



        //writing new data to files:
        exchange_plan_current.write_begin() = new2dcords;
        exchange_plan_current.write_end();

        exchange_plan_target.write_begin() = new3dcords;
        exchange_plan_target.write_end();\

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}
