// Basic skeleton file for bac-kend data exchange.
// BAC: Basic Autonomy Control 
// manages the autonomy state machine and pilot comms (over UDP)
// KEND: Keep Electronics Not Dying 
// talks to the Arduino, and restarts it if needed.

#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

int main() {
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_drive_encoders();
    MAKE_exchange_stepper_request();
    MAKE_exchange_plan_target();

    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_drive_commands();

    while (true) {
        // Reading current drive commands to command drive control
        aurora::drive_commands currentDrive = exchange_drive_commands.read();

        aurora::drive_encoders newDriveEncode;
        // newDriveEncode.left = ?;
        // newDriveEncode.right = ?;
        exchange_drive_encoders.write_begin() = newDriveEncode;
        exchange_drive_encoders.write_end();

        aurora::stepper_pointing newStepperPointing;
        // newStepperPointing.angle = ?;
        exchange_stepper_request.write_begin() = newStepperPointing;
        exchange_stepper_request.write_end();

        aurora::robot_loc2D plannedCoords;
        // plannedCoords.angle = ?;
        // plannedCoords.x = ?;
        // plannedCoords.y = ?;
        // plannedCoords.percent = ?;
        exchange_plan_target.write_begin() = plannedCoords;
        exchange_plan_target.write_end();

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}