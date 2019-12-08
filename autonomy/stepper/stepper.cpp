// Basic skeleton file for stepper data exchange.
//stepper talks to the camera pointing stepper motor
// Reads requested view angle from backend
// Publishes current view angle to the localizer

#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

int main() {
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_report();
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_request();

    while (true) {
        aurora::stepper_pointing oldDiretion = exchange_stepper_request.read();

        aurora::stepper_pointing newDiretion;
        //writing new data to files:
        // newDiretion.angle = ?
        // newDiretion.stable = ?

        exchange_stepper_report.write_begin() = newDiretion;
        exchange_stepper_report.write_end();

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }
    return 0;
}

