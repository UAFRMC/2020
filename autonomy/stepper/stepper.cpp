// Basic skeleton file for stepper data exchange.
//stepper talks to the camera pointing stepper motor
// Reads requested view angle from backend
// Publishes current view angle to the localizer

#include <iostream>
#include <unistd.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "../include/serial.h"
#include "../include/serial.cpp"
#include "../include/aurora/data_exchange.h"
#include "../include/aurora/lunatic.h"

//SerialPort Serial;

int main() {
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_report();
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_request();

    Serial.begin(115200);

    static int MIL = 1000000; // to convert microseconds to seconds

    bool leave = false, once = false;
    std::string received;
    std::string received_str = "";
    char str[] = "Hello world";

    Serial.Output_flush();

    while(Serial.Is_open())
    {
        if(!once)
        {
            std::cout << "<Serial  Port Ready>" << std::endl;
            usleep(1 * MIL); // must wait for nano
            Serial.write(str);
            once = true;
        }

        while(Serial.available() > 0)
        {
            received = Serial.read(); // one byte at a time

            if(received == "#")
            {
                std::cout << received_str << std::endl;

                if(received_str == "<Arduino Nano Ready>")
                {
                    std::cout << std::endl;
                }
                else if(received_str == "Welcome home")
                {
                    leave = true;
                }

                received_str = "";
            }
            else // end of packet
            {
                received_str += received;
            }
        }

        if(leave)
        {
            break;
        }
    }
    std::cout << std::endl;

    /*while (true) {
        aurora::stepper_pointing oldDir = exchange_stepper_request.read();

        aurora::stepper_pointing newDir;
        //writing new data to files:
        // newDir.angle = ?
        // newDir.stable = ?

        exchange_stepper_report.write_begin() = newDir;
        exchange_stepper_report.write_end();

        //Sleep? Forced latency?
        aurora::data_exchange_sleep(10);
    }*/

    return 0;
}

