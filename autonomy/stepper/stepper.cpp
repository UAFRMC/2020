// Basic skeleton file for stepper data exchange.
//stepper talks to the camera pointing stepper motor
// Reads requested view angle from backend
// Publishes current view angle to the localizer

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include "serial.h"
#include "serial.cpp"
#include "aurora/stepper.h"
#include "aurora/lunatic.h"

#include "nano_stepper/serial_byte.h"

int main(int argc, char * argv[])
{
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_report();
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_request();

    bool res;
    float curLoc = 0.0;
    bool do_serial_comms = true;
    std::string serial_port = "/dev/ttyUSB0";
    for (int i=1;i<argc;i++)
    {
        if(0==strcmp(argv[i],"--sim"))
        {   std::cout<<"SIM" <<std::endl;
            do_serial_comms = false;
        }
        else if(0==strcmp(argv[i],"--dev"))
        {
            serial_port = argv[++i];
        }
        else {
            printf("Unknown command line argument '%s'\n",argv[i]);
            return 1;
        }
    }

    if(do_serial_comms)
    {
        Serial.Open(serial_port);
        Serial.Set_baud(115200);
        //Serial.begin(115200);
        if(Serial.Is_open())
        {
            std::cout << "Opened serial port, waiting for bootloader" << std::endl;
            usleep(2 * 1000000);  // wait through bootloader
        }
    }

    bool first_time=true;
    while (true)
    {
        if (first_time) // or sent home?
        { // Send out a motion request
            Serial.write(BYTE_HOME);
            printf("Sending the stepper home\n");
            first_time=false;

            // Mark the stepper as moving in the exchange
            exchange_stepper_report.write_begin().stable=0;
            exchange_stepper_report.write_end();
        }
        else if (exchange_stepper_request.updated())
        { // Send out a motion request
            aurora::stepper_pointing reqDir = exchange_stepper_request.read();
            float angle=aurora::normalize_angle(reqDir.angle);
            Serial.write(deg2byte(angle));
            printf("Moving stepper to angle %.1f\n",angle);

            if (do_serial_comms) {
                // Mark the stepper as moving in the exchange
                exchange_stepper_report.write_begin().stable=0;
                exchange_stepper_report.write_end();
            } else { // sim mode
                reqDir.stable=1;
                exchange_stepper_report.write_begin()=reqDir;
                exchange_stepper_report.write_end();
            }
        }

        while (Serial.available()>0)
        { // Read back the nano's state, and post it to the data exchange
            serial_byte b=Serial.read();

            aurora::stepper_pointing newDir;
            newDir.angle=0;
            newDir.stable=0;
            if (b<=BYTE_MAX_ANGLE) { // stepper has reached a new position
                newDir.angle=byte2deg(b);
                newDir.stable=1;
                printf("Stepper has now reached angle %.1f\n",newDir.angle);
            }
            else if (b==BYTE_HOME) {
                printf("Arduino reports that it has reached HOME.\n");
            }
            else if (b==BYTE_OK) {
                printf("Arduino reports that it started up OK.\n");
            }
            else {  /* Error or weird from Arduino */
                printf("Arduino sent unknown / error byte: %02x\n",b);
            }

            // Post the new status to the data exchange
            exchange_stepper_report.write_begin() = newDir;
            exchange_stepper_report.write_end();
        }

        aurora::data_exchange_sleep(10);
    }

    return 0;
}
