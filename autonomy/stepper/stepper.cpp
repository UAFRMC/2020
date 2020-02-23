// Basic skeleton file for stepper data exchange.
//stepper talks to the camera pointing stepper motor
// Reads requested view angle from backend
// Publishes current view angle to the localizer

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include "../include/serial.h"
#include "../include/serial.cpp"
#include "../include/aurora/data_exchange.h"
#include "../include/aurora/stepper.h"
#include "../include/aurora/lunatic.h"

//SerialPort Serial;

bool nanoComm(float loc, float & curLoc)
{
    static int MIL = 1000000; // to convert microseconds to seconds

    int count = 0;
    bool leave = false, once = false, ret = false;
    std::string received;
    std::string received_str = "";

    std::stringstream ss;
    ss << std::fixed << std::setprecision(2) << loc;
    std::string tmp = ss.str(); // float to string, precision 2
    char cstr[tmp.length() + 1];
    strcpy(cstr, tmp.c_str()); // string to char array

    while(Serial.Is_open())
    {
        if(!once)
        {
            usleep(1 * MIL); // must wait for nano
            Serial.write(cstr);
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
                    received_str = "";
                }
                else if(received_str.substr(0,21) == "RECEIVED: New Angle (")
                {
                    received_str = "";
                    continue;//leave = true;
                }
                else if(received_str.substr(0,21) == "CONFIRM: Curr Angle (")
                {
                    std::string newDeg = received_str.substr(21, received_str.length());
                    newDeg = newDeg.substr(0, newDeg.length() - 2);

                    curLoc = std::stof(newDeg);
                    received_str = "";
                    leave = true;
                }
            }
            else // end of packet
            {
                received_str += received;
            }
        }

        ++count;

        if(leave)
        {
            ret = true;
            break;
        }
        else if(count == 15 * MIL)
        {
            break;
        }
    }
    std::cout << std::endl;

    return ret;
}

int main(int argc, char * argv[])
{
    //Data sources need to write to, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_report();
    //Data source needed to read from, these are defined by lunatic.h for what files we will be communicating through
    MAKE_exchange_stepper_request();

    Stepper spyglass;
    bool res;
    float curLoc = 0.0;
    bool do_serial_comms = true;

    if (argc > 1)
    {
        if(0==strcmp(argv[1],"--sim"))
        {   std::cout<<"SIM" <<std::endl;
            do_serial_comms = false;
        }
            
    }

    if(do_serial_comms)
    {
        Serial.begin(115200);
        if(Serial.Is_open())
        {
            std::cout << "<Serial  Port Ready>" << std::endl;
        }
    }
    aurora::stepper_pointing reqDir;
    while (true)
    {
        if(exchange_stepper_request.updated()){
            reqDir = exchange_stepper_request.read();
        }
        spyglass.loc = reqDir.angle; // where and how should we be setting this?
        if(do_serial_comms)
        {
            res = nanoComm(spyglass.loc, curLoc);
        }
        else
        {
            res = true;
        }
        
        // curLoc -- where is it needed?

        if(res)
        {
            std::cout << "SUCCESS\n\n" << std::endl;
            aurora::stepper_pointing newDir;
            //writing new data to files:
            newDir.angle = curLoc; // is this where the stepper's current location is supposed to go?
            //newDir.stable = ? // what's this?

            exchange_stepper_report.write_begin() = newDir;
            exchange_stepper_report.write_end();
        }
        else
        {
            std::cout << "FAILED\n\n" << std::endl;
        }

        aurora::stepper_pointing oldDir = exchange_stepper_request.read();

        aurora::data_exchange_sleep(10);
    }

    return 0;
}