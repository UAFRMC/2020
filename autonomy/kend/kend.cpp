// Basic skeleton file for bac-kend data exchange.
// BAC: Basic Autonomy Control 
// manages the autonomy state machine and pilot comms (over UDP)
// KEND: Keep Electronics Not Dying 
// talks to the Arduino, and restarts it if needed.

#include <iostream>
#include <stdio.h>

#include "aurora/lunatic.h"
#include "serial.cpp"
#include "cyberalaska/serial_packet.h"


int main(int argc, char * argv[]) 
{
    std::string serial_port = "/dev/ttyUSB0"; // default nano serial port
    int n=0; // number of our nano
    for (int i=1;i<argc;i++)
    {
        if(0==strcmp(argv[i],"--dev"))
        {
            serial_port = argv[++i];
        }
        else if(0==strcmp(argv[i],"--nano"))
        {
            n = atoi(argv[++i]);
        }
        else {
            printf("Unknown command line argument '%s'\n",argv[i]);
            return 1;
        }
    }

    Serial.Open(serial_port);
    Serial.Set_baud(115200);
    if(Serial.Is_open())
    {
        std::cout << "Opened serial port, waiting for bootloader" << std::endl;
        usleep(2 * 1000000);  // wait through bootloader
    }
    A_packet_formatter<SerialPort> pkt(Serial);
    
    // We exchange data with the nanos:
    MAKE_exchange_nano_net();

    bool first_time=true;
    int print_count=0;
    int send_wait=5;
    while (true) {
        // Read setup and command data
        bool exchange_updated=exchange_nano_net.updated();
        aurora::nano_net_data nano=exchange_nano_net.read();
        
        // Consider sending a setup packet
        if (first_time || nano.sensor[n].nosetup) 
        { 
            first_time=false;
            pkt.write_packet(0xB,sizeof(nano.setup[n]),&nano.setup[n]);
            aurora::data_exchange_sleep(100); // <- don't spam these
        }
        
        // Consider sending a command packet
        if (exchange_updated || (--send_wait < 0)) { // new data to send to nano
            pkt.write_packet(0xC,sizeof(nano.command[n]),&nano.command[n]);
            send_wait=5; // don't spam nano unless the backend sends updates
        }
        
        // Receive data back from nano
        A_packet p;
        while (-1==pkt.read_packet(p)) {}
        if (p.valid) {
            if (p.command==0xE) {
                printf("Nano hit error 0xE: %s\n",(char *)p.data);
            }
            else if (p.command==0x0) {
                printf("Nano sent ping request\n");
                send_wait=0; //<- they're ready for a command packet
            }
            else if (p.command==0x5) { // nano sent sensor data
                if (!p.get(exchange_nano_net.write_begin().sensor[n]))
                {
                    printf("Nano sent weird sensor packet of length %d\n",p.length);
                }
                exchange_nano_net.write_end();
            }
            else {
                printf("Nano sent unknown packet type %02x / length %d\n",
                    p.command,p.length);
            }
        }
        
        // Periodically print the data coming and going
        if ((++print_count%32)==0)
        {
            nano.print(n);
            printf("\n"); fflush(stdout);
        }
        
        // Limit this loop to 200Hz (5ms/loop)
        aurora::data_exchange_sleep(5);
    }
    return 0;
}
