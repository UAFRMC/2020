/* Debug print the Localizers positional data */
#include "aurora/lunatic.h"
#include <iostream>
#include <iomanip>
int main(int argc,char *argv[]) {
    if (argc<=1) { 
        printf("Usage: lunatic_print_3Dpos <name of file to print>\n");
        exit(1);
    }
    const char *filename=argv[1];
    aurora::data_exchange<aurora::robot_coord3D> exchange(filename);    
    while (true) {
        if (exchange.updated()) {
            aurora::robot_coord3D loc3D= exchange.read();
            loc3D.print();
        }
        
        aurora::data_exchange_sleep(100);
    }
}

