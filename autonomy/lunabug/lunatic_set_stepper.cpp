/* Debug tool: manually set the autonomy target position */
#include "aurora/lunatic.h"
#include <iostream>

int main(int argc,char *argv[]) {
    if (argc!=2) { 
        printf("Usage: set angle \n");
        exit(1);
    }
    MAKE_exchange_stepper_report();
    MAKE_exchange_stepper_request();
    aurora::stepper_pointing pointdir;
    pointdir.angle = atof(argv[1]);
    std::cout << "setting stepper angle to: " << pointdir.angle << "\n";
    exchange_stepper_request.write_begin() = pointdir;
    exchange_stepper_request.write_end();

    return 0;
}
