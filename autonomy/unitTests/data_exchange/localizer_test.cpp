/* Debug print the Localizers positional data */
#include "aurora/lunatic.h"
#include <iostream>
int main() {
    MAKE_exchange_plan_current();
    
    while (true) {
        if (exchange_plan_current.updated());
        aurora::robot_loc2D new2dcords = exchange_plan_current.read();
        std::cout << "The Current 2DCords are x:" << new2dcords.x << ", y:" << new2dcords.y << ", angle:";
        std::cout << new2dcords.angle << " and we are this precent sure:" << new2dcords.percent << std::endl;
        
        aurora::data_exchange_sleep(100);
    }
}

