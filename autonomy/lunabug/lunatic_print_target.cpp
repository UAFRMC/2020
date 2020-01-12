/* Debug print the autonomous navigation target */
#include "aurora/lunatic.h"
#include <iostream>
#include <iomanip>
int main() {
    MAKE_exchange_plan_target();
    
    while (true) {
        if (exchange_plan_target.updated()) {
            aurora::robot_loc2D loc2D= exchange_plan_target.read();
            loc2D.print();
        }
        
        aurora::data_exchange_sleep(100);
    }
}

