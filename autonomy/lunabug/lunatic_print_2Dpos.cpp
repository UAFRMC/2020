/* Debug print the Localizers positional data */
#include "aurora/lunatic.h"
#include <iostream>
#include <iomanip>
int main() {
    MAKE_exchange_plan_current();
    
    while (true) {
        if (exchange_plan_current.updated()) {
            aurora::robot_loc2D loc2D= exchange_plan_current.read();
            loc2D.print();
        }
        
        aurora::data_exchange_sleep(100);
    }
}

