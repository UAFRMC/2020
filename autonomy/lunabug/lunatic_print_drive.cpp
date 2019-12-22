/* Debug print the backend's drive commands */
#include "aurora/lunatic.h"

int main() {
    MAKE_exchange_drive_commands();
    
    while (true) {
        if (exchange_drive_commands.updated()) printf("+");
        exchange_drive_commands.read().print();
        
        aurora::data_exchange_sleep(100);
    }
}


