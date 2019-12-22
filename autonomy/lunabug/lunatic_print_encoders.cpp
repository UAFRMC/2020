/* Debug print the backend's encoder output */
#include "aurora/lunatic.h"

int main() {
    MAKE_exchange_drive_encoders();
    
    aurora::drive_encoders last;
    last.left=last.right=0.0f;
    while (true) {
        if (exchange_drive_encoders.updated()) printf("+");
        aurora::drive_encoders cur=exchange_drive_encoders.read();
        
        aurora::drive_encoders change = cur - last;
        change.print();
        
        last=cur;
        
        aurora::data_exchange_sleep(100);
    }
}


