/* Debug print a stepper command or report */
#include "aurora/lunatic.h"

int main(int argc,const char *argv[]) {
    const char *filename="stepper_request.angle";
    if (argc>1) {
        filename=argv[1];
        // Trim pathname up to trailing slash, so you can pass
        //    /tmp/data_exchange/foo.angle
        const char *lastslash = strrchr(filename,'/');
        if (lastslash) filename = lastslash+1;
    }
    
    aurora::data_exchange<aurora::stepper_pointing> exchange_stepper(filename);
    
    while (true) {
        if (exchange_stepper.updated()) printf("+");
        exchange_stepper.read().print();
        
        aurora::data_exchange_sleep(100);
    }
}


