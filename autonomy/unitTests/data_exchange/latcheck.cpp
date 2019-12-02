/* Write the time, in milliseconds, to this file. */
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"

int main() {
    typedef uint64_t millitime_t;
    aurora::data_exchange<millitime_t> millitime("millitime.u64");
    millitime_t previous = 0;
    while (true) {
        millitime_t currenttime = millitime.read();
        long realLat = currenttime - previous;
        if(realLat < 0 || realLat > 2)
            std::cout << realLat << std::endl;
        previous = currenttime;
    }
    return 0;
}

