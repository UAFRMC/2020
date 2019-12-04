/* Write the time, in milliseconds, to this file. */
#include <iostream>
#include <stdio.h>
#include <time.h>
#include "aurora/data_exchange.h"

int main() {
    typedef uint64_t millitime_t;
    aurora::data_exchange<millitime_t> exchange_millitime("millitime.u64");
    
    while (true) {
        struct timespec tp;
        clock_gettime(CLOCK_REALTIME,&tp);
        millitime_t millis = tp.tv_sec*1000UL + tp.tv_nsec/NANO_TO_MILLI;
        static millitime_t start = millis;
        
        exchange_millitime.write_begin() = millis - start;
        exchange_millitime.write_end();
        
        aurora::data_exchange_sleep(1);
    }
    return 0;
}

