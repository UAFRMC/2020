/* Write the time, in milliseconds, to this file. */
#include <iostream>
#include <stdio.h>
#include <time.h>
#include "aurora/data_exchange.h"

#define NANO_TO_MILLI 1000000UL

int main() {
    typedef uint64_t millitime_t;
    aurora::data_exchange<millitime_t> millitime("millitime.u64");
    
	while (true) {
    	struct timespec tp;
    	clock_gettime(CLOCK_REALTIME,&tp);
    	millitime_t millis = tp.tv_sec*1000UL + tp.tv_nsec/NANO_TO_MILLI;
    	static millitime_t start = millis;
	    millitime.write() = millis - start;
	    
	    // Don't hog the CPU, give up our timeslice
	    struct timespec sleeptime;
	    sleeptime.tv_sec=0;
	    sleeptime.tv_nsec=1*NANO_TO_MILLI;
	    nanosleep(&sleeptime,NULL);
	}
	return 0;
}

