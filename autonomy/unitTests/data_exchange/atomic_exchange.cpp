/* Write the time, in milliseconds, to this file. */
#include <iostream>
#include <stdio.h>
#include <time.h>
#include "aurora/data_exchange.h"

#include <atomic>
class atomic_exchange_test {
public:
    std::atomic<uint32_t> index;
};

int main(int argc,char *argv[]) {
    aurora::data_exchange<atomic_exchange_test> exch("atomic.u64");
    if (argc>1) {
        exch.write_begin().index=0;
        exch.write_end();
    }
    
    for (int i=0;i<100000000;i++) {
        exch.write_begin().index++;
        exch.write_end();
    }
    
    printf("Final index: %d\n", (int)exch.read().index);
    
    return 0;
}

