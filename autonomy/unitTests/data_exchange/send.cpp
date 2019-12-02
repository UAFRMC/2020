/* Test program for data exchange */
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"

aurora::data_exchange<uint64_t> time_exchange("time.u64");

int main() {
    printf("Sender running.\n");
    return 0;
}

