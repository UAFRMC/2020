/*
 Reads from /tmp/data_exchange, and writes to stdout.
 Redirect this output to a file to save the data_exchange:
    exchange_read /tmp/data_exchange/ * > robot_test3.xcg
*/
#include "exchange_datatypes.h"

int main(int argc,char *argv[]) {
    std::vector<std::string> filenames;
    for (int i=1;i<argc;i++)
        filenames.push_back(argv[i]);
    
    FILE *f=stdout;
    exchange_send(filenames,f);
    return 0;
}

