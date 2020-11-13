/*
 Reads from this file (or stdin), and writes files to /tmp/data_exchange.
*/
#include "exchange_datatypes.h"

int main(int argc,char *argv[]) {
    FILE *f=stdin;
    std::vector<std::string> filenames;
    exchange_recv(filenames,f);
    for (size_t i=0;i<filenames.size();i++)
        printf("Filename: '%s'\n",filenames[i].c_str());
    return 0;
}

