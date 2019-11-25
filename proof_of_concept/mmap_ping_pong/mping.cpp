/**
 Proof of concept: exchange data between processes by mmaping the same file.
*/
#include <iostream>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <fcntl.h>

int main(int argc,char *argv[]) {
    int fd=open("exchange.bin",O_RDWR);
    volatile int *shared_area=(volatile int *)mmap(0,4096,
        PROT_READ|PROT_WRITE, MAP_SHARED,
        fd,0);
    int me=atoi(argv[1]);
    int count=atoi(argv[2]);
    printf("Exchanging %d at pointer %p\n", me,shared_area);
    for (int repeat=0;repeat<count;repeat++)
    {
        *shared_area=me;
        // Wait for another process to write to the area
        while (*shared_area == me) {}
    }
    return 0;
}


