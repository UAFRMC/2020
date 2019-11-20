/*
Linux framebuffer example program
Dr. Orion Lawlor, lawlor@alaska.edu, 2019-11-19 (Public Domain)

Originally from the Qt Qtopia Core examples:
https://cep.xray.aps.anl.gov/software/qt4-x11-4.2.2/qtopiacore-testingframebuffer.html

Compile with:
    g++ framebuffer.cpp -o fb
Run with:
    ./fb

On my 4k laptop, this only works inside an old-school terminal,
and it seems to think I'm at 800x600 even when running X in 4k.
*/

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#if 0  /* 32 bit pixels, for desktop */
# define PIXELBITS 32
# define RMAX 255
# define GMAX 255
# define BMAX 255
#else  /* 16 bit pixels, for raspberry pi / mobile */
# define PIXELBITS 16
# define RMAX 31
# define GMAX 63
# define BMAX 31
#endif

// This class matches how the pixels are actually stored in memory
struct pixel {
#if PIXELBITS == 32
    unsigned char b; // blue channel, 0 is black, 255 is bright
    unsigned char g; // green
    unsigned char r; // red
    unsigned char a; // alpha transparency (unused)
#else
    unsigned short b:5; // blue channel, 0 is black, 255 is bright
    unsigned short g:6; // green (some machines use 5 bits here)
    unsigned short r:5; // red
#endif
};

int main()
{
    // Open the framebuffer device file 
    int fbfd = open("/dev/fb0", O_RDWR);
    if (fbfd == -1) {
        perror("Error: cannot open framebuffer device");
        exit(1);
    }
    printf("The framebuffer device was opened successfully.\n");

    // Get fixed screen information
    struct fb_fix_screeninfo finfo;
    if (ioctl(fbfd, FBIOGET_FSCREENINFO, &finfo) == -1) {
        perror("Error reading fixed information");
        exit(2);
    }

    // Get variable screen information
    struct fb_var_screeninfo vinfo;
    if (ioctl(fbfd, FBIOGET_VSCREENINFO, &vinfo) == -1) {
        perror("Error reading variable information");
        exit(3);
    }

    printf("Resolution %dx%d, %dbpp, off +%dx%d\n", 
        vinfo.xres, vinfo.yres, vinfo.bits_per_pixel,
        vinfo.xoffset, vinfo.yoffset);
    if (vinfo.bits_per_pixel!=PIXELBITS) {
        printf("Sorry, we only support %d bit framebuffer\n",PIXELBITS);
        exit(4);
    }

    // Figure out the size of the screen in bytes
    long screensize = vinfo.xres * vinfo.yres * vinfo.bits_per_pixel / 8;
    long wid=finfo.line_length / sizeof(pixel);
    long ht=vinfo.yres;

    // Map the device to memory
    pixel *framebuffer = (pixel *)mmap(0, screensize, 
        PROT_READ | PROT_WRITE, MAP_SHARED, fbfd, 0);
    if ((long)framebuffer == -1) {
        perror("Error: failed to map framebuffer device to memory");
        exit(5);
    }

    // Figure out where in memory to put the pixel
    for (int y = 0; y < ht; y++)
        for (int x = 0; x < wid/2; x++) {
            long index = x+y*wid;
            pixel &p = framebuffer[index];
            p.r=x*RMAX/wid; // red == x
            p.g = GMAX - p.g; // invert green channel
            p.b=y*BMAX/ht; // blue == y
        }
    
    
    // Clean up
    munmap(framebuffer, screensize);
    close(fbfd);
    return 0;
}
