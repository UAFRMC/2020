/*
Renders the Mandelbrot Set fractal to the left half of the framebuffer.
Dr. Orion Lawlor, lawlor@alaska.edu, 2019-11-19 (Public Domain)

Linux framebuffer example program, originally from the Qt Qtopia Core examples:
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
    unsigned short g:6; // green
    unsigned short r:5; // red
#endif
};

/* Draw one Mandelbrot set pixel */
float center_x=0.3576, center_y=0.648;
float zoom=1.0;

inline void draw_pixel(pixel *image,int x,int y,int wid,int ht) {
    int i=y*wid+x;
	float fx=center_x+zoom*(x*(2.0f/ht)-1.0);
    float fy=center_y+zoom*(y*(2.0f/ht)-1.0);
	float scale=1.0; // amount of the mandelbrot set to draw
	fx*=scale; fy*=scale;
	
	float ci=fy, cr=fx; // complex constant: x,y coordinates
	float zi=ci, zr=cr; // complex number to iterate
	int iter;
	for (iter=0;iter<50;iter++) {
		if (zi*zi+zr*zr>4.0f) break; // number too big--stop iterating
		// z = z*z + c
		float zr_new=zr*zr-zi*zi+cr;
		float zi_new=2.0f*zr*zi+ci;
		zr=zr_new; zi=zi_new;
	}
	
	image[i].r=zr*RMAX/4.0;
	image[i].g=zi*GMAX/4.0;
	image[i].b=iter;
}

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
    for (zoom=2.0;zoom!=0.0f;zoom*=0.95) //<- animate!
    for (int y = 0; y < ht; y++)
        for (int x = 0; x < wid/2; x++) {
            draw_pixel(framebuffer,x,y,wid,ht);
        }
    
    
    // Clean up
    munmap(framebuffer, screensize);
    close(fbfd);
    return 0;
}
