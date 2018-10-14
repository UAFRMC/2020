/*
  Detect LunaBin location & orientation using Kinect sensor,
  then drive to commanded positions using that location.
  
  This is *exactly* the libfreenect/example/glview.c code, except
  I've added a glob of C++ in the middle to do more interesting 
  data analysis, and some motor driver stuff in the display loop.
  
  Dr. Orion Lawlor, lawlor@alaska.edu, 2013-05-16 (Public Domain)
*/


/*
 * This file was part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>

#include "osl/transform.h"
#include "osl/file_ipc.h"
#include "bitgrid_RMC.h"

#include "libfreenect.h"

#include <pthread.h>

#include "osl/vec4.h"

#if defined(__APPLE__)
#include <GLUT/glut.h>
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#else
#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#endif

#include <math.h>

#define VEC3_TO_XYZ(v) (v).x,(v).y,(v).z

vec3 upVec; // accelerometer-derived "up vector" estimate, kinect coords
float binConfidence=0.0; // 0.0: no idea where lunabin is.  1.0: lunabin in sight.
vec3 binNormal=vec3(1,0,0); // lunabin's outward-facing normal, in kinect coords
vec3 binLoc=vec3(0,0,0); // lunabin's last-seen location, in kinect coords

template <class T>
void blendInto(T &target,const T& newValue,float newWeight)
{
	target=newWeight*newValue+(1.0-newWeight)*target;
}


/*
  To use less CPU time, we decimate the depth output from the Kinect
  by this factor.  This is needed because the Kinect output 
  resolution seems to be fixed.
*/
enum {decimate=2};
enum {KINECT_w=640/decimate};
enum {KINECT_h=480/decimate};

//Video output size
enum {KINECT_vw=640};
enum {KINECT_vh=480};

/* If true, show the RGB image.  
   If false, only show depth (for a bit lower CPU usage). */
bool do_video=false;


pthread_t freenect_thread;
volatile int die = 0;

int g_argc;
char **g_argv;

int window;

pthread_mutex_t gl_backbuf_mutex = PTHREAD_MUTEX_INITIALIZER;

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"
uint8_t *depth_mid, *depth_front;
uint8_t *rgb_back, *rgb_mid, *rgb_front;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

freenect_context *f_ctx;
freenect_device *f_dev;
int freenect_angle = 0;
int freenect_led;

freenect_video_format requested_format = FREENECT_VIDEO_RGB;
freenect_video_format current_format = FREENECT_VIDEO_RGB;

pthread_cond_t gl_frame_cond = PTHREAD_COND_INITIALIZER;
int got_rgb = 0;
int got_depth = 0;

void DrawGLScene()
{
	glViewport(0,0,glutGet(GLUT_WINDOW_WIDTH),glutGet(GLUT_WINDOW_HEIGHT));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (0, KINECT_w, KINECT_h, 0, -1.0f, 1.0f);
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

	pthread_mutex_lock(&gl_backbuf_mutex);

	// When using YUV_RGB mode, RGB frames only arrive at 15Hz, so we shouldn't force them to draw in lock-step.
	// However, this is CPU/GPU intensive when we are receiving frames in lockstep.
	if (current_format == FREENECT_VIDEO_YUV_RGB) {
		while (!got_depth && !got_rgb) {
			pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
		}
	} else {
		while ((!got_depth || !got_rgb) && requested_format != current_format) {
			pthread_cond_wait(&gl_frame_cond, &gl_backbuf_mutex);
		}
	}

	if (requested_format != current_format) {
		pthread_mutex_unlock(&gl_backbuf_mutex);
		return;
	}

	uint8_t *tmp;

	if (got_depth) {
		tmp = depth_front;
		depth_front = depth_mid;
		depth_mid = tmp;
		got_depth = 0;
	}
	if (got_rgb) {
		tmp = rgb_front;
		rgb_front = rgb_mid;
		rgb_mid = tmp;
		got_rgb = 0;
	}

	pthread_mutex_unlock(&gl_backbuf_mutex);

	glDisable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	glTexImage2D(GL_TEXTURE_2D, 0, 3, KINECT_w, KINECT_h, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_front);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 0); glVertex3f(0,0,0);
	glTexCoord2f(1, 0); glVertex3f(KINECT_w,0,0);
	glTexCoord2f(1, 1); glVertex3f(KINECT_w,KINECT_h,0);
	glTexCoord2f(0, 1); glVertex3f(0,KINECT_h,0);
	glEnd();

if (do_video) {
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glEnable(GL_TEXTURE_2D);
	glPixelStorei(GL_UNPACK_ALIGNMENT,1);
	if (current_format == FREENECT_VIDEO_RGB || current_format == FREENECT_VIDEO_YUV_RGB)
		glTexImage2D(GL_TEXTURE_2D, 0, 3, KINECT_vw, KINECT_vh, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_front);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, 1, KINECT_vw, KINECT_vh, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, rgb_front+KINECT_vw*4);

	glBegin(GL_TRIANGLE_FAN);
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glTexCoord2f(0, 0); glVertex3f(KINECT_w,0,0);
	glTexCoord2f(1, 0); glVertex3f(KINECT_w*2,0,0);
	glTexCoord2f(1, 1); glVertex3f(KINECT_w*2,KINECT_h,0);
	glTexCoord2f(0, 1); glVertex3f(KINECT_w,KINECT_h,0);
	glEnd();
	
}

	glDisable(GL_TEXTURE_2D);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glViewport(0,0,glutGet(GLUT_WINDOW_WIDTH),glutGet(GLUT_WINDOW_HEIGHT));
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho (-0.2, 8.0, /* LunArena X dimensions, plus a buffer zone */
	       -2.0, +2.0f,  // Y 
		   -10.0, +10.0f); // Z
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	double time=0.001*glutGet(GLUT_ELAPSED_TIME);
	glTranslatef(4.0,0.0,0.0);
	glRotatef(-10.0, 0.0,1.0,0.0); // slight 3D rotation
	glTranslatef(-4.0,0.0,0.0);
	
	glLineWidth(8.0); // fat white lines for important stuff
	glBegin(GL_LINES);
	// lunabin
	glColor4f(0.0f, 0.0f, 0.0f, 0.5f); // shadow
	glVertex3fv(vec3(0.0,-1.65/2.0,0.0)); 
	glVertex3fv(vec3(0.0,+1.65/2.0,0.0));
	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glVertex3fv(vec3(0.0,-1.65/2.0,0.5)); 
	glVertex3fv(vec3(0.0,+1.65/2.0,0.5));
	glEnd();
	
	glutSwapBuffers();
	usleep(40*1000); // run every 40ms
	glutPostRedisplay();
}

void keyPressed(unsigned char key, int x, int y)
{
	if (key == 27) {
		die = 1;
		pthread_join(freenect_thread, NULL);
		glutDestroyWindow(window);
		free(depth_mid);
		free(depth_front);
		free(rgb_back);
		free(rgb_mid);
		free(rgb_front);
		// Not pthread_exit because OSX leaves a thread lying around and doesn't exit
		exit(0);
	}
	if (key == 'w') {
		freenect_angle++;
		if (freenect_angle > 30) {
			freenect_angle = 30;
		}
	}
	if (key == 's') {
		freenect_angle = 0;
	}
	if (key == 'f') {
		if (requested_format == FREENECT_VIDEO_IR_8BIT)
			requested_format = FREENECT_VIDEO_RGB;
		else if (requested_format == FREENECT_VIDEO_RGB)
			requested_format = FREENECT_VIDEO_YUV_RGB;
		else
			requested_format = FREENECT_VIDEO_IR_8BIT;
	}
	if (key == 'x') {
		freenect_angle--;
		if (freenect_angle < -30) {
			freenect_angle = -30;
		}
	}
	if (key == '1') {
		freenect_set_led(f_dev,LED_GREEN);
	}
	if (key == '2') {
		freenect_set_led(f_dev,LED_RED);
	}
	if (key == '3') {
		freenect_set_led(f_dev,LED_YELLOW);
	}
	if (key == '4') {
		freenect_set_led(f_dev,LED_BLINK_GREEN);
	}
	if (key == '5') {
		// 5 is the same as 4
		freenect_set_led(f_dev,LED_BLINK_GREEN);
	}
	if (key == '6') {
		freenect_set_led(f_dev,LED_BLINK_RED_YELLOW);
	}
	if (key == '0') {
		freenect_set_led(f_dev,LED_OFF);
	}
	freenect_set_tilt_degs(f_dev,freenect_angle);
}

void ReSizeGLScene(int Width, int Height)
{
}

void InitGL(int Width, int Height)
{
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClearDepth(1.0);
	glDepthFunc(GL_LESS);
    glDepthMask(GL_FALSE);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_BLEND);
    glDisable(GL_ALPHA_TEST);
    glEnable(GL_TEXTURE_2D);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_FLAT);

	glGenTextures(1, &gl_depth_tex);
	glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	glGenTextures(1, &gl_rgb_tex);
	glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	ReSizeGLScene(Width, Height);
}

void *gl_threadfunc(void *arg)
{
	printf("GL thread\n");

	glutInit(&g_argc, g_argv);

	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
	//glutInitWindowSize(640/decimate, 480/decimate);
  glutInitWindowSize(640, 480);
		glutInitWindowPosition(0, 0);

	window = glutCreateWindow("Kinect Localization");

	glutDisplayFunc(&DrawGLScene);
	// glutIdleFunc(&DrawGLScene); // only requested repaints...
	glutReshapeFunc(&ReSizeGLScene);
	glutKeyboardFunc(&keyPressed);

	InitGL(640, 480);

	glutMainLoop();

	return NULL;
}

uint16_t t_gamma[2048];


/***************** The GOOD STUFF ****************/

enum {KINECT_bad=0x7fe};

/**
  This class is used to get 3D positions from kinect depths.
  
  Camera-style coordinate system:
    Origin is Kinect's IR receiver
	+X faces to the left (from Kinect's point of view)
    +Y is up
	+Z is away from Kinect
*/
class kinect_depth_image {
public:
	const uint16_t *depthi;
	enum {NATIVE_KINECT_w=640};
	int w,h; /* dimensions of image */
	/* Unit-depth field of view offset per X or Y pixel */
	float pixelFOV;
	
	kinect_depth_image(const uint16_t *d_,int w_,int h_) 
		:depthi(d_), w(w_), h(h_) 
	{
		pixelFOV=tan(0.5 * (M_PI / 180.0) * 57.8)/(NATIVE_KINECT_w*0.5)*decimate;
	}

	/* Return raw data number at this pixel */
	int raw(int x,int y) const {
		return depthi[y*decimate*NATIVE_KINECT_w+x*decimate];
	}
	
	/* Return depth, in meters, at this pixel */
	float depth(int x,int y) const {
		uint16_t disp=depthi[y*decimate*NATIVE_KINECT_w+x*decimate];
		if (disp>KINECT_bad) return 0.0;
		return disp_to_depth(disp);
	}
	
	/* Given stereo disparity number, return depth in meters */
	float disp_to_depth(uint16_t disp) const {
		//From Stephane Magnenat's depth-to-distance conversion function:
		return 0.1236 * tan(disp / 2842.5 + 1.1863) - 0.037; // (meters)
	}
	
	/* Return 3D direction pointing from the sensor out through this pixel 
	   (not a unit vector, due to Kinect's projection) */
	vec3 dir(int x,int y) const {
		return vec3((w*0.5-x)*pixelFOV,(h*0.5-y)*pixelFOV,1.0);
	}
	
	/* Return 3D location, in meters, at this pixel */
	vec3 loc(int x,int y) const {
		// Project view ray out for that pixel
		return dir(x,y)*depth(x,y);
	}
};


/**
 This class classifies pixels as matching our target, or not.
*/
class kinectPixelWatcher {
public:
	kinect_depth_image &img;
	osl::transform sensor_tf;
	vec3 up;
	
	typedef rmc_navigator nav;
	
	// Grid cells are in one of these states:
	enum {
	  grid_unknown=0, grid_obstacle=1, grid_straddle=2, grid_driveable=3 
	};
	typedef nav::navigator_t::grid2D<unsigned char> gridstate_t;
	
	// Gridded temporary data:
	nav::navigator_t::grid2D<float> zmin, zmax; // Z height range (cm)
	nav::navigator_t::grid2D<float> zsum; // total Z's observed 
	nav::navigator_t::grid2D<unsigned short> zcount; // count of Z's observed
	
	// Gridded outputs:
	bitgrid driveable; // we can see this area is driveable (drive tracks here).  Even terrain.
	bitgrid straddle; // we can see this object can be straddled (drive over it between tracks).  Uneven terrain.
	bitgrid obstacle; // we can see this object is too high to be straddled.  High obstacle.
	
	kinectPixelWatcher(kinect_depth_image &img_,vec3 up_) 
		:img(img_), up(normalize(up_)) 
	{
	  zmin.clear(+10000.0);
	  zmax.clear(-10000.0);
	  zsum.clear(0.0);
	  zcount.clear(0);
	}
	
	
	// Debug outputs for this pixel.
	class debug_t {
	public:
		unsigned char r,g,b; // onscreen color
	};
	
	// Classify this pixel: 0 for bad, small number for close, >=10 for match.
	int classify_pixel(int x,int y,debug_t &debug,gridstate_t &gridstate);
	
	// Check grid cells for driveability based on Z:
	void classify_grid(gridstate_t &gridstate);
};

inline int kinectPixelWatcher::classify_pixel(int x,int y,debug_t &debug,gridstate_t &gridstate)
{
	const float min_up=-200.0; // cm along up vector to start search (below sensor)
	const float max_up=70; // cm along up vector to end search (above sensor)
	
	const float max_distance=6.0; //  meters to farthest possible target
	const float min_distance=0.5; // meters to closest possible target
	
	const int delx=9/decimate,dely=9/decimate; // pixel shifts for neighbor search	
	
	int raw=img.raw(x,y);
	if (raw>=KINECT_bad) return 0; // out of bounds

	vec3 loc=img.loc(x,y); // meters
	
	//float toFloor=loc.dot(up); // m to floor	
	float m=loc.mag(); // total distance
	
	// Compute global-coordinates position
	vec3 global=sensor_tf.global_from_local(vec3(loc.x,loc.z,loc.y)*100.0); // to cm

  // Add 1 meter grid lines to everything
	const float grid_mod=100.0; // draw 1 meter grid cells
	bool in_gridline=false;
	for (int axis=0;axis<3;axis++) 
	  if ((0xff&(int)(global[axis]*(256/grid_mod)))<10)
	    in_gridline=true;
	debug.b=in_gridline?255:0; // draw global grid (coordinate system debug)
	debug.r=global.z*(256.0/10.0); // Up vector, 10cm wrap
	
	int grid_x=global.x/rmc_navigator::GRIDSIZE;
	int grid_y=global.y/rmc_navigator::GRIDSIZE;
	if (!driveable.inbounds(grid_x,grid_y)) { // not inside the field--ignore it
	  debug.b >>= 2; debug.r >>= 2; // darken
	  return 10; // out of grid bounds
	}
	
	// Driveability from last grid state: 
	//    enums make bright green driveable, dimmer is straddle, dimmest is full obstacle
	debug.g=gridstate.at(grid_x,grid_y)*(255.0/3);
	
	if ( m<min_distance || m>max_distance) return 0; // too close or far
	if (global.z<min_up || global.z>max_up) return 1; // bad up vector distance
	
	// Update z range for this grid cell:
	zsum.at(grid_x,grid_y)+=global.z;
	zcount.at(grid_x,grid_y)++;
	float &lo=zmin.at(grid_x,grid_y);
	float &hi=zmax.at(grid_x,grid_y);
	if (lo>global.z) lo=global.z;
	if (hi<global.z) hi=global.z;

	return 5;
}

// Check grid cells for driveability based on Z:
void kinectPixelWatcher::classify_grid(gridstate_t &gridstate)
{
  int count_drive=0, count_straddle=0, count_obstacle=0;
  enum {nbor=1};
  for (int y=nbor;y<nav::GRIDY-nbor;y++)
  for (int x=nbor;x<nav::GRIDX-nbor;x++) {  
  
    // Find Z range over neighborhood
  	float lo=10000.0,hi=-10000.0;
    float sum=0.0; int count=0;
    for (int dy=-nbor;dy<=nbor;dy++)
    for (int dx=-nbor;dx<=nbor;dx++) {
      float nlo=zmin.at(x+dx,y+dy);
      float nhi=zmax.at(x+dx,y+dy);
      if (lo>nlo) lo=nlo;
      if (hi<nhi) hi=nhi;
      sum+=zsum.at(x+dx,y+dy);
      count+=zcount.at(x+dx,y+dy);
    }
    int state=grid_unknown;
    if (count>=7 && zcount.at(x,y)>3) 
    { // enough samples to be plausible:
      const float drive_range=6.0; // can drive over humps / holes this high (cm)
      const float straddle_range=18.0; // can straddle humps this high (cm)
      
      float mean=sum/count;
      float hump=hi-mean; // obstacle height (always >=0)
      float hole=mean-lo; // hole depth (always >=0)
      
      if (hump <= drive_range && hole <=drive_range) 
      {  // really flat--can put tracks here.
        state=grid_driveable;
        driveable.write(x,y,true);
        count_drive++;
      } else if (hump <= straddle_range) { // can straddle with mining head?
        state=grid_straddle;
        straddle.write(x,y,true);
        count_straddle++;
      } else { // hump >straddle_range: can't drive over it at all
        state=grid_obstacle;
        obstacle.write(x,y,true);
        count_obstacle++;
      }
    }
    gridstate.at(x,y)=state;
    
  }
  printf("Grid cells: %d driveable, %d straddleable, %d obstacle\n",
    count_drive,count_straddle,count_obstacle);
}

void depth_cb(freenect_device *dev, void *v_depth, uint32_t timestamp)
{
	int i;
	uint16_t *depth = (uint16_t*)v_depth;
	
	// Grab latest vive localization
	static osl::transform sensor_tf(vec3(2.0,1.0,0.4)); // default origin in middle of field
	
	static file_ipc_link<osl::transform> sensor_link("sensor.tf");
	sensor_link.subscribe(sensor_tf);

	kinect_depth_image img(depth,KINECT_w,KINECT_h);
	vec3 up(upVec.x,upVec.y,upVec.z); 
	// Kinect accelerometer calibration fixes (possibly device dependent)
	//  I picked these to make the floor be *flat*
	//up.x+=0.03; // correct a tilt to the left
	//up.z+=0.02; // correct weak back tilt
	up=normalize(up);
	kinectPixelWatcher watch(img,up);
	static kinectPixelWatcher::gridstate_t gridstate; // <- static to use frame coherence for display operation
	watch.sensor_tf=sensor_tf;
	
	pthread_mutex_lock(&gl_backbuf_mutex);

// #pragma omp parallel for schedule(dynamic,2) // <- very fast, but breaks region grower.
	for (int y=0;y<img.h;y++)
	for (int x=0;x<img.w;x++)
	{
		int i=x+img.w*y;

		kinectPixelWatcher::debug_t debug;
		debug.r=debug.g=debug.b=0;
		watch.classify_pixel(x,y,debug,gridstate);

		depth_mid[3*i+0]=debug.r;
		depth_mid[3*i+1]=debug.g;
		depth_mid[3*i+2]=debug.b;
	}
	got_depth++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
	
	watch.classify_grid(gridstate);
	
	static file_ipc_link<bitgrid> driveable_link("driveable.grid");  driveable_link.publish(watch.driveable);
	static file_ipc_link<bitgrid> straddle_link("straddle.grid");  straddle_link.publish(watch.straddle);
	static file_ipc_link<bitgrid> obstacle_link("obstacle.grid");  obstacle_link.publish(watch.obstacle);
}

/*********************** Back to verbatim libfreenect/examples/glview.c code ****************/

void rgb_cb(freenect_device *dev, void *rgb, uint32_t timestamp)
{
	pthread_mutex_lock(&gl_backbuf_mutex);

	// swap buffers
	assert (rgb_back == rgb);
	rgb_back = rgb_mid;
	freenect_set_video_buffer(dev, rgb_back);
	rgb_mid = (uint8_t*)rgb;

	got_rgb++;
	pthread_cond_signal(&gl_frame_cond);
	pthread_mutex_unlock(&gl_backbuf_mutex);
}

void *freenect_threadfunc(void *arg)
{
	int accelCount = 0;

//	freenect_set_tilt_degs(f_dev,freenect_angle);
	freenect_set_led(f_dev,LED_RED);
	freenect_set_depth_callback(f_dev, depth_cb);
	freenect_set_video_callback(f_dev, rgb_cb);
	freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, current_format));
	freenect_set_depth_mode(f_dev, freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_11BIT));
	freenect_set_video_buffer(f_dev, rgb_back);

	freenect_start_depth(f_dev);

if (do_video) {	freenect_start_video(f_dev); }

	printf("'w'-tilt up, 's'-level, 'x'-tilt down, '0'-'6'-select LED mode, 'f'-video format\n");

	while (!die && freenect_process_events(f_ctx) >= 0) {
		//Throttle the text output
		if (accelCount++ >= 10)
		{
			accelCount = 0;
			freenect_raw_tilt_state* state;
			freenect_update_tilt_state(f_dev);
			state = freenect_get_tilt_state(f_dev);
			double dx,dy,dz; // OSL: made global
			freenect_get_mks_accel(state, &dx, &dy, &dz);
			upVec=normalize(upVec+0.2*normalize(vec3(dx,dy,dz)));
			
			//printf("\r raw acceleration: %4d %4d %4d  mks acceleration: %4f %4f %4f", state->accelerometer_x, state->accelerometer_y, state->accelerometer_z, upVec.x, upVec.y, upVec.z);
			fflush(stdout);
		}

		if (do_video && requested_format != current_format) {
			freenect_stop_video(f_dev);
			freenect_set_video_mode(f_dev, freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, requested_format));
			freenect_start_video(f_dev);
			current_format = requested_format;
		}
		usleep(1000);
	}

	printf("\nshutting down streams...\n");

	freenect_stop_depth(f_dev);
	freenect_stop_video(f_dev);

	freenect_close_device(f_dev);
	freenect_shutdown(f_ctx);

	printf("-- done!\n");
	return NULL;
}

int main(int argc, char **argv)
{
	int res;

	//motors_setup();

	depth_mid = (uint8_t*)malloc(KINECT_w*KINECT_h*3);
	depth_front = (uint8_t*)malloc(KINECT_w*KINECT_h*3);
	rgb_back = (uint8_t*)malloc(KINECT_vw*KINECT_vh*3);
	rgb_mid = (uint8_t*)malloc(KINECT_vw*KINECT_vh*3);
	rgb_front = (uint8_t*)malloc(KINECT_vw*KINECT_vh*3);

	printf("Kinect camera test\n");

	int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}

	g_argc = argc;
	g_argv = argv;

	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}

	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));

	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);

	int user_device_number = 0;
	if (argc > 1)
		user_device_number = atoi(argv[1]);

	if (nr_devices < 1)
		return 1;

	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		return 1;
	}

	res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create failed\n");
		return 1;
	}

	// OS X requires GLUT to run on the main thread
	gl_threadfunc(NULL);

	return 0;
}
