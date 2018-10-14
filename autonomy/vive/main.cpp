//Data recorder mod with GUI showing light positions.
/*
 Modded by OSL 2017-04-16 to try to figure out calibration for RMC configuration:
  Emitters facing in the same direction, 1.5 meters apart.
 

*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include <string.h>
#include <os_generic.h>

#include "osl/file_ipc.h"
#include "osl/transform.h"
#include <vector>

#include "src/survive_cal.h"
#include <CNFGFunctions.h>
#ifdef __unix__
#include <unistd.h>
#endif

#include "src/survive_config.h"


/**
  Return the current time in seconds (since something or other).
*/
#if defined(_WIN32)
#  include <sys/timeb.h>
#  define time_in_seconds_granularity 0.1 /* seconds */
double time_in_seconds(void) { /* This seems to give terrible resolution (60ms!) */
        struct _timeb t;
        _ftime(&t);
        return t.millitm*1.0e-3+t.time*1.0;
}
#else /* UNIX or other system */
#  include <sys/time.h> //For gettimeofday time implementation
#  define time_in_seconds_granularity 0.01 /* seconds */
double time_in_seconds(void) {
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_usec*1.0e-6+tv.tv_sec*1.0;
}
#endif

#include "osl/vec4.h"



void survive_vec3_print(const char *heading, vec3 v)
{
  printf("%s: ( %.5f %.5f %.5f )\n", heading, v[0], v[1], v[2]);
}



/* A coordinate frame made from three unit vectors */
typedef struct SurviveObjectOrientation {
  // Orientation matrix (orthogonal unit vectors)
  vec3 x,y,z;
  
} SurviveObjectOrientation;

// Initialize this orientation to the identity
void survive_orient_init(SurviveObjectOrientation *o) {
  o->x=vec3(1,0,0);
  o->y=vec3(0,1,0);
  o->z=vec3(0,0,1);
}

void survive_orient_print(SurviveObjectOrientation *o) {
  survive_vec3_print("   X",o->x);
  survive_vec3_print("   Y",o->y);
  survive_vec3_print("   Z",o->z);
}

// Convert local coordinates out into global coordinates
vec3 survive_orient_global_from_local(SurviveObjectOrientation *o, vec3 local)
{
  return local.x*o->x + local.y*o->y + local.z*o->z;
}

// Convert global coordinates down into local coordinates
vec3 survive_orient_local_from_global(SurviveObjectOrientation *o, vec3 global)
{
  return vec3(dot(global,o->x), dot(global,o->y), dot(global,o->z));
}


// Apply these incremental rotation angles to this orientation
//    angle.x rotates about the x axis, etc.
//    angle vector is in local coordinates, radians, and right handed
void survive_orient_rotate(SurviveObjectOrientation *o, vec3 angle)
{
  o->z+= angle.y*o->x - angle.x*o->y;
  o->y+=-angle.z*o->x;
	o->x=normalize(cross(o->y,o->z));
	o->y=normalize(cross(o->z,o->x));
	o->z=normalize(o->z);
}

// Nudge our coordinate frame to make these two vectors equal:
//    local_measured (local coordinates) is what it's reading
//    global_should is what it should read
//    strength is how fast it should get there (0-1)
void survive_orient_nudge(SurviveObjectOrientation *o, vec3 local_measured, vec3 global_should, FLT strength)
{
  local_measured=normalize(local_measured);
  vec3 local_should=normalize(survive_orient_local_from_global(o,global_should));
  vec3 shift=strength*(local_measured-local_should); // rotation force
  float max_shift=0.2; // maximum radian rotation to ever apply
  if (length(shift)>max_shift) shift=max_shift*normalize(shift);
  vec3 rotate=cross(shift,normalize(local_measured)); // torque vector
  
  // Rotate to apply that torque
  survive_orient_rotate(o,rotate);
}



/* The hardware configuration of a tracked sensor */
typedef struct SurviveSensorHardware {
  // The sensor's position, relative to the IMU
  vec3 position; 
  
  // The sensor's facing normal vector
  vec3 normal;
} SurviveSensorHardware;

/* Keeps track of the simulated position of the sensor */
typedef struct SurviveSensorTracker {
  
  // Angle, in radians, of last detection by this lighthouse.
  //   These are re-zeroed every integration step.
  double angle[NUM_LIGHTHOUSES][2];
  
  // Time, in seconds, when the flash was visible
  double length[NUM_LIGHTHOUSES][2];
  
} SurviveSensorTracker;

/* Simulates position of a tracked object */
typedef struct SurviveObjectSimulation {
  // time_in_seconds at our last integration step.  
  //    Used to compute timesteps.
  double last_integrate_time;
  
  // Position of the lighthouses in global coordinates
  //    FIXME: for multi-object tracking, this needs to be a shared pointer.
  vec3 lighthouse_position[NUM_LIGHTHOUSES];
  
  // Orientation of the lighthouses in global coordinates
  //    FIXME: for multi-object tracking, this needs to be a shared pointer.
  SurviveObjectOrientation lighthouse_orient[NUM_LIGHTHOUSES];
  
  // Position of our center in global coordinates (meters)
  vec3 position;
  
  // Velocity of our center in global coordinates (m/s)
  vec3 velocity;
  
  // Our orientation in global coordinates (orient.x points along our X axis)
  SurviveObjectOrientation orient;
  // IMU timesteps are faster:
  double last_imu_time;
  
  // Down vector, measured in local orient frame
  vec3 down;

  // Number of sensors actually present in these arrays:
  int nsensor;
  
  // Static info about the sensor hardware  
  const SurviveSensorHardware *hardware;
  
  // Dynamic tracking info for each sensor
  SurviveSensorTracker sensor[SENSORS_PER_OBJECT];
  
} SurviveObjectSimulation;


SurviveObjectSimulation *survive_sim_init(const char *type) {
  double now=time_in_seconds();
  SurviveObjectSimulation *o=(SurviveObjectSimulation *)malloc(sizeof(SurviveObjectSimulation));
  
  o->last_integrate_time=now;
  
  // HACK: hardcode lighthouse positions for robot setup.
  o->lighthouse_position[0]=vec3(0,0,0);
  o->lighthouse_position[1]=vec3(1.5,0,0);
  survive_orient_init(&o->lighthouse_orient[0]);
  survive_orient_init(&o->lighthouse_orient[1]);
  
  o->position=vec3(0.0,1.0,0);
  o->velocity=vec3(0,0,0);
  survive_orient_init(&o->orient);
  // flip initial estimate upside down, since controller Z+ is world Z-
  o->orient.z*=-1.0; o->orient.y*=-1.0;
  
  o->last_imu_time=now;
  o->down=vec3(0,0,0);
  
  const static SurviveSensorHardware hardware[]={
#include "survive_models/controller_cal.h"
  };
  o->hardware=hardware;
  
  // Zero out the sensors:
  for (int S=0;S<SENSORS_PER_OBJECT;S++)
    for (int L=0;L<NUM_LIGHTHOUSES;L++)
      for (int A=0;A<2;A++)
        o->sensor[S].angle[L][A]=0.0;
  
  return o;
}

// Return the global coordinates normal of the plane swept by
//   lighthouse L's axis A at angle ang radians.
vec3 survive_lighthouse_normal(SurviveObjectSimulation *o,int L,int A,FLT ang)
{
  vec3 N;
  FLT s=sin(ang), c=cos(ang);
  if (A==0) { // Sweeping across X axis
    N=vec3(-c,-s,0.0);
  } else { // Y axis sweep
    N=vec3(0.0,-s,c);
  }
  return survive_orient_global_from_local(&o->lighthouse_orient[L],N);
}

// Integrate the collected sensor sweep data
void survive_sim_integrate(SurviveObjectSimulation *o) {
  double now=time_in_seconds();
  double dt=now-o->last_integrate_time;
  o->last_integrate_time=now;
  
  
  // Update predicted global position based on velocity
  vec3 nextP=o->position; // +dt*o->velocity;    //  (velocity is worse than useless)
  
  // Iteratively update position and orientation based on sensed angles
  enum {NPASS=5};
  float last_avg_err=10.0;
  for (int pass=0;pass<NPASS;pass++) {
    vec3 sum_motion=vec3(0.0); // linear residual, global coordinates
    float n_motion=0.0;
    
    vec3 sum_torque=vec3(0.0); // rotational residual, global coordinates
    float n_torque=0.0;
    
    int n_lighthouse[NUM_LIGHTHOUSES]={0};
    
    float tot_err=0.0; int n_err=0; int n_outlier=0;
    //if (pass==NPASS-1) printf("Per-sensor error (m): ");
    for (int L=0;L<NUM_LIGHTHOUSES;L++)
      for (int A=0;A<2;A++)
      {
#define ANGULAR_CHECK 1
#if ANGULAR_CHECK
      // Angular sweep speed check:
      //   predicted = geometric predicted angles
      //   sensed = observed angles
        float all_predicted[SENSORS_PER_OBJECT];
        float all_sensed[SENSORS_PER_OBJECT];
        int n_sensed=0;
#endif

      // float sensor_err=0.0; int n_sensor=0;
        for (int S=0;S<SENSORS_PER_OBJECT;S++)
        {
          FLT a=o->sensor[S].angle[L][A];
          if (a==0.0) continue;  // no data
          if (a<-1.5 || a>1.5) throw "Invalid angle";
          
          // Origin of sweep plane is at point where sweep axes cross:
          vec3 LP=o->lighthouse_position[L];
          // Normal of sweep plane
          vec3 LN=survive_lighthouse_normal(o,L,A,a);
           
          // Expected global sensed location is at our origin plus the sensor position offset
          vec3 sensor_offset=survive_orient_global_from_local(&o->orient,o->hardware[S].position);
          vec3 E=nextP+sensor_offset;
          
          // Detected location is the projection D of E onto the (P,N) plane:
          //  We want dot(D-LP,LN)=0
          FLT err=dot(E-LP,LN);  
          
          float ferr=fabs(err);
          tot_err+=ferr; n_err++; // sensor_err+=ferr; n_sensor++;
          if (ferr<3.0*last_avg_err) 
          { // Include this point in the average:
            n_lighthouse[L]++;
            
            
#if ANGULAR_CHECK
            // Angular velocity estimate: radians
            //    numerator is projection of sensor into LN plane
            //    denominator is distance to sensor
            float predicted=dot(E-LP,survive_lighthouse_normal(o,L,A,0.0))/length(E-LP);
            all_predicted[n_sensed]=predicted;
            all_sensed[n_sensed]=a;
            // printf(" %2d sweep velocity: pred %.4f  sensed %.4f\n",S,predicted,a);
            n_sensed++;
#endif
          
            vec3 correction=-err*LN;
            // vec3 D=E+correction; // actual location on sweep plane
            
            sum_motion+=correction;
            n_motion+=1.0;
            
            vec3 torque=cross(sensor_offset,correction);
            // magnitude of cross product: |s||c|sin(angle)
            //  Rotation required for correction: |c|/|s| radians
            //  so divide by |s|^2
            torque*=1.0/(0.01*0.01+dot(sensor_offset,sensor_offset));
            sum_torque+=torque;
            n_torque+=1.0;
            
            // FLT speed=0.5; // m/s of position correction per sensor
            // tracking+=speed*dt*correction; // move toward detected value
            
            // FLT sanity_check=dot(D-LP,LN); printf("Sanity check: D is off by %.5f meters\n",sanity_check);
          }
          else n_outlier++;
        }
      //if (pass==NPASS-1) if (sensor_err>0) printf("%.4f(%d)\t",sensor_err/n_sensor,n_sensor);

#if ANGULAR_CHECK
        if (n_sensed>=2 && pass>2) 
        { // use angular ratio as distance estimate:
          
          // Find mean slope: radio of expected and actual angles
          float sum_slopes=0.0;
          int n_slopes=0;
          for (int i=0;i<n_sensed;i++)
            for (int j=i+1;j<n_sensed;j++) 
            {
              // sensed = slope*predicted+offset;
              float num=(all_sensed[i]-all_sensed[j]);
              float denom=(all_predicted[i]-all_predicted[j]);
              if (isnormal(num) && isnormal(denom) && fabs(denom)>0.0001) {
                float slope=num/denom;
                if (isnormal(slope)) {
                  sum_slopes+=slope;
                  n_slopes++;
                }
              }
            }
          if (n_slopes>=1) {
            float slope=sum_slopes/n_slopes;
            
            // Find consensus offset:
            float offset=0.0;
            for (int i=0;i<n_sensed;i++) {
              offset += -(slope*all_predicted[i]-all_sensed[i]);
            }
            offset *= (1.0/n_sensed);
            
            // Measure mean total error
            float err=0.0;
            for (int i=0;i<n_sensed;i++) {
              err += fabs(slope*all_predicted[i]+offset-all_sensed[i]);
            }
            err*=(1.0/n_sensed);
            
#define verbose_angular 0
#if verbose_angular
            printf("  L%d A%d angular model = *%.4f + %.4f   (err %.5f)\n",
              L,A, slope, offset, err);
            for (int i=0;i<n_sensed;i++) 
                printf("  pred = %.4f    sensed = %.4f\n",
                  slope*all_predicted[i]+offset, all_sensed[i]);
#endif
            if (n_sensed>=4 && fabs(err)<0.004) 
            { // Apply slope to fix scale factor on position:
              // If distance is correct, slope==1.0
              float scale=1.0/fabs(slope);
              float push=0.01*(scale-1.0);
              float limit=0.005; // maximum position scale factor per pass
              if (push>limit) push=limit;
              if (push<-limit) push=-limit;
              vec3 dir=nextP-o->lighthouse_position[L];
#if verbose_angular
              printf("  slope-based push = %.4f\n",push);
#endif
              nextP += push*dir;
             // nextP += (0.01*n_sensed)*push*dir;
            }
          }
        }
#endif
    }
    //if (pass==NPASS-1) printf("\n");
    vec3 motion=sum_motion*(1.0/n_motion);
    vec3 torque=sum_torque*(1.0/n_torque);
    
    float avg_err=tot_err/n_err;
    
    printf("Pass %d: %d points, error %.4f meters, %d/%d visible, %d outliers removed\n",
        pass,(int)n_motion,avg_err,n_lighthouse[0],n_lighthouse[1],n_outlier);
    survive_vec3_print("    motion: ",motion);
    survive_vec3_print("                     torque: ",torque);
    
    if (pass>0 && n_motion>0 && (n_lighthouse[0]>1 || n_lighthouse[1]>1) ) {
      // Apply position offset:
      float strength=0.03*n_motion;
      const float max_strength=0.5;
      if (strength>max_strength) strength=max_strength;

      // If the error is big, chop down the strength:
      float extra_err=avg_err-0.03;
      if (extra_err>0.0) {
        strength*=1/(50.0*extra_err+1.0);
      }
      
      nextP+=strength*motion;
    }
    
    // Wait until position is correct to apply torque:
    if (false && pass>3 && n_torque>4) 
    { // Apply torque
      vec3 local_torque=survive_orient_local_from_global(&o->orient,torque);
      
      survive_orient_rotate(&o->orient,
        (dt*10.0)*local_torque);
    }
    last_avg_err=avg_err;
  }
  
  // Update position and estimate velocity
  vec3 nextV=(nextP-o->position)/dt;
  FLT velfilter=10.0*dt;
  o->velocity=velfilter*nextV+(1.0-velfilter)*o->velocity;
  o->position=nextP;
  
  // Coarse distance updater:
  double tot_length=0; int n_lengths=0;
  for (int L=0;L<NUM_LIGHTHOUSES;L++) {
    for (int S=0;S<SENSORS_PER_OBJECT;S++)
      for (int A=0;A<2;A++)
        if (o->sensor[S].length[L][A]) {
          tot_length += o->sensor[S].length[L][A];
          n_lengths++;
        }
  }
  if (n_lengths>=3) {
    double avg_length=tot_length/n_lengths;
    printf("Average pulse length: %.2g\n",avg_length);
  }
  
  // Coarse orientation updater:
  // If the lighthouse is visible from this sensor,
  //   then the sensor normal must point generally toward the lighthouse
  for (int L=0;L<NUM_LIGHTHOUSES;L++) {
    vec3 to_LH=vec3(0.0); // points toward lighthouse (local coords)
    float n_LH=0;
    for (int S=0;S<SENSORS_PER_OBJECT;S++)
      for (int A=0;A<2;A++)
        if (o->sensor[S].angle[L][A]!=0.0) 
        { // this sensor sees the lighthouse
          FLT weight=1.0;
          to_LH+=weight*o->hardware[S].normal;
          n_LH+=weight;
        }
    if (length(to_LH)!=0.0 && n_LH>1) {
      to_LH=normalize(to_LH);
      // survive_vec3_print("  local to LH: ",to_LH);
      
      // Update orientation to reflect visibility.
      //   Be gentle, because this is low precision,
      //   and it's wrong if our position is wrong.
      survive_orient_nudge(&o->orient,
        to_LH,o->lighthouse_position[L]-o->position, 
        dt*0.5);
    }
  }
  
  // Zero out the sensed angles:
  for (int S=0;S<SENSORS_PER_OBJECT;S++)
    for (int L=0;L<NUM_LIGHTHOUSES;L++)
      for (int A=0;A<2;A++)
      {
        o->sensor[S].angle[L][A]=0.0;
        o->sensor[S].length[L][A]=0.0;
      }
  
  
  // Create robot and sensor transforms, and publish them.
  
  SurviveObjectOrientation &basis=o->orient;
  vec3 vive=o->position; // meters, relative to lighthouse 0
  vive*=100; // to cm
  vive.x-=75; // x==0 on centerline between the sensors
  vive.x+=378/2; // set X=0 to left edge of arena (driving coords, not centered)
  vive.y-=40; // origin at the front of the dump area
  
  osl::transform tf_robot;
  tf_robot.origin=vive+basis.x*(-65)+basis.y*(-25); // vive to robot turning center
  tf_robot.basis.z=-basis.z; // robot Z is straight up
  tf_robot.basis.x=basis.y; // robot X is driving forward
  tf_robot.basis.y=basis.x; // robot Y points to robot's left
  
  static file_ipc_link<osl::transform> robot_link("robot.tf");
  robot_link.publish(tf_robot);
  
  osl::transform tf_sensor;
  tf_sensor.origin=vive+basis.x*(-18); // sensor position
  
  /*
    Sensors are rotated by 45 degrees in vive's Y-Z plane:
    Vive controller facing up and backward:
      v-> Y
      |
      v
      Z
    
    Sensor facing down and forward:
    Z
     \
      s
     /
    Y 
  */
  
  tf_sensor.basis.y=normalize(basis.y+basis.z);
  tf_sensor.basis.z=normalize(basis.y-basis.z);
  tf_sensor.basis.x=basis.x;
  
  static file_ipc_link<osl::transform> sensor_link("sensor.tf");
  sensor_link.publish(tf_sensor);
}
// Dump important stuff to the screen:
void survive_sim_print(SurviveObjectSimulation *o)
{
  survive_vec3_print("position",o->position);
  survive_vec3_print("velocity",o->velocity);
  survive_orient_print(&o->orient);
  survive_vec3_print("down",o->down);
}

// Update simulation for these IMU readings
void survive_sim_imu(SurviveObjectSimulation *o, FLT * accelgyro)
{
  double now=time_in_seconds();
  double dt=now-o->last_imu_time;
  o->last_imu_time=now;
  vec3 accel=vec3(accelgyro[0], accelgyro[2], accelgyro[1]);
  accel*=1.0/4140.0; // emperical scale factor to 1.0g 
  o->down=accel;
  
  vec3 gyro=vec3( -accelgyro[3], -accelgyro[5], -accelgyro[4] );
  gyro*=1.0/1000.0; // units seem to be milliradians per second?
  gyro*=dt; // radians of rotation this frame
  survive_orient_rotate(&o->orient,gyro);
  
  // Keep gravity's down vector pointing to global down
  survive_orient_nudge(&o->orient,
    o->down,vec3(0,0,-1.0), dt*2.0);
}







SurviveObjectSimulation *ww0=0;
struct SurviveContext * ctx;
const char *caldesc="Initializing...";
int  quit = 0;

void HandleKey( int keycode, int bDown )
{
	if( !bDown ) return;

	if( keycode == 'O' || keycode == 'o' )
	{
		survive_send_magic(ctx,1,0,0);
	}
	if( keycode == 'F' || keycode == 'f' )
	{
		survive_send_magic(ctx,0,0,0);
	}
	if( keycode == 'Q' || keycode == 'q' )
	{
		quit = 1;
	}
}

void HandleButton( int x, int y, int button, int bDown )
{
}

void HandleMotion( int x, int y, int mask )
{
}

void HandleDestroy()
{
}

//int bufferpts[32*2*3][2];
int bufferpts[32*3][4];


char buffermts[32*128*3];
unsigned int buffertimeto[32*3][4];

void my_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lh)
{
//	if( timeinsweep < 0 ) return;
	survive_default_light_process( so, sensor_id, acode, timeinsweep, timecode, length, lh);
	if( sensor_id < 0 ) return;
	if( acode < 0 ) return;
	
//return;
	int jumpoffset = sensor_id;
	//if( strcmp( so->codename, "WM0" ) == 0 || strcmp( so->codename, "WW0" ) == 0) jumpoffset += 32;
	//else if( strcmp( so->codename, "WM1" ) == 0 ) jumpoffset += 64;

/*
	if( acode % 2 == 0 && lh == 0) //data = 0
	{
		bufferpts[jumpoffset*2+0][0] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][0] = 0;
	}
	if(  acode % 2 == 1 && lh == 0 ) //data = 1
	{
		bufferpts[jumpoffset*2+1][0] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][0] = 0;
	}


	if( acode % 2 == 0 && lh == 1 ) //data = 0
	{
		bufferpts[jumpoffset*2+0][1] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][1] = 0;
	}
	if( acode % 2 == 1 && lh == 1 ) //data = 1
	{
		bufferpts[jumpoffset*2+1][1] = (timeinsweep-100000)/300;
		buffertimeto[jumpoffset][1] = 0;
	}
	*/
	acode=(acode%2)+2*lh;
	bufferpts[jumpoffset][acode] = (timeinsweep-80000)/500;
	buffertimeto[jumpoffset][acode]=0;
}

int imu_updates=0;
FLT accelgyro[6];

void my_imu_process( struct SurviveObject * so, int mask, FLT * accelgyro_new, uint32_t timecode, int id )
{
	survive_default_imu_process( so, mask, accelgyro, timecode, id );
	
	if (ww0) survive_sim_imu(ww0,accelgyro_new);
	imu_updates++;
	memcpy(accelgyro,accelgyro_new,6*sizeof(FLT));

	//if( so->codename[0] == 'H' )
	if( 0 )
	{
		printf( "I %s ( %f %f %f ) ( %f %f %f ) %08X %d\n", so->codename, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], timecode, id );
	}
}

/*

lh: lighthouse sending the pulse:
  0 is lighthouse B, for me the left.
  1 is lighthouse C, for me the right.

acode: angle code:
  0-1 for X-Y angles from lighthouse 0
  2-3 for X-Y angles from lighthouse 1

angle: radians

*/
FLT all_angles[32][4]; // sensor ID, then acode

void my_angle_process( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh)
{
	survive_default_angle_process( so, sensor_id, acode, timecode, length, angle, lh );
	
	const float max_angle=1.2;
  if (angle<-max_angle || angle>max_angle) {} // time delays cause really weird angles
  else {
	  if (angle==0) angle=0.00000001; //  <- 0 is sentinal value
	  if (ww0) {
	    ww0->sensor[sensor_id].angle[lh][acode%2]=angle;
	    ww0->sensor[sensor_id].length[lh][acode%2]=length;
	  }
  }	
	
	if (1)
	printf("RAWANGLES:  %d  %d  %d  %.5f %.3g\n",
	  sensor_id, acode, lh, angle, length);

	
	acode=(acode%2)+2*lh;
	all_angles[sensor_id][acode]=angle;
}

#define MAX_SENSOR_NAME (3*32)
char* sensor_name[MAX_SENSOR_NAME];

void * GuiThread( void * v )
{
	short screenx, screeny;
	CNFGBGColor = 0x000000;
	CNFGDialogColor = 0x444444;
	CNFGSetup( "Survive Robot Controller Tracking", 700, 700 );

	while(1)
	{
	  int sensor_id, acode;

#ifdef __unix__
usleep(10*1000); // limit to 100fps
#endif
		CNFGHandleInput();
		CNFGClearFrame();
		CNFGColor( 0xFFFFFF );
		CNFGGetDimensions( &screenx, &screeny );

		
		//printf("\033[0;0f"); // seek to start of screen
		printf("\033[2J"); // seek to (0,0) and clear screen
		
	  if (ww0) {
	    survive_sim_integrate(ww0);
	    survive_sim_print(ww0);

		  float scaleX=-screenx/5.0, scaleY=screeny/5.0;
		  float offX=screenx+2.0*scaleX, offY=0;
		
		  // Segments for tracked controller position
		  for (int axis=0;axis<3;axis++) {
		    CNFGColor(0x0000ff<<(8*axis));
		    vec3 del=0.2*ww0->orient.x;
		    if (axis==1) del=0.2*ww0->orient.y;
		    if (axis==2) del=0.4*ww0->orient.z;
		    vec3 S=ww0->position, E=ww0->position + del;
		    
		    CNFGTackSegment( S.x*scaleX+offX, S.y*scaleY+offY,
		                     E.x*scaleX+offX, E.y*scaleY+offY);
		  }
		  
		
		}
		
		printf( "RAWIMU ( %7.1f %7.1f %7.1f ) ( %5.1f %5.1f %5.1f ) %d\n", accelgyro[0], accelgyro[2], accelgyro[1], -accelgyro[3], -accelgyro[5], -accelgyro[4], imu_updates);
		imu_updates=0;
		
		if (0) // dump sensor data as simple table:
	  for (sensor_id=0;sensor_id<32;sensor_id++)
	  {
	    printf("Sensor %2d: ",sensor_id);
	    for (acode=0;acode<4;acode++) 
	    {
	      float v=all_angles[sensor_id][acode];
	      if (buffertimeto[sensor_id][(acode/2)*2]<3) 
  	      printf("%8.4f  ",v);
  	    else
  	      printf("          ");
	    }
	    printf("\n");
	  }
		
		
		
		

		int i,nn;
		for( i = 0; i < 32*3; i++ )
		{
			for( nn = 0; nn < 4; nn+=2 )
			{
				if( buffertimeto[i][nn] < 5 ) // frames to draw old points
				{
					buffertimeto[i][nn]++;
					
					int pip=2;
					// uint32_t color = i * 3231349;
					int x=bufferpts[i][nn];
					int y=bufferpts[i][nn+1];
					uint8_t r = 0xff;
					uint8_t g = 0xff;
					uint8_t b = 0xff;

					if (nn==0) { b = 0; g = 50; } //lighthouse B, red, master
					if (nn==2) r = 0; //lighthouse C, blue, slave

//					r = (r * (5-buffertimeto[i][nn])) / 5 ;
//					g = (g * (5-buffertimeto[i][nn])) / 5 ;
//					b = (b * (5-buffertimeto[i][nn])) / 5 ;
					CNFGColor( (b<<16) | (g<<8) | r );

					if (x == 0 || y==0) continue; //do not draw if aither coordinate is 0

					CNFGTackRectangle( x, y, x + pip, y + pip );
					CNFGPenX = x; CNFGPenY = y;
					CNFGDrawText( buffermts, 2 );

					if (i<MAX_SENSOR_NAME) {
						CNFGPenX = x+5; CNFGPenY = y+5;
						CNFGDrawText( sensor_name[i], 2 );
					}
				}
			}
		}

		CNFGColor( 0xffffff );
    /*
		char caldesc[256];
		survive_cal_get_status( ctx, caldesc, sizeof( caldesc ) );
		*/
		CNFGPenX = 3;
		CNFGPenY = 3;
		CNFGDrawText( caldesc, 4 );
		
    fflush(stdout);

		CNFGSwapBuffers();
		OGUSleep( 10000 );
	}
	return 0;
}




int main(int argc,char * const *argv)
{
	ctx = survive_init( argc,argv );
	
	if (ctx == 0) {
	  fprintf(stderr,"No vive objects connected. Exiting.\n");
	  return 1;
	}
	
	ww0=survive_sim_init("WW0");

	uint8_t i =0;
	for (i=0;i<MAX_SENSOR_NAME;++i) {
		sensor_name[i] = (char *)malloc(8);
		sprintf(sensor_name[i],"%d",i%32);
	}

	survive_install_light_fn( ctx,  my_light_process );
	survive_install_imu_fn( ctx,  my_imu_process );
	survive_install_angle_fn( ctx, my_angle_process );

	// survive_cal_install( ctx );

	OGCreateThread( GuiThread, 0 );
	

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	while(survive_poll(ctx) == 0 && !quit)
	{
		//Do stuff.
	}

	survive_close( ctx );

	printf( "Returned\n" );
}


