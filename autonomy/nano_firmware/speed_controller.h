/* 
 Non-BTS PID motor speed controller.
*/
#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <Arduino.h>
#include "encoder.h"

template<uint8_t AVERAGES> 
class speed_controller_t
{
public:
  const encoder_t *encoder;
  int last_encoder; 
  milli_t times[AVERAGES]; // milli at recent encoder ticks
  uint8_t times_ptr; // index into times array
  int16_t average_without_div; // running average of times array
  milli_t target_time; // motor target period (ms)
  milli_t last_power; // time we last sent power to the motor
  milli_t last_change; // time of last value change
  int last_dir; // direction at last check
  int last_err; // speed error at last check
  bool stalled;

  speed_controller_t(const encoder_t *encoder_=0)
    :encoder(encoder_), average_without_div(0),times_ptr(0),target_time(1000)
  {
    memset(times,0,sizeof(times));
    stalled=false;
    last_dir=0;
    last_change=last_power=milli;
    last_err=0;
  }

  // Update state for new commanded direction
  void set_dir(int dir) {
      if (dir!=last_dir) {
        last_dir=dir;
        last_change=milli; // update timing for start of new command
        last_err=0;
      }
  }


  //target is from -100 .. 0 .. +100, either meaning:
  //   raw PWM in torque control mode (torque_only == true)
  //   target RPM speed in speed control mode 
  int update(int target,const bool torque_only,int rpm_scaler=3)
  {
    // Follow the encoder
    if (encoder && encoder->value != last_encoder)
    { // encoder value just changed, update timings
      times[times_ptr++]=milli-last_change;
      if(times_ptr>=AVERAGES)
        times_ptr=0;

      average_without_div=0;
      for(int16_t ii=0;ii<AVERAGES;++ii)
        average_without_div+=times[ii];

      last_encoder=encoder->value;
      last_change=milli;
      stalled=false; // evidently we're spinning!
    }
    
    // If torque control, leave immediately
    if((!encoder) || torque_only) {
      last_change=milli;
      stalled=false;
      return target; // unmodified value
    }
    
    bool backwards=false;
    if(target<-3) { // running backwards
      backwards=true;
      set_dir(-1);
      target=-target; // target positive in remaining code
    }
    else if (target>3) { // running forwards
      set_dir(+1);
    }
    else /* target is very small, nearly stopped */ {
      set_dir(0);
      stalled=false; //No power,clear stall
      return 0;
    }
    
    // target == 0 (stop) .. 100 (full power)

    //Change pwm speed to time in ms
    int rpm=target*rpm_scaler;
    if(rpm<=0)
      rpm=1;
    
    // target_time is ms per encoder tick
    target_time=(1000*60)/(rpm*36); // ms/tick = ms/sec * sec/min / rev/min * ticks/rev

    int min_time=2;
    int max_time=200;

    // Sanity clamp the tick time
    if(target_time>max_time)
      target_time=max_time;
    if(target_time<min_time)
      target_time=min_time;

    // Stall detection: keep from frying motors
    int stall_time=500;
    if (last_power>last_change+stall_time) { stalled=true; }
    if (stalled) return 0;
    
    const int motor_min=5; // never give less power than this (keep spinning)
    const int motor_max=100; // never give more power than this
    const int motor_idle=10;
    int32_t motor_value=0;
    {
      milli_t real_average=average_without_div;
      int count=AVERAGES;

      milli_t since_last=milli-last_change;
      if(since_last>real_average/AVERAGES) // next tick is late--include it
      {
        real_average+=since_last;
        ++count;
      }

      real_average=real_average/count;

      // Figure the current percent speed error:
      int32_t err=((int32_t)real_average-(int32_t)target_time)*100/target_time;
      // Figure PID terms:
      int32_t P=8*err;
      int32_t D=0; // FIXME * (err - last_err)
      motor_value=P+D+motor_idle;

#if 0
      // Debug prints
      static milli_t last_print=0;
      if(milli-last_print>=100)
      {
        Serial.print("tPIDpow:\t");
        Serial.print(milli);
        Serial.print("\t");
        Serial.print(average_without_div/AVERAGES);
        Serial.print("\t");
        Serial.println(motor_value);

        last_print=milli;
      }
#endif

      // avoid spinning too slow
      if(motor_value<motor_min)
        motor_value=motor_min;
    }

    // avoid spinning too fast
    if(motor_value>motor_max)
      motor_value=motor_max;


    if(motor_value!=0)
      last_power=milli;

    if(backwards) //running in reverse
      return -motor_value;
    else
      return +motor_value;
  }
};

#endif
