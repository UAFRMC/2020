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
  int last_encoder;  // encoder value at last check
  milli_t last_change; // time of last encoder value change

  milli_t times[AVERAGES]; // milli between recent encoder ticks
  uint8_t times_ptr; // index into times array (cyclic buffer)

  milli_t last_power; // time we last sent power to the motor
  int last_dir; // direction at last check
  int last_err; // speed error (percent) at last check
  bool stalled;
  int debug;

  speed_controller_t(const encoder_t *encoder_=0)
    :encoder(encoder_), times_ptr(0)
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
        stalled=false; // clear stall bit on direction change

        // Make fake buffer of slow encoder tick times--
        //   we're basically at a standing start
        for (int a=0;a<AVERAGES;a++) times[a]=10;
        times_ptr=0;
      }
  }

  //target is from -100 .. 0 .. +100, either meaning:
  //   raw PWM in torque control mode (torque_only == true)
  //   target RPM speed in speed control mode
  int update(int target,const bool torque_only,int rpm_scaler=3)
  {
    // If torque control, leave immediately
    if((!encoder) || torque_only) {
      last_change=milli;
      stalled=false;
      return target; // unmodified value
    }

    // Follow the encoder
    if (encoder && encoder->value != last_encoder)
    { // encoder value just changed, update timings
      times[times_ptr++]=milli-last_change;
      if(times_ptr>=AVERAGES)
        times_ptr=0;

      last_encoder=encoder->value;
      last_change=milli;
      stalled=false; // evidently we're spinning!
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
    int target_time=(1000L*60)/(rpm*36L); // ms/tick = ms/sec * sec/min / rev/min * ticks/rev

    int min_time=2;
    int max_time=200;

    // Sanity clamp the tick time
    if(target_time>max_time)
      target_time=max_time;
    if(target_time<min_time)
      target_time=min_time;

    // Stall detection: keep from frying motors
    int stall_time=800; // ms
    milli_t since_change = milli-last_change;
    if (since_change>target_time+stall_time) { stalled=true; }
    if (stalled) return 0; // turn off motor when stalled

    const int motor_min=10; // never give less power than this (keep spinning)
    const int motor_max=100; // never give more power than this
    const int motor_idle=10;
    int32_t motor_value=0;
    {
      int16_t total=0;
      for(int16_t ii=0;ii<AVERAGES;++ii)
        total+=times[ii];
      int count=AVERAGES;

      milli_t since_last=milli-last_change;
      if(since_last>(total/AVERAGES)) // next tick is late--include it
      {
        total+=since_last;
        ++count;
      }

      milli_t real_average=total/count;
      debug=real_average;


      // Figure the current percent speed error:
      int32_t divide=target_time;
      // if (real_average>divide) divide=real_average;
      int32_t err=(((int32_t)real_average-(int32_t)target_time)*100)/divide;
      if (err<0) err=0;
      if (err>200) err=200; // clamp range of error term

      // Figure the corresponding PID terms, in percent motor power
      int32_t P=err/2; // proportional term
      int32_t D=0; // FIXME * (err - last_err)
      motor_value=P+D+motor_idle;

      last_err=err;

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
