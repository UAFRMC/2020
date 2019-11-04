/*
 Non-BTS PID motor speed controller.
*/
#ifndef SPEED_CONTROLLER_H
#define SPEED_CONTROLLER_H

#include <Arduino.h>
#include "encoder.h"

// AVERAGES: Number of ticks
// HISTORY: Buffer of encoder value 
template<uint8_t AVERAGES, uint8_t HISTORY>
class speed_controller_t
{
public:
  const encoder_t *encoder;
  int last_encoder;  // encoder value at last check
  milli_t last_change; // time of last encoder value change

  milli_t times[AVERAGES]; // milli between recent encoder ticks
  uint8_t times_ptr; // index into times array (cyclic buffer)
  uint16_t count_history[HISTORY];
  int8_t count_index;
   
  milli_t last_power; // time we last sent power to the motor
  int last_dir; // direction at last check
  int last_err; // speed error (percent) at last check
  int32_t last_I;
  bool stalled;
  int debug;
  int pterm;
  int dterm;
  int iterm;

  speed_controller_t(const encoder_t *encoder_=0)
    :encoder(encoder_), times_ptr(0)
  {
    memset(times,0,sizeof(times));
    memset(count_history,0,sizeof(count_history));
    count_index=HISTORY-1;
    stalled=false;
    last_dir=0;
    last_change=last_power=milli;
    last_err=0;
    last_I=0;
    pterm=0;
    dterm=0;
    iterm=0;
  }

  int history_wrap(int index){
    while(index >= HISTORY-1)
        index -= HISTORY;
    while(index < 0)
        index += HISTORY;
   return index;
  }

  int get_speed(int start, int duration){
    uint16_t speed = count_history[history_wrap(count_index-start)] - count_history[history_wrap(count_index-start-duration)]; 
    return (int)speed;
  }
  // Update state for new commanded direction
  void set_dir(int dir) {
      if (dir!=last_dir) {
        last_dir=dir;
        last_change=milli; // update timing for start of new command
        last_err=0;
        last_I=0;
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
     
    // If torque control,leave immediately
    if((!encoder) || torque_only) {
      last_change=milli;
      stalled=false;
      return target; // unmodified value
    }
    count_index ++;
    if (count_index>HISTORY-1)
    {
      count_index = 0;
    }
    count_history[count_index] = encoder->count_mono;
    
    
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
    const int dt = 5; // iteration_time


    // If you want to readjust these, start with Kp, Ki, and Kd at zero
    
    const float Kp = 1.0f;     // Proportion constant (Adjust me first until 
                                 // Adjust me first until oscillation are steadyish
    
    const float Kd = -16.0f;    // Derivative constant
                                 // Adjust me second to achieve tight oscillations 
                                 // or until the system is semi-critically damp
                           
    const float Ki = 100.0f;   // Integral constant
                                // Adjust me third till the desired oscillations
                                // are achieved. (You might want this at 0)
    
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
      int32_t window_ms=60;
      int32_t window=window_ms/dt;//window for D term
      // Figure the current percent speed error:
      int32_t divide=target_time;
      // if (real_average>divide) divide=real_average;
      //int32_t err=(((int32_t)real_average-(int32_t)target_time)*100)/divide;
      int32_t encoder_ticks_per_rev=36;
      int32_t windows_per_minute=60000L/(window_ms*2);
      int32_t current_rpm=get_speed(0,window*2)*windows_per_minute/encoder_ticks_per_rev;
      int32_t err=(2*target-current_rpm);
      //if (err<0) err=0;
      //if (err>200) err=200; // clamp range of error term
      
      // Figure the corresponding PID terms, in percent motor power
      int32_t P=err; // proportional term
      int32_t D=(get_speed(0,window)-get_speed(window,window));//(err - last_err) / dt; // FIXME * (err - last_err)
      int32_t I=last_I + (err*dt);
      int32_t max_I=10000;
      if(I>max_I)
      {
        I=max_I; 
      }
      if(I<-max_I)
      {
        I=-max_I;
      }
      motor_value=Kp*P+(Ki/max_I)*I+(Kd/dt)*D+motor_idle;
      pterm=current_rpm;
      dterm=D;
      iterm=I*100/max_I;
      last_err=err;
      last_I=I;

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
