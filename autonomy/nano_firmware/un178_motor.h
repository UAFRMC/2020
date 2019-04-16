
//Arsh Chauhan
//Last Edited: 4/8/2019

#ifndef UN178_MOTOR_H
#define UN178_MOTOR_H

#include <Arduino.h>


class un178_motor_single_t
{
    //Needs 2 digital pins and 1 PWM pin

    uint8_t pwm_;
    uint8_t dir_1_, dir_2_;

public:

    un178_motor_single_t(){}
    un178_motor_single_t(uint8_t pwm, uint8_t dir_1, uint8_t dir_2 ):
    pwm_(pwm), dir_1_(dir_1), dir_2_(dir_2){
        stop();
    }

    void set_pins(uint8_t pwm, uint8_t dir_1, uint8_t dir_2 )
    {
        pwm_ = pwm;
        dir_1_ = dir_1;
        dir_2_ = dir_2;
        stop();
    }

    void drive (uint8_t pwm, uint8_t dir_1, uint8_t dir_2)const
    {
        if (pwm == 255 )
            pwm = 254; //UN178 stops working if it gets 255 PWM
        digitalWrite(dir_1_,dir_1);
        digitalWrite(dir_2_,dir_2);
        analogWrite(pwm_,pwm);
    }

    void drive_green(uint8_t pwm)const
    {
        drive(pwm,1,0);
    }

    void drive_red(uint8_t pwm)const
    {
        drive(pwm,0,1);
    }

    void stop()const
    {
        drive(0,0,0);
    }
};

class un178_motor_t
{

    uint8_t A_pwm_, B_pwm_;
    uint8_t A_dir_1_, A_dir_2_, B_dir_1_, B_dir_2_;

public:
    un178_motor_single_t motor_A;
    un178_motor_single_t motor_B;
    
    un178_motor_t(uint8_t A_pwm, uint8_t A_dir_1, uint8_t A_dir_2, uint8_t B_pwm, uint8_t B_dir_1, uint8_t B_dir_2 ) :
    A_pwm_(A_pwm), A_dir_1_(A_dir_1), A_dir_2_(A_dir_2), B_pwm_(B_pwm), B_dir_1_(B_dir_1), B_dir_2_(B_dir_2)
    {
        
        motor_A.set_pins(A_pwm_, A_dir_1_ , A_dir_2_);
        motor_B.set_pins(B_pwm_, B_dir_1_, B_dir_2_);
    }
 
};
#endif
