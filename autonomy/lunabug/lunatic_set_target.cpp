/* Debug tool: manually set the autonomy target position */
#include "aurora/lunatic.h"

int main(int argc,char *argv[]) {
    if (argc!=7) { 
        printf("Usage: set_target  x_cm  y_cm  angle_deg  error_x error_y error_target\n");
        exit(1);
    }
    
    aurora::robot_navtarget target;
    target.x=atof(argv[1]);
    target.y=atof(argv[2]);
    target.angle=atof(argv[3]);
    target.error.x=atof(argv[4]);
    target.error.y=atof(argv[5]);
    target.error.angle=atof(argv[6]);
    target.print();
    
    MAKE_exchange_plan_target();
    exchange_plan_target.write_begin()=target;
    exchange_plan_target.write_end();

    return 0;
}


