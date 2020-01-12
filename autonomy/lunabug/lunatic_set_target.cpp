/* Debug tool: manually set the autonomy target position */
#include "aurora/lunatic.h"

int main(int argc,char *argv[]) {
    if (argc!=5) { 
        printf("Usage: set_target  x_cm  y_cm  angle_deg  percent\n");
        exit(1);
    }
    
    aurora::robot_loc2D target;
    target.x=atof(argv[1]);
    target.y=atof(argv[2]);
    target.angle=atof(argv[3]);
    target.percent=atof(argv[4]);
    target.print();
    
    MAKE_exchange_plan_target();
    exchange_plan_target.write_begin()=target;
    exchange_plan_target.write_end();

    return 0;
}


