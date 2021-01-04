// Basic skeleton read/write files for the andretti, used to make sure we can read out all the data,
// as well as write out all the data. Contains no logic, allows for a testing of the whole system
// reading data.

//
#include <iostream>
#include <stdio.h>
#include "aurora/data_exchange.h"
#include "aurora/lunatic.h"

bool verbose=true;

// cap turning power
float turncap(float turnpower) {
    const float cap=1.0;
    if (turnpower>+cap) return +cap;
    if (turnpower<-cap) return -cap;
    return turnpower;
}

/*
  Find the drive commands to use when at this point along this path.
*/
aurora::drive_commands path_to_drive(const aurora::path_plan &path,
    const aurora::robot_loc2D &cur)
{
    aurora::drive_commands drive;
    drive.left = drive.right = 0.0;
    int best_index = -1;
    double best_distance=20.0; //<- cm distance--if nobody's closer than this, just stop and wait
    const double angle_to_cm=0.5; // scales angle mismatch (deg) to position mismatch (cm)
    
    for(unsigned short i = 0; i < path.plan_len-1; i++)
    {
        aurora::robot_loc2D step = path.path_plan[i];
        double distance = 
           length(step.center()-cur.center())
           + angle_to_cm*aurora::angle_abs_diff(step.angle,cur.angle);
        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
        }
    }
    if (verbose) printf("andretti: index %2d matches with distance %.1f\n", best_index,best_distance);
    
    // Figure out the drive commands to follow the path from this position
    if (best_index>=0) 
    { 
        aurora::robot_loc2D step = path.path_plan[best_index];
        aurora::robot_loc2D next = path.path_plan[best_index+1];
        vec2 move=next.center()-step.center(); //<- fixme: should this be next - cur?
        if (length(move)>0.01) move=normalize(move); // we just want the drive direction
        float forward = dot(step.forward(),move);
        float turn = aurora::angle_signed_diff(next.angle,cur.angle); 
        const double turnscale = 0.1; // degrees to 0-1 drive power (steering agressiveness)
        
        const float   autonomous_speed=100.0f; //<- fixme: put this scale factor into backend
        drive.left  = autonomous_speed * (forward - turncap(turnscale*turn));
        drive.right = autonomous_speed * (forward + turncap(turnscale*turn));
        if (verbose) {
            printf("   fw %.2f  turn %.2f     ",forward,turn); drive.print();
            /*
            printf("      "); cur.print(stdout," =cur\n"); 
            printf("         "); step.print(stdout," =step\n"); 
            printf("             "); next.print(stdout," =next\n");
            */
        }
        
    }
    return drive;
}

int main()
{

    MAKE_exchange_drive_commands();
    MAKE_exchange_plan_current();
    MAKE_exchange_path_plan();
    
    aurora::robot_loc2D cur = exchange_plan_current.read();
    aurora::path_plan path = exchange_path_plan.read();
    bool updated=true;
    while (true) {
        if(exchange_path_plan.updated()){
            path = exchange_path_plan.read();
            updated=true;
        }
        if(exchange_plan_current.updated()){
            cur = exchange_plan_current.read();
            updated=true;
        }
        
        if (updated) {
            updated=false;
            
            // Figure out the step of the path most similar to our current position
            aurora::drive_commands drive = path_to_drive(path,cur);
            
            exchange_drive_commands.write_begin() = drive;
            exchange_drive_commands.write_end();
        
        }

        // Limit our cycle rate (to save CPU)
        aurora::data_exchange_sleep(50);
    }

    return 0;
}
