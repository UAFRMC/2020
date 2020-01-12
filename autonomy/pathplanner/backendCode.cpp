    void robot_manager_t::autonomous_state()
            {
            robot.power.stop(); // each state starts from scratch

            double time_in_state=cur_time-state_start_time;
            robotPrintln("In state %s for %.1f seconds...\n", state_to_string(robot.state), time_in_state);

            // full autonomy start
            if (robot.state==state_autonomy) {
                robot.autonomous=true;
                enter_state(state_setup_raise);
            }
            // raise: raise the mining head to clear ground for driving
            else if (robot.state==state_setup_raise)
            {
                if(robot.sensor.bucket<head_mine_drive && time_in_state<2.0)// raises until bucket_drive
                {
                robot.power.dump=power_full_fw; // raise bin
                }
                else{
                enter_state(state_setup_extend);
                }
            }
            // state_setup_extend: extend the mining head so it does not get dragged
            else if (robot.state==state_setup_extend)
            {
                if (time_in_state<7.0)
                {
                robot.power.dump=power_full_fw; // raise bin
                robot.power.head_extend = 127; // 127 for extend, 1 for tuck
                }
                else
                {
                    mining_head_extended = true;
                    enter_state(state_setup_lower);
                }
            }
            // state_setup_lower: lower the box
            else if (robot.state==state_setup_lower)
            {
                if (time_in_state<8.0 && robot.sensor.limit_bottom==1)
                {
                robot.power.roll = 30; // lower box somewhat slowly
                robot.power.head_extend = 127;
                }
                else
                {
                    enter_state(state_find_camera);
                }
            }
            //state_find_camera: line up with centerline
            else if (robot.state==state_find_camera)
            {
                if (!drive_posture()) { /* correct posture first */ }
                else if (locator.merged.percent>=15.0) { // we know where we are!
                sim.loc=locator.merged; // reset simulator to real detected values

                // if(exchange_plan_current.updated())
                // {
                //   currentLocation = exchange_plan_current.read();
                // }
                // else if (currentLocation.percent>=0.1) { // we know where we are!
                // Need some interefacing with the simulation stuff

                sim.loc=locator.merged; // reset simulator to real detected values
                enter_state(state_scan_obstacles);
                }
                else // don't know where we are yet--change pointing
                {
                if (time_in_state<5.0) point_beacon(0);
                else if (time_in_state<10.0) point_beacon(-10);
                else if (time_in_state<15.0) point_beacon(-30);
                }
            }
            //state_scan_obstacles: Scan for obstacles
            else if (robot.state==state_scan_obstacles)
            {
                int scan_angle=45;
                if (time_in_state<10.0) { // line up the beacon correctly
                point_beacon(scan_angle);
                }
            }
            //state_drive_to_mine: Drive to mining area
            else if (robot.state==state_drive_to_mine)
            {
                if (drive_posture()) {
                double target_Y=field_y_mine_start; // mining area distance (plus buffer)
                double distance=target_Y-locator.merged.y;
                if (autonomous_drive(mine_target_loc,mine_target_angle) ||
                    distance<0.0)  // we're basically there now
                {
                    if (driver_test) enter_state(state_drive_to_dump);
                    else enter_state(state_mine_lower); // start mining!
                }
                
                }
            }

            //Enter Semiauto mine modes

            //state_mine_lower: enter mining state
            else if (robot.state==state_mine_lower) {
                tryMineMode();
                mine_start_time=cur_time; // update mine start time
                enter_state(state_mine);
            }
            else if (robot.state==state_mine)
            {
                if (!tryMineMode()) { // too high to mine (sanity check)
                robot.power.dump=power_full_bw; // lower bucket
                mining_head_lowered=true;
                }

                double mine_time=cur_time-mine_start_time;
                double mine_duration=250.0;
                if(mine_time>mine_duration)
                {
                    enter_state(state_mine_raise);
                } // done mining
                
                if (robot.sensor.Mstall) enter_state(state_mine_stall);
            }

            // state_mine_stall: Detect mining head stall. Raise head until cleared
            else if (robot.state==state_mine_stall)
            {
                tryMineMode(); // Start PID based mining
                if(robot.sensor.Mstall && time_in_state<1)
                {
                robot.power.dump=power_full_fw; // raise bucket
                }
                else {enter_state(state_mine);} // not stalled? Then back to mining
            }

            //state_mine_raise: Raise mining conveyor before starting to backup towards Lunarbin
            else if (robot.state==state_mine_raise)
            {
                if(drive_posture())
                enter_state(state_drive_to_dump);
            }

            // Drive back to trough
            else if (robot.state==state_drive_to_dump)
            {
                if (autonomous_drive(dump_target_loc,dump_target_angle) )
                {
                enter_state(state_dump_align);
                }
            }
            else if (robot.state==state_dump_align)
            {
                vec2 target=dump_align_loc;
                    target.y=locator.merged.y; // don't try to turn when this close
                if (autonomous_drive(target,dump_target_angle)
                || (fabs(locator.merged.y-target.y)<30 && fabs(locator.merged.x-field_x_trough_stop)<=10) )
                {
                if (driver_test) {
                    mine_target_loc.x=50+(rand()%250); // retarget in mining area every run
                    enter_state(state_drive_to_mine);
                }
                else enter_state(state_dump_contact);
                }
            }

             //Semiauto dump mode entry point: dock and dump mode
            else if (robot.state==state_dump_contact) // final backup to Lunarbin
            {
                back_up();
                if (time_in_state>1.0)
                {
                enter_state(state_dump_raise);
                }
            }

            // raise bucket to dump
            else if (robot.state==state_dump_raise)
            {
                enter_state(state_dump_pull);//2-19: Already in dump position
            }
            // Raise box
            else if(robot.state==state_dump_pull)
            {
                int howfast=32;
                int cur=(signed short)robot.sensor.Rcount;
                int target=box_raise_max;
                if (!speed_limit(howfast,cur,target,+1)  || time_in_state>15.0)
                enter_state(state_dump_rattle);
                else
                robot.power.roll=64+howfast; // forward
            }
            // Give dust time to flow out (maybe gentle rattle?)
            else if (robot.state==state_dump_rattle)
            {
                robot.power.dump=(fmod(time_in_state,0.2)>0.1)?power_full_fw:power_full_bw; // empty out conveyor (and rattle)
                if(time_in_state>2.0) {
                enter_state(state_dump_push);
                }
            }
            // Push box back down after dumping
            else if(robot.state==state_dump_push)
            {
                int howfast=32;
                int cur=(signed short)robot.sensor.Rcount;
                int target=0;
                if (!speed_limit(howfast,cur,target,-1)  || time_in_state>15.0)
                enter_state(state_drive_to_mine);
                else
                robot.power.roll=64-howfast; // backward
            }
            else if (robot.state==state_stow)
            {
                if(mining_head_lowered)
                drive_posture();
                if(time_in_state<20)
                robot.power.dump=127;
                enter_state(state_stowed);

            }
            else if (robot.state==state_stowed)
            {
                /* wait here forever */
            }
            else
            { // what?  unrecognized state?!  manual mode...
                robotPrintln("Autonomy: unrecognized state %s (%d)!?\n",state_to_string(robot.state), robot.state);
                enter_state(state_drive);
            }

            if (nodrive)
            { // do not drive!  (except for state_drive)
                robotPrintln("NODRIVE");
                set_drive_powers(0.0,0.0);
            }
        }