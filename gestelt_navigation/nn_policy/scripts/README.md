Key learning points:
Things to take note:
1. The state and odom output from PX4 has to match the inference rate of the innerloop controller. It has to be at least 100Hz. The way the diff sim trains has to comensurate with the implementation. That means the time step used in the training has to be 1/100. 
    a. To adjust the state and odom, go into PX4-Autopilot and find the mavlink_main.cpp and change the configure_stream_local("HIGHRES_IMU", 100.0f);
    configure_stream_local("LOCAL_POSITION_NED", 100.0f);
2. The output to setpoint_raw/attitude has to be 100 Hz and has to correspond to the training as well with the time step used in training 1/100
3. For position control, make sure the position and the target set during testing is the same as that with training.
3. For velocity control, make sure that the target velocity set during testing is the same as that with training.
4. For training, it is important that the max single thrust and the max attitude rate is the same as that with training. Perhaps there should be a common config file.



5. Real drone weight is 242.5g. nominal collective thrust is 0.33 during hover. Diff sim drone weight is currently 234g. action_ref is currently 0.286. Need to increase it.