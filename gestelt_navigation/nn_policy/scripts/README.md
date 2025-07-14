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


Scripts And their Different Functionality. Basically listen to the correct input topics, runs this through policies and then outputs. No transformation is done here. The only transformations are done in trajectory_servers.

1. Main script - policy_vel.py
    This script is only used for running warp training files where y axis is the upright position. This requires the trajectory_server to publish odom in the warp global frame. The policy then outputs thrust in body frame and body rates in the warp global frame. Then it gets sent to trajectory_server where it gets converted to PX4 body frame. In order to run, we can do the following
        a. rostopic pub /mode_change std_msgs/Bool "data: true"  - This changes attitude mode. Because I can feed either attitude or attitude rate control mode to PX4. If it is true then it is in the attitude rate control mode.
        b. rostopic pub /traj_server/warp_mission_command std_msgs/Int8 "data: 2"  - This changes the different modes that trajectory_server feeds actions to PX4. 1. is position mode. 2. is attitude/attitude rate control mode. 3. velocity control mode (Note that velocity setpoints are in the nwu global frame). and 4. is the geometric controller mode, which is supposed to be used with mavros geometric controller. Will update on this more next time.

2. Updated main script - policy_vel_combined.py
    This script is used and can work with running warp trained policies where either y axis is the upright position (as per point 1) or z axis is the upright position (nwu frame). If z axis is in the upright frame, then trajectory server do not have to transform the global nwu frame to anything. The policy basically just takes in global position and quaternion (from mavros/local_position/pose) and body px4 frame (from mavros/local_position/odom). Also the output of the policy is in px4 body frame as well, including the body rates. So no transform is necessary. This makes it faster. The following needs to be taken note of:
        a. The script reads the training config file. In the training config file, there's an indication of whether it is trained using the first or second mode.
        b. Depending on the mode, it will then listen out to different inputs and pass it through the neural network accordingly.

3. policy_vel_onnx.py - This works for onboard computing. Converting the policy from torch to onnx speeds things up.