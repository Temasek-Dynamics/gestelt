traj_server.cpp

This file serves as the manager to take off, execute mission mode using the PX4 flight controller.
It listens out for the config file in gestelt_bringup/config/traj_server_default.yaml. Important parameters are: 
    a. mission_command_mode: 1 - always start with 1 because this brings it to position hold mode
    b. pub_cmd_freq: This controls the controls publication to PX4. Keep it at 100Hz. The policy_vel_*.py files will send the correct frequency over.
    c. to_transform_odom: This decides if there is a need to perform frame transformation for odom and quaternions. 
    d. to_transform_policy: This decides if there is a need to perform frame transformation for the control actions.
    e. warp_jax: Warp_jax has different needs. Jax needs the odom to be transformed to global nwu frame from the body frame. But does not need output policy actions to be transformed. For warp, look at policy_vel_*.py README files.
    f. position_control: This is if the policy is trained for position control. Generally true.
    g. training params are not quite so important for traj_server. More for policy_vel_*.py files.

Also note that policy_vel_*.py file will cross-check these params with the training config to prevent errors.

