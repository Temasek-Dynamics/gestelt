# PX4_SITL_setup

# Setup (Gazebo)
1. Refer to https://docs.px4.io/main/en/simulation/hitl.html for setup instructions on PX4 device

2. Set in [raynor.sdf](../gestelt_bringup/simulation/models/raynor/raynor.sdf):
    - `serialEnabled` to `1` 
    - `serialDevice` to `/dev/ttyACM0`. One way to check the port is to do `dmesg | grep "tty"`

Make sure this file is copied over to the PX4_Autopilot simulation folder. For example, do the following:
```bash
cp -r ~/gestelt_ws/src/gestelt/gestelt_bringup/simulation/models/raynor ~/gestelt_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
```

# Setup (QGroundControl)
1. Set the following vehicle parameters
    - COM_RC_IN_MODE to "Joystick/No RC Checks". This allows joystick input and disables RC input checks.
    - SYS_HITL to "1"
2. 


# Setup (PX4 HITL)
2. Build the firmware 
```bash 
# Default build
make px4_fmu-v2_default 
# HITL/SIH build for Flywoo F405 AIO
make flywoo_f405s_aio_sih
```
3. Upload firmware
```bash
make px4_fmu-v2_default upload
```

4. On PX4 nsh (Nuttx shell)
```bash
mavlink status streams
```


# Quick Start

```bash
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_central_hitl.sh http://192.168.31.173:11311 192.168.31.173

cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh http://192.168.31.173:11311 192.168.31.173
```


Start QGroundControl
```bash
~/Documents/QGroundControl.AppImage
```


# Example for testing

```bash
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default 
gazebo Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world
```

```bash
./radxa_central_hitl.sh http://localhost:11311 localhost
```

# Additional tools
https://mavlink.io/en/guide/wireshark.html

# Log
1. Test in SITL and get mavlink messages from wireshark
    - Seems to have the same type of messages as in HITL, one mesage of interest is the target_attitude, which publishes the thrust value. Both HITL and SITL publish the same message but somehow only Gazebo is able to receive these values in SITL.
2. Test with PX4 FMUV2 but with an older firmware
    - Same results
3. Build custom firmware
    - One possible problem is that `pwm_out_sim` is disabled
        - Look at https://github.com/PX4/PX4-Autopilot/issues/11203
        - Try adding CONFIG_MODULES_SIMULATION_PWM_OUT_SIM=y and buiding an image out of it
            - pwm_out_sim
4. Study source code to see what topics are being published/subscribed to 
    - Mavlink interface
    - pwm_sim_out
        - Takes `actuator_control` uORB messages,
mix them with any loaded mixer and output the result to the
`actuator_output` uORB topic.
    - 

5. Looked at the following issues:
    - [px4io: When running HITL, don't publish actuator_outputs. Fixes #13471](https://github.com/PX4/PX4-Autopilot/pull/13488)
    - [Custom PX4 Simulator - No HIL_ACTUATOR_CONTROLS message](https://discuss.px4.io/t/custom-px4-simulator-no-hil-actuator-controls-message/23791/4)


6. Potential issues
    - datarate not enough ("mav_link_main.cpp" and "mavlink_receiver.cpp")
        - Try different cable and USB Port with higher data rate 
    - 

7. Test with other PX4 devices (SAFMC FCU running PX4)
    - 

# TODO 
2. Change geometry for px4 params

# Data

Opened serial device /dev/ttyACM0
[ INFO] [1688458435.199605432]: udp0: Remote address: 127.0.0.1:57640
[ INFO] [1688458435.199754190]: IMU: Attitude quaternion IMU detected!
[ INFO] [1688458435.199797309]: IMU: High resolution IMU detected!
[ INFO] [1688458435.201001995]: CON: Got HEARTBEAT, connected. FCU: PX4 Autopilot
[ INFO] [1688458435.217695834, 0.200000000]: IMU: Scaled IMU message used.
[ INFO] [1688458435.223244042, 0.204000000]: IMU: Attitude quaternion IMU detected!
[ INFO] [1688458435.223692447, 0.204000000]: IMU: High resolution IMU detected!
[ WARN] [1688458435.224656021, 0.208000000]: TM : RTT too high for timesync: 18446743973910.26 ms.
[drone0/iris_0_spawn-7] process has finished cleanly
log file: /home/john/.ros/log/b667090a-1a42-11ee-b316-91f43256decb/drone0-iris_0_spawn-7*.log
[ INFO] [1688458436.209256699, 1.188000000]: GF: Using MISSION_ITEM_INT
[ INFO] [1688458436.209288206, 1.188000000]: RP: Using MISSION_ITEM_INT
[ INFO] [1688458436.209301082, 1.188000000]: WP: Using MISSION_ITEM_INT
[ INFO] [1688458436.209324921, 1.188000000]: VER: 1.1: Capabilities         0x000000000000ecff
[ INFO] [1688458436.209344490, 1.188000000]: VER: 1.1: Flight software:     010e0000 (bb0f2875a9000000)
[ INFO] [1688458436.209357603, 1.188000000]: VER: 1.1: Middleware software: 010e0000 (bb0f2875a9000000)
[ INFO] [1688458436.209373907, 1.188000000]: VER: 1.1: OS software:         0b0000ff (3f77354c0dc88793)
[ INFO] [1688458436.209386968, 1.188000000]: VER: 1.1: Board hardware:      00000011
[ INFO] [1688458436.209399558, 1.188000000]: VER: 1.1: VID/PID:             26ac:0011
[ INFO] [1688458436.209415832, 1.188000000]: VER: 1.1: UID:                 3233470e31363032
[ INFO] [1688458450.027096851, 15.004000000]: GF: mission received
[ INFO] [1688458450.031056392, 15.008000000]: WP: mission received
[ INFO] [1688458450.035047079, 15.012000000]: RP: mission received

