# radxa_utils
This package contains useful scripts and launch files to setup and test the radxa communications.

# Setting up the Radxa

```bash

# Install tools
sudo pip3 install pyamlboot

# Use lsusb command to check if radxa is connected
lsusb
# Use dmesg to find the device filepath
sudo dmesg

# To erase the eMMC 
# https://wiki.radxa.com/Zero/install/eMMC_erase
sudo boot-g12.py radxa-zero-erase-emmc.bin

# Get images from https://wiki.radxa.com/Zero/downloads
xz -v --decompress IMAGE_COMPRESSED

# Flash the image using balena etcher

# Proceed on activating WIFI using https://wiki.radxa.com/Zero/Ubuntu
# Root username and pasword is rock/rock
sudo su
nmcli r wifi on
nmcli dev wifi
nmcli dev wifi connect "wifi_name" password "wifi_password"                   

# Install ROS at http://wiki.ros.org/noetic/Installation/Ubuntu

# Run setup script
./radxa_setup.sh

# Copy .bashrc configuration file over
export ROS_DISTRO="noetic"
source /opt/ros/noetic/setup.bash
alias sros="source /opt/ros/noetic/setup.bash"
alias sws="source devel/setup.bash"
alias sbash="source ~/.bashrc"

# Set up network
# http://wiki.ros.org/ROS/NetworkSetup
export ROS_MASTER_URI=http://MASTER_IP:11311
export ROS_HOSTNAME=OWN_IP
export ROS_IP=OWN_IP

# Convenience function
alias pull_repo="git -C ~/gestelt_ws/src/gestelt/ pull"
alias ez_make="cd ~/gestelt_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES='rviz_plugins;'"
alias cd_scripts="cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts/"
```

# Testing communications
Assuming we just have 2 machines A (192.168.1.112) and B (192.168.1.134) connected to the same LAN.
They all have to have the same ROS_MASTER_URI. In this case, the ROS_MASTER_URI is "http://192.168.1.112:11311". A is the talker and B is the listener.

```bash
# Use ifconfig to figure out their IPs within the LAN

# On machine A:
roslaunch radxa_utils talker.launch ros_master_uri:=http://192.168.1.112:11311 ros_ip:=192.168.1.112

# On machine B:
roslaunch radxa_utils listener.launch ros_master_uri:=http://192.168.1.112:11311 ros_ip:=192.168.1.134
```

Setting environment variables manually
```bash
# Xiaomi network
    # Central PC
        export MASTER_IP=192.168.31.173
        export SELF_IP=192.168.31.173
    # UAV
        export MASTER_IP=192.168.31.173
        export SELF_IP=192.168.31.166

# Nvidia network
    # Central PC
        export MASTER_IP=192.168.1.112
        export SELF_IP=192.168.1.112
    # UAV
        export MASTER_IP=192.168.1.112
        export SELF_IP=192.168.1.134


# Matchine A (Central PC)
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP

# Matchine B (UAV PC)
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP
```

# Quick Start

## Central Computer and radxa
```bash
# On the central flight control computer
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_central_sitl.sh 

# On the UAV (ID 0)
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh 0
```

# TODO
1. (Egoplanner on Radxa) <--wireless--> (PX4 SITL and Gazebo on PC) 
    - Xiaomi Network (Central PC and Radxa connected wirelessly to network)
        - Bandwidth: 
            - TCP: 57.3 Mbits/sec -> 7.1625 mb/s
            - UDP: 90Mbits/sec - 101 Mbits/sec  -> 11.25 mb/s
        - Latency
            - Latency averages 14.8 ms. Best is 8.7 ms, Worst is 32.1 ms.
    - Nvidia Network (Central PC Wired to network, while Radxa is connected wirelessly)
        - ROS Topics:
            - On radxa
                - /drone0/camera/depth/image_raw (840 * 640)
                    - 2.4 ghz 
                        - Delay: 0.75 (0.004)
                        - Bandwidth: 2.88mb/s (16.65 mb/s)
                    - 5.0 ghz
                        - Delay:
                        - Bandwidth:
                - /drone0/mavros/local_position/pose
                    - 2.4 ghz
                        - Delay: 0.03 (0.006)
                        - Bandwidth: 2.6 kb/s (2.6 kb/s)
                    - 5.0 ghz
                        - Delay:
                        - Bandwidth:
            - On PC
                - /drone0/mavros/setpoint_raw/local
                    - 2.4 ghz
                        - Delay: 0.7 ()
                        - Bandwidth: 2.5kb/s (2.7kb/s)
                    - 5.0 ghz
                        - Delay:
                        - Bandwidth:
                - /drone_0_ego_planner_node/grid_map/occupancy
                    - 2.4 ghz
                        - Delay: 2280 (2500)
                        - Bandwidth: 68 kb/s (2.5kb/s)
                    - 5.0 ghz
                        - Delay:
                        - Bandwidth:
2. (PX4 HITL) + (Gazebo + Egoplanner on PC)
    - Cannot be done with PX4 FMUV2 (Pixhawk 1 FCU). FCU's flash memory is too low to take off additional HITL modules.
    - Can be done with Flywoo F405S
3. (PX4 Drone <--serial--> Egoplanner on Radxa) <--wireless--> (Gazebo on PC)

# Metrics to measure
MAKE SURE TO BUILD IN RELEASE MODE

- Explanation of metrics 
    - Network 
        - Signal strength 
            - `iw dev wlan0 link` or `watch -n1 iwconfig`
        - Bandwidth (Network capacity), total messages sizes
            - Maximum rate that information can be transferred
            - `sudo iftop`
        - Throughput 
            - Actual rate that information is transferred
            - TCP Mode
                - On PC A, set up server `iperf -s`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP`
            - UDP Mode
                - On PC A, set up server `iperf -s -u`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP -u -b 1000m`
        - Latency
            - Delay between sender and receiver decoding it. Function of signals travel time, and processing time at any nodes the information traverses 
            - use `sudo mtr --no-dns --report --report-cycles 60 IP_ADDR` or `ping`
        - Jitter
            - Variation in packet delay at the receiver of the information
        - ROS
            - Measure message size: `rostopic bw`
            - Measure frequency of publishing: `rostopic hz`
            - Measure delay: `rostopic delay`
    - Hardware (Radxa)
        - CPU Usage `htop`
        - Wall time (aka 'real' time )
            - Elapsed time from start to finish of the call. Includes time slices used by other processes and the time the process spends blocked (waiting for I/O to complete
            )
        - CPU Runtime (aka 'user' time)
            - Amount of CPU time spent in user-mode code (outside the kernel) within the process. Only actual CPU time used in executing the process.
        - System time
            - Amount of time spent within the kernel, as opposed to library code. Could include I/O, allocating memory.
        - Sometimes Sys + user > real, as multiple processors work in parallel.
        - What is important to us is the 'real' time as we want to ensure that the planner is able to plan with a high enough frequency
        - Measure for planner, Depth map

    - Integration tests 
        - Replan frequency
        - Tracking error
        - Success rate
        - Maximum flight speed it can enable in sparse/dense environment
        - Maximum number of UAVs it can handle

## PID Tuning Guide
- Tune PID 
    - Rate Controller
        - ROLL
            - MC_ROLLRATE_K: 1.0
            - MC_ROLLRATE_P: 0.249
                - Enhance damping of the roll channel, faster attenuation of the oscillation
            - MC_ROLLRATE_D: 0.0046
            - MC_ROLLRATE_I: 0.325
        - PITCH
            - MC_PITCHRATE_K: 1.0
            - MC_PITCHRATE_P: 0.233
            - MC_PITCHRATE_D: 0.0044
            - MC_PITCHRATE_I: 0.3
        - YAW
            - MC_YAWRATE_K: 1.0
            - MC_YAWRATE_P: 0.18
            - MC_YAWRATE_I: 0.18
    - Attitude Controller
        - MC_ROLL_P
            - 3.84
            - To reduce amplitude of oscillation
        - MC_PITCH_P
            - 4.1
        - MC_YAW_P
            - 5.26
    - Velocity Controller
        - (horizontal)
            - MPC_XY_VEL_P_ACC
                - 3.25
            - MPC_XY_VEL_I_ACC
                - 0.4
            - MPC_XY_VEL_D_ACC
                - 0.2
        - (vertical)
            - MPC_Z_VEL_P_ACC
                - 4.0
            - MPC_Z_VEL_I_ACC
                - 2.0
            - MPC_Z_VEL_D_ACC
                - 0.0
    - Position Controller
        - (horizontal)
            - MPC_XY_P 
                - 0.8
                - Reduce position control gain
        - (vertical)
            - MPC_Z_P 
                - 1.0
                - Reduce overshoot of position.