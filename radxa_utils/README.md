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
alias ez_make="cd ~/gestelt_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES='rviz_plugins;manual_take_over;odom_visualization;pose_utils;uav_utils;drone_detect'"
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
# Matchine A
export ROS_MASTER_URI=http://192.168.1.112:11311
export ROS_HOSTNAME=192.168.1.112
export ROS_IP=192.168.1.112

# Matchine B
export ROS_MASTER_URI=http://192.168.1.112:11311
export ROS_HOSTNAME=192.168.1.134
export ROS_IP=192.168.1.134
```

# Quick Start

## Central Computer and radxa
```bash
# On the central flight control computer
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_central_sitl.sh http://192.168.1.112:11311 192.168.1.112

# On the UAV (ID 0)
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh 0 http://192.168.1.112:11311 192.168.1.134
```

# TODO
Communication between different machines happen on a wifi network

1. (PX4 SITL + Gazebo on PC) <-> (Egoplanner on Radxa)   
    -   Data
        - Bandwidth: Using `iperf`
            - Bandwidth:
                - TCP: 57.3 Mbits/sec 
                - UDP: 90Mbits/sec - 101 Mbits/sec
        - Latency: Using `sudo mtr --no-dns --report --report-cycles 60 IP_ADDR`
            - Latency averages 14.8 ms. Best is 8.7 ms, Worst is 32.1 ms.
        - CPU Usage: Using `htop`
            - it is shown to use around (55%, 30%, 30%, 30%) for the Radxa's 4 cores
    - Sources of large bandwidth
        - Depth camera sensor data
    - Potential solutions to reduce bandwidth required
        - Compress depth image before sending over. Or only send point clouds over (Which have been compressed)
2. (PX4 HITL) + (Gazebo + Egoplanner on PC)
    - Can be done with Flywoo F405S
    - Cannot be done with PX4 FMUV2 (Pixhawk 1 FCU). FCU's flash memory is too low to take off additional HITL modules.
    
3. (PX4 HITL) <-> (Gazebo on PC) <-> (Egoplanner on Radxa)


# Metrics to measure
MAKE SURE TO BUILD IN RELEASE MODE

- Record metrics 
    - Communications 
        - Signal strength to wifi network
            - `iw dev wlan0 link` or `watch -n1 iwconfig`
        - Bandwidth (Network capacity), total messages sizes
            - `sudo iftop`
        - Network Speed 
            - TCP Mode
                - On PC A, set up server `iperf -s`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP`
            - UDP Mode
                - On PC A, set up server `iperf -s -u`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP -u -b 1000m`
        - Network Latency
            - use `sudo mtr --no-dns --report --report-cycles 60 IP_ADDR` or `ping`
        - ROS
            - Measure message size: `rostopic bw`
            - Measure frequency of publishing: `rostopic hz`
            - Measure delay: `rostopic delay`
            - Topics of interest:
                - On radxa
                    - /drone0/camera/depth/image_raw
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

    - Hardware (Radxa)
        - CPU Usage
            - Use `htop`
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



# Replan 