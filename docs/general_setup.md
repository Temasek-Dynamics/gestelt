# Gestelt setup

## Requirements
1. Operating System/Frameworks
    - Ubuntu 20.04 (ROS Noetic)

## Installation instructions

1. Install binaries
```bash 
export ROS_DISTRO="noetic"

# Install ROS dependencies
sudo apt-get install ros-noetic-octomap -y
# sudo apt install ros-${ROS_DISTRO}-tf2-sensor-msgs -y
sudo apt install ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
# Install external dependencies
sudo apt install git build-essential tmux python3-catkin-tools python3-vcstool xmlstarlet -y
sudo apt install libopencv-dev ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-pcl-ros libeigen3-dev libgoogle-glog-dev -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
# Install tools for VICON 
sudo apt-get install ros-${ROS_DISTRO}-vrpn-client-ros
# Extra tools for debugging
sudo snap install plotjuggler
```

2. Clone repositories
```bash
mkdir -p ~/gestelt_ws/src/
cd ~/gestelt_ws/src
git clone https://github.com/JohnTGZ/gestelt.git
cd gestelt
vcs import < simulators.repos --recursive
vcs import < thirdparty.repos --recursive
```

3. Install PX4 firmware
```bash
# cd to PX4-Autopilot repo
cd ~/gestelt_ws/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh 
# Make SITL target for Gazebo simulation
DONT_RUN=1 make px4_sitl_default gazebo-classic
cp -r ~/gestelt_ws/src/gestelt/gestelt_bringup/simulation/models/raynor ~/gestelt_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/

# IF you screw up the PX4 Autopilot build at any point, clean up the build files via the following command:
make distclean
```

4. Building the workspace
```bash
# Assuming your workspace is named as follows
cd ~/gestelt_ws/

# Building for debugging/development
catkin build 
# Building for release mode (For use on Radxa)
catkin build -DCMAKE_BUILD_TYPE=Release
```

5. Setting up the Offboard computer
We assume that Radxa is being used as the offboard computer. Refer to [radxa_setup](./radxa_setup.md) for more instructions.

6. Setting up PX4 (Flywoo board)
Refer to [px4_setup](./px4_setup.md) for more instructions.

7. Vicon setup
Use the vicon if you would like to get the vehicle ground truth. Refer to [vicon_setup](./vicon_setup.md) for more instructions.

## Quick start
1. Start up simulation scripts
```bash
# Multi vehicle simulation using gazebo
~/gestelt_ws/src/gestelt/gestelt_bringup/scripts/gz_multi_uav/gz_sim_multi_uav.sh
# Single vehicle simulation using gazebo
~/gestelt_ws/src/gestelt/gestelt_bringup/scripts/gz_single_uav/gz_sim_single_uav.sh

```
2. This should start up a Tmux terminal, and one of the windows should show an option to run the trajectory server.

```bash
# Taking off
rostopic pub /traj_server_event std_msgs/Int8 "data: 0" --once
# Landing
rostopic pub /traj_server_event std_msgs/Int8 "data: 1" --once
# Switch to Mission mode
rostopic pub /traj_server_event std_msgs/Int8 "data: 2" --once
# Switch to hover mode
rostopic pub /traj_server_event std_msgs/Int8 "data: 3" --once
# EMERGENCY STOP!
rostopic pub /traj_server_event std_msgs/Int8 "data: 4" --once
```
