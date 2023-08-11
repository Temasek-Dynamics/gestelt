#!/bin/bash
export ROS_DISTRO="noetic"

# Install ROS-Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# Install dependency packages
sudo apt install git build-essential tmux python3-vcstool python3-catkin-tools xmlstarlet cron chrony -y
sudo apt install ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
sudo apt install ros-${ROS_DISTRO}-roslint ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs -y
sudo apt install libopencv-dev ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-pcl-ros libeigen3-dev -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Add your usergroup to dialout
sudo usermod -a -G dialout $USER

# Set up workspace and clone gestelt framework
cd ~/
mkdir -p ~/gestelt_ws/src/
cd ~/gestelt_ws/src
git clone https://github.com/JohnTGZ/gestelt.git --branch sitl
cd gestelt
