#!/bin/bash
export ROS_DISTRO="noetic"

# Install dependency packages
sudo apt install git build-essential tmux python3-vcstool xmlstarlet -y
sudo apt install ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
sudo apt install ros-${ROS_DISTRO}-roslint ros-${ROS_DISTRO}-gazebo-msgs ros-${ROS_DISTRO}-tf2-geometry-msgs -y
sudo apt install libopencv-dev ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-pcl-ros libeigen3-dev -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Set up workspace and clone gestelt framework
mkdir -p ~/gestelt_ws/src/
cd ~/gestelt_ws/src
git clone https://github.com/JohnTGZ/gestelt.git --branch sitl
cd gestelt

# Build workspace
cd ~/gestelt_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES="rviz_plugins"
