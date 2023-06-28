#!/bin/bash
export ROS_DISTRO="noetic"

# Install dependency packages
sudo apt install ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
sudo apt install git build-essential tmux python3-vcstool xmlstarlet -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh

# Set up workspace and clone gestelt framework
mkdir -p ~/gestelt_ws/src/
cd ~/gestelt_ws/src
git clone https://github.com/JohnTGZ/gestelt.git
cd gestelt

# Build workspace
cd ~/gestelt_ws/
catkin_make -DCMAKE_BUILD_TYPE=Release