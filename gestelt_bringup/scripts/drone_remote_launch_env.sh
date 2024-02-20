#!/bin/bash
export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_HOSTNAME=172.20.10.3 
export ROS_MASTER_URI=http://172.20.10.4:11311
source /opt/ros/noetic/setup.bash
source ~/gestelt_ws/devel/setup.bash
exec "$@"
