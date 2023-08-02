# ROS-related
export ROS_DISTRO="noetic"
source /opt/ros/noetic/setup.bash
alias sros="source /opt/ros/noetic/setup.bash"
alias sws="source devel/setup.bash"
alias sbash="source /home/rock/.bashrc"

# Multi-machine ROS communication
export MASTER_IP=192.168.31.173
export SELF_IP=192.168.31.166
#export MASTER_IP=localhost
#export SELF_IP=localhost

export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP

# Convenience function
alias pull_repo="git -C ~/gestelt_ws/src/gestelt/ pull"
alias ez_make="cd ~/gestelt_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES='rviz_plugins;swarm_bridge;central_benchmark;'"
alias cd_scripts="cd /home/rock/gestelt_ws/src/gestelt/gestelt_bringup/scripts/"

