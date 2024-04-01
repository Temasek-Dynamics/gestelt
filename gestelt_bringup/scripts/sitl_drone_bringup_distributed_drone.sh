#!/bin/bash

SESSION="gz_sim_single_uav"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
gestelt_bringup_DIR="$SCRIPT_DIR/.."
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../PX4-Autopilot"

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../../devel/setup.bash &&
"
# export ROS_MASTER_URI (for distributed simulation)
# drone's side ROS_MASTER_URI should be the laptop
EXPORT_ROS_MASTER_URI="
export ROS_HOSTNAME=172.20.10.2 &&
export ROS_MASTER_URI=http://172.20.10.4:11311
"

# PX4 v1.13.0
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/sitl_gazebo &&
"

# let the cpu run in the highest performance
CPU_PERFORMANCE="
cpufreq-set -g performance
"

#####
# Commands
#####
# Start Gazebo and PX4 SITL instances
CMD_0="
roslaunch gestelt_bringup sitl_drone.launch 
"

# Start up drone commander (Handles taking off, execution of mission and landing etc.)
# trajectory_server_SE3_node: geometric controller
# trajectory_server_node: PX4 RPT controller
CMD_1="
roslaunch trajectory_server trajectory_server_SE3_node.launch rqt_reconfigure:=false
"

# Start up minimum snap trajectory planner and sampler 
CMD_2="
roslaunch trajectory_planner trajectory_planner_node.launch
"

# Start up script to send commands
CMD_3="roslaunch gestelt_bringup mission_sitl.launch record:=false"


# Start up a separate SE3 controller
CMD_4="taskset -c 0 roslaunch se3_controller se3_controller.launch"
# disarm drone
# CMD_4="rosservice call /drone_commander/disarm"
# CMD_4="rosrun mavros mavparam set COM_RCL_EXCEPT 4"
if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $EXPORT_ROS_MASTER_URI $CMD_1" C-m 
    sleep 2
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $EXPORT_ROS_MASTER_URI $CMD_2" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $EXPORT_ROS_MASTER_URI $CMD_3" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $EXPORT_ROS_MASTER_URI $CMD_4" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
