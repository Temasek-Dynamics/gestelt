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
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR
"
# export ROS_MASTER_URI (for distributed simulation)
# drone's side ROS_MASTER_URI should be the laptop
EXPORT_ROS_MASTER_URI="
export ROS_HOSTNAME=$SELF_IP &&
export ROS_MASTER_URI=http://$MASTER_IP:11311
"

# PX4 v1.13.0
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/sitl_gazebo &&
"



#####
# Commands
#####
# Start Gazebo and PX4 SITL instances
# CMD_0="
# roslaunch gestelt_bringup sitl_drone.launch 
# "

# Start up drone commander (Handles taking off, execution of mission and landing etc.)
CMD_1="
taskset -c 2 roslaunch trajectory_server trajectory_server_node.launch rviz_config:=gz_sim
"

# Start up script to send commands
CMD_2="roslaunch gestelt_bringup circular_mission.launch"

CMD_3="roslaunch gestelt_bringup standard_trajectory_publisher.launch simulation:=true"

CMD_4="
roslaunch gestelt_bringup record.launch record_platform:=drone test_mode:=HITL
"
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
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $EXPORT_ROS_MASTER_URI" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
