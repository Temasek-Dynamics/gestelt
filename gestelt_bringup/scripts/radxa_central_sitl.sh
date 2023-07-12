#!/bin/bash

SESSION="radxa_central"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
gestelt_bringup_DIR="$SCRIPT_DIR/.."
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../../PX4-Autopilot"

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../../../devel/setup.bash &&
"
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic &&
"

#####
# Commands
#####
# CMD_0="
# roslaunch gestelt_bringup central_sim.launch world_name:=$SCRIPT_DIR/../simulation/worlds/empty.world ros_master_uri:=${ros_master_uri} ros_ip:=${ros_ip}
# "

CMD_0="
roslaunch gestelt_bringup central_sim.launch world_name:=$SCRIPT_DIR/../simulation/worlds/ego_test.world ros_master_uri:=$ROS_MASTER_URI ros_ip:=$ROS_IP
"

CMD_1="
roslaunch gestelt_bringup central_bridge.launch ros_master_uri:=$ROS_MASTER_URI ros_ip:=$ROS_IP
"

CMD_2="
roslaunch gestelt_bringup rviz.launch config:=gz_sim ros_master_uri:=$ROS_MASTER_URI ros_ip:=$ROS_IP
"

CMD_3="roslaunch gestelt_bringup radxa_mission.launch ros_master_uri:=$ROS_MASTER_URI ros_ip:=$ROS_IP"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $CMD_0" C-m 
    sleep 3
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" 

fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
