#!/bin/bash

SESSION="docker_sitl_laptop"
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
export ROS_IP=10.42.0.1
export ROS_HOSTNAME=10.42.0.1 &&
export ROS_MASTER_URI=http://10.42.0.1:11311
"

# PX4 v1.13.0
# SOURCE_PX4_AUTOPILOT="
# source $PX4_AUTOPILOT_REPO_DIR/Tools/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/sitl_gazebo &&
# "

# PX4 v1.14.0
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic &&
"
#####
# Commands
#####
# Start Gazebo and PX4 SITL instances
CMD_0="
roslaunch gestelt_bringup sitl_drone.launch 
"


CMD_2="
roslaunch gestelt_bringup record.launch --wait
"

# disarm drone
# CMD_4="rosservice call /drone_commander/disarm"
# CMD_4="rosrun mavros mavparam set COM_RCL_EXCEPT 4"
if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $EXPORT_ROS_MASTER_URI $CMD_0" C-m 
    # sleep 1
    # tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $EXPORT_ROS_MASTER_URI " #C-m $CMD_1
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $EXPORT_ROS_MASTER_URI $CMD_2" C-m 
    # sleep 1
    # tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $EXPORT_ROS_MASTER_URI " #C-m $CMD_3
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
