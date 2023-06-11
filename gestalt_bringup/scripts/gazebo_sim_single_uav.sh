#!/bin/bash

SESSION="quick_start_sesh"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

# Directories
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../../PX4-Autopilot"

# SOURCE_PX4_AUTOPILOT="
# source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
# source $SCRIPT_DIR/../../../../devel/setup.bash &&
# "
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
source $SCRIPT_DIR/../../../../devel/setup.bash &&
"

ADD_PX4_PACKAGE_PATH="
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic &&
"

CMD_0="
roslaunch gestalt_bringup gazebo_single_uav.launch world_name:=$SCRIPT_DIR/../simulation/worlds/ego_test.world
"
CMD_1="roslaunch px4 px4.launch ID:=0"
CMD_2="roslaunch px4 px4.launch ID:=1"

# CMD_2="roslaunch mavros px4.launch fcu_url:="udp://:14540@0.0.0.0:14550""

# UDP ports (PX4 <-> Mavlink communication)
# 14550: Communication with ground control stations
# 14540: Communication with offboard APIs (for multi vehicle simulations, PX4 sequentially allocates a seperate remote port from 14540 to 14549)

# UDP ports (PX4 <-> Simulator)
# 4560: Simulator listens to this port and PX4 initiates a TCP connection to it

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $CMD_0" C-m 
    sleep 3
    tmux send-keys -t $SESSION:0.1 "$ADD_PX4_PACKAGE_PATH $CMD_1" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$ADD_PX4_PACKAGE_PATH $CMD_2" C-m 
    # tmux send-keys -t $SESSION:0.2 "$CMD_2" C-m 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
