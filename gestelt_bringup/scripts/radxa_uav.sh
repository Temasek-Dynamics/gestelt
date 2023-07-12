#!/bin/bash

if [ "$#" != 1 ]; then 
    echo -e "usage: ./radxa_uav uav_id \n"
fi

uav_id=$1

SESSION="radxa_uav"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../../../devel/setup.bash &&
"

#####
# Commands
#####
CMD_0="
roslaunch gestelt_bringup radxa_ego_planner.launch drone_id:=${uav_id} ros_master_uri:=$ROS_MASTER_URI ros_ip:=$ROS_IP
"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
