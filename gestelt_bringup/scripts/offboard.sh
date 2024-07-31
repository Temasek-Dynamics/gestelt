#!/bin/bash

SESSION="offboard"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Get arguments
#####
# getopts: function to read flags in input
# OPTARG: refers to corresponding values
while getopts i: flag
do
    case "${flag}" in
        i) DRONE_ID=${OPTARG};; 
    esac
done

#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
gestelt_bringup_DIR="$SCRIPT_DIR/.."

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../../devel/setup.bash &&
"

#####
# Commands
#####
# Start bridge with FCU
CMD_0="
roslaunch --wait gestelt_bringup offboard_px4.launch drone_id:=${DRONE_ID}
"

# Start up rviz
CMD_1="
roslaunch --wait gestelt_bringup offboard_fake_sensor.launch drone_id:=${DRONE_ID}
"

# Start up central bridge and nodes
CMD_2="
roslaunch --wait gestelt_bringup offboard_planner.launch drone_id:=${DRONE_ID}
"

# Start up script to send commands
CMD_3=""

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    # tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
