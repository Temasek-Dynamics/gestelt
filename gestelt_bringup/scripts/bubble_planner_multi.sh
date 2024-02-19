#!/bin/bash

SESSION="bubble_planner_test"
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

#####
# Commands
#####
# Start drones with planner modules
CMD_0="
roslaunch gestelt_bringup multi_fake_drones.launch 
"

# Start up rviz
CMD_1="
roslaunch gestelt_bringup fake_map_central.launch rviz_config:=bubble
"

# Start up central bridge and nodes
# CMD_2="
# "

# Start up script to send commands
CMD_3="roslaunch gestelt_bringup mission_bubble_planner.launch"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    sleep 2
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    sleep 0.5
    # tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    # sleep 0.5
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
