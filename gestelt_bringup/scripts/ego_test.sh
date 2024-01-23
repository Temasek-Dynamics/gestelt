#!/bin/bash

SESSION="ego_test"
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
# Start up ego planner and planner adaptor
CMD_0="
roslaunch gestelt_bringup back_end_planner.launch drone_id:=0
"

# CMD_0="
# roslaunch gestelt_bringup ego_planner_test.launch drone_id:=0
# "



# Start up central nodes
CMD_2="
roslaunch gestelt_bringup fake_map_central.launch rviz_config:=bubble
"

# Start up script to send commands
CMD_3="roslaunch gestelt_bringup mission_ego_test.launch"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    sleep 1
    # tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    # sleep 1
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
