#!/bin/bash

SESSION="central_sim_uav"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."

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
roslaunch gestelt_bringup offboard_ego_planner.launch drone_id:=1
"
CMD_1="
roslaunch gestelt_bringup offboard_ego_planner.launch drone_id:=2
"
CMD_2="
roslaunch gestelt_bringup offboard_ego_planner.launch drone_id:=3
"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 

fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
