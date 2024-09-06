#!/bin/bash

SESSION="scenario_sh"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Get arguments
#####
# getopts: function to read flags in input
# OPTARG: refers to corresponding values
while getopts s: flag
do
    case "${flag}" in
        s) SCENARIO=${OPTARG};; 
    esac
done

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
roslaunch gestelt_bringup sitl_central.launch 
"

# Start up rviz
CMD_1="
roslaunch --wait gestelt_bringup fake_map_central.launch scenario:=$SCENARIO
"

# CMD_2="killall -9 gazebo; killall -9 gzserver; killall -9 gzclient; killall -9 rosmaster; tmux kill-server;"
CMD_2="rosrun mpc_controller mpc_node.py"

# Start up script to send commands
CMD_3="roslaunch --wait gestelt_bringup scenario_mission.launch scenario:=$SCENARIO"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" 
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
