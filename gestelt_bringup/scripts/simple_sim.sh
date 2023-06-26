#!/bin/bash

SESSION="formation_sesh"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#####
# Sourcing
#####
SOURCE_WS="source $SCRIPT_DIR/../../../../devel/setup.bash &&"

#####
# Commands
#####
CMD_0="roslaunch gestelt_bringup rviz.launch config:=gz_sim"
CMD_1="roslaunch gestelt_bringup simple_multi_uav.launch"
CMD_2="rosrun gestelt_bringup mission_startup_and_send_wp.py"
# CMD_3="rostopic pub /traj_server_event std_msgs/Int8 \"data: 0 \" "

# rostopic pub /traj_server_event std_msgs/Int8 "data: 0" 
# rostopic pub /traj_server_event std_msgs/Int8 "data: 2" 

if [ "$SESSIONEXISTS" = "" ]
then 
    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_WS $CMD_0" C-m 
    sleep 2
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    # tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
