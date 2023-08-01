#!/bin/bash

SESSION="test_vicon"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
gestelt_bringup_DIR="$SCRIPT_DIR/.."

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
roslaunch gestelt_bringup vicon_client.launch 
"

CMD_1="
roslaunch gestelt_bringup rviz.launch config:=gz_sim 
"

CMD_2="
roslaunch gestelt_bringup vicon_tf.launch
"

if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_0" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_2" C-m 
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
