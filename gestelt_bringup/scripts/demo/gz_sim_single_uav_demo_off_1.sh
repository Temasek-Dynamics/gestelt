#!/bin/bash

SESSION="gz_sim_single_uav_demo_off_1"
SESSIONEXISTS=$(tmux list-sessions | grep $SESSION)

#####
# Directories
#####
SCRIPT_DIR="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )/.."
gestelt_bringup_DIR="$SCRIPT_DIR/.."
PX4_AUTOPILOT_REPO_DIR="$SCRIPT_DIR/../../../../PX4-Autopilot"

#####
# Sourcing
#####
SOURCE_WS="
source $SCRIPT_DIR/../../../../devel/setup.bash &&
"
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic &&
"

#####
# Commands
#####
CMD_0="
roslaunch gestelt_bringup single_uav_sim.launch drone_id:=1 init_x:=0.5 init_y:=-0.5
"

CMD_1="roslaunch gestelt_bringup single_fake_map.launch drone_id:=1"

CMD_2="
roslaunch gestelt_bringup single_ego_planner.launch drone_id:=1 init_x:=0.4 init_y:=-0.4 POSE_TYPE:=3 SENSOR_TYPE:=1
"

if [ "$SESSIONEXISTS" = "" ]
then 
    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $CMD_0" C-m 
    sleep 3
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    # tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3"
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
