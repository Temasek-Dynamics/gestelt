#!/bin/bash

SESSION="gz_sim_single_uav"
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
# PX4 v1.14.0
# SOURCE_PX4_AUTOPILOT="
# source $PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic &&
# "

# PX4 v1.13.0
SOURCE_PX4_AUTOPILOT="
source $PX4_AUTOPILOT_REPO_DIR/Tools/setup_gazebo.bash $PX4_AUTOPILOT_REPO_DIR $PX4_AUTOPILOT_REPO_DIR/build/px4_sitl_default &&
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$gestelt_bringup_DIR:$PX4_AUTOPILOT_REPO_DIR:$PX4_AUTOPILOT_REPO_DIR/Tools/sitl_gazebo &&
"

# temporally source the standard PX4, will move to gestelt PX4 later
# SOURCE_PX4_AUTOPILOT="
# source $SCRIPT_DIR/../../../devel/setup.bash &&
# source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot/ ~/PX4-Autopilot/build/px4_sitl_default; 
# export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot:~/PX4-Autopilot/Tools/sitl_gazebo
# "
#####
# Commands
#####
# Start Gazebo and PX4 SITL instances
CMD_0="
roslaunch gestelt_bringup sitl_drone.launch 
"

# # Start up drone commander (Handles taking off, execution of mission and landing etc.)
CMD_1="
roslaunch trajectory_server trajectory_server_SE3_node.launch rviz_config:=gz_sim
"

# Start up minimum snap trajectory planner and sampler 
CMD_2="
roslaunch trajectory_planner trajectory_planner_node.launch
"

# Start up script to send commands
CMD_3="roslaunch gestelt_bringup multiple_circle_mission.launch"


# SE3
CMD_4="roslaunch se3_controller sitl_se3_controller.launch"


if [ "$SESSIONEXISTS" = "" ]
then 

    tmux new-session -d -s $SESSION

    tmux split-window -t $SESSION:0.0 -v
    tmux split-window -t $SESSION:0.1 -h
    tmux split-window -t $SESSION:0.0 -h
    # tmux split-window -t $SESSION:0.1 -h

    tmux send-keys -t $SESSION:0.0 "$SOURCE_PX4_AUTOPILOT $CMD_0" C-m 
    sleep 2
    # tmux send-keys -t $SESSION:0.1 "$SOURCE_PX4_AUTOPILOT $CMD_4" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.1 "$SOURCE_WS $CMD_1" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.2 "$SOURCE_WS $CMD_2" C-m 
    sleep 1
    tmux send-keys -t $SESSION:0.3 "$SOURCE_WS $CMD_3" C-m
fi

# Attach session on the first window
tmux attach-session -t "$SESSION:0"
