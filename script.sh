# Please run with sudo.
# This bash will launch all the components of the system, in the order of:
# 0. px4 (mavros & gcs_bridge) 


init_path=~/gestelt_ws


# delay between launching various modules
module_delay=2.0

# check whether is running as sudo
if [ "$EUID" -eq 0 ]
    then echo "Please DO NOT run as root."
    exit
fi

if [ "${STY}" != "" ]
    then echo "You are running the script in a screen environment. Please quit the screen."
    exit
fi

output="$(screen -ls)"
if [[ $output != *"No Sockets found"* ]]; then
    echo "There are some screen sessions alive. Please run 'pkill screen' before launching uavos."
    exit
fi

echo "The system is booting..."

cd ${init_path}

# roscore
screen -d -m -S roscore bash -c "source devel/setup.bash; roscore; exec bash -i"
sleep ${module_delay}
sleep ${module_delay}
sleep ${module_delay}
echo "roscore ready."


# --------------------------------------------
# prepareation
cd ${init_path}
source devel/setup.bash

#################################################################################################################################
# -1 vectornav & gprmc
screen -d -m -S vicon_bridge bash -c "source devel/setup.bash; roslaunch vicon_bridge vicon.launch  ; exec bash -i"
sleep ${module_delay}
sleep ${module_delay}
sleep ${module_delay}
echo "vicon_bridge ready."

######################################################################################################


#################################################################################################################################
# -1 mavros
screen -d -m -S mavros bash -c "source devel/setup.bash; roslaunch mavros px4.launch  ; exec bash -i"
sleep ${module_delay}
sleep ${module_delay}
sleep ${module_delay}
echo "mavros ready."

#################################################################################################################################
# -2 trajectory server
screen -d -m -S trajectory_server bash -c "source devel/setup.bash; roslaunch trajectory_server trajectory_server_node.launch; exec bash -i"
sleep ${module_delay}
echo "trajectory_server ready."

#################################################################################################################################
# 3. trajectory planner
screen -d -m -S trajectory_planner bash -c "source devel/setup.bash;roslaunch trajectory_planner trajectory_planner_node.launch 
; exec bash -i"
sleep ${module_delay}
echo "trajectory_planner ready."

#################################################################################################################################


#################################################################################################################################
sleep ${module_delay}
sleep ${module_delay}
sleep ${module_delay}




echo "ALL GREEN ! System is started."


