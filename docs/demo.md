# Demo
These are instructions for demo in the Vicon room with multiple drones.

## Preparation
1. Turn on Vicon computer
2. Batteries (charged)
3. Laptop (charged)
4. Radio Controller (Charged)
5. Drones 

## Set-up 
1. Make sure drone is connected to the right wifi network
```bash
sudo nmcli dev wifi connect "wifi_name"
```

## Offboard computer (Drone)
```bash 
uav_startup
```

## Host PC
1. Start ROSCore
2. Start Vicon central
```bash 
cd_scripts 
./vicon_central.sh
```

## Commands
```bash
# Land the drone
rostopic pub /traj_server_event std_msgs/Int8 "data: 1" --once
# Switch to hover mode
rostopic pub /traj_server_event std_msgs/Int8 "data: 3" --once
```

# Demo Preparation tests

## On central computer
```bash
cd_scripts 
cd testing
./gz_sim_single_uav_demo_ctl.sh
# When ready, launhc the mission
```

## On actual drone
```bash
cd_scripts 
cd testing
./gz_sim_single_uav_demo_off.sh
```