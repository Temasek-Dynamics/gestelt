# Demo
These are instructions for demo in the Vicon room with multiple drones.

## Preparation
1. Turn on Vicon computer
2. Batteries (charged)
3. Laptop (charged)
4. Radio Controller (Charged)
5. Drones 

### Set-up 
1. Make sure drone is connected to the right wifi network
```bash
sudo nmcli dev wifi connect "wifi_name"
```

### Offboard computer (Drone)
```bash 
# Drone 0 (192.168.31.205)
ssh rock@192.168.31.205
uav_startup 0 
# Drone 1 (192.168.31.150)
ssh rock@192.168.31.150
uav_startup 1
```

### Host PC
1. Start ROSCore
2. Start Vicon central
```bash 
cd_scripts && cd vicon
./vicon_central.sh
```

### Commands
```bash
# Land the drone
rostopic pub /traj_server_event std_msgs/Int8 "data: 1" --once
# Switch to hover mode
rostopic pub /traj_server_event std_msgs/Int8 "data: 3" --once
# Emergency stop
rostopic pub /traj_server_event std_msgs/Int8 "data: 4" --once
```

### Copying bringup files
```bash
# Radxa 0
scp -r /home/john/gestelt_ws/src/gestelt/gestelt_bringup/ rock@192.168.31.205:/home/rock/gestelt_ws/src/gestelt/
# Radxa 1
scp -r /home/john/gestelt_ws/src/gestelt/gestelt_bringup/ rock@192.168.31.150:/home/rock/gestelt_ws/src/gestelt/
```

# Demo Preparation tests

## On central computer
```bash
cd_scripts && cd demo
./demo_ctl.sh
# When ready, launhc the mission
```

## To simulate the actual drone
```bash
cd_scripts && cd demo
./gz_sim_single_uav_demo_off_0.sh
./gz_sim_single_uav_demo_off_1.sh
```

## Connection tests
1. Make sure self IP is correct
2. Can connect to FCU via Mavros?

