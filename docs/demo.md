# Demo

## Prepare
1. Vicon computer
2. Batteries (charged)
3. Laptop (charged)
4. RC (Charged)


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
