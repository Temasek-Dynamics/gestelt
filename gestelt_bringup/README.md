# File Hierarchy 

## Launch files

- fake_drone: For simulated drones (without dynamics)
- sitl: For Gazebo simulated drones
- offboard: For actual drone deployment 
- rosbag: Rosbag recording
- rviz: RVIZ Visualization helper files
- scenarios: Hosts a set of scenario spawn configurations for use with fake_drone

## scripts

- offboard.sh: Script for starting up all required nodes on an actual drone

# Launching demos
```bash
# On the base station 
./offboard.sh -i <DRONE_ID>

# On the drones
./base_station -s <SCENARIO>
```
