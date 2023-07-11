# gestelt_bringup
This package contains bringup files for Gestalt.

# Quick start
```bash
cd ~/gestelt_ws/src/gestelt_bringup/gestelt_bringup/scripts
# Simulation using simple quad simulator
./simple_sim.sh
# Simulation using gazebo
./gazebo_sim_multi_uav.sh
```

```bash
# Taking off
rostopic pub /traj_server_event std_msgs/Int8 "data: 0" --once
# Taking Mission
rostopic pub /traj_server_event std_msgs/Int8 "data: 2" --once

TAKEOFF_E,        // 0
LAND_E,           // 1
MISSION_E,        // 2
CANCEL_MISSION_E, // 3
E_STOP_E,         // 4
EMPTY_E,          // 5
```

# Demo (13/7/23)
- Gridmap
    - Change gridmap to use PCL and octreeOccupancyGrid for occupancy grid.
    - Point cloud
        - Transform point cloud on gridmap node. 
        - Downsample point clouds before sending
            - Voxel downsampling and publishing
    - Depth image
        - Reduce width and height (to 320 x 240)
        - Rewrite function to create PC from depth map
            - http://docs.ros.org/en/fuerte/api/rgbd2cloud/html/depth2cloud_8cpp_source.html
    - Add message filtering to ensure that poses and point cloud are aligned
    - set ros::TransportHints()
        - TCP_NODELAY: Improves the efficiency of TCP/IP networks by reducing the number of packets that need to be sent. For small messages: More efficient but higher latency. For big messages: More efficient and potentially lower latency
    
- Remove unnecessary nodes and dependencies for an easier build on radxa
    - Get rid of quadrotor_msgs 
    - Take out traj_server from plan_manage
    - Clean up cmakelists dependencies

- Benchmarking
    - Add CPU usage
    - https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
# TODO
- Gridmap
    - Depth image
        - Find out how to create organized point clouds
        - Compress, publish and subscribe using ImageTransport
    - Compare compressed depth map to downsampled point cloud
        - Bandwidth and delay
        - Does queue time affect latency
    - Look at using udp to send over point clouds or depth images

- Benchmark
    - Add network params 
        - Bandwidth, latency, signal strength
    - Use ddynamic_reconfigure to toggle on/off benchmarking

- Get the code working on the actual drone
- Perform physical tests to determine physical characteristics
    - Use the actual mass in Gazebo params
        - 0.25 g
    - Motor coefficients
        - Motor 6000V
    - Battery: 2S 7.4V
    - Propellers: 3 inch three-blade propellers
- Correct tracking error algo

## Simulation
- Explore weird phenomenom between drone_num/formation_num and path planning problems
    - When actual number of drones are 2 
        - If num_drone == 2, then the planned path is abnormal and goes very close to the ground
        - If num_drone == 3, the planned path is normal. 

## gridmap

## Trajectory Server
- Add mutexes
- For trajectory server, read the current state of the mavros/state topic before determining the starting state machine state.
- Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to be able to reliably check that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).
- Support Cancel/Start/Pause of waypoints execution
- Handle goals in obstacle regions (Cancel the goal?)
- Check if every UAV in formation has finished execution of current waypoint before planning for the next one

## Issues
- When rounding corners of obstacles, if the goal lies about a sharp turn around the corner, a trajectory with a sharp turn is planned, this could lead to issues if the obstacles is especially large as the drone will not be able to detect the other wall of the obstacle until it has turned around. 
- When the agents are too close to each other and the replan fails upon detecting a potential collsion between swarm agents. This condition happens more often when the obstacle_inflation is increased to a significantly higher value (1.2) and drones are navigating through narrow corridors.

# Future Roadmap
- Consider a mixed ecosystem approach
    - Look at porting from ROS1 to ROS2
        - Simulation
        - Egoplanner Library
        - Launch files
        - gestelt_bringup executables
    - Aim: 
        - To be able to use third party libraries from ROS easily
        - Reduce unnecessary components
