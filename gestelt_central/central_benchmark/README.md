# central_benchmark
This package contains bringup files for Gestalt.

# Quick start
```bash
cd ~/gestelt_ws/src/central_benchmark/central_benchmark/scripts
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
    - Add message filtering to ensure that poses and point cloud are aligned
    - Point cloud
        - Add transformation for point clouds to global uav frame
        - Added voxel filter for downsampling point clouds
            - Voxel downsampling and publishing (10cm size)
                - (320 x 240) resolution
                    - Before: 28.10 MB/s, after: 414.70KB/s
                - (640 x 480) resolution
                    - Before: 105 MB/s, after: ~ 400KB/s
                - (848 x 480) resolution
                    - Before: 140 MB/s, after: ~ 400KB/s
                - Max bandwidth is proportional to density of obstacles in environment
    - Depth image
        - Measure utilized bandwidth for different resolutions
            - (848 x 480): 16.46MB/s 
            - (640 x 480): 12.48MB/s
            - (320 x 240): 3.52MB/s
        - Rewrite function to create PC from depth map
            - http://docs.ros.org/en/fuerte/api/rgbd2cloud/html/depth2cloud_8cpp_source.html
    - set ros::TransportHints()
        - Use TCP_NODELAY for image/point cloud topics: Improves the efficiency of TCP/IP networks by reducing the number of packets that need to be sent. For small messages: More efficient but higher latency. For big messages: More efficient and potentially lower latency
        - Use UDP for pose messages topics
    
- Remove unnecessary nodes and dependencies for an easier build on radxa
    - Get rid of quadrotor_msgs 
    - Separate traj_server from plan_manage
    - Clean up cmakelists dependencies

- Benchmarking
    - Add CPU usage
    - https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process

    

# TODO
- Test with other virtual drones

- Set optimization flags
    - set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
    - https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html

- Get the code working on the actual drone

- Gridmap
    - Compare compressed depth map to downsampled point cloud
        - Bandwidth and delay
    - Implement pose timeout
    - Rename `plan_env` to `gestelt_mapping`
    - Depth image
        - Find out how to create organized point clouds
        - Compress, publish and subscribe using ImageTransport
    - Look at using udp to send over point clouds or depth images
    - Create map with decaying voxels

- Benchmark
    - Add network params 
        - Bandwidth, latency, signal strength
    - Use ddynamic_reconfigure to toggle on/off benchmarking

- Perform physical tests to determine physical characteristics
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

## Issues
- When rounding corners of obstacles, if the goal lies about a sharp turn around the corner, a trajectory with a sharp turn is planned, this could lead to issues if the obstacles is especially large as the drone will not be able to detect the other wall of the obstacle until it has turned around. 
- When the agents are too close to each other and the replan fails upon detecting a potential collsion between swarm agents. This condition happens more often when the obstacle_inflation is increased to a significantly higher value (1.2) and drones are navigating through narrow corridors.

# Future Roadmap
- Consider a mixed ecosystem approach
    - Look at porting from ROS1 to ROS2
        - Simulation
        - Egoplanner Library
        - Launch files
        - central_benchmark executables
    - Aim: 
        - To be able to use third party libraries from ROS easily
        - Reduce unnecessary components