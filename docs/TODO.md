# TODO
- Constrain the trajectories of the uav to be within the bounds of the map
    - Set map size
    - Edit point cloud file to include ceiling and floor

- Allow trajectory server to take in waypoint commands from elsewhere like PBTM

- Improvements
    - Sensor
        - Solve laser range finder issue
    - Vicon improvements:
        - Check logs and see vicon delay
        - Set vicon noise values from computer
    - Software
        - Change parameters for trajectory server/ego planner to param config files

- Documentation
    - Vicon system and PX4 params
    - TF tree structure

- Ego Planner
    - Split up into multiple packages to make builds faster on radxa
    - Add feature to dynamically set formation number

- Benchmark
    - Add network params 
        - Bandwidth, latency, signal strength

- Radxa 
    - Create startup script for radxa
        - Should startup mavros script and egoplanner
    - Clone radxa device
    - Set optimization flags
        - set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
        - https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html

- Gridmap
    - Solve bug with `cloud.points.size () == cloud.width * cloud.height assertion failure
    - Implement pose timeout
        - flag_sensor_timeout_
    - Depth image
        - Find out how to create organized point clouds
        - Compress, publish and subscribe using ImageTransport
    - Look at using udp to send over point clouds or depth images
    - Create map with decaying voxels
    - Rename `plan_env` to `gestelt_mapping`

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
- Push a swarm through cluttered environments at 5m/s
- Things to try out:
    - Share compresssed position/vecoity
    - SOTA Trajectory tracker (vs PID)

- Add DDS message output
    - Port to ROS2