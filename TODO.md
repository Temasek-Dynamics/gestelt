# TODO

- Create ENU->FRD frame for vicon. 

- Integrate Vicon positioning with PX4 
    - Test position mode:
        - Check that this is able to localize the drone in position mode
    - Test Offboard mode
        - 
    - Set vicon noise values from computer
    - Try with vision speed?

- Use TF published from PX4. 
    - Subscribe to `/mavros/global_position/local` instead of `/mavros/local_position/pose`

- Benchmark
    - Add network params 
        - Bandwidth, latency, signal strength
    - Use ddynamic_reconfigure to toggle on/off benchmarking

- Radxa 
    - Create startup script for radxa
        - Should startup mavros script and egoplanner
    - Clone radxa device

- Radxa
    - Set optimization flags
        - set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
        - https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html

- Gridmap
    - Solve bug with `cloud.points.size () == cloud.width * cloud.height assertion failure
    - Implement pose timeout
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

- Consider a mixed ecosystem approach
    - Look at porting from ROS1 to ROS2
        - Simulation
        - Egoplanner Library
        - Launch files
        - gestelt_bringup executables
    - Aim: 
        - To be able to use third party libraries from ROS easily
        - Reduce unnecessary components
