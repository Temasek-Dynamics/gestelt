# TODO
Grid map
- inflate points for visualization

Add instructions on how to run 
 - With pictures
 - Vicon
- Add instructions on how to send custom commands
- Use config files for params

Simulation
- Can we put all the drones in the same world frame?
- remove formation_num
- Is there a way to abstract away planning and replanning?
- test swarm navigation in a large map to check if formation flying still holds

Fix swarm collision checker:
- malloc(): corrupted top size for swarm collision checker

Add gn405s ICM support for official repo (Use omnibus bootloader)
test trajectory tracker

-Metrics collection
    - Trajectory Tracking
        - Create a few test trajectories and track the errors
            - Add waypoint computation as message
    - Get CPU load benchmarking

- RVIZ GUI for take off and landing

- Grid_map
    - Apply passtheough filter on point cloud obtained from octomap 
    https://github.com/OctoMap/octomap/issues/283
    - Remove all octree elements outside of local map bound
    - Parallerization? Figure out how octomap uses openMP to parallerize ray casting

- Actual deployment
    - Logging of mavlink topics and ROS topics to analyze issues

- Fix formation and num_drones issue
    - Get rid of num_drones, be able to dynamically add/remove agents
    -  Add feature to dynamically set formation number

- Benchmark
    - Add network params 
        - Bandwidth, latency, signal strength

- Port to ROS 2

- Gridmap
    - Implement pose timeout
        - flag_sensor_timeout_
    - Depth image
        - Find out how to create organized point clouds
        - Compress, publish and subscribe using ImageTransport
    - Look at using udp to send over point clouds or depth images
    - Create map with decaying voxels
        - Start by visualizing the underlying stored octree first

## Simulation
- Explore weird phenomenom between drone_num/formation_num and path planning problems
    - When actual number of drones are 2 
        - If num_drone == 2, then the planned path is abnormal and goes very close to the ground
        - If num_drone == 3, the planned path is normal. 

## Trajectory Server
- Handle goals in obstacle regions (Cancel the goal?)
- Add mutexes
- For trajectory server, read the current state of the mavros/state topic before determining the starting state machine state.
- Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to be able to reliably check that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).
- Support Cancel/Start/Pause of waypoints execution

## Issues
- When rounding corners of obstacles, if the goal lies about a sharp turn around the corner, a trajectory with a sharp turn is planned, this could lead to issues if the obstacles is especially large as the drone will not be able to detect the other wall of the obstacle until it has turned around. 
- When the agents are too close to each other and the replan fails upon detecting a potential collsion between swarm agents. This condition happens more often when the obstacle_inflation is increased to a significantly higher value (1.2) and drones are navigating through narrow corridors.

# Future Roadmap
- Push a swarm through cluttered environments at 5m/s
- Things to try out:
    - SOTA Trajectory tracker (vs PID)