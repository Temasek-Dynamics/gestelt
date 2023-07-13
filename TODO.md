# TODO

# Demo (13/7/23)
- Gridmap
    - Change gridmap to use PCL and octreeOccupancyGrid for occupancy grid.
        - Previously they pre-allocated a 3d array sized by the entire map. This is memory intensive, but access might be faster. Octree might incur a small overhead for access (depending on the resolution) but definitely has better stability and APIs. 
    - Add message filtering to ensure that poses and point cloud are aligned (http://wiki.ros.org/message_filters/ApproximateTime)
    - Point cloud
        - Add proper support to take in point clouds
            - transformation for point clouds to global uav frame
            - Added voxel filter for downsampling 
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
    - Improve communication  ros::TransportHints()
        - Use TCP_NODELAY for image/point cloud topics: Improves the efficiency of TCP/IP networks by reducing the number of packets that need to be sent. For small messages: More efficient but higher latency. For big messages: More efficient and potentially lower latency
        - Use UDP for pose messages topics
        - Sensor_msgs type must use TCP as the ROS UDP messages provide no mechanism to reconstruct the fragmented images/point clouds
    
    - Compare compressed depth map to downsampled point cloud
        - Using depth image (320 x 240) as input ceases to work properly after a while as the delay gets increasingly longer.
            - (higher throughput req. at ~ 3.5 mb/s) 
        - DOWNSAMPLED Point clouds as input works well, and there seems to be minimal delay.
            - (lower throughput req. at ~ 400kb/s)

- Improve build time on radxa by removing unnecessary dependencies 
    - Get rid of quadrotor_msgs 
    - Separate traj_server from plan_manage
    - Clean up cmakelists dependencies
    - Reorganize packages to ensure that radxa only builds the necessary packages

- Benchmarking
    - Add CPU usage (https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process)

- Test with other virtual drones

- Challenges
    - Unable to build the flywoo firmware with the latest version of PX4. However, building it with the custom version of PX4 works. Should we look into how to port it to the latest version (so it remains relevant with any updates?)
    - Increasing the frequency of the depth/point cloud topics so as to prevent mis-mapped point cloud topics
    - Potential to improve latency for streaming depth images and point clouds when using UDP, but need to use a different transport mechanism than ROS (Low level UDP sockets for e.g.).
    - 

# TODO

- Set optimization flags
    - set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
    - https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html

- Get the code working on the actual drone

- Gridmap
    - Implement pose timeout
    - Depth image
        - Find out how to create organized point clouds
        - Compress, publish and subscribe using ImageTransport
    - Look at using udp to send over point clouds or depth images
    - Create map with decaying voxels
    - Rename `plan_env` to `gestelt_mapping`

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
        - gestelt_bringup executables
    - Aim: 
        - To be able to use third party libraries from ROS easily
        - Reduce unnecessary components
