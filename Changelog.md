1. Added a "complete" state machine to trajectory server with safety features such as Emergency stop and the ability to switch between HOVER and MISSION mode.
2. The occupancy map not aligned with the actual depth camera point cloud: Need to set the intrinsic parameters (This can be obtained by the 'K' variable in the `camera_info` topic )
4. Fixed projected points from depth camera not being intialized with the right size.
5. Fixed issue where drone's heading does not always face it's direction of travel (results in depth camera not facing the direction of travel and knocking into obstacles it didn't detect in it's desired trajectory). The fix is to ignore the yaw rate being supplied by the command.
6. Got at least 2 drones to plan using ego planner in gazebo.
7. Set the physical form of the drone properly 
    - Motor to motor: 0.12m
    - Rotor diameter: 0.09m
    - Size can be approximated as a cuboid: 0.21 * 0.21 * 0.12
8. Offset the starting mavros local position of the drone by creating new frames and using those as the origin frame for each drone
9. Fixed issues with simulating multiple robots in Gazebo Classic
10. Visualization 
    - Add a visualization mesh to each drone (Similar to simple quad simulator's implementation)
    - Use a new mesh (fake_drone.dae) model to represent the drone in gazebo
12. Create script for takeoff, switching to mission mode and sending waypoints.
13. Fix frame transformation issue that comes with PX4 firmware
    - Each UAV has their own origin frame relative to the world (Why? PX4 will always start from (0,0,0) in any given frame, so we create our own origin frame for each drone and provide an offset from world for said frame)
        - Input Data
            - Sensor input
                - Odom
                    - NO TRANSFORM
                        - Input Odom (Already in UAV frame)
                - Point Cloud
                    - Received in camera frame
                    - TRANSFORM
                        - Need to transform (CAMERA -> UAV Origin frame)
            - User input
                - TRANSFORM
                    - Waypoints 
                        - Received in WORLD frame
                        - Need to transform (WORLD -> UAV Origin frame)
            - Plans
                - TRANSFORM
                    - Other drones' MINCO Trajectories
                        - Received in WORLD frame
                        - Need to transform (WORLD -> UAV Origin frame)
        - Output Data
            - NO TRANSFORM
                - Planned trajectory (not MINCO) 
                    - Originally in (UAV frame)
                    - Passed to trajectory server
            - TRANSFROM
                - Broadcasted Trajectories 
                    - Need to transform (UAV Origin -> WORLD)
                    - Passed to other planner servers
14. Tested with 4 drones
15. Refactor code to make it cleaner, easier to modify for additional functionality in the future
    - FSM 
        - Refactor the state machine execution
        - Make transition tables between states more explicit
    - Waypoint execution 
        - Added functionality to accept waypoints via topics rather than through ros parameters
        - Created a new class to abstract away handling of waypoints
16. Added logging of uav trajectory and tracking error to Trajectory Server node
    - Used Plotjuggler to visualize error
17. Investigate tracking error and maximum vel, acc, jerk parameters provided to planner
    - With default Gazebo Iris Model (In obstacle free environment)
        - Planner params
            - Max acc (0.5), max Jerk (5.0)
                - Max vel (0.5)
                    - Max XY Tracking error: 0.11 ~ 0.2
                    - Average XY Tracking error: Averages about 0.04 ~ 0.11
            - Max acc (1.0), max Jerk (5.0)
                - Max vel (0.5)
                    - Max XY Tracking error: 0.19 ~ 0.35
                        - 0.35 for a single drone
                        - Around 0.2 - 0.25 for the other 3 drones
                    - Average XY Tracking error: 0.05 ~ 0.15
                - Max vel (1.5)
                    - Max XY Tracking error: 0.4 ~ 0.55
                    - Average XY Tracking error: 0.05 ~ 0.45 
            - Max acc (3.0), max Jerk (20.0)
                - Max vel (0.5)
                    - Max XY Tracking error: 0.15
                    - Average XY Tracking error: 0.03 ~ 0.14
18. Parameters to play with for reducing collision with obstacles
    - grid_map/inflation
        - Affects inflation of obstacles. This would be more visually intuitive, and creates a virtual cushion against planning too near to obstacles
    - obstacle_clearance
        - Somehow even with a relatively high value, the uav still plans a trajectory through tight corridors. COuld it it be due to it optimizing on a path provided by the global planner, and the global planner does not provide an alternative path because it does not take into account obstacle clearance?
19. (12/6/23) Added swarm collision checker to check for collision between the swarm agents. This node subscribes to the pose of each agent and publishes spheres visualizing where the inter-agent distance falls below the user-defined collision tolerance.
20. Created global_planner module, with an example 3d a_star planner. Makes use of PCL point clouds and octree search to check the occupancy grid.
21. (22/6/23) Added PID params for Raynor vehicle setup
22. (23/6/23) Added obstacle collision sensor to base_link of Raynor gazebo model. And added parsing and visualization of collision with obstacles.
23. (23/6/23) Added launch files to record/playback rosbags of simulation runs.
24. (26/6/23) Added option to launch files for running on Radxa
25. (28/6/23) Add tested Radxa setup file
26. (7/7/23) Tested Egoplanner with Radxa HITL and Gazebo simulation
27. (7/7/23) To gridmap module, added subscription to camera_info for intrinsic parameters and added setting of extrinsic parameter via rosparams. 
28. (13/7/23) Gridmap
    - Change gridmap to use PCL and octreeOccupancyGrid for occupancy grid.
        - Previously they pre-allocated a 3d array sized by the entire map. This is memory intensive, but access might be faster. Octree might incur a small overhead for access (depending on the resolution) but definitely has better stability and APIs. 
    - Add message filtering to ensure that poses and point cloud are aligned (http://wiki.ros.org/message_filters/ApproximateTime)
    - Point cloud
        - Add proper support to take in point clouds
            - transformation for point clouds to global uav frame
            - Added voxel filter for downsampling 
                - Voxel downsampling and publishing (10cm voxel size)
                    - (320 x 240) resolution
                        - Before: 28.10 MB/s, after: 414.70KB/s
                    - (640 x 480) resolution
                        - Before: 105 MB/s, after: ~ 400KB/s
                    - (848 x 480) resolution
                        - Before: 140 MB/s, after: ~ 400KB/s
                    - Max bandwidth is proportional to density of obstacles in environment
    - Depth image
        - Rewrite function to create PC from depth map
            - http://docs.ros.org/en/fuerte/api/rgbd2cloud/html/depth2cloud_8cpp_source.html
    - Improve communication  ros::TransportHints()
        - Use TCP_NODELAY for image/point cloud topics: Improves the efficiency of TCP/IP networks by reducing the number of packets that need to be sent. For small messages: More efficient but higher latency. For big messages: More efficient and potentially lower latency
        - Use UDP for pose messages topics
        - Sensor_msgs type must use TCP as the ROS UDP messages provide no mechanism to reconstruct the fragmented images/point clouds
29. (13/7/23) Improve build time on radxa by removing unnecessary dependencies 
    - Get rid of quadrotor_msgs 
    - Separate traj_server from plan_manage
    - Clean up cmakelists dependencies
    - Reorganize packages to ensure that radxa only builds the necessary packages
30. (13/7/23) CPU Benchmarking
    - Add CPU usage (https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process)
31. (31/7/23) Added launch file including interface of Radxa with PX4, as well as taking in pose input from VICON system
    - Add instructions to synchronize time between radxa and PX4
        - http://www.ubuntugeek.com/network-time-protocol-ntp-server-and-clients-setup-in-ubuntu.html
32. (8/8/23) Added documentation on tweaking the PX4 for usage with the Vicon system
33. (10/8/23) Added fake sensor data node for simulating virtual obstacles on an actual drone.
34. (11/8/23) Updated gridmap with additional option to use tf instead of pose to get camera to world transform
35. (16/8/23) Successfully tested offboard mode, added scripts for testing offboard and mission for single drone.
36. 