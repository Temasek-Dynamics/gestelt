# gestalt_bringup
This package contains bringup files for Gestalt.

# Quick start
```bash
cd ~/gestelt_ws/src/gestelt_bringup/gestalt_bringup/scripts
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
```

# Changes made to EGOSwarm V2
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
        - Input 
            - Sensor 
                - NO TRANSFORM
                    - Input Odom (Already in UAV frame)
            - User 
                - TRANSFORM
                    - Received waypoints (WORLD -> UAV Origin frame)
            - Plans
                - TRANSFORM
                    - Other drone's MINCO trajectories (World -> UAV Origin frame)
        - Output 
            - NO TRANSFORM
                - Planned trajectory (not MINCO) (Already in UAV frame)
                    - Passed to trajectory server
            - TRANSFROM
                - Broadcasted Trajectories (UAV Origin -> world frame)
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
    - grid_map/obstacles_inflation
        - Affects inflation of obstacles. This would be more visually intuitive, and creates a virtual cushion against planning too near to obstacles
    - obstacle_clearance
        - Somehow even with a relatively high value, the uav still plans a trajectory through tight corridors. COuld it it be due to it optimizing on a path provided by the global planner, and the global planner does not provide an alternative path because it does not take into account obstacle clearance?

# TODO
    - Share params (with description) in gazebo plugin required for gazebo simulation
    - Automated checking of collision avoidance
    - Total error = sensor error + tracking error + radius of drone
        - Drone could end up closer to the obstacle due to tracking error
    - Use drone_detection module to remove the drone point cloud, starting with simulation.
    - Investigate
        - Unknown regions are assumed to be obstacle free?
            - Planning trajectories into the unknown
    - When rounding corners of obstacles, if the goal lies about a sharp turn around the corner, a trajectory with a sharp turn is planned, this could lead to issues if the obstacles is especially large as the drone will not be able to detect the other wall of the obstacle until it has turned around. 

## Simulation
- Add publish server state to ego replan fsm, so that trajectory server can aggregate it.
- Explore weird phenomenom between drone_num/formation_num and path planning problems
    - When actual number of drones are 2 
        - If num_drone == 2, then the planned path is abnormal and goes very close to the ground
        - If num_drone == 3, the planned path is normal. 
- Set up a more complex simulation world

## gridmap
- Add body to camera transform as a matrix ROS Param (Make sure that it is same as that in simulation)
- Take in intrinsic params of camera via camera_info topic

## Trajectory Server
- Add mutexes
- For trajectory server, read the current state of the mavros/state topic before determining the starting state machine state.
- Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to be able to reliably check that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).

## Benchmarking/Diagnostics
- Benchmark the replanning time for each drone's planner (are they close enough to the specified replanning frequency?)

## Hardware
- Test compilation on radxa
