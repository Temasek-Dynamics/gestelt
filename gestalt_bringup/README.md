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

# TODO
    - Automated checking of collision avoidance
        - Agent-Obstacle collision
            - Collision occurs when sensor data is within a certain range of the base link frame.
    - Investigate
        - Unknown regions are assumed to be obstacle free?
            - Planning trajectories into the unknown
    - Add flag for building in release mode
    - Use drone_detection module to remove the drone point cloud, starting with simulation.

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

## Issues
- When rounding corners of obstacles, if the goal lies about a sharp turn around the corner, a trajectory with a sharp turn is planned, this could lead to issues if the obstacles is especially large as the drone will not be able to detect the other wall of the obstacle until it has turned around. 