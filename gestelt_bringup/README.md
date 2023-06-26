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
 
# TODO
    - Add option to:
        - Add flag for building in release mode
        - enable/disable running on radxa
    - Record metrics 
        - Maximum possible speed
        - Communications reqd – message size, frequency of sending
        - Communications latency tolerated – milliseconds
        - Algo run time on Radxa – milliseconds
        - Success rate
        - Maximum flight speed it can enable in sparse/dense environment
        - Maximum number of UAVs it can handle

    - Consider a mixed ecosystem approach
        - Look at porting from ROS1 to ROS2
            - Simulation
            - Egoplanner Library
            - Launch files
            - gestelt_bringup executables
        - Aim: 
            - To be able to use third party libraries from ROS easily
            - Reduce unnecessary components
    - Perform physical tests to determine physical characteristics
        - Use the actual mass in Gazebo params
            - 0.25 g
        - Motor coefficients
            - Motor 6000V
        - Battery: 2S 7.4V
        - Propellers: 3 inch three-blade propellers


## Simulation
- Add publish server state to ego replan fsm, so that trajectory server can aggregate it.
- Explore weird phenomenom between drone_num/formation_num and path planning problems
    - When actual number of drones are 2 
        - If num_drone == 2, then the planned path is abnormal and goes very close to the ground
        - If num_drone == 3, the planned path is normal. 

## gridmap
- Change gridmap to use PCL and octree search for occupancy grid
- Add body to camera transform as a matrix ROS Param (Make sure that it is same as that in simulation)
- Take in intrinsic params of camera via camera_info topic

## Trajectory Server
- Add mutexes
- For trajectory server, read the current state of the mavros/state topic before determining the starting state machine state.
- Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to be able to reliably check that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).

## Benchmarking/Diagnostics
- Benchmark the replanning time for each drone's planner (are they close enough to the specified replanning frequency?)

## PID Tuning Guide
- Tune PID 
    - Rate Controller
        - ROLL
            - MC_ROLLRATE_K: 1.0
            - MC_ROLLRATE_P: 0.249
                - Enhance damping of the roll channel, faster attenuation of the oscillation
            - MC_ROLLRATE_D: 0.0046
            - MC_ROLLRATE_I: 0.325
        - PITCH
            - MC_PITCHRATE_K: 1.0
            - MC_PITCHRATE_P: 0.233
            - MC_PITCHRATE_D: 0.0044
            - MC_PITCHRATE_I: 0.3
        - YAW
            - MC_YAWRATE_K: 1.0
            - MC_YAWRATE_P: 0.18
            - MC_YAWRATE_I: 0.18
    - Attitude Controller
        - MC_ROLL_P
            - 3.84
            - To reduce amplitude of oscillation
        - MC_PITCH_P
            - 4.1
        - MC_YAW_P
            - 5.26
    - Velocity Controller
        - (horizontal)
            - MPC_XY_VEL_P_ACC
                - 3.25
            - MPC_XY_VEL_I_ACC
                - 0.4
            - MPC_XY_VEL_D_ACC
                - 0.2
        - (vertical)
            - MPC_Z_VEL_P_ACC
                - 4.0
            - MPC_Z_VEL_I_ACC
                - 2.0
            - MPC_Z_VEL_D_ACC
                - 0.0
    - Position Controller
        - (horizontal)
            - MPC_XY_P 
                - 0.8
                - Reduce position control gain
        - (vertical)
            - MPC_Z_P 
                - 1.0
                - Reduce overshoot of position.


## Issues
- When rounding corners of obstacles, if the goal lies about a sharp turn around the corner, a trajectory with a sharp turn is planned, this could lead to issues if the obstacles is especially large as the drone will not be able to detect the other wall of the obstacle until it has turned around. 
- When the agents are too close to each other and the replan fails upon detecting a potential collsion between swarm agents. This condition happens more often when the obstacle_inflation is increased to a significantly higher value (1.2) and drones are navigating through narrow corridors.