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

# Metrics to measure
MAKE SURE TO BUILD IN RELEASE MODE

- Record metrics 
    - Communications 
        - Signal strength to wifi network
            - `iw dev wlan0 link`
        - Bandwidth (Network capacity), total messages sizes
            - `sudo iftop`
        - Network Speed 
            - TCP Mode
                - On PC A, set up server `iperf -s`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP`
            - UDP Mode
                - On PC A, set up server `iperf -s -u`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP -u -b 1000m`
        - Network Latency
            - use `sudo mtr --no-dns --report --report-cycles 60 IP_ADDR` or `ping`
        - ROS
            - Measure message size for incoming point clouds

    - Hardware (Radxa)
        - CPU Usage
            - Use `htop`
        - Wall time (aka 'real' time )
            - Elapsed time from start to finish of the call. Includes time slices used by other processes and the time the process spends blocked (waiting for I/O to complete
            )
        - CPU Runtime (aka 'user' time)
            - Amount of CPU time spent in user-mode code (outside the kernel) within the process. Only actual CPU time used in executing the process.
        - System time
            - Amount of time spent within the kernel, as opposed to library code. Could include I/O, allocating memory.
        - Sometimes Sys + user > real, as multiple processors work in parallel.
        - What is important to us is the 'real' time as we want to ensure that the planner is able to plan with a high enough frequency
        - Measure for planner, Depth map

    - Integration tests 
        - Replan frequency
        - Tracking error
        - Success rate
        - Maximum flight speed it can enable in sparse/dense environment
        - Maximum number of UAVs it can handle

# TODO
- Add timebenchmark class to 
    - planner
    - gridmap
- Measure point cloud size in gridmap
- Create message type that shows debug information such as the metrics
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
- Change gridmap to use PCL and octree search for occupancy grid
- Add body to camera transform as a matrix ROS Param (Make sure that it is same as that in simulation)
- Take in intrinsic params of camera via camera_info topic

## Trajectory Server
- Add mutexes
- For trajectory server, read the current state of the mavros/state topic before determining the starting state machine state.
- Disabling of offboard mode for land state would be a good feature. Current challenge to implement it is to be able to reliably check that the drone has actually landed (Otherwise it will be stuck in AUTO.LOITER while hovering in the air, being unable to disarm).

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
- Trajectory Server
    - Support Cancel/Start/Pause of waypoints execution
    - Handle goals in obstacle regions (Cancel the goal?)
    - Check if every UAV in formation has finished execution of current waypoint before planning for the next one