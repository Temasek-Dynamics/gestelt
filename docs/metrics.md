
# Network Metrics
- Xiaomi Network (Central PC and Radxa connected wirelessly to network)
    - Bandwidth: 
        - TCP: 57.3 Mbits/sec -> 7.1625 mb/s
        - UDP: 90Mbits/sec - 101 Mbits/sec  -> 11.25 mb/s
    - Latency
        - Latency averages 14.8 ms. Best is 8.7 ms, Worst is 32.1 ms.

# ROS Topic metrics
All tests conducted on 5 Ghz Wifi network
- Radxa subscribed topics

    - Point clouds (/drone0/camera/depth/points_downsampled)
        - Delay: AVG 0.005, MAX 0.04
        - Throughput: AVG 81 kb/s, MAX 440kb/s
            - throughput is proportional to density of obstacles in environment

    - Depth Image (/drone0/camera/depth/image_raw)
        - (840 * 640)
            - Delay: 0.75 
            - Throughput: 2.88mb/s (Required 16.65 mb/s)
        - (640 * 480)
            - Delay: 
            - Throughput: (Required 12.48MB/s)
        - (320 x 240)
            - Delay: 0.027, Max 0.26 
            - Throughput: 2.75 mb/s 

    - Pose input (/drone0/mavros/local_position/pose)
        - Delay: AVG -0.006, MAX 0.012
        - Throughput: 2.82KB/s

- PC subscribed topics

    - PVA Commands (/drone0/mavros/setpoint_raw/local)
        - Delay: AVG 0.015, MAX 0.04
        - Throughput: 2.55KB/s

    - Occupancy Grid (/drone_0_ego_planner_node/grid_map/occupancy)
        - Delay: AVG 0.08, MAX 0.128
        - Throughput: 76.5KB/S, MAX 440kb/s

# Integration Tests
    - Replan wall time
    - Replan frequency
    - Maximum flight speed it can enable in sparse/dense environment
    - Maximum number of UAVs it can handle
    - Tracking error
    - Success rate

# Metrics to measure
MAKE SURE TO BUILD IN RELEASE MODE

- Explanation of metrics 
    - Network 
        - Signal strength 
            - `iw dev wlan0 link` or `watch -n1 iwconfig`
        - Bandwidth (Network capacity), total messages sizes
            - Maximum rate that information can be transferred
            - `sudo iftop`
        - Throughput 
            - Actual rate that information is transferred
            - TCP Mode
                - On PC A, set up server `iperf -s`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP`
            - UDP Mode
                - On PC A, set up server `iperf -s -u`, take note of tcp port number
                - On PC B, set up client connecting to IP of PC A: `iperf -c PC_A_IP -u -b 1000m`
        - Latency
            - Delay between sender and receiver decoding it. Function of signals travel time, and processing time at any nodes the information traverses 
            - use `sudo mtr --no-dns --report --report-cycles 60 IP_ADDR` or `ping`
        - Jitter
            - Variation in packet delay at the receiver of the information
    - Hardware (Radxa)
        - CPU Usage `htop`
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
    - ROS
        - Measure message size: `rostopic bw`
        - Measure frequency of publishing: `rostopic hz`
        - Measure delay: `rostopic delay`
    - Integration tests 
        - Maximum flight speed it can enable in sparse/dense environment
        - Maximum number of UAVs it can handle
        - Replan frequency
        - Tracking error
        - Success rate

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




