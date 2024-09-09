# gestelt
A Receding Horizon Planning (RHP) framework with a focus on multi-UAV navigation in cluttered environments. 

For simulation and deployment on a physical drone, PX4 is the firmware of choice, although it is possible to remap the topics for use with Ardupilot or any other Mavlink-compatible system.

# Architecture
<img src="docs/pictures/gestelt_architecture_24_10.png" alt="Gestelt Architecture" style="width: 1200px;"/>

# Installation and Setup for Simulation
1. Install dependencies
- ROS 2 Jazzy

2. Required Repos
- PX4-msgs: bcb3d020bd2f2a994b0633a6fccf8ae47190d867
- PX4-Autopilot: 3d36c8519de83afd7b4617c3496d0304fb17cc28
- px4-ros2-interface-lib: fe9d3785384b4d1ca5399384f8480cdfee9030e8
- eProsima/Micro-XRCE-DDS-Agent: v2.4.3
    -  XRCE DDS installation
```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git --recursive -b v2.4.3
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

3. Install PX4 firmware
```bash
# cd to PX4-Autopilot repo
cd ~/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh 
# Make SITL target for simulation
DONT_RUN=1 make px4_sitl 

# Copy the custom drone model over
cp -r ~/gestelt_ws/src/gestelt/gestelt_bringup/simulation/models/raynor ~/gestelt_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/

# [FOR EMERGENCY USE] IF you screw up the PX4 Autopilot build at any point, clean up the build files via the following command:
make distclean
```

4. Building the workspace
```bash
# Assuming your workspace is named as follows
cd ~/gestelt_ws/
colcon build
```

5. Additional checks
```bash
# Check if px4_msg definitions match those in PX4 Firmware
./src/px4-ros2-interface-lib/scripts/check-message-compatibility.py -v ./src/px4_msgs/ ../PX4-Autopilot/
```

6. Configuration
- MAV_MODE_FLAG: needs to be set to 1
- COM_RC_IN_MODE: Set to 4 to disable need for manual controller to arm
- COM_OF_LOSS_T: Time-out (in seconds) to wait when offboard connection is lost before triggering offboard lost failsafe (COM_OBL_RC_ACT)
- COM_OBL_RC_ACT: Flight mode to switch to if offboard control is lost (Values are - 0: Position, 1: Altitude, 2: Manual, 3: *Return, 4: *Land*).
- COM_RC_OVERRIDE: Controls whether stick movement on a multicopter (or VTOL in MC mode) causes a mode change to Position mode. This is not enabled for offboard mode by default.

# Important notes
1. All topics that you can use are defined in `dds_topics.yaml`

2. Specifically, nodes should subscribe using the ROS 2 predefined QoS sensor data (from the listener example source code):
```cpp
...
rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

subscription_ = this->create_subscription<px4_msgs::msg::SensorCombined>("/fmu/out/sensor_combined", qos,
...
```

3. Examples of vectors that require rotation are:
- all fields in TrajectorySetpoint message; ENU to NED conversion is required before sending them.
    - first a pi/2 rotation around the Z-axis (up),
    - then a pi rotation around the X-axis (old East/new North).
- all fields in VehicleThrustSetpoint message; FLU to FRD conversion is required before sending them.
    - a pi rotation around the X-axis (front) is sufficient.


# Quick start

## Initial setup
```bash
# Start gazebo and PX4 SITL
make px4_sitl gz_x500
# Start QGround Control
~/Documents/QGroundControl.AppImage
# Start XRCE DDS agent  
MicroXRCEAgent udp4 -p 8888
```

## Demo 1: Listener example
```bash
ros2 launch px4_ros_com sensor_combined_listener.launch.py
```
## Demo 2: Offboard control
```bash
ros2 run px4_ros_com offboard_control
```

## Demo 1 example
```bash
# Example
ros2 run example_mode_manual_cpp example_mode_manual

# Check on PX4 shell that the custom mode is registered
commander status
```


## Debugging
```bash
ros2 topic echo /fmu/out/vehicle_status
```

# Message types
1. Control input topics
    - **Position, Velocity and Acceleration**
        - [TrajectorySetpoint](https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint.html): PVA
            - All values are interpreted in NED (Nord, East, Down) coordinate system and the units are [m], [m/s] and [m/s^2] for position, velocity and acceleration, respectively.
    - **Collective thrust, attitude**
        - [VehicleAttitudeSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleAttitudeSetpoint.html) Normalized thrust vector, attitude (quaternion)
            - The quaternion represents the rotation between the drone body FRD (front, right, down) frame and the NED frame. The thrust is in the drone body FRD frame and expressed in normalized [-1, 1] values.
    - Collective thrust, rates
        - [VehicleRatesSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleRatesSetpoint.html)
            - All the values are in the drone body FRD frame. The rates are in [rad/s] while thrust_body is normalized in [-1, 1].
    - **Thrust and torque**
        - [VehicleThrustSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleThrustSetpoint.html) Thrust setpoint along X, Y, Z body axis [-1, 1]
        - [VehicleTorqueSetpoint](https://docs.px4.io/main/en/msg_docs/VehicleTorqueSetpoint.html) Torque setpoint about X, Y, Z body axis (normalized)
            - All the values are in the drone body FRD frame and normalized in [-1, 1].
    - **Individual motors**
        - [ActuatorMotors](https://docs.px4.io/main/en/msg_docs/ActuatorMotors.html) Individual motor values
            - `Topic: "/fmu/in/actuator_motors"`
            - All the values normalized in [-1, 1]. For outputs that do not support negative values, negative entries map to NaN. 
            - NaN maps to disarmed.

2. [VehicleCommand](https://docs.px4.io/main/en/msg_docs/VehicleCommand.html)
    - `Topic: "/fmu/in/vehicle_command"`
    - Used to set to offboard mode

3. [OffboardControlMode](https://docs.px4.io/main/en/msg_docs/OffboardControlMode.html)
    - `Topic: "/fmu/in/offboard_control_mode"`
    - The vehicle must be already be receiving a stream of MAVLink setpoint messages or ROS 2 OffboardControlMode messages before arming in offboard mode or switching to offboard mode when flying.
    - The vehicle will exit offboard mode if MAVLink setpoint messages or OffboardControlMode are not received at a rate of > 2Hz.

## Gazebo PX4 targets
```bash
# make px4_sitl 
gz_x500
gz_x500_depth
gz_x500_vision
gz_x500_lidar
gz_standard_vtol
gz_rc_cessna
gz_advanced_plane
gz_r1_rover
gz_rover_ackermann
```


# Reference
1. [Virtual env usage](https://github.com/ros2/ros2/issues/1094)
2. [Gazebo Simulation](https://docs.px4.io/main/en/sim_gazebo_gz/)

# Acknowledgements
1. [EGO-Planner-V2 repo](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2)
2. [ETHZ-ASL/mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation)


