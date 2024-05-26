# PX4 setup

## Building the FCU firmware
1. Building and uploading the bootloader
```bash
# Install dfu-util 
sudo apt install dfu-util python-is-python2
# Clone the PX4 bootloader repository
git clone https://github.com/PX4/PX4-Bootloader.git
cd ./PX4-Bootloader
make omnibusf4sd_bl
# bootloader file with ".hex" extension in PX4-Bootloader/build/omnibusf4sd_bl/omnibusf4sd_bl.hex
dfu-util -a 0 --dfuse-address 0x08000000:force:mass-erase:leave -D build/omnibusf4sd_bl/omnibusf4sd_bl.bin
dfu-util -a 0 --dfuse-address 0x08000000 -D  build/omnibusf4sd_bl/omnibusf4sd_bl.bin
```
Please refer to ["PX4 Bootloader Flashing onto Betaflight Systems"](https://docs.px4.io/main/en/advanced_config/bootloader_update_from_betaflight.html) for flashing of bootloader onto the FCU

2. Build the board firmware 
```bash 
# Clone the custom version of PX4 
git clone https://github.com/JohnTGZ/PX4-Autopilot.git --recursive -b f405-v1.13.0
cd ./PX4-Autopilot/
make flywoo_f405s_aio
```

3. Upload firmware
```bash
# MPU6000 Build (Old out of production flywoo gn405s AIO)
make flywoo_f405s_aio_mpu6000 upload
# ICM42688P Build (New flywoo gn405s AIO)
make flywoo_f405s_aio_icm42688p upload
```

### (Advanced) Configuring the board configuration
1. Launch GUI 
```bash
# Older PX4 firmwares ( < V1.11)
make omnibus_f4sd menuconfig

# Newer PX4 firmwares (V1.13)
make flywoo_f405s_aio boardguiconfig

# Save the file as ".config", the default filename, NO EXCEPTIONS! This will be saved in "PX4-Autopilot/platforms/nuttx/NuttX/nuttx" 
# This ".config" file is the full configuration of your board and needs to be configured to a "defconfig" file which is usable by the FCU board.
```
2. Make `defconfig` files from `.config` files
```bash
# Navigate to "PX4-Autopilot/platforms/nuttx/NuttX/nuttx"
# Generate a defconfig file using the following command:
make savedefconfig
# Copy the newly generated "defconfig" file in this directory to "PX4-Autopilot/boards/flywoo/f405s_aio/nuttx-config/nsh"
```

3. [Optional] Recover `.config` file from the `defconfig` file
```
make olddefconfig
```

### Troubleshooting
1. If you get the following error, `the git tag '137b7a4a8a' does not match the expected format.`:

Make sure you create a git tag like so:
```bash
git tag v1.13.0-0.1.0
```
2. Unable to detect flywoo board from QGroundControl, and doing `dmesg` shows `USB disconnect` for the Flywoo FCU. Refer to [link](https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html#ubuntu)

## Configuration
### PX4 Parameters
```yaml
################
# Communication/Serial ports
################
  baud rates:
    SER_TEL1_BAUD: 230400 
    SER_TEL2_BAUD: 230400 # Can be used for companion computer 
    SER_UART6_BAUD: 9600 # Can be used for lidar 
  GPS:
    GPS_1_CONFIG: Set to 0 # Disabled unless using
  Lidar:
    SENS_TFMINI_CFG: Set to UART6 
  Mavlink:
    Connection to companion computer:
      MAV_0_CONFIG: TEL2/102 
      MAV_0_MODE: 2 (ONBOARD)
      MAV_0_RATE: 0 (Half maximum)
      MAV_0_FORWARD: 0 (DISABLED)
      MAV_SYS_ID: (THIS NUMBER AFFECTS COMMUNICATION VIA MAVROS, SET TO MATCHING "target_system_id" on MAVROS)
    Connection to Lidar:
      MAV_1_CONFIG: UART6 
      MAV_1_MODE: Normal
      MAV_1_RATE: 1200 Byte/s
      MAV_1_FORWARD: True


################
# Lidar/Range finder
################
  EKF2_RNG_AID: Set to 0 to disallow use of range sensor.
  Position offset should be 0.0:
    EKF2_RNG_POS_X, EKF2_RNG_POS_Y, EKF2_RNG_POS_Z 
  Other values:
    EKF2_MIN_RNG: In units of meters. Set to 0.04m (or whatever your sensor reading tells you when your drone is on the ground). This is the expected rangefinder value when on the ground.

################
# Vicon (If using)
################
  EKF2_AID_MASK: 
    All other bits to false
    Bit 3: Set to true to enable vision position fusion
    Bit 4: Set to true to enable vision yaw fusion
  EKF2_EV_NOISE_MD: Set to true. This is so that we can use noise values from PX4 parameters directly.

################
# EKF2 
################
  SYS_MC_EST_GROUP: Set to EKF2
  EKF2_HGT_MODE: Set to 2 (If lidar is used)
  EKF2_MAG_CHECK: Set to 0
  EKF2_MAG_TYPE: If no magnetometer, set to NONE. Else AUTOMATIC

################
# Position control 
################
  MPC_ALT_HOLD: Set to 2 (requires distance sensor)
  MPC_THR_HOVER: Set to % required to achieve hover
  MPC_LAND_SPEED: Rate of descent, can leave as default

################
# Others 
################
  Checks
    COM_ARM_WO_GPS: Allow arming without GPS Values
    COM_RC_IN_MODE: Set to enable RC checks (if testing with a single drone)
    CBRK_SUPPLY_CHK: If disabling voltage check, set to 894281
  Auto disarm timeouts
    COM_DISARM_PRFLT: Timeout for auto disarm if vehicle is too slow to takeoff.
  Arming/Disarming
    MAN_ARM_GESTURE: Enable arming/disarming via stick gestures
  Motor ordering
    MOT_ORDERING: Set to 1 for betaflight. This is because the default motor ordering on the Flywoo GN405 is for betaflight.
  Sensor
    SENS_BOARD_ROT: Set to Yaw 270 degrees

################
# Battery
################
  Set the number of cells. Should be 3S.
  Set the voltage divider value (There is a bug that it cannot be changed if the initial value is -1.0, so set it to a random positive value first. Then calculate it with the real value)
```

- Reference: 
    - [V1.9 Reference](https://dev.px4.io/v1.9.0_noredirect/en/advanced/parameter_reference.html)
    - [tfmini module refence](https://dev.px4.io/v1.9.0_noredirect/en/middleware/modules_driver_distance_sensor.html)
    - For serial porting, refer to file `boards/flywoo/f405s_aio/nuttx-config/include/board.h`
        - [PINOUT] <-> <Firmware Port> <-> (PX4 Mapping) <-> |USB Port| <-> {Hardware Connected} 
        - [TX/RX1] <-> <UART 1> <-> (N/A) <->      |/dev/ttyS0|  <-> {Not used}
        - [TX/RX2] <-> <UART 2> <-> (TEL1/101) <-> |/dev/ttyS1|  <-> {Not used}
        - [TX/RX3] <-> <UART 3> <-> (N/A) <->      |N/A|         <-> {Not used}
        - [TX/RX4] <-> <UART 4> <-> (TEL2/102) <-> |/dev/ttyS2| <->  {Companion computer}
        - [TX/RX6] <-> <UART 6> <-> (UART6) <->    |/dev/ttyS3| <->  {Lidar}
        - [VTX] <->    <UART 5> <-> (N/A) <->      |N/A|        <->  {Not used, dedicated for VTX SBUS}

        - [SPI1] <-> {MPU6000 ICM42688P IMU}
        - [SPI2] <-> {SD Card}
        - [SPI3] <-> {BMP280 Barometer}
        - [I2C] <-> {GPS/Compass}

    - [VICON with PX4](https://docs.px4.io/main/en/ros/external_position_estimation.html)

## Interface with Radxa
1. Connection between FCU and Radxa
    - FCU: Connected as MAV_0_CONFIG to one of the UART (should be UART4, which is registered as TEL2 in PX4)
    - Radxa: Connected to UART_AO_B (/dev/ttyAML1)
2. Launch mavros bridge
    - Might need to perform the following in root mode. Enter with `sudo su`
    - `source /home/rock/.bashrc`
    - `roslaunch mavros apm.launch fcu_url:=/dev/ttyAML1:230400`

- References:
    - https://wiki.radxa.com/Zero/dev/serial-console
    - https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html
    - https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html

### Params:
1. VICON IP: 192.168.31.248
2. Central computer IP: 192.168.31.22 or 192.168.31.173
3. Radxa IP: 192.168.31.205
4. GCS IP: 192.168.31.61

### Transmitter-Receiver Binding (XSR Faast):
1. Hold bind button (while Receiver has no power)
2. Enter binding  mode on transmitter
3. Provide power to receiver 
4. If connection is successful, the receiver should have a solid green light
5. Restart receiver
Failsafe set to no pulse

## Troubleshooting

### PX4
```bash
#####
# Status
#####
# List topics
uorb status
mavlink status streams
# List processes
top

# Get diagnostic messages
dmesg

# List all modules
ls /bin/

#####
# Listening
#####

# Listen to uorb topics
listener TOPIC 
# Common topics:
# actuator_outputs
# actuator_outputs_sim
# actuator_motors

# Listen to mavlink topics
Use the MAVLink inspector in QGroundControl

#####
# Interacting with Modules
#####
## Distance sensor
tfmini status
## Sensors
sensors status
```

### PX4 SITL: A detailed introduction to creating new models

#### How actuator command work in SITL (Simulation-In-The-Loop)
PX4 sitl will send actuator commands via uorb to the [`SimulatorMavlink`](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp) on the [`HIL_ACTUATOR_CONTROLS`](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS) message. The values of the controls are from -1 to 1. `SimulatorMavlink` should then publish the array of controls (each corresponding to a motor) as a mavlink message.
[gazebo_mavlink_interface.cpp](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/main/src/gazebo_mavlink_interface.cpp) subscribes to this mavlink message and scales the actuator controls value (from -1 to 1) by a factor defined by the `input_scaling` parameter. This can be seen [here](https://github.com/PX4/PX4-SITL_gazebo-classic/blob/20ded0757b4f2cb362833538716caf1e938b162a/src/gazebo_mavlink_interface.cpp#L1190). 

What results is the reference input, which will be published as a Gazebo message of type `mav_msgs::msgs::CommandMotorSpeed`.
[gazebo_motor_model.cpp]() subscribes to the reference input topic defined by the `motorSpeedPubTopic` parameter and outputs the final force vector of the motor in gazebo.

#### Creating a new model
We will need to change the following parameters:
1. Objects making up drone model
  - Mass
  - Mass moment of inertia (Can be obtained from solidworks model under `mass properties`)
2. `libgazebo_mavlink_interface` plugin (Controls the actuator interface)
  - Under each channel in `control_channels`, for each motor make sure that `input_scaling` is set to the maximum angular velocity (in rad/s) of the drone. 
    - Practically, we would want to set the maximum angular velocity based on the maximum allowable continuous current specified on the UAS. 
  - Source code found in [SimulatorMavlink.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp)
3. `libgazebo_motor_model` plugin (Set the motor and propeller coefficients)
  - `maxRotVelocity`: Maximum angular velocity in rad/s
  - `motorConstant`: Also known as K_T or C_T in the literature, the motor constant coefficient. The thrust of each motor, `T = rot_velocity^2 * motorConstant`. The units of `motorConstant` are in `[kg][m]`.
  - `momentConstant`: This is calculated as C_T/C_M, where C_M is the moment constant coefficient. The units of `motorConstant` are in `[m]`.

#### Additional Info
- [SimulatorMavlink.cpp](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp) handles UORB messages from the PX4 SITL instance and publishes to mavlink messages. 
Note that [ESC_STATUS](https://mavlink.io/en/messages/common.html#ESC_STATUS) message published from PX4 SITL will not be accurate, as the [maximum RPM value is hardcoded](https://github.com/PX4/PX4-Autopilot/blob/d1266c856fbd759cbc6cf583c5221aab49962b30/src/modules/simulation/simulator_mavlink/SimulatorMavlink.cpp#L159) as well as the current. 
- [gazebo_motor_model.cpp]() subscribes to motor command from the topic defined by the `motorSpeedPubTopic` parameter, takes the command and puts it through a filter, after which it will set the final angular velocity of the motor joint accordingly accordingly. 

### Gazebo simulation
To change Gazebo world physics parameters
[Gazebo Physics Parameters](https://classic.gazebosim.org/tutorials?tut=physics_params&cat=physics)
Some level of damping and stiffness is required
```xml
      <collision name='...'>
        ...
        <surface>
          <contact>
            <ode>
              <min_depth>0.001</min_depth>
              <max_vel>0</max_vel>
              <kp>1e15</kp>
              <kd>1e13</kd>
            </ode>
          </contact>
        </surface>
      </collision>
```

#### Use dart with gazebo
1. Do source installation of DART: https://dartsim.github.io/install_dart_on_ubuntu.html 
  - NOTE: You will have to build from the latest version of DART
2. Full installation of Gazebo from source: https://classic.gazebosim.org/tutorials?tut=install_from_source&ver=4.0&cat=install
  - For noetic, we use the Gazebo 11 branch
  - If you have installed in `usr/local/`, and gazebo throws the error `gazebo: error while loading shared libraries: libgazebo_common.so.1: cannot open shared object file: No such file or directory`, you will have to run the foollowing commands to put `/usr/local/lib` into the load path.
    ```
      echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
      sudo ldconfig
    ```
3. Build `gazebo_ros_pkgs` from source for interfacing ROS with Gazebo: https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros 
  - You can't install from Debian packages because when Gazebo is compiled from source, Ubuntu's package manager apt is unable to find Gazebo. Therefore building `gazebo_ros_pkgs` from source is required.
4. How to use DART: https://dartsim.github.io/faq.html

References:
1. SDF Format for specifying physics engines: http://sdformat.org/spec?elem=physics
2. Examples: https://classic.gazebosim.org/tutorials?tut=haptix_world_sim_api&cat=haptix
