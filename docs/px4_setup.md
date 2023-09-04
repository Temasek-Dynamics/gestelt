# PX4 setup

## Building the firmware
1. Build the board firmware 
```bash 
# Clone the custom version of PX4 
git clone https://github.com/matthewoots/PX4-Autopilot.git
cd ./PX4-Autopilot/
# Default build for Flywoo F405 AIO
make flywoo_f405s_aio_default
```
2. Upload firmware
```bash
make flywoo_f405s_aio_default upload
```

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
        - UART 1, /dev/ttyS0
        - UART 2 <-> TEL1/101 (/dev/ttyS1)
        - UART 4 <-> TEL2/102 (/dev/ttyS2)
        - UART 6 <-> UART6 (/dev/ttyS3)
    - [VICON with PX4](https://docs.px4.io/main/en/ros/external_position_estimation.html)

## Interface with Radxa
1. Set up headless mode on Radxa:
    - edit `/boot/uEnv.txt` and remove line with `console=ttyAML0,115200`, which removes this port as a debugging console
2. Connection between FCU and Radxa
    - FCU: Connected as MAV_0_CONFIG to one of the UART (should be UART4, which is registered as TEL2 in PX4)
    - Radxa: Connected to UART_AO_A (/dev/ttyAML0)
3. Launch mavros bridge
    - Might need to perform the following in root mode. Enter with `sudo su`
    - `source /home/rock/.bashrc`
    - `roslaunch mavros apm.launch fcu_url:=/dev/ttyAML0:230400`

- References:
    - https://wiki.radxa.com/Zero/dev/serial-console
    - https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html
    - https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html

### Params:
1. VICON IP: 192.168.31.248
2. Central computer IP: 192.168.31.22 or 192.168.31.173
3. Radxa IP: 192.168.31.205
4. GCS IP: 192.168.31.61

## Troubleshooting
```bash
# Communication
uorb status
mavlink status streams

# List processes
top

# List all modules
ls /bin/

# Listen to topics
listener TOPIC 

# Modules
## Distance sensor
tfmini status
## Sensors
sensors status
```