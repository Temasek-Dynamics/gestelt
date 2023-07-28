# PX4 setup

# Building the firmware
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

# Configuration
## Parameters
1. Set Parameters
- RC 
    - COM_RC_IN_MODE: Set to 1 to disable RC Checks
    - COM_FAIL_ACT_T: Failsafe reaction delay
    - NAV_RCL_ACT: Failsafe action (Disabled, loiter, return, land, disarm, terminate)
    - COM_RCL_EXCEPT: Disable RC loss failsafe in specific automatic modes (mission, hold, offboard)
- GPS
    - COM_ARM_WO_GPS: Allow arming without GPS Values
- Land mode
    - COM_DISARM_LAND: Select checkbox to specify that the vehicle will disarm after landing. The value must be non-zero but can be a fraction of a second.
    - MPC_LAND_SPEED: Rate of descent
- Offboard loss failsafe
    - COM_OF_LOSS_T: Delay after loss of offboard connection before the failsafe is triggered.
    - COM_OBL_RC_ACT: 	Failsafe action if RC is available: Position mode, Altitude mode, Manual mode, Return mode, Land mode, Hold mode.
- Attitude Trigger:
    - FD_FAIL_P: Maximum allowed pitch (in degrees).
    - FD_FAIL_R: Maximum allowed roll (in degrees).
    - FD_FAIL_P_TTRI: Time to exceed FD_FAIL_P for failure detection (default 0.3s).
    - FD_FAIL_R_TTRI: Time to exceed FD_FAIL_R for failure detection (default 0.3s).
- Auto disarm timeouts
    - COM_DISARM_PRFLT: Timeout for auto disarm if vehicle is too slow to takeoff.
- Arming/Disarming
    - MAN_ARM_GESTURE: Enable arming/disarming via stick gestures
- Checks:
    - CBRK_FLIGHTTERM: this check is always enabled on takeoff, irrespective of the CBRK_FLIGHTTERM parameter. Failure detection during flight is deactivated by default. Enable by setting to 0.
    - CBRK_SUPPLY_CHK: Set to 894281
- Motor ordering:
    - MOT_ORDERING: Set to 1 for betaflight
- Sensor rotation:
    - SENS_BOARD_ROT: Set to Yaw 270 degrees

- EKF:
    - SYS_MC_EST_GROUP: Set to EKF2
    - EKF2_HGT_MODE: Set to 2 (If lidar is used)
    - EKF2_MAG_CHECK: Set to 0
    - EKF2_MAG_TYPE: If no magnetometer, set to NONE. Else AUTOMATIC

- Serial ports
    - baud rates:
        - SER_TEL1_BAUD: 57600 
        - SER_TEL2_BAUD: 921600
        - SER_UART6_BAUD: 9600
    - GPS:
        - GPS_1_CONFIG: Set to 0
    - Lidar
        - SENS_TFMINI_CFG: Set to TEL1/101 (UART 2, /dev/ttyS1)
    - Mavlink:
        - MAV_0_CONFIG: TEL2/102 (UART 4, /dev/ttyS2)
        - MAV_0_MODE: 2 (ONBOARD)
        - MAV_0_RATE: 0 (Half maximum)
        - MAV_0_FORWARD: 0 (DISABLED)
        - MAV_1_CONFIG: 0

- Lidar: 
    - Position offset
        - EKF2_RNG_POS_X, EKF2_RNG_POS_Y, EKF2_RNG_POS_Z 
    - Other values
        - EKF2_RNG_SFE, EKF2_RNG_DELAY, EKF2_RNG_NOISE


- Reference: 
    - [V1.9 Reference](https://dev.px4.io/v1.9.0_noredirect/en/advanced/parameter_reference.html)
    - [tfmini module refence](https://dev.px4.io/v1.9.0_noredirect/en/middleware/modules_driver_distance_sensor.html)
    - For serial porting, refer to file `boards/flywoo/f405s_aio/nuttx-config/include/board.h`
        - UART 1, /dev/ttyS0
        - UART 2 <-> TEL1/101 (/dev/ttyS1)
        - UART 4 <-> TEL2/102 (/dev/ttyS2)
        - UART 6 <-> UART6 (/dev/ttyS3)

2. Set flight modes on transmitter
    - https://docs.px4.io/main/en/config/flight_mode.html
    - Set kill switch

## Interface with Radxa
1. Set up headless mode on Radxa:
    - edit `/boot/uEnv.txt` and remove line with `console=ttyAML0,115200`, which removes this port as a debugging console
2. Connection between FCU and Radxa
    - FCU: Connected as MAV_0_CONFIG to one of the UART (should be UART4, which is registered as TEL2 in PX4)
    - Radxa: Connected to UART_AO_A (/dev/ttyAML0)
3. Testing: 
    - `roslaunch mavros apm.launch fcu_url:=/dev/ttyAML0:921600`
    - ls /dev/ttyAML0



- References:
    - https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html
    - https://docs.px4.io/main/en/companion_computer/pixhawk_companion.html


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

# References

## Hardware 
Refer to [gn405s_wiring_diagram](./images/gn405s_wiring_diagram.png)
1. Serial Receiver
    - FM800 (FASST RX)
    - OR FrSky R-XSR/RXSR Ultra mini S.BUS Smart Port Redundancy Receiver for FPV Drone
2. Flywoo F405 AIO BOARD
    - MCU: STM32F405 BGA
    - Integrated ESCs
    - Built-in Sensors: BMP280 Barometer AND MPU6000 IMU.  
    - [Link](https://flywoo.net/products/goku-gn-405s-20a-aio-bmi270-25-5-x-25-5)
3. Sensors
    - Lidar: TF-Luna V1.2
4. Motors
    - Flywoo NIN 1404 (4850 kV)
5. Frame
    - TODO 
6. Companion computer
    - Radxa Zero V1.5
        - [GPIO Wiring](https://wiki.radxa.com/Zero/hardware/gpio)
        - [Pin location](https://wiki.radxa.com/Zero)
        - [Github docu](https://github.com/radxa/documentation/tree/master/rs102)

## Others
- [PX4 Tune meanings](https://docs.px4.io/v1.9.0/en/getting_started/tunes.html)