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


2. Set flight modes on transmitter
    - https://docs.px4.io/main/en/config/flight_mode.html
    - Set kill switch

## Test plan
1. Test position/manual mode with transmitter
    - Calibrate IMU
    - Test [arming/disarming](https://docs.px4.io/main/en/advanced_config/prearm_arm_disarm.html)
    - Test Stabilized mode flying (or altitude and position)
    - Run failsafe simulations
    - [Autotune vehicle](https://docs.px4.io/main/en/config/autotune.html)

2. 

## Troubleshooting
1. mav
```bash
mavlink status streams
```

# References

## Hardware 
Refer to [gn405s_wiring_diagram](./images/gn405s_wiring_diagram.png)
1. Serial Receiver
    - FM800 (FASST RX)
2. Flywoo F405 AIO BOARD
    - MCU: STM32F405 BGA
    - Integrated ESCs
    - Built-in Sensors: BMP280 Barometer AND MPU6000 IMU.  
    - [Link](https://flywoo.net/products/goku-gn-405s-20a-aio-bmi270-25-5-x-25-5)
3. Sensors
    - Optical flow sensor
4. Motors
    - Flywoo NIN 1404 (4850 kV)
5. Frame
    - TODO 
6. Companion computer
    - Radxa Zero V1.5
        - [GPIO Wiring](https://wiki.radxa.com/Zero/hardware/gpio)
        - [Pin location](https://wiki.radxa.com/Zero)

## Others
- [PX4 Tune meanings](https://docs.px4.io/v1.9.0/en/getting_started/tunes.html)