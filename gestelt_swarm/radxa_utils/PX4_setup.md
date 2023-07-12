# PX4 setup

# Building the firmware
1. Build the board firmware 
```bash 
cd ~/gestelt_ws/PX4-Autopilot/
# Default build for Flywoo F405 AIO
make flywoo_f405s_aio_default
```
2. Upload firmware
```bash
make flywoo_f405s_aio_default upload
```

# Hardware
## Hardware List
1. Receiver
2. Flywoo F405 AIO BOARD
    - Integrated ESCs
3. Sensors
    - Optical flow sensor
4. Motors
    - Flywoo NIN 1404 (4850 kV)
5. Frame
    - 
6. Onboard computer
    - Radxa Zero V1.5
7.  

# Quick Start
1. Start ROS Nodes
```bash
```

2. Start QGroundControl
```bash
~/Documents/QGroundControl.AppImage
```


# Troubleshooting
1. mav
```bash
mavlink status streams
```
