# PX4_SITL_setup

# Setup
1. Refer to https://docs.px4.io/main/en/simulation/hitl.html for setup instructions on PX4 device

2. Set in [raynor.sdf](../gestelt_bringup/simulation/models/raynor/raynor.sdf):
    - `serialEnabled` to `1` 
    - `serialDevice` to `/dev/ttyACM0`. One way to check the port is to do `dmesg | grep "tty"`
    - `hil_mode` to `1`

Make sure this file is copied over to the PX4_Autopilot simulation folder. For example, do the following:
```bash
cp -r ~/gestelt_ws/src/gestelt/gestelt_bringup/simulation/models/raynor ~/gestelt_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/
```

# Quick Start

```bash
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_central_hitl.sh http://192.168.31.173:11311 192.168.31.173

cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh http://192.168.31.173:11311 192.168.31.173
```