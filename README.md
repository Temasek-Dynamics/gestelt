# gestelt
A simple path planning framework for swarm robots. This is a work in progress, and a lot of existing components are refactored parts of the work done by ZJU in [EGO-Planner-V2 repo](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2).

# Requirements
1. Operating System/Frameworks
    - Ubuntu 20.04 (ROS Noetic)
2. Tools
    - vcstool: For repo setup 
    - xmlstarlet: For multi-vehicle simulation
3. Simulation
    - Gazebo Classic (Version 11)
    - PX4-Autopilot (Main branch)
    - Simple Quad Simulator
4. Other packages:
    - MavROS 
    - plotjuggler (For visualization of data over time)
    - Listed in "Setup" section

# Setup
Refer to [setup.md](./setup.md)

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

# Acknowledgements
Plenty of inspiration was taken from the [EGO-Planner-V2 repo](https://github.com/ZJU-FAST-Lab/EGO-Planner-v2) from ZJU, without which, this would not have been possible.

# License
The MIT License (MIT)

Copyright (c) 2023 John Tan

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.