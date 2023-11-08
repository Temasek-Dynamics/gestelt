# gestelt
A swarm-focused path planning framework. 

For simulation and deployment on a physical drone, PX4 is the firmware of choice, although it is possible to remap the topics for use with Ardupilot or any other Mavlink-compatible firmware.

# Architecture
<img src="docs/pictures/gestelt_architecture_24_10.png" alt="Gestelt Architecture" style="width: 1200px;"/>

# Installation and Setup for Simulation
1. Clone repositories
```bash
mkdir -p ~/gestelt_ws/src/
cd ~/gestelt_ws/src
git clone https://github.com/JohnTGZ/gestelt.git
cd gestelt
vcs import < simulators.repos --recursive
vcs import < thirdparty.repos --recursive
```

2. Install PX4 firmware
```bash
# cd to PX4-Autopilot repo
cd ~/gestelt_ws/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh 
# Make SITL target for Gazebo simulation
DONT_RUN=1 make px4_sitl_default gazebo-classic

# Copy the custom drone model over
cp -r ~/gestelt_ws/src/gestelt/gestelt_bringup/simulation/models/raynor ~/gestelt_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/

# [EMERGENCY USE] IF you screw up the PX4 Autopilot build at any point, clean up the build files via the following command:
make distclean
```

3. Building the workspace
```bash
# Assuming your workspace is named as follows
cd ~/gestelt_ws/

# Building for debugging/development
catkin build 
# Building for release mode (For use on Radxa)
catkin build -DCMAKE_BUILD_TYPE=Release
```

# Acknowledgements