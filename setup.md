# Gestelt setup

1. Install binaries
```bash 
export ROS_DISTRO="noetic"

# Install ROS dependencies
# sudo apt install ros-${ROS_DISTRO}-tf2-sensor-msgs -y
sudo apt install ros-${ROS_DISTRO}-mavlink ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-msgs ros-${ROS_DISTRO}-mavros-extras -y
# Install external dependencies
sudo apt install git build-essential tmux python3-vcstool xmlstarlet -y
sudo apt install libopencv-dev ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-pcl-ros libeigen3-dev -y
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
# Extra tools for debugging
sudo snap install plotjuggler
```

2. Clone repositories
```bash
mkdir -p ~/gestelt_ws/src/
cd ~/gestelt_ws/src
git clone https://github.com/JohnTGZ/gestelt.git
cd gestelt
vcs import < simulators.repos --recursive
```

3. Install PX4 firmware
```bash
# cd to PX4-Autopilot repo
cd ~/gestelt_ws/PX4-Autopilot
bash ./Tools/setup/ubuntu.sh --no-nuttx
# Make SITL target for Gazebo simulation
DONT_RUN=1 make px4_sitl_default gazebo-classic
cp -r ~/gestelt_ws/src/gestelt/gestelt_bringup/simulation/models/raynor ~/gestelt_ws/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/

# If you screw up the PX4 Autopilot build at any point, clean up the build files via the following command:
make distclean
```

4. Building 
```bash
# Assuming your workspace is named as follows
cd ~/gestelt_ws/

# Building for debugging/development
catkin_make 
# Building for release mode (For use on Radxa)
catkin_make -DCMAKE_BUILD_TYPE=Release
```

5. Setting up the Offboard computer
We assume that Radxa is being used as the offboard computer. Refer to [radxa_setup](./radxa_setup/README.md) for more instructions.
