
# Installation of dependencies 

```bash
# Install tools
sudo pip3 install pyamlboot

# Use lsusb command to check if radxa is connected
lsusb
# Use dmesg to find the device filepath
sudo dmesg

# To erase the eMMC 
# https://wiki.radxa.com/Zero/install/eMMC_erase
sudo boot-g12.py radxa-zero-erase-emmc.bin

# Get images from https://wiki.radxa.com/Zero/downloads
xz -v --decompress IMAGE_COMPRESSED

# Flash the image using balena etcher

# Proceed on activating WIFI using https://wiki.radxa.com/Zero/Ubuntu
# Root username and pasword is rock/rock
sudo su
nmcli r wifi on
nmcli dev wifi
nmcli dev wifi connect "wifi_name" password "wifi_password"                   

# Install ROS at http://wiki.ros.org/noetic/Installation/Ubuntu

# Run setup script
./radxa_setup.sh
```

# Append the following to .bashrc

```bash
# Copy .bashrc configuration file over
export ROS_DISTRO="noetic"
source /opt/ros/noetic/setup.bash
alias sros="source /opt/ros/noetic/setup.bash"
alias sws="source devel/setup.bash"
alias sbash="source ~/.bashrc"

# Set up network
# http://wiki.ros.org/ROS/NetworkSetup
export MASTER_IP=192.168.31.173
export SELF_IP=192.168.31.173

export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP

# Convenience function
alias pull_repo="git -C ~/gestelt_ws/src/gestelt/ pull"
alias ez_make="cd ~/gestelt_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES='rviz_plugins;'"
alias cd_scripts="cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts/"
```

# Network setup
Setting environment variables manually
```bash
# Xiaomi network
    # Central PC
        export MASTER_IP=192.168.31.173
        export SELF_IP=192.168.31.173
    # UAV
        export MASTER_IP=192.168.31.173
        export SELF_IP=192.168.31.166

# Nvidia network
    # Central PC
        export MASTER_IP=192.168.1.112
        export SELF_IP=192.168.1.112
    # UAV
        export MASTER_IP=192.168.1.112
        export SELF_IP=192.168.1.134


# To any machine
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP

```