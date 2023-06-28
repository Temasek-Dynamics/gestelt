# Setting up the Radxa
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

# Copy .bashrc configuration file over
export ROS_DISTRO="noetic"
source /opt/ros/noetic/setup.bash
alias sros="source /opt/ros/noetic/setup.bash"
alias sws="source devel/setup.bash"
alias sbash="source ~/.bashrc"

# Set up network
# http://wiki.ros.org/ROS/NetworkSetup
export ROS_MASTER_URI=http://PC_IP:11311

```