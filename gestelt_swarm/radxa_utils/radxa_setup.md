
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
./scripts/radxa_setup.sh
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
alias ez_make="cd ~/gestelt_ws && catkin_make -DCMAKE_BUILD_TYPE=Release -DCATKIN_BLACKLIST_PACKAGES='rviz_plugins;swarm_bridge;central_benchmark;'"
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

# To any machine
export ROS_MASTER_URI=http://$MASTER_IP:11311
export ROS_HOSTNAME=$SELF_IP
export ROS_IP=$SELF_IP
```

# Remove autoboot on radxa
1. Hold down boot button on radxa and connect to PC
2. `lsusb` should show Amlogic, Inc. 
3. `sudo boot-g12.py rz-udisk-loader.bin` should expose the Radxa as a mountable disk
4. Follow [this guide to remove autoboot countdown](https://github.com/matthewoots/documentation/blob/main/radxa-zero/radxa-remove-autoboot-countdown.md)
    - Notes: use the `git checkout radxa-zero-v2021.07` branch for u-boot

# Synchronize time between ubuntu machines (radxas and central computer)

## Set up an NTP server central computer
1. Install ntp
```bash
sudo apt install ntp
``` 

2. Add the following as pool servers using `sudo vim /etc/ntp.conf `
```bash
pool 0.sg.pool.ntp.org iburst
pool 1.sg.pool.ntp.org iburst
pool 2.sg.pool.ntp.org iburst
pool 3.sg.pool.ntp.org iburst
```

3. Check that server is up and running
```bash
sudo service ntp status
```

4. Configure firewall to allow access to ntp server from clients
```bash
sudo ufw allow from any to any port 123 proto udp
```

## Set up an NTP client on client computer (Radxa), using central computer as host
1. Set up time zone
```bash 
sudo timedatectl set-timezone Singapore
```

2. Install chrony
```bash 
sudo apt update
sudo apt install chrony -y
sudo systemctl start chronyd
```

3. Specify host name. Set iburst to ensure that it synchronizes as soon as it establishes a connection with the host.
```bash
sudo vim /etc/hosts
# Add IP_ADDR HOST_NAME iburst
```

4. 
```bash
# Check status
sudo systemctl status chronyd
chronyc activity
chronyc sourcestats -v
```

5. Set NTP host
```bash
sudo vim /etc/chrony/chrony.conf
# Add the line: server NTP-server-host
sudo timedatectl set-ntp true
sudo systemctl restart chronyd
# Enable chronyd on boot
systemctl enable chronyd
```

6. Check NTP Status
```bash
sudo chronyc clients
chronyc sources
chronyc tracking
```

# Run script on startup on Radxa
This is useful if we want to run mavros and the flight manager nodes on startup.
```bash
crontab -e 
# Add the line: @reboot sh SCRIPT_TO_RUN
# Example: @reboot sh /home/rock/gestelt_ws/src/gestelt/gestelt_swarm/radxa_utils/scripts/radxa_startup.sh
```

# Clone an image on Radxa
1. https://github.com/matthewoots/documentation/blob/main/radxa-zero/radxa-flash-backup-image.md

# Useful commands

## Copy source code over to radxa
```bash
# Copy with override
scp -r ~/gestelt_ws/src/gestelt rock@192.168.31.166:/home/rock/gestelt_ws/src/ 
```


# Troubleshooting
1. Permission denied when accessing serial port
- Make sure Baud rate matches what is set as PX4 params
- Make sure your user (default is "rock") is added to the dialout group through `sudo usermod -a -G dialout $USER`
2. 