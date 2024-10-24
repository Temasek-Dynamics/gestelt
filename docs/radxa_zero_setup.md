# Radxa Zero Setup
Instructions for setting up of Radxa Zero computer 

## A: Flashing Ubuntu OS onto Radxa
1. Install the following tools on your main computer:
    ```bash
    sudo apt update
    sudo apt install python3-pip
    sudo pip3 install pyamlboot
    ```

2. Erase eMMC of Radxa
    - Press down boot button on Radxa before powering it on and connecting it to the PC.
    - Download `radxa-zero-erase-emmc.bin` AND `rz-udisk-loader.bin`
    - Run `sudo boot-g12.py radxa-zero-erase-emmc.bin`
        - This will automatically erase eMMC, then present eMMC as a USB storage device. No need to run `rz-udisk-loader.bin` afterwards
    - [Reference](https://wiki.radxa.com/Zero/install/eMMC_erase)

3. Remove autoboot on radxa
    - Install dependencies: 
        - `sudo apt-get install -y wget bc nano mc build-essential autoconf libtool cmake pkg-config git python-dev swig libpcre3-dev libnode-dev gawk wget diffstat bison flex device-tree-compiler libncurses5-dev gcc-aarch64-linux-gnu g++-aarch64-linux-gnu binfmt-support binfmt-support qemu-user-static gcc-aarch64-linux-gnu gcc-arm-linux-gnueabihf gcc-aarch64-linux-gnu fastboot`
    - Git clone 2 packages
        - `git clone  --branch radxa-zero-v2021.07 https://github.com/radxa/u-boot.git`
        - `git clone https://github.com/radxa/fip.git`
    - Edit the radxa zero config in the u-boot repo
        - `nano u-boot/configs/radxa-zero_defconfig`
        - Add "CONFIG_BOOTDELAY=-2" before "#CONFIG_DISPLAY_CPUINFO is not set"
    - Compile from `u-boot`
        ```bash
        cd u-boot
        export ARCH=arm
        export CROSS_COMPILE=aarch64-linux-gnu-
        make radxa-zero_defconfig
        make
        ```
    - Compile from `fip/radxa-zero`. Note: "make" here will have no output as it is disabled
        ```bash
        cp u-boot.bin ../fip/radxa-zero/bl33.bin
        cd ../fip/radxa-zero
        make
        ```
    - Copy u-boot.bin over
        - Sideload (Safer option) 
            - Hold down boot button on radxa and connect to PC, then run:
                - `sudo boot-g12.py /path/to/fip/radxa-zero/u-boot.bin`
        - (ONLY AS LAST RESORT) Use dd to write over the eMMC
            - Hold down boot button on radxa and connect to PC. Run:
                - `sudo boot-g12.py rz-udisk-loader.bin` to expose the Radxa as a mountable disk
            - Use dmesg/lsblk to figure out what `sdX` is Radxa
            - In fip/radxa-zero, run `sudo dd if=u-boot.bin of=/dev/sdX bs=512 seek=1` to copy the files over
    - Reboot device
    - [Reference](https://github.com/matthewoots/documentation/blob/main/radxa-zero/radxa-remove-autoboot-countdown.md)

4. Install [Balena Etcher](https://github.com/balena-io/etcher)

5. Flash Ubuntu OS onto Radxa
    - If you are doing a fresh image flash, download images from [here](https://wiki.radxa.com/Zero/downloads), the file name should be something like `radxa-zero-ubuntu-focal-server-arm64-XXX-mbr.img`.
        - Decompress images using `xz -v --decompress IMAGE_COMPRESSED`
    - Else, if you are flashing from an existing image, get the .iso image ready.
    - Hold the boot button on Radxa before connecting to the PC, then run `sudo boot-g12.py rz-udisk-loader.bin`, the radxa eMMC should appear as a flashable device.
    - Use BalenaEtcher to mount the image.

6. Enable serial port connection with FCU
```bash
vim /etc/ssh/sshd_config
# change ChallengeResponseAuthentication to no
sudo systemcyl reload sshd

# Edit /boot/uEnv.txt to open serial port with FCUs 
vim /boot/uEnv.txt
# change the overlays line to "overlays=meson-g12a-uart-ao-b-on-gpioao-2-gpioao-3"
```

7. Connect to a wifi network

```bash
# Proceed on activating WIFI using https://wiki.radxa.com/Zero/Ubuntu
# Root username and pasword is rock/rock
sudo su
sudo nmcli r wifi on
sudo nmcli dev wifi
sudo nmcli dev wifi connect "wifi_name" password "wifi_password"                   
```

8. Install ROS and dependencies
```bash
# Copy radxa_setup.sh script over to the radxa 
scp path/to/radxa_setup.sh rock@IP_ADDR:/home/rock/radxa_setup.sh

# Run setup script
./scripts/radxa_setup.sh
```

9. Synchronize time between ubuntu machines (radxas and central computer)
    - Make sure ntp daemon is installed on both host and client computer: `sudo apt-get install ntp`
    - (Optional) Add network alias to /etc/hosts: `192.168.31.22 master0`

    - Host computer
        1. modify the ntp config: `sudo vim /etc/ntp.conf` and add the following lines
            ```bash
                # Add local clock as server in the event there is no internet connection
                server 127.127.1.0
                # fudge forces the system to treat local clock as a ntp server
                fudge 127.127.1.0 stratum 10
                # Restrict access to only within the local network subnet
                restrict 192.168.31.0 mask 255.255.255.0 nomodify notrap
            ```
        2. Restart ntp: `sudo /etc/init.d/ntp restart`
        3. Monitor system log to see if you can synchronize with a time server: `tail -f /var/log/syslog`

    - Client computer
        1. modify the ntp config: `sudo vim /etc/ntp.conf` and add the following lines
            ```bash
                # uncomment the following lines
                disable auth
                broadcastclient
                
                # Add the local ntp server as a master, please DO NOT ever add 'prefer' keyword, it will not work
                server master0 iburst
            ```
        2. Restart ntp: `sudo /etc/init.d/ntp restart`
        3. Monitor connections to peer: `ntpq -c lpeer`
    
    - [Reference: how-do-i-setup-a-local-ntp-server](https://askubuntu.com/questions/14558/how-do-i-setup-a-local-ntp-server)


    - More commands
        ```bash
            #####
            # ntp
            #####
            # Restart ntp server
            sudo /etc/init.d/ntp restart

            # Check for synchronization 
            tail -f /var/log/syslog

            # Check list of peers
            ntpq -c lpeer

            # systemctl
            systemctl status chronyd
            sudo systemctl restart chronyd
            
            # chronyc
            chronyc -n sources 
            chronyc sources -v
            chronyc sourcestats -v
            chronyc tracking
            chronyc activity

            # chronyc (host)
            chronyc clients

            # timedatectl
            timedatectl status
            timedatectl set-ntp 1
            timedatectl set-local-rtc 1

            # To manually set time, disable ntp first 'timedatectl set-ntp 0'
            timedatectl set-time '2023-08-18 15:05:30'
        ```

10. MAVROS set-up
    - Enable publishing of TF for the `global_position` plugin in `px4_config.yaml`. This provides us with the TF between map and drone base_link frame:
    ```yaml
    global_position:
    frame_id: "map"
    ...
    tf:
        send: true # Set to true
        ...
    ```
    - In the mavros launch file, set the FCU url to the address of the serial port connecting to the flight controller. 
        - [DEPRECATED] For pins (6, 8, 10) or (GND, UART_AO_A_TXD, UART_AO_A_RXD), use `/dev/ttyAML0:230400`
        - For pins (7, 9, 11) or (UART_AO_B_RX, GND, UART_AO_B_TX), use `/dev/ttyAML1:230400`
    - Set the GCS URL to the ip address of the computer running QGroundControl.

## Useful info

### Clone an image on Radxa
```bash
# Hold boot button and connect radxa to pc, run the following command
sudo boot-g12.py rz-udisk-loader.bin

# Get device name, should be something like "/dev/sdX" where X is a letter
lsblk -p

# Copy device image
export SD_CARD_DEVICE_NAME=/dev/sdX
export IMAGE_FILE_NAME=~/Downloads/gestelt_os_15_8_23.img

sudo dd bs=4M if=$SD_CARD_DEVICE_NAME of=$IMAGE_FILE_NAME conv=fsync status=progress
sudo chown $USER: $IMAGE_FILE_NAME

# [For the first time] Installation of PiShrink
cd ~
wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
chmod +x pishrink.sh
sudo mv pishrink.sh /usr/local/sbin/pishrink

# Shrink image
sudo pishrink $IMAGE_FILE_NAME

# Compress image if necessary 
tar -czf $IMAGE_NAME.tar.gz $IMAGE_NAME.img

# Proceed to load image onto board using BalenaEtcher or other methods
```
[Reference](https://github.com/matthewoots/documentation/blob/main/radxa-zero/radxa-flash-backup-image.md)

### Copy files over to radxa
```bash
# Copy with override
scp -r path/to/dir rock@IP_ADDR:path/to/dir

scp -r /home/john/gestelt_ws/src/gestelt/gestelt_bringup/ rock@192.168.31.205:/home/rock/gestelt_ws/src/gestelt/

scp -r /home/john/gestelt_ws/src/gestelt/ rock@192.168.31.205:/home/rock/gestelt_ws/src/
```

### Generate ssh keys to make your life easier
```bash
# On central computer, generate a public key
ssh-keygen -t ecdsa -b 521
# Copy public key over to robot
ssh-copy-id -i ~/.ssh/id_ecdsa rock@ROBOT_IP
```
On central computer, setup ssh config file. Save as `~/.ssh/config`
```bash
Host robot
    User rock
    Hostname ROBOT_IP
    IdentityFile ~/.ssh/id_ecdsa
    ForwardX11 yes
```
Now you can access the Radxa using `ssh robot`

### Run script on startup on Radxa
This is useful if we want to run mavros and the flight manager nodes on startup.
```bash
crontab -e 
# Add the line: @reboot sh SCRIPT_TO_RUN
# Example: @reboot sh /home/rock/gestelt_ws/src/gestelt/gestelt_swarm/radxa_utils/scripts/radxa_startup.sh
```

### Set wireless network priority
```bash
nmcli -f NAME,UUID,AUTOCONNECT,AUTOCONNECT-PRIORITY c

nmcli connection modify WIFINAME connection.autoconnect-priority 10
```

### Enable performance mode
```bash
sudo apt-get install cpufrequtils
echo 'GOVERNOR="performance"' | sudo tee /etc/default/cpufrequtils
sudo systemctl disable ondemand
# Reboot system and check using the following command
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
```

# Troubleshooting
1. Permission denied when accessing serial port of FCU from MAVROS on Radxa
    - Make sure Baud rate when launching MAVROS matches what is set as PX4 params
    - Make sure your current user (on Radxa) is added to the dialout group through `sudo usermod -a -G dialout $USER`
2. Unable to boot up Radxa.
    - Did you miss out on any installation steps?
    - If not, proceed to reflash the bootloader. This is done by pressing the boot button on radxa and connecting it to the PC. You would then run the following command: `sudo boot-g12.py /path/to/fip/radxa-zero/u-boot.bin` 