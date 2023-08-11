# Radxa Setup

## Flashing Ubuntu OS onto Radxa
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
        - This will automatically erase eMMC, then present eMMC as a USB storage device.
        - A successful erase should show something like:
        ```
        Firmware Version :
        ROM: 3.2 Stage: 0.0
        Need Password: 0 Password OK: 1
        Writing radxa-zero-erase-emmc.bin at 0xfffa0000...
        [DONE]
        Running at 0xfffa0000...
        [DONE]
        AMLC dataSize=16384, offset=65536, seq=0...
        [DONE]
        AMLC dataSize=49152, offset=393216, seq=1...
        [DONE]
        AMLC dataSize=16384, offset=229376, seq=2...
        [DONE]
        AMLC dataSize=49152, offset=245760, seq=3...
        [DONE]
        AMLC dataSize=16384, offset=65536, seq=4...
        [DONE]
        AMLC dataSize=49152, offset=393216, seq=5...
        [DONE]
        AMLC dataSize=16384, offset=229376, seq=6...
        [DONE]
        AMLC dataSize=49152, offset=245760, seq=7...
        [DONE]
        [BL2 END]
        ```
    - Run `sudo boot-g12.py radxa-zero-erase-emmc.bin`
    - [Reference](https://wiki.radxa.com/Zero/install/eMMC_erase)

3. Install [Balena Etcher](https://github.com/balena-io/etcher)

4. Flash Ubuntu OS onto Radxa
    - Download images from [here](), the file name should be something like `radxa-zero-ubuntu-focal-server-arm64-XXX-mbr.img`
    - Decompress images using `xz -v --decompress IMAGE_COMPRESSED`
    - Use BalenaEtcher to mount the image. After step 2 (erasing the eMMC), the radxa eMMC should appear as a flashable device

5. Remove autoboot on radxa
    - Hold down boot button on radxa and connect to PC
    - `sudo boot-g12.py rz-udisk-loader.bin` should expose the Radxa as a mountable disk
    - Install dependencies: 
        - `sudo apt-get install -y wget bc nano mc build-essential autoconf libtool cmake pkg-config git python-dev swig libpcre3-dev libnode-dev gawk wget diffstat bison flex device-tree-compiler libncurses5-dev gcc-aarch64-linux-gnu g++-aarch64-linux-gnu binfmt-support binfmt-support qemu-user-static gcc-aarch64-linux-gnu gcc-arm-linux-gnueabihf fastboot`
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
    - Compile from `fip/radxa-zero`
        ```bash
        cp u-boot.bin ../fip/radxa-zero/bl33.bin
        cd ../fip/radxa-zero
        make
        ```
    - Use dd to write over the eMMC
        - Use dmesg to figure out what `sdX` is Radxa
        - `sudo dd if=u-boot.bin of=/dev/sdX bs=512 seek=1`
    - [Reference](https://github.com/matthewoots/documentation/blob/main/radxa-zero/radxa-remove-autoboot-countdown.md)

6. Connect to a wifi network

```bash
# Proceed on activating WIFI using https://wiki.radxa.com/Zero/Ubuntu
# Root username and pasword is rock/rock
sudo su
nmcli r wifi on
nmcli dev wifi
nmcli dev wifi connect "wifi_name" password "wifi_password"                   
```

7. Install ROS and dependencies
```bash
# Run setup script
./scripts/radxa_setup.sh
```

8. Synchronize time between ubuntu machines (radxas and central computer)
    - On host computer 
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

    - On client computer
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
        # Add IP_ADDR HOST_NAME 
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
        # Add the line: server NTP-server-host iburst
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

9. MAVROS set-up
    - Enable publishing of TF for the `global_position` plugin in `px4_config.yaml`. This provides us with the TF between map and drone base_link frame:
    ```yaml
    global_position:
    frame_id: "map"
    ...
    tf:
        send: true # Set to true
        ...
    ```
    - In the mavros launch file, set the FCU url to the address of the serial port connecting to the flight controller. And set the GCS URL to the ip address of the computer runnig QGroundControl.

## Useful info

### Clone an image on Radxa
1. https://github.com/matthewoots/documentation/blob/main/radxa-zero/radxa-flash-backup-image.md

### Copy source code over to radxa
```bash
# Copy with override
scp -r ~/gestelt_ws/src/gestelt rock@192.168.31.166:/home/rock/gestelt_ws/src/ 
```
### Run script on startup on Radxa
This is useful if we want to run mavros and the flight manager nodes on startup.
```bash
crontab -e 
# Add the line: @reboot sh SCRIPT_TO_RUN
# Example: @reboot sh /home/rock/gestelt_ws/src/gestelt/gestelt_swarm/radxa_utils/scripts/radxa_startup.sh
```

# Troubleshooting
1. Permission denied when accessing serial port
- Make sure Baud rate matches what is set as PX4 params
- Make sure your user (default is "rock") is added to the dialout group through `sudo usermod -a -G dialout $USER`
2. 