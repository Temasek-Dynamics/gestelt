# radxa 2 pro

## Ubuntu Compatibility 
So far, only Ubuntu 22.04 images from release [b32](https://github.com/radxa-build/radxa-zero-2pro/releases/tag/b32) are tested and working.
Ubuntu 20.04 images from release [20230301-0102](https://github.com/radxa-build/radxa-zero-2pro/releases/tag/20230301-0102) do not work.

## Setup

### Flashing images
1. Hold maskrom button and plug in USB-C cable to OTG USB-C Port of radxa
2. Download `radxa-zero2-2pro-erase-emmc.bin` from this [link](https://docs.radxa.com/en/zero/zero2pro/getting-started/download) 
3. 
4. Run `sudo boot-g12.py radxa-zero2-2pro-erase-emmc.bin` to erase existing image and load radxa v2 pro as a storage device
5. Follow instructions for balena-etcher from this [link](https://docs.radxa.com/en/zero/zero2pro/getting-started/install-os?Platform=Linux) to load the [latest radxa 2 ubuntu image release e.g. radxa-zero-2pro_ubuntu_jammy_cli_b32.img.xz ](https://github.com/radxa-build/radxa-zero-2pro).

### Login
username and password is `radxa`.

### GPIO 
```bash
# rsetup is used to activate GPIO pins
rsetup
# Restart to update GPIO settings
sudo nano /etc/kernel/cmdline && sudo u-boot-update 
```
[Radxa Zero2 GPIO](https://wiki.radxa.com/Zero2/Hardware/GPIO)

### Set performance mode for Radxa 2 pro
```bash
sudo apt-get install cpufrequtils
sudo cpufreq-set -g performance
```

### Network setup
```bash
# Get current address 
ip addr
```

## References
1. [Pre-built radxa 2 pro linux images](https://github.com/radxa-build/radxa-zero-2pro)
2. [Zero 2 Pro vs Zero](https://wiki.radxa.com/Zero2/Hardware/compare)
3. [Zero 2 Pro documentation](https://docs.radxa.com/en/zero/zero2pro)