# radxa 2 pro

## Ubuntu Compatibility 
So far, only Ubuntu 22.04 images from release [b32](https://github.com/radxa-build/radxa-zero-2pro/releases/tag/b32) are tested and working.
Ubuntu 20.04 images from release [20230301-0102](https://github.com/radxa-build/radxa-zero-2pro/releases/tag/20230301-0102) do not work.

## Setup

### Login
username and password is `radxa`.

### GPIO 
```bash
rsetup
sudo nano /etc/kernel/cmdline && sudo u-boot-update 
```
[Radxa Zero2 GPIO](https://wiki.radxa.com/Zero2/Hardware/GPIO)

### Network setup
```bash
# Get current address 
ip addr
```


## References
1. [Pre-built radxa 2 pro linux images](https://github.com/radxa-build/radxa-zero-2pro)
2. [Zero 2 Pro vs Zero](https://wiki.radxa.com/Zero2/Hardware/compare)
3. [Zero 2 Pro documentation](https://docs.radxa.com/en/zero/zero2pro)