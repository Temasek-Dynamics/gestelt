# radxa_utils
This package contains useful scripts and launch files to setup and test the radxa communications.

# Setting up the Radxa
Refer to [radxa_setup.md](./radxa_setup.md)

# Quick Start

## Central Computer and radxa(s)
```bash
# On the central flight control computer
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_central_sitl.sh 

# On UAV (ID 0)
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh 0

# On UAV (ID 1)
cd ~/gestelt_ws/src/gestelt/gestelt_bringup/scripts
./radxa_uav.sh 1
```
