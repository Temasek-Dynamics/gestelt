# demo setup

## Configuration
1. Ensure DRONE_ID in ~/.bashrc of radxa host is correct
2. Check MAV_SYS_ID in px4 parameters, it's value should be DRONE_ID+1

## Code
1. Check that latest code is pulled from docker
    ```bash
    docker pull johntgz95/radxa-gestelt:latest
    ```

## Updating the configuration
1. `docker commit CONTAINER_ID johntgz95/radxa-gestelt:latest`
2. ` docker push johntgz95/radxa-gestelt:latest`

## Launching demos
```bash
# On the base station 
./base_station -s vicon_empty3
# ./base_station -s vicon_obs3

# On the drones
uav_startup

# Taking off, landing
rosrun gestelt_bringup takeoff.py
rosrun gestelt_bringup land.py
```

## Test checklist (18/7/24)
1. Test take off and landing
2. Test 0.5m/s vicon_empty3
3. Test 0.5m/s vicon_obs3
4. Test 2.0m/s vicon_empty3
5. Test 2.0m/s vicon_obs3
5. Test 3.0m/s vicon_empty3
