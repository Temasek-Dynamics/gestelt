# The flight required distributed ROS nodes

# Real flight
1. config the IP, see [here]( #deployment-with-docker])
1. `prepare_real_flight_mpc_script.sh`
2. on the laptop: `real_flight_learning_gile_record.sh`
3. on the quadrotor in the ssh: `real_flight_learning_agile_drone.sh`

# Workstation SITL simulation
1. `sitl_learning_agile.sh`


# Distributed HITL simulation
1. connect the drone and the workstation by Ethernet.
2. please follow [this page](https://blog.csdn.net/xuzhengzhe/article/details/136271161) to config the IPv4 address, Netmask, Gateway for both two ubuntu devices.
4. config the `$SELF_IP`, `$MASTER_IP` mentioned in the next chapter (e.g. `10.40.0.96`)
4. on the workstation: `hitl_learning_agile_laptop_distributed.sh`
5. on the quadrotor: `hitl_bringup_learning_agile_drone_distributed.sh`


# Deployment with Docker
1. Drone Dependent parameters
    a. Drone`$SELF_IP`,  and Laptop IP `$MASTER_IP` address,
    b. Drone name in VICON, 
    is defined **outside** the docker container, in the raw ubuntu `~/.bashrc` as variables, will be transmitted to the container when run the container. 