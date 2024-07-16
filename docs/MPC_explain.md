# MPC Explanation


## 1. The MPC problem definition is modified from the work [LearningAgileFlight_SE3](https://github.com/BinghengNUS/LearningAgileFlight_SE3), in the folder :

```plaintext
gestelt_bringup/
├── src/
│   ├── learning_agile_agent.py
│   ├── c_generated_code
│   ├── acados_template
│   ├── ...
```
where:
1. `learning_agile_agent.py` is the main file for constructing the MPC solver and running the python simulation.
2. The `acados_template` is the required by the `acados` framework, also you need to install `acados` and setup the environment, please refer to [acados installation](https://docs.acados.org/installation/index.html)
3. The generated C code of the solver works as a shared library, saved in the folder `c_generated_code`. This shared library will be linked with the MPC ROS wrapper (The package `/learning_agile`). This C++ ROS node will request the solver during both the **Gazebo SITL simulation** and the **real flight**.
4. **Each time after modifying the MPC problem definition, like the weight and the model parameters, you need to run the `learning_agile_agent.py` to regenerate the solver.** (I will refine the procedure)
## 2. The MPC ROS node is in the package:
```plaintext
learning_agile/
├── src/
│   ├── learning_agile_node.cpp (useless now)
│   ├── learning_agile.cpp

```
1. The MPC ROS wrapper is in the file `learning_agile.cpp`, it is not a single ROS node, instead, this class will be called by the node `traj_server` by using the shared pointer.

## 3. The FSM for safety protection
```plaintext
trajectory_server/
├── src/
│   ├── traj_server_node.cpp 
│   ├── traj_server.cpp

```
1. This node manages the drone flight state, from `ARM`->`Take-Off`->`Hover`->`MPC`
2. if the `MPC` has no solution, or the period is too long, the `traj_server` will switch the drone back to `Hover` and the `MPC` will never be called.
## 4. Running the Gazebo SITL simulation and the real flight.
## 4.1. The simulation
1. Please firstly refer to the main `README.md` to install dependence. 
2. All scripts are in the folder:
    ```plaintext
    gestelt_bringup/
    ├── script/
    │   ├── ...

    ```
3. I provide two kind of simulations.   
    1. **All in PC.** 
        This simulation will run both `PX4_SITL` and `MPC`in the PC.
        run `./sitl_bringup_learning_agile.sh`
    2. **HITL simulation**
        One critical issue is to verify whether the drone onboard computer could run the MPC in the desired frequency or not, so here I provided the **HITL** simulation:
        ![Alt text](pictures/MPC_HITL.png)
        1. Your PC will run the `PX4_SITL` and `Gazebo` by running:
        `./sitl_bringup_learning_agile_laptop_distributed.sh` 
        2. Connect the drone with the PC by wifi, run:
        `./sitl_bringup_learning_agile_drone_distributed.sh`
## 4.2 Real flight:
1. On your PC, run: `./realflight_bringup_learning_agile_record_remote.sh`  
2. ssh on the drone, run:
`./realflight_bringup_learning_agile_drone.sh`

## 4.3 Log
Both simulation and real flight logs are in folder  
 ```plaintext
    gestelt_bringup/
    ├── data/
    │   ├── ...

```