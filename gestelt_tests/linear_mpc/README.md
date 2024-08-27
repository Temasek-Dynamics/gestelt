# Linear MPC

# Structure
1. mpc_node.py: ROS node subscribing to front-end path and calling the MPC solver routines.
2. helper.py: Contains `TrackManager` for conversion between frenet-serret and cartesian representations
3. acados_settings.py: Set up ocp problem using `AcadosCustomOcp` class
4. params.py: Contains all the parameters used for MPC

# Acknowledgements
[1] [OSQP MPC Example](https://osqp.org/docs/examples/mpc.html)
