"""Parameters used by solver and dynamic model
"""
import numpy as np

INF = 1e5

# CrazyFlie 2.1 physical parameters

'''Dynamic Model'''
g0  = 9.80665       # [m.s^2] gravitational acceleration

mq  = 31e-3         # [kg] total mass
Ix = 1.395e-5       # [kg.m^2] Inertial moment around x-axis
Iy = 1.395e-5       # [kg.m^2] Inertial moment around y-axis
Iz = 2.173e-5       # [kg.m^2] Inertia moment around z-axis
dq  = 92e-3         # [m] distance between motors' center
l   = dq/2          # [m] distance between motors' center and the axis of rotation

Cd  = 7.9379e-06    # [N/krpm^2] Drag coefficient
Ct  = 3.25e-4       # [N/krpm^2] Thrust coefficient

drone_rad = 0.04                           # radius of the drone covering sphere

'''MatPlotLib animation settings'''
f_plot = 10       # Interval of sampling the optimal state, controls. Value of 1 means plot everything
refresh_ms = 10
sphere_scale = 20000 #TODO make dependant on map size. (10000/ 20 obst)

'''Constraints'''
# U_MAX = 22                                              # [krpm]
U_HOV = int(np.sqrt(.25 * 1e6* mq * g0 /Ct)) /1000      # [krpm]
U_REF = np.array([U_HOV, U_HOV, U_HOV, U_HOV])
zeta_0_default = np.array([0.05, 0, 0,       # s,  n,  b          -> frenet-serret coords
                          1, 0, 0, 0,       # qw, qx, qy, qz,    -> Quaternions
                          .02, 0, 0,        # sdot, ndot, bdot   -> rate of change of frenet-serret coords
                          0, 0, 0,          # ohmr,  ohmp,  ohmy -> angular rate
                          0, 0, 0,          # vx, vy, vz         -> linear velocity
                          U_HOV, U_HOV, U_HOV, U_HOV ])     # ohm1, ohm2, ohm3, ohm4

'''Weights'''
Q = np.diag([1, 1e-1, 1e-1,               # frenet position
             1e-5, 1e-5, 1e-5, 1e-5,      # quaternion
             1e-5, 1e-5, 1e-5,            # frenet velocity
             1e-5, 1e-5, 1e-5,            # drone angular velocity
             1e-5, 1e-5, 1e-5,            # cartesian velocity
             1e-8, 1e-8, 1e-8, 1e-8])     # rotor angular velocity

                                          # Terminal state weights on
Qn = np.diag([10, 1e-3, 1e-2,             # frenet position
             1e-5, 1e-5, 1e-5, 1e-5,      # quaternion
             1e-5, 1e-5, 1e-5,            # frenet velocity
             1e-5, 1e-5, 1e-5,            # drone angular velocity
             1e-5, 1e-5, 1e-5,            # cartesian velocity
             1e-8, 1e-8, 1e-8, 1e-8])     # rotor angular velocity

R = np.diag([1e-4, 1e-4, 1e-4, 1e-4])     # Weights on controls

'''Solver'''
T_del = 0.05                # [s] time between steps in seconds
N = 30                      # number of shooting nodes
Tf = N * T_del              # [s] Prediction time horizon

Tsim = 25                   # [s] Simulation time
Nsim = int(Tsim / T_del)    # Number of simulations required to simulate for Tsim 

S_REF = 0.1
