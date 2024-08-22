import casadi as ca
import numpy as np

from params import *
from helper import *

'''Global symbolic variables'''

# Quaternion

# # Quarternion heading (body frame )
q1 = ca.MX.sym('q1')
q2 = ca.MX.sym('q2')
q3 = ca.MX.sym('q3')
q4 = ca.MX.sym('q4')

# Transaltional velocities (inertial frame, m/s)
vx = ca.MX.sym('vx')
vy = ca.MX.sym('vy')
vz = ca.MX.sym('vz')
v_c = ca.vertcat(vx, vy, vz)

# Angular velocities w.r.t phi(roll), theta(pitch), psi(yaw)
# (body frame, m/s)
wr = ca.MX.sym('wr')
wp = ca.MX.sym('wp')
wy = ca.MX.sym('wy')
omg = ca.vertcat(wr, wp, wy)

# Frenet states
# curve displacements in meter
s = ca.MX.sym('s')
n = ca.MX.sym('n')
b = ca.MX.sym('n')

# curve velocities in meter/ second
sDot = ca.MX.sym('sDot')    # Rate of change of progress variable
nDot = ca.MX.sym('nDot')    # Rate of change of normal
bDot = ca.MX.sym('bDot')    # Rate of change of binormal

# Control variable angles (Motor RPM)
ohm1 = ca.MX.sym('ohm1')
ohm2 = ca.MX.sym('ohm2')
ohm3 = ca.MX.sym('ohm3')
ohm4 = ca.MX.sym('ohm4')

#zeta_f: state of quadrotor
zeta_f = ca.vertcat(s, n, b, q1, q2, q3, q4, sDot, nDot, bDot, wr, wp, wy, vx, vy, vz, ohm1, ohm2, ohm3, ohm4)

# Quadrotor control
alpha1 = ca.MX.sym('alpha1')
alpha2 = ca.MX.sym('alpha2')
alpha3 = ca.MX.sym('alpha3')
alpha4 = ca.MX.sym('alpha4')
u = ca.vertcat( alpha1, alpha2, alpha3, alpha4)

class SysDyn():
    def __init__(self):
        self.n_samples = 0 #?

    def setupODE(self):
        """System dynamic model
        """

        # D: Where is drag term obtained from???
        # D = (Cd / mq) * ca.vertcat(vx**2, vy**2, vz**2) 
        # F: Forces
        F = Ct * ca.vertcat(0,  # Body force on x-axis
                            0,  # body force on y-axis
                            ohm1**2  + ohm2**2  + ohm3**2  + ohm4**2 )  # body z
        # G: gravity
        G = ca.vertcat(0, 0, g0) 
        # J: Moment of inertia
        J = np.diag([Ix, Iy, Iz]) 
        # M: Moments
        M = ca.vertcat(Ct * l * (ohm1**2 + ohm2**2 - ohm3**2 - ohm4**2),    # Body torque about x-axis
                      Ct * l * (ohm1**2 - ohm2**2 - ohm3**2 + ohm4**2),     # Body torque about y-axis
                      Cd * (ohm1**2 - ohm2**2 + ohm3**2 - ohm4**2))         # Body torque about z-axis
        
        # Rq: Rotation matrix
        Rq = ca.vertcat(ca.horzcat( 2 * (q1**2 + q2**2) - 1,    -2 * (q1*q4 - q2*q3),       2 * (q1*q3 + q2*q4)),
                        ca.horzcat( 2 * (q1*q4 + q2*q3),         2 * (q1**2 + q3**2) - 1,   2 * (q1*q2 - q3*q4)),
                        ca.horzcat( 2 * (q1*q3 - q2*q4),         2 * (q1*q2 + q3*q4),       2 * (q1**2 + q4**2) - 1))

        # Orientation ODEs ( quaternion)
        q1Dot = (-(q2 * wr) - (q3 * wp) - (q4 * wy))/2
        q2Dot = ( (q1 * wr) - (q4 * wp) + (q3 * wy))/2
        q3Dot = ( (q4 * wr) + (q1 * wp) - (q2 * wy))/2
        q4Dot = (-(q3 * wr) + (q2 * wp) + (q1 * wy))/2

        # Cartesian acceleration ODEs ( including drag)
        vDot_c = -G + (1/ mq) * Rq @ F 

        # Angular acceleration ODEs (rate of change of projected Euler angles)
        omgDot = ca.inv(J) @ (M - ca.cross(omg, J @ omg))

        ohm1Dot = alpha1
        ohm2Dot = alpha2
        ohm3Dot = alpha3
        ohm4Dot = alpha4

        # Frenet Serret Dynamics
        # project to frenet serret space 
        kap, tau, et, en, eb, kapBar, tauBar, etBar, enBar, ebBar = projFrenSerretBasis(zeta_f[0], x_ref_MX, y_ref_MX, z_ref_MX)

        # Path progress dynamics
        sDot = (et.T @ v_c) / (1- kap * n)
        nDot = en.T @ v_c + tau *  sDot @ b
        bDot = eb.T @ v_c - tau * sDot @ n

        kapDot = kapBar * sDot
        tauDot = tauBar * sDot
        etDot = etBar * sDot
        enDot = enBar * sDot
        ebDot = ebBar * sDot

        s2Dot = (et.T @ vDot_c + etDot.T @ v_c)/ (1 - kap * n) + et.T @ v_c * ((n * kapDot + nDot * kap)/ (1 - kap * n)**2 )
        n2Dot = en.T @ vDot_c + enDot.T @ v_c + tauDot * sDot * b + tau * s2Dot * b + tau * sDot * bDot
        b2Dot = eb.T @ vDot_c + ebDot.T @ v_c - tauDot * sDot * n - tau * s2Dot * n - tau * sDot * nDot

