"""
Helper functions
"""

import casadi as ca
import numpy as np

from typing import Union

class TrackManager:
    def __init__(self):
        pass

    def setReferences(self, s_ref, x_ref, y_ref, z_ref):
        """Create interpolating splines for points on reference trajectory 

        Args:
            s_ref (_type_): _description_
            x_ref (_type_): _description_
            y_ref (_type_): _description_
            z_ref (_type_): _description_
        """
        self.s_ref = np.asarray(s_ref) 
        self.x_ref = np.asarray(x_ref) 
        self.y_ref = np.asarray(y_ref) 
        self.z_ref = np.asarray(z_ref) 

        InterOpts = {'degree': [5]}

        self.x_ref_MX = ca.interpolant("x_ref", "bspline", [s_ref], x_ref, InterOpts)
        self.y_ref_MX = ca.interpolant("y_ref", "bspline", [s_ref], y_ref, InterOpts)
        self.z_ref_MX = ca.interpolant("z_ref", "bspline", [s_ref], z_ref, InterOpts)

    def projFrenSerretBasis(self, s: Union[ca.MX, float] ):
        ''' project to the Frenet Serret space
        <-- kap : Curvature
        <-- tau : torsion
        <-- dGamma_ds, d2Gamma_ds2, d3Gamma_ds3 : First 3 derivates of the curve w.r.t arc length
        <-- et_MX, en_MX, eb_MX : tangent , normal , binormal unit vectors'''
        
        # gamma: continuous progress function
        # Second and first derivative of gamma w.r.t s
        [d2GammaX_ds2, dGammaX_ds] = ca.hessian(self.x_ref_MX(s), s)
        [d2GammaY_ds2, dGammaY_ds] = ca.hessian(self.y_ref_MX(s), s)
        [d2GammaZ_ds2, dGammaZ_ds] = ca.hessian(self.z_ref_MX(s), s)

        # fourth and third derivative of gamma w.r.t s
        [d4GammaX_ds4, d3GammaX_ds3] = ca.hessian(d2GammaX_ds2, s)
        [d4GammaY_ds4, d3GammaY_ds3] = ca.hessian(d2GammaY_ds2, s)
        [d4GammaZ_ds4, d3GammaZ_ds3] = ca.hessian(d2GammaZ_ds2, s)
        
        # Derivatives of gamma
        dGamma_ds = ca.vertcat(dGammaX_ds, dGammaY_ds, dGammaZ_ds)
        d2Gamma_ds2 = ca.vertcat(d2GammaX_ds2, d2GammaY_ds2, d2GammaZ_ds2)
        d3Gamma_ds3 = ca.vertcat(d3GammaX_ds3, d3GammaY_ds3, d3GammaZ_ds3)
        d4Gamma_ds4 = ca.vertcat(d4GammaX_ds4, d4GammaY_ds4, d4GammaZ_ds4)

        # kap: curvature (matrix), magnitude of rate of change of tangent
        kap = ca.norm_2(d2Gamma_ds2)    # k(s)
        kapBar_MX = ca.jacobian(kap, s) # k(s)'

        # tangent, normal, binormal (t, n, b)
        et_MX = dGamma_ds               # t(s)
        en_MX = d2Gamma_ds2/ kap        # n(s)
        eb_MX = ca.cross(et_MX, en_MX)  # b(s) = t(s) x n(s)

        # Rate of change of (t, n, b)
        etBar_MX = d2Gamma_ds2          # rate of change of tangent
        enBar_MX = (1/ kap**3) * (kap**2 * d3Gamma_ds3 - d2Gamma_ds2 @ d3Gamma_ds3.T @ d2Gamma_ds2)  # rate of change of normal
        ebBar_MX = (1/ kap) * ca.cross(dGamma_ds, d3Gamma_ds3)  # rate of change of binormal

        tau_MX =  ca.dot(en_MX, ebBar_MX)   # tau(s) = n(s) * b(s)'
        # tau_MX =  ca.dot(enBar_MX, ca.cross(et_MX, eb_MX))
        tauBar_MX =  ca.jacobian(tau_MX, s) # tau(s)'

        return kap, tau_MX, et_MX, en_MX, eb_MX, kapBar_MX, tauBar_MX, etBar_MX, enBar_MX, ebBar_MX

    def Fren2CartT(self, zetaMX, s_list, n_list, b_list):
        ''' (Used for visualization) Frenet to Cartesian transform
        --> s : lateral deviation from reference curve
        --> n : lateral deviation from reference curve
        --> b : vertiacal deviation from reference curve
        --> et : unit tangent vector (3x1)
        --> en : unit normal vector (3x1)
        --> eb : unit binormal vector (3x1)
        <-- p_x, p_y, p_z : 3D position projection w.r.t reference curve '''

        len = s_list.shape[0]

        gamma_x, gamma_y, gamma_z  = self.InterpolLuT(s_list)

        kap_MX, tau_MX, et_MX, en_MX, eb_MX, _, _, _, _, _ = self.projFrenSerretBasis(zetaMX[0])
        _, _, _, en_fun, eb_fun = self.evalFrenSerretBasis(zetaMX[0], kap_MX, tau_MX, et_MX, en_MX, eb_MX)
        en_list = []
        eb_list = []
        for i in range(0, len):
            en_list.append(en_fun(s_list[i]))
            eb_list.append(eb_fun(s_list[i]))
        en_arr = np.reshape(en_list, (3, len))
        eb_arr = np.reshape(eb_list, (3, len))
        p_x = gamma_x + en_arr[0, :] * n_list + eb_arr[0, :] * b_list
        p_y = gamma_y + en_arr[1, :] * n_list + eb_arr[1, :] * b_list
        p_z = gamma_z + en_arr[2, :] * n_list + eb_arr[2, :] * b_list

        return p_x, p_y, p_z

    def InterpolLuT(self, s: Union[ca.MX, float]):
        ''' 3rd bspline interpolation of curve x, y, zeta based on longitudinal progress (s)
        <-- xref, yref, zref : position reference curve interpol function'''

        # x_ref_curve = ca.interpolant("x_ref", "bspline", [s_ref], x_ref)
        # y_ref_curve = ca.interpolant("y_ref", "bspline", [s_ref], y_ref)
        # z_ref_curve = ca.interpolant("z_ref", "bspline", [s_ref], z_ref)

        return self.x_ref_MX(s), self.y_ref_MX(s), self.z_ref_MX(s)
    
    def evalFrenSerretBasis(self, s: ca.MX, kap_MX, tau_MX, et_MX, en_MX, eb_MX):
        '''evaluation functions for curve in Frenet Serret space
        <-- kap_fun, tau_fun : Curvature, torsion casADi functions
        <-- et_fun, en_fun, eb_fun : tangent , normal , binormal casADi functions '''

        tau_fun = ca.Function('tau', [s], [tau_MX])
        kap_fun = ca.Function('kap', [s], [kap_MX])
        et_fun = ca.Function('et', [s], [et_MX])
        en_fun = ca.Function('en', [s], [en_MX])
        eb_fun = ca.Function('en', [s], [eb_MX])

        return kap_fun, tau_fun, et_fun, en_fun, eb_fun
    
    def isPathEnded(self, s):
        return (s >= self.s_ref[-1])

    def getTrack(self):
        """Get track (s,x,y,z) references

        Returns:
            np.array: 1d array of (s,x,y,z) track references
        """
        return self.s_ref, self.x_ref, self.y_ref, self.z_ref

# def quat2rpy(quat):
#     ''' quat -> [qw, qx, qy, qz]
#         returns euler angles in degrees
#         reference math3d.h crazyflie-firmware'''

#     r	  =  ca.atan2( 2 * (quat[0]*quat[1] + quat[2]*quat[3]), 1 - 2 * (quat[1]**2 + quat[2]**2 ))
#     p     =  ca.asin( 2 *  (quat[0]*quat[2] - quat[1]*quat[3]))
#     y	  =  ca.atan2( 2 * (quat[0]*quat[3] + quat[1]*quat[2]), 1 - 2 * (quat[2]**2 + quat[3]**2 ))

#     r_d = r * 180 / np.pi          # roll in degrees
#     p_d = p * 180 / np.pi          # pitch in degrees
#     y_d = y * 180 / np.pi          # yaw in degrees

#     return [r_d, p_d, y_d]