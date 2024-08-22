import casadi as ca
import numpy as np

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


def projFrenSerretBasis(s: Union[ca.MX, float], x_ref_MX, y_ref_MX, z_ref_MX):
    ''' project to the Frenet Serret space
    <-- kap : Curvature
    <-- tau : torsion
    <-- dGamma_ds, d2Gamma_ds2, d3Gamma_ds3 : First 3 derivates of the curve w.r.t arc length
    <-- et_MX, en_MX, eb_MX : tangent , normal , binormal unit vectors'''

    # InterOpts = {'degree': [5]}
    # y_ref_MX = ca.interpolant("y_ref", "bspline", [s_ref], y_ref, InterOpts)
    # z_ref_MX = ca.interpolant("z_ref", "bspline", [s_ref], z_ref, InterOpts)
    # x_ref_MX = ca.interpolant("x_ref", "bspline", [s_ref], x_ref, InterOpts)
    
    # gamma: continuous progress function
    # Second and first derivative of gamma w.r.t s
    [d2GammaX_ds2, dGammaX_ds] = ca.hessian(x_ref_MX(s), s)
    [d2GammaY_ds2, dGammaY_ds] = ca.hessian(y_ref_MX(s), s)
    [d2GammaZ_ds2, dGammaZ_ds] = ca.hessian(z_ref_MX(s), s)

    # fourth and third derivative of gamma w.r.t s
    [d4GammaX_ds4, d3GammaX_ds3] = ca.hessian(d2GammaX_ds2, s)
    [d4GammaY_ds4, d3GammaY_ds3] = ca.hessian(d2GammaY_ds2, s)
    [d4GammaZ_ds4, d3GammaZ_ds3] = ca.hessian(d2GammaZ_ds2, s)
    
    # Derivatives of gamma
    dGamma_ds = ca.vertcat(dGammaX_ds, dGammaY_ds, dGammaZ_ds)
    d2Gamma_ds2 = ca.vertcat(d2GammaX_ds2, d2GammaY_ds2, d2GammaZ_ds2)
    d3Gamma_ds3 = ca.vertcat(d3GammaX_ds3, d3GammaY_ds3, d3GammaZ_ds3)
    d4Gamma_ds4 = ca.vertcat(d4GammaX_ds4, d4GammaY_ds4, d4GammaZ_ds4)

    # kap: curvature (matrix)
    kap = ca.norm_2(d2Gamma_ds2)
    kapBar_MX = ca.jacobian(kap, s)

    et_MX = dGamma_ds
    en_MX = d2Gamma_ds2/ kap
    eb_MX = ca.cross(et_MX, en_MX)

    etBar_MX = d2Gamma_ds2
    enBar_MX = (1/ kap**3) * (kap**2 * d3Gamma_ds3 - d2Gamma_ds2 @ d3Gamma_ds3.T @ d2Gamma_ds2)
    ebBar_MX = (1/ kap) * ca.cross(dGamma_ds, d3Gamma_ds3)
    tau_MX =  ca.dot(enBar_MX, ca.cross(et_MX, eb_MX))
    tauBar_MX =  ca.jacobian(tau_MX, s)

    return kap, tau_MX, et_MX, en_MX, eb_MX, kapBar_MX, tauBar_MX, etBar_MX, enBar_MX, ebBar_MX
