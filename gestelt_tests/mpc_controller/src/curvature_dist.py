#!/usr/bin/env python3

from time import time

from dataclasses import dataclass, field
from typing import List

import pickle

import numpy as np
import casadi as ca

from helper import TrackManager
from sys_dynamics import SysDyn
from acados_settings import AcadosCustomOcp
from params import *
from visualize_mpl import animOptVars


from matplotlib import pyplot, animation

# import rospy

# from std_msgs.msg import Header, ColorRGBA
# from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
# from nav_msgs.msg import Odometry
# from visualization_msgs.msg import Marker
# from gestelt_msgs.msg import FrontEndPlan

@dataclass
class SpaceTimePath:
    """Each member is a vector and they collective represent a spatio-temporal path
    """
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    z: List[float] = field(default_factory=list)
    t: List[int] = field(default_factory=list)

    def reset(self):
        self.x.clear()
        self.y.clear()
        self.z.clear()
        self.t.clear()

    def printPath(self):
        for i in range(0, len(self.x)):
            print(f" ({self.t[i]}, {self.x[i]}, {self.y[i]}, {self.z[i]} )")

class MPCController:
    
    def __init__(self):
        # Initialize publishers, subscribers
        # self.spline_pub = rospy.Publisher(f'/drone0/fe_spline', Marker, queue_size=10, latch=True)
        # self.fe_sub = rospy.Subscriber(f'/drone0/fe_plan', FrontEndPlan, self.fePlanCB, queue_size=10)
        # self.odom_sub = rospy.Subscriber(f'/drone0/mavros/local_position/odom', Odometry, self.odomCB, queue_size=10)

        # Params
        self.t_unit = 0.1

        # data structs
        self.path_4d = SpaceTimePath()  
        self.spline_samp_4d = SpaceTimePath()  
        self.odom = None

        # Set up trackManager to manage tracked trajectory
        self.track_mngr = TrackManager()

        # Flags
        self.planObtained = False
        self.planOnce = True

    def loadTrackFromPickle(self, path_4d_f_name, zeta_0_f_name, track_mngr_f_name = ""):
        path_4d_tmp = self.loadObject(path_4d_f_name)
        self.zeta_0 =   np.array([ 0.02, 0.0, 0.0,       # s,  n,  b          -> frenet-serret coords
                                    1.0, 0.0, 0.0, 0.0,       # qw, qx, qy, qz,    -> Quaternions
                                    .02, 0.0, 0.0,        # sdot, ndot, bdot   -> rate of change of frenet-serret coords
                                    0.0, 0.0, 0.0,  # ohmr,  ohmp,  ohmy -> angular rate
                                    0.0, 0.0, 0.0,     # vx, vy, vz         -> linear velocity
                                    U_HOV, U_HOV, U_HOV, U_HOV ])     # ohm1, ohm2, ohm3, ohm4
        # print(f"Loaded zeta_0: {self.zeta_0}")
        print(f"Loaded path_4d with shape: {len(path_4d_tmp.x)}")

        stride = 2
        # Get front-end path
        for i in range(0, len(path_4d_tmp.x), stride):
            self.path_4d.x.append(path_4d_tmp.x[i])
            self.path_4d.y.append(path_4d_tmp.y[i])
            self.path_4d.z.append(path_4d_tmp.z[i])
            self.path_4d.t.append(path_4d_tmp.t[i] * self.t_unit)

        print(f"Downsample path_4d to shape: {len(self.path_4d.x)}")
        # self.path_4d.printPath()

        self.track_mngr.setReferences(self.path_4d.t, self.path_4d.x, self.path_4d.y, self.path_4d.z )
    
    def plotPathParams(self):
        """Plot path parameters such as curvature and torsion
        """
        [s_ref, xref_track, yref_track, zref_track] = self.track_mngr.getTrack()

        sysModel = SysDyn()
        zetaMx, _, _, _, _ = sysModel.setupODE(self.track_mngr)
        kap, tau, et, en, eb, kapBar, tauBar, _, _, _ = self.track_mngr.projFrenSerretBasis(zetaMx[0])
        kap_fun, tau_fun, _, _, _ = self.track_mngr.evalFrenSerretBasis(zetaMx[0], kap, tau, et, en, eb)

        list_kap = []   # List of curvatures at each sampled s
        list_tau = []   # List of torsion at each sampled s

        for i in range((s_ref.shape[0])): # For each sampled arc length s
            #et_fun1 = et_fun(s_ref[i]/2)
            #print("\t", gam4(s_ref[i]))
            list_kap.append(float(kap_fun(s_ref[i])))
            list_tau.append(float(tau_fun(s_ref[i])))

        fig = pyplot.figure(figsize=(15, 10))

        """Plot 
        """
        # add_subplot(nrows, ncols, index)
        param = fig.add_subplot(1, 1, 1)
        tauAx = param.stairs(list_tau[:-1], s_ref, baseline=None,label="$\\tau$", color="red" )
        kapAx = param.stairs(list_kap[:-1], s_ref, baseline=None,label="$\\kappa$", color="blue")
        param.set_xlim(0, np.amax(s_ref) + 0.2)
        param.set_ylim( ymin = np.amin(np.ravel(list_tau[:] + list_kap[:]))*1.10 - 0.2,
                        ymax = np.amax(np.ravel(list_tau[:] + list_kap[:]))*1.10 + 0.2)
        param.set_xlabel('$s\,$(m)')
        param.legend(loc='upper right')
        param.grid()

        pyplot.show()

    def loadObject(self, f_name):
        try:
            with open(f_name, "rb") as f:
                return pickle.load(f)
        except Exception as ex:
            print("Error during unpickling object (Possibly unsupported):", ex)

def main():
    mpc_controller = MPCController()
    mpc_controller.loadTrackFromPickle("pickles/path_4d.pickle", "pickles/zeta_0.pickle")
    mpc_controller.plotPathParams()

    print("mpc_controller: Finished execution")

if __name__ == '__main__':
    main()          
