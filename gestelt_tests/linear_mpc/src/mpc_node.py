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
        print(f"Loaded zeta_0: {self.zeta_0}")
        print(f"Loaded path_4d with shape: {len(path_4d_tmp.x)}")

        stride = 3
        # Get front-end path
        for i in range(0, len(path_4d_tmp.x), stride):
            self.path_4d.x.append(path_4d_tmp.x[i])
            self.path_4d.y.append(path_4d_tmp.y[i])
            self.path_4d.z.append(path_4d_tmp.z[i])
            self.path_4d.t.append(path_4d_tmp.t[i] * self.t_unit)

        print(f"Downsample path_4d to shape: {len(self.path_4d.x)}")
        self.path_4d.printPath()

        self.track_mngr.setReferences(self.path_4d.t, self.path_4d.x, self.path_4d.y, self.path_4d.z )

    def runControllerOnce(self, zeta_0):
        """Setup MPC controller
        """
        ocp_wrapper = AcadosCustomOcp(self.track_mngr)
        ocp_wrapper.setupAcadosOCP()

        # dimensions
        nx = ocp_wrapper.nx
        nu = ocp_wrapper.nu

        # Initialize iteration variables
        t0 = 0                      # computation time
        discr_step = np.array([])   # Temporary array for storing (computation time, solver cost)
        times = np.array([0])       # Array solving times
        mpc_iter = 0                # MPC Iteration counter
        cost = 0                    # Solver cost at each iteration

        # Define states, controls, and slacks as column vectors         
        ocp_wrapper.setInitialCondition(zeta_0)
        u_0 = np.copy(U_REF)

        # Vertically repeat array (nx,) to (nx, N+1)
        zeta_N_tmp = ca.repmat(np.reshape(zeta_0, (nx,1)), 1, N+1)

        #####
        # For visualization
        #####
        # (nx, mpc_iter) to plot state during entire Horizon
        state_steps = ca.repmat(zeta_0, 1, N+1)     # state per step
        # extend (nu, ) to (nu, 1)
        control_steps = ca.repmat(u_0, 1, 1)
        # (2, mpc_iter) to represent (iteration time, cost) for each mpc iteration
        misc_step = ca.repmat(np.array([0, 0]).T, 1)

        # Control loop entry point
        for i in range(Nsim):   # For each simulation iteration
            # store previous iterate data for plots
            discr_step = ca.reshape(np.array([t0, cost]), 2, 1)
            misc_step = np.concatenate( (misc_step, discr_step), axis=1)
            state_steps = np.dstack((state_steps, ocp_wrapper.zeta_N))
            control_steps = np.concatenate((control_steps,
                                            np.reshape(ocp_wrapper.u_N[:, 0], (nu, 1))),
                                            axis = 1)
            t1 = time()

            isPathEnded = ocp_wrapper.cost_update_ref(zeta_N_tmp[:, 0])
            if isPathEnded:
                print("Reached end of track!")
                break

            # Solve the OCP with updated state and controls
            ocp_wrapper.solve_and_sim()
            cost = ocp_wrapper.get_cost()
                
            t0 = round(t0 + T_del, 3)
            t2 = time()
            ocp_soln_time = t2 - t1
            times = np.vstack(( times, ocp_soln_time))
            mpc_iter = mpc_iter + 1

            zeta_N_tmp = ocp_wrapper.zeta_N

            print(f'\n Soln. {mpc_iter} Sim: {(np.round(ocp_wrapper.zeta_N[:, 0],2).T)} at {round(t0,2)} s\t')

        sqp_max_sec = round(np.array(times).max(), 3)
        sqp_avg_sec = round(np.array(times).mean(), 3)

        avg_n = round(np.abs(state_steps[1, 0, :]).mean(), 4)   # Normal 
        avg_b = round(np.abs(state_steps[2, 0, :]).mean(), 4)   # Binormal

        print(f'Max. solver time\t\t: {sqp_max_sec * 1000} ms')
        print(f'Avg. solver time\t\t: {sqp_avg_sec * 1000} ms')
        print(f'Avg. lateral deviation n\t: {avg_n} m')
        print(f'Avg. vertical deviation b\t: {avg_b} m')

        animOptVars(misc_step, state_steps, control_steps, self.track_mngr)

    def fePlanCB(self, msg):
        """Front end plan callback

        Args:
            msg (FrontEndPlan): _description_
        """
        if self.planOnce and self.planObtained:
            return

        if (len(msg.plan) < 3):
            return 

        self.path_4d.reset() 

        # Get front-end path
        for i in range(0, len(msg.plan), 1):
            self.path_4d.x.append(msg.plan[i].position.x)
            self.path_4d.y.append(msg.plan[i].position.y)
            self.path_4d.z.append(msg.plan[i].position.z)
            self.path_4d.t.append(msg.plan_time[i])

        # Generate spline
        self.x_ref_MX, self.y_ref_MX, self.z_ref_MX = self.generateSpline(self.path_4d)

        # Sample spline 
        self.spline_samp_4d.reset() 
        for s in np.arange(0, self.path_4d.t[-1], 0.1):
            self.spline_samp_4d.x.append(self.x_ref_MX(s))
            self.spline_samp_4d.y.append(self.y_ref_MX(s))
            self.spline_samp_4d.z.append(self.z_ref_MX(s))
            self.spline_samp_4d.t.append(s)

        # Set references for track manager
        self.track_mngr.setReferences(self.path_4d.t, self.path_4d.x, self.path_4d.y, self.path_4d.z )

        zeta_0 = np.array([ 0.05, 0.0, 0.0,       # s,  n,  b          -> frenet-serret coords
                            1.0, 0.0, 0.0, 0.0,       # qw, qx, qy, qz,    -> Quaternions
                            .02, 0.0, 0.0,        # sdot, ndot, bdot   -> rate of change of frenet-serret coords
                            0.0, 0.0, 0.0,  # ohmr,  ohmp,  ohmy -> angular rate
                            0.0, 0.0, 0.0,     # vx, vy, vz         -> linear velocity
                            U_HOV, U_HOV, U_HOV, U_HOV ])     # ohm1, ohm2, ohm3, ohm4

        self.saveObject(self.path_4d, "path_4d.pickle")
        self.saveObject(self.track_mngr, "track_manager.pickle")
        self.saveObject(zeta_0, "zeta_0.pickle")        

        self.planObtained = True

        # Publish spline
        self.publishSpline(self.spline_samp_4d)

        # self.runControllerOnce(zeta_0)

    def odomCB(self, msg):
        self.odom = msg

    def generateSpline(self, path):
        x_ref = path.x
        y_ref = path.y
        z_ref = path.z
        s_ref = path.t
        
        InterOpts = {'degree': [5]}
        x_ref_MX = ca.interpolant("x_ref", "bspline", [s_ref], x_ref, InterOpts)
        y_ref_MX = ca.interpolant("y_ref", "bspline", [s_ref], y_ref, InterOpts)
        z_ref_MX = ca.interpolant("z_ref", "bspline", [s_ref], z_ref, InterOpts)

        return x_ref_MX, y_ref_MX, z_ref_MX

    def publishSpline(self, path):
        r1 = 0.3
        r2 = 0.1
        r3 = 0.05
        start = Marker(
                    ns='spline',
                    type=Marker.SPHERE,
                    action=Marker.ADD,
                    id=0,
                    pose=Pose(Point(path.x[0], path.y[0], path.z[0]), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(r1, r1, r1),
                    header=Header(frame_id='world'),
                    color=ColorRGBA(1.0, 1.0, 0.0, 0.5),
                    frame_locked=False)
        goal = Marker(
                    ns='spline',
                    type=Marker.SPHERE,
                    action=Marker.ADD,
                    id=1,
                    pose=Pose(Point(path.x[-1], path.y[-1], path.z[-1]), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(r1, r1, r1),
                    header=Header(frame_id='world'),
                    color=ColorRGBA(0.0, 1.0, 0.0, 0.5),
                    frame_locked=False)

        wp_list = Marker(
                    ns='spline',
                    type=Marker.SPHERE_LIST,
                    action=Marker.ADD,
                    id=2,
                    # pose=Pose(Point(0.0, 0.0, 0,0), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(r2, r2, r2),
                    header=Header(frame_id='world'),
                    color=ColorRGBA(1.0, 0.5, 0.0, 0.5),
                    frame_locked=False)

        line_strip  = Marker(
                        ns='spline',
                        type=Marker.LINE_STRIP,
                        action=Marker.ADD,
                        id=3,
                        # pose=Pose(Point(0.0, 0.0, 0,0), Quaternion(0, 0, 0, 1)),
                        scale=Vector3(r3, r3, 0.0),
                        header=Header(frame_id='world'),
                        color=ColorRGBA(0.0, 0.0, 1.0, 0.5),
                        frame_locked=False)
        
        for i in np.arange(1, len(path.x) -1):
            wp_list.points.append(Point(path.x[i], path.y[i], path.z[i]))
            line_strip.points.append(Point(path.x[i], path.y[i], path.z[i]))
        
        self.spline_pub.publish(start)
        self.spline_pub.publish(goal)
        self.spline_pub.publish(wp_list)
        self.spline_pub.publish(line_strip)

    def saveObject(self, obj, f_name):
        try:
            with open(f_name, "wb") as f:
                pickle.dump(obj, f, protocol=pickle.HIGHEST_PROTOCOL)
            print(f"Saved obj as {f_name}")
        except Exception as ex:
            print(f"Error during pickling object (Possibly unsupported): {ex}")

    def loadObject(self, f_name):
        try:
            with open(f_name, "rb") as f:
                return pickle.load(f)
        except Exception as ex:
            print("Error during unpickling object (Possibly unsupported):", ex)

def main():
    # rospy.init_node('mpc_controller', disable_signals=True)
    # mpc_controller = MPCController()
    # rospy.spin()

    mpc_controller = MPCController()
    # mpc_controller.loadTrackFromPickle("pickles/path_4d.pickle", "pickles/zeta_0.pickle", "pickles/track_manager.pickle")
    mpc_controller.loadTrackFromPickle("pickles/path_4d.pickle", "pickles/zeta_0.pickle")
    mpc_controller.runControllerOnce(mpc_controller.zeta_0)

    print("mpc_controller: Finished execution")

if __name__ == '__main__':
    main()          
