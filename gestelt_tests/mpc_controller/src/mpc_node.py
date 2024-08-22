#!/usr/bin/env python3

import rospy

from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Pose, Point, Quaternion
from gestelt_msgs.msg import FrontEndPlan

from visualization_msgs.msg import Marker

import numpy as np
import casadi as ca

# import matplotlib.pyplot as plt
# pytransform3d
# from pytransform3d.transformations import plot_transform
# from pytransform3d.plot_utils import make_3d_axis, plot_spheres, plot_vector
# from pytransform3d.camera import make_world_grid

from dataclasses import dataclass, field
from typing import List

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



class TrajGen:
    
    def __init__(self):
        # Initialize publishers, subscribers
        self.spline_pub = rospy.Publisher(f'/drone0/fe_spline', Marker, queue_size=10, latch=True)

        self.fe_sub = rospy.Subscriber(f'/drone0/fe_plan', FrontEndPlan, self.fePlanCB, queue_size=10)

        self.path_4d = SpaceTimePath()  
        self.spline_samp_4d = SpaceTimePath()  





    def fePlanCB(self, msg):
        if (len(msg.plan) < 3):
            return 

        self.path_4d.reset() 

        # Get front-end path
        for i in range(0, len(msg.plan)-1, 2):
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

        # Publish spline
        self.publishSpline(self.spline_samp_4d)

        self.solveMPC()

    def solveMPC(self):
        path_length= self.spline_samp_4d.t[-1] 
        

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

    #     #####
    #     # Plotting
    #     #####
    #     self.fig1 = plt.figure(0, figsize=(25, 25))
    #     self.ax = make_3d_axis(ax_s=1, unit="m", n_ticks=5)
    #     plot_transform(ax=self.ax)

    #     # 1a) Plot front_end path 
    #     # self.ax.scatter(front_end_path.x, front_end_path.y, front_end_path.z, s=1.0, c='grey', marker='s')
        
    #     # 1b) Plot guide points used to construct sampling vector
    #     # self.ax.scatter(guide_pts.x, guide_pts.y, guide_pts.z, s=5.0, c='r', marker='o')
        
    #     # # 1c) Plot SFC Waypoints 
    #     # self.ax.scatter(sfc_waypoints.x, sfc_waypoints.y, sfc_waypoints.z, s=5.0, c='b', marker='.')
    #     # for i in range(0, len(sfc_waypoints.x)-1): # Plot line connecting all SFC waypoints
    #     #     self.ax.plot([sfc_waypoints.x[i], sfc_waypoints.x[i+1]], 
    #     #                 [sfc_waypoints.y[i],sfc_waypoints.y[i+1]],
    #     #                 [sfc_waypoints.z[i],sfc_waypoints.z[i+1]], c='b')

    #     # # 1d) Plot q transformed coordinates of SFC Waypoints 
    #     # self.ax.scatter(sfc_waypoints_q.x, sfc_waypoints_q.y, sfc_waypoints_q.z, s=10.0, c='m', marker='o')
    #     # for i in range(0, len(sfc_waypoints_q.x)-1): # Plot line connecting all SFC waypoints
    #     #     self.ax.plot([sfc_waypoints_q.x[i], sfc_waypoints_q.x[i+1]], 
    #     #                 [sfc_waypoints_q.y[i],sfc_waypoints_q.y[i+1]],
    #     #                 [sfc_waypoints_q.z[i],sfc_waypoints_q.z[i+1]], c='m')

    #     # 1e) Plot original constraint points 
    #     self.ax.scatter(cstr_pts.x, cstr_pts.y, cstr_pts.z, s=20.0, c='b', marker='.', label='constraint_pts')
    #     for i in range(0, len(cstr_pts.x)-1): # Plot line connecting all SFC waypoints
    #         self.ax.plot([cstr_pts.x[i], cstr_pts.x[i+1]], 
    #                     [cstr_pts.y[i],cstr_pts.y[i+1]],
    #                     [cstr_pts.z[i],cstr_pts.z[i+1]], c='b')

    #     # 1e) Plot constraint points in q 
    #     # self.ax.scatter(cstr_pts_q.x, cstr_pts_q.y, cstr_pts_q.z, s=20.0, c='m', marker='o', label='cstr_pts_q')
    #     # for i in range(0, len(cstr_pts_q.x)-1): # Plot line connecting all SFC waypoints
    #     #     self.ax.plot([cstr_pts_q.x[i], cstr_pts_q.x[i+1]], 
    #     #                 [cstr_pts_q.y[i],cstr_pts_q.y[i+1]],
    #     #                 [cstr_pts_q.z[i],cstr_pts_q.z[i+1]], c='m')

    #     # 1e) Plot constraint points in xi
    #     self.ax.scatter(cstr_pts_xi.x, cstr_pts_xi.y, cstr_pts_xi.z, s=20.0, c='g', marker='x', label='cstr_pts_xi')
    #     for i in range(0, len(cstr_pts_xi.x)-1): # Plot line connecting all SFC waypoints
    #         self.ax.plot([cstr_pts_xi.x[i], cstr_pts_xi.x[i+1]], 
    #                     [cstr_pts_xi.y[i],cstr_pts_xi.y[i+1]],
    #                     [cstr_pts_xi.z[i],cstr_pts_xi.z[i+1]], c='g')

    #     # 2a) Plot all sampled sphere
    #     # self.plotSpheres(all_seg_samp_spheres[2])
    #     # 2b) Plot SFC final spheres
    #     self.plotSpheres(sfc_spheres)
        
    #     # Plot histogram of sampled spheres
    #     # self.plotHistogram(all_seg_samp_spheres[2])

    #     self.ax.set_xlim(-0.1, 6)
    #     self.ax.set_ylim(-0.1, 6)
    #     self.ax.set_zlim(-0.1, 6)

    #     # plt.tight_layout()
    #     plt.legend()
    #     plt.show()  


    # def plotHistogram(self, spheres):
    #     self.fig2, self.ax_hist = plt.subplots(nrows=2, ncols=2, figsize=(15, 15))
    #     self.fig2.suptitle("Segment i")

    #     self.ax_hist[0,0].set_title("Radii (m)")
    #     self.ax_hist[0,0].hist(spheres.radii)

    # def plotSpheres(self, spheres):
    #     # Plot sampled spheres
    #     plot_spheres(p= spheres.centers, 
    #                 radius= spheres.radii, 
    #                 alpha= spheres.alphas,
    #                 color= spheres.colors,
    #                 )

    # # Markers: https://matplotlib.org/stable/api/markers_api.html#module-matplotlib.markers
    # # Colors: https://matplotlib.org/stable/users/explain/colors/colors.html#colors-def

def main():
    rospy.init_node('traj_gen', disable_signals=True)

    traj_gen = TrajGen()

    rospy.spin()

    print("traj_gen: Finished execution")

if __name__ == '__main__':
    main()          
