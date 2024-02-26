#!/usr/bin/env python3

import rospy

import numpy as np
import matplotlib.pyplot as plt

from gestelt_debug_msgs.msg import SFCSegment, SFCTrajectory
from gestelt_msgs.msg import Sphere

# pytransform3d
from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis, plot_spheres, plot_vector
from pytransform3d.camera import make_world_grid

from dataclasses import dataclass, field
from typing import List

@dataclass
class PointVec:
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    z: List[float] = field(default_factory=list)

class Spheres:
    def __init__(self, size):
        self.centers = np.empty([size, 3], dtype=float)
        self.radii = np.empty([size], dtype=float)

        self.colors = np.empty([size, 3], dtype=float)
        self.alphas = np.empty([size], dtype=float)

class Projection:
    def __init__(self):
        pass

    def f_B(self, xi, r, o):
        """Projection from n-dim state x to n-dim closed ball.
            Or from xi to q

        Args:
            r (_type_): radius of sphere
            o (_type_): center of sphere
        """
        return o + (2 * r * xi) / (np.dot(xi.transpose(),xi) + 1.0)
        
    def f_BInv_ctrl_pts(self, q_i, o_i, r_i):
        """Local inverse of f_B(xi)

        Args:
            q_i (np.array): waypoint i
            o_i (np.array): center of sphere i
            r_i (float): radius of sphere i
        """
        q_i_ = q_i.flatten()
        o_i_ = o_i.flatten()
        v_i = q_i_ - o_i_
        
        a = r_i + np.sqrt(r_i**2 - np.dot(v_i, v_i) )  # Works for points both inside nad outside the circle
        # a = r_i - np.sqrt(r_i**2 - np.dot(v_i, v_i) )  # Works only for points inside the circle
        
        b = np.dot(v_i, v_i)

        xi_i = ( a ) * (v_i / b) 
        return xi_i

class TrajectoryInspector:
    
    def __init__(self):
        msg = rospy.wait_for_message("drone0/sfc/debug_trajectory", SFCTrajectory, timeout=10.0)
        rospy.loginfo("Got SFCTraj")
 
        front_end_path = PointVec()                 # A* path
        guide_pts = PointVec()                      # Guide points for constructing sampling vector
        sampling_vector = PointVec()                # Sampling vector for direction of sampling distribution
        all_seg_samp_spheres = []                        # Each element (the i-th segment) contains a list of sampled spheres 
        sfc_spheres = Spheres(len(msg.sfc_spheres)) # Vector of final SFC spheres
        sfc_waypoints = PointVec()                  # Vector of final SFC Trajectory waypoints
        sfc_waypoints_xi = PointVec()               # Vector of waypoints in xi coordinates

        #####
        # Get data from ROS msg
        #####

        # Get Guide path
        for pt in msg.front_end_path:
            front_end_path.x.append(pt.x)
            front_end_path.y.append(pt.y)
            front_end_path.z.append(pt.z)

        # Get Sampling guide points
        for segment in msg.segments:
            # Add guide points
            guide_pts.x.append(segment.guide_point.x)
            guide_pts.y.append(segment.guide_point.y)
            guide_pts.z.append(segment.guide_point.z)
            
            # Add sampled spheres
            seg_spheres = Spheres(len(segment.sampled_spheres))
            for i in range(0, len(segment.sampled_spheres)):
                sph = segment.sampled_spheres[i]
                seg_spheres.centers[i, 0:3] = ([sph.center.x, sph.center.y, sph.center.z])
                seg_spheres.radii[i] = sph.radius 
                seg_spheres.alphas[i] = 0.2
                seg_spheres.colors[i, 0:3] = ([0.0, 1.0, 1.0])

            all_seg_samp_spheres.append(seg_spheres)

            # Add Sampling vector
            sampling_vector.x.append(segment.sampling_vector.x)
            sampling_vector.y.append(segment.sampling_vector.y)
            sampling_vector.z.append(segment.sampling_vector.z)

        # Get actual SFC sphere 
        for i in range(0, len(msg.sfc_spheres)):
            sph = msg.sfc_spheres[i]
            sfc_spheres.centers[i, 0:3] = ([sph.center.x, sph.center.y, sph.center.z])
            sfc_spheres.radii[i] = sph.radius 
            sfc_spheres.alphas[i] = 0.3
            sfc_spheres.colors[i, 0:3] = ([1.0, 0.0, 0.0])
        
        # Get final SFC trajectory
        for pt in msg.sfc_waypoints:
            sfc_waypoints.x.append(pt.x)
            sfc_waypoints.y.append(pt.y)
            sfc_waypoints.z.append(pt.z)

        #####
        # Transfrom from q to xi
        #####
        # for i in range(1, len(sfc_waypoints.x)-1): # For each waypoint except starting and final waypoint
            

        #####
        # Plotting
        #####
        self.fig1 = plt.figure(0)
        self.ax = make_3d_axis(ax_s=1, unit="m", n_ticks=5)
        plot_transform(ax=self.ax)

        # 1a) Plot front_end path 
        # self.ax.scatter(front_end_path.x, front_end_path.y, front_end_path.z, s=1.0, c='grey', marker='s')
        
        # 1b) Plot guide points used to construct sampling vector
        # self.ax.scatter(guide_pts.x, guide_pts.y, guide_pts.z, s=5.0, c='r', marker='o')
        
        # 1c) Plot SFC Waypoints 
        self.ax.scatter(sfc_waypoints.x, sfc_waypoints.y, sfc_waypoints.z, s=5.0, c='b', marker='.')
        for i in range(0, len(sfc_waypoints.x)-1): # Plot line connecting all SFC waypoints
            self.ax.plot([sfc_waypoints.x[i], sfc_waypoints.x[i+1]], 
                        [sfc_waypoints.y[i],sfc_waypoints.y[i+1]],
                        [sfc_waypoints.z[i],sfc_waypoints.z[i+1]])

        # 2a) Plot all sampled sphere
        # self.plotSpheres(all_seg_samp_spheres[2])
        # 2b) Plot SFC final spheres
        self.plotSpheres(sfc_spheres)
        
        # Plot histogram of sampled spheres
        # self.plotHistogram(all_seg_samp_spheres[2])

        self.ax.set_xlim(-0.1, 6)
        self.ax.set_ylim(-0.1, 6)
        self.ax.set_zlim(-0.1, 6)

        # plt.tight_layout()
        plt.show()  


    

    def plotHistogram(self, spheres):
        self.fig2, self.ax_hist = plt.subplots(nrows=2, ncols=2, figsize=(15, 8))
        self.fig2.suptitle("Segment i")

        self.ax_hist[0,0].set_title("Radii (m)")
        self.ax_hist[0,0].hist(spheres.radii)

    def plotSpheres(self, spheres):
        # Plot sampled spheres
        plot_spheres(p= spheres.centers, 
                    radius= spheres.radii, 
                    alpha= spheres.alphas,
                    color= spheres.colors,
                    )

    # Markers: https://matplotlib.org/stable/api/markers_api.html#module-matplotlib.markers
    # Colors: https://matplotlib.org/stable/users/explain/colors/colors.html#colors-def

def main():
    rospy.init_node('trajectory_inspector', disable_signals=True)

    traj_inspector = TrajectoryInspector()

if __name__ == '__main__':
    main()          



# INPUT FROM SFC: xi
#     inner_ctrl_pts is of size (3, M-1)
#     Eigen::MatrixXd inner_ctrl_pts_xi = f_BInv_ctrl_pts(inner_ctrl_pts, spheres_center_, spheres_radius_); // (3, num_segs - 1);

# 1) Optimization is done using xi as decision variables
#     a) Forward direction: cost evaluation done using q_i = f_B(xi)
#           This means that the following is calculated in q_i
#               a) swarm
#               b) closeness to sphere center
#     b) Backward direction: p.d.(H/xi) = (2 * r_i * p.d.(H/q))/(xi_i.T * xi_i) - (4 * r_i * ( ... ))/(xi_i.T * xi_i + 1)**2 
#           This means that 
# 2) Transform back to q?
#       Transforming back to q would mean taking the original unconstrained xi and constraining 
#       it to be within the safe flight corriddor

# OUTPUT TO PLANNER: q