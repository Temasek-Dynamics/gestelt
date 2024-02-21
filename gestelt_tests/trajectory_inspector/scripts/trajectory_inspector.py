#!/usr/bin/env python3

import rospy

import numpy as np
import matplotlib.pyplot as plt

from gestelt_debug_msgs.msg import SFCSegment, SFCTrajectory
from gestelt_msgs.msg import Sphere

from pytransform3d.transformations import plot_transform
from pytransform3d.plot_utils import make_3d_axis, plot_spheres

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

class TrajectoryInspector:
    
    def __init__(self):
        msg = rospy.wait_for_message("drone0/sfc/debug_trajectory", SFCTrajectory, timeout=10.0)
        rospy.loginfo("Got SFCTraj")

        guide_pts = PointVec()
        front_end_path = PointVec()
        sampling_vector = PointVec()
        all_seg_spheres = [] # Each element (the i-th segment) contains a list of sampled spheres 
        sfc_spheres = Spheres(len(msg.sfc_spheres)) # Actual sfc sphere

        # Guide path
        for pt in msg.front_end_path:
            front_end_path.x.append(pt.x)
            front_end_path.y.append(pt.y)
            front_end_path.z.append(pt.z)

        # Sampling guide points
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

            all_seg_spheres.append(seg_spheres)

            # Add Sampling vector
            sampling_vector.x.append(segment.sampling_vector.x)
            sampling_vector.y.append(segment.sampling_vector.y)
            sampling_vector.z.append(segment.sampling_vector.z)

        # Construct actual SFC sphere 
        for i in range(0, len(msg.sfc_spheres)):
            sph = msg.sfc_spheres[i]
            sfc_spheres.centers[i, 0:3] = ([sph.center.x, sph.center.y, sph.center.z])
            sfc_spheres.radii[i] = sph.radius 
            sfc_spheres.alphas[i] = 0.3
            sfc_spheres.colors[i, 0:3] = ([1.0, 0.0, 0.0])
        
        self.fig1 = plt.figure(0)
        self.ax = make_3d_axis(ax_s=1, unit="m", n_ticks=5)
        plot_transform(ax=self.ax)

        # Plot front_end path and guide points used to sample.
        self.ax.scatter(guide_pts.x, guide_pts.y, guide_pts.z, s=5.0, c='r', marker='o')
        self.ax.scatter(front_end_path.x, front_end_path.y, front_end_path.z, s=1.0, c='grey', marker='s')

        self.plotSpheres(all_seg_spheres[2])
        
        self.plotSpheres(sfc_spheres)

        self.plotHistogram(all_seg_spheres[2])

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