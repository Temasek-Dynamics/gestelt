#!/usr/bin/env python3
import sys
import os
import subprocess
import yaml
import numpy as np
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append("../")
sys.path.append(subdirectory_path)

import rospy
from geometry_msgs.msg import  PoseStamped, Point, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
class DroneEllipsoidVisualizer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('drone_ellipsoid_vis', anonymous=True)

        # Publisher for the ellipsoid marker
        self.drone_ellipsoid_pub = rospy.Publisher(
            '/visual/drone_ellipsoid', Marker, queue_size=10
        )
        # Publisher for GATE
        self.gate_vis_pub = rospy.Publisher(
            "/visual/gate_vis", Marker, queue_size=1
        )
        # Subscriber for the drone state
        self.drone_state_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.drone_state_callback
        )

        # Subscriber for the gate point
        self.gate_state_sub = rospy.Subscriber(
            '/visual/gate_points', PoseArray, self.gate_points_callback
        )

        self.drone_model_pub = rospy.Publisher(
            '/visual/drone_model', Marker, queue_size=10
        )

        self.drone_wing_len=rospy.get_param('/drone/wing_len')
        self.drone_height=rospy.get_param('/drone/height')
    def drone_state_callback(self, msg):
        # Create and configure the ellipsoid marker
        drone_ellipsoid = Marker()
        drone_ellipsoid.header.frame_id = "world"
        drone_ellipsoid.header.stamp = rospy.Time.now()
        drone_ellipsoid.ns = "drone_ellipsoid"
        drone_ellipsoid.id = 0
        drone_ellipsoid.type = Marker.SPHERE
        drone_ellipsoid.action = Marker.ADD

        drone_ellipsoid.pose.position = msg.pose.position
        drone_ellipsoid.pose.orientation = msg.pose.orientation

        drone_ellipsoid.scale.x = self.drone_wing_len
        drone_ellipsoid.scale.y = self.drone_wing_len
        drone_ellipsoid.scale.z = self.drone_height

        drone_ellipsoid.color.a = 0.4
        drone_ellipsoid.color.r = 0.0
        drone_ellipsoid.color.g = 1.0
        drone_ellipsoid.color.b = 0.0

        ## create the model maker
        drone_model = Marker()
        drone_model.header.frame_id = "world"
        drone_model.header.stamp = rospy.Time.now()
        drone_model.ns = "drone_model"
        drone_model.id = 0
        drone_model.type = Marker.MESH_RESOURCE
        drone_model.action = Marker.ADD
        drone_model.pose.position = msg.pose.position
        drone_model.pose.orientation = msg.pose.orientation
        drone_model.scale.x = 0.5
        drone_model.scale.y = 0.5
        drone_model.scale.z = 2
        drone_model.color.a = 1
        drone_model.color.r = 0
        drone_model.color.g = 1
        drone_model.color.b = 0
        drone_model.mesh_resource = "package://gestelt_bringup/simulation/models/raynor/meshes/fake_drone.dae"

        # Publish the marker
        self.drone_ellipsoid_pub.publish(drone_ellipsoid)
        self.drone_model_pub.publish(drone_model)
    def gate_points_callback(self, msg):
        # Extract the gate points from the message
        gate_points = np.zeros((len(msg.poses), 3))
        for i in range(len(msg.poses)):
            gate_points[i, 0] = msg.poses[i].position.x
            gate_points[i, 1] = msg.poses[i].position.y
            gate_points[i, 2] = msg.poses[i].position.z
            
         ##============================ gate visualization =========================##
        gate_vis_msg = Marker()
        gate_vis_msg.header.frame_id = "world"
        gate_vis_msg.header.stamp = rospy.Time.now()
        gate_vis_msg.ns = "gate"
        gate_vis_msg.id = 0
        gate_vis_msg.type = Marker.LINE_STRIP
        gate_vis_msg.action = Marker.ADD
        for k in range(len(gate_points)):
            p = gate_points[k,:]
            gate_vis_msg.points.append(Point(x=p[0],
                                             y=p[1],
                                             z=p[2]))
        gate_vis_msg.points.append(Point(x=gate_points[0,0],
                                         y=gate_points[0,1],
                                         z=gate_points[0,2]))
        gate_vis_msg.color.a = 1.0
        gate_vis_msg.color.r = 1.0
        gate_vis_msg.color.g = 0.0
        gate_vis_msg.color.b = 0.0
        gate_vis_msg.scale.x = 0.05   # control the width of the line
        gate_vis_msg.pose.orientation.w = 1.0
        gate_vis_msg.pose.orientation.x = 0.0
        gate_vis_msg.pose.orientation.y = 0.0
        gate_vis_msg.pose.orientation.z = 0.0
        gate_vis_msg.pose.position.x = 0 
        gate_vis_msg.pose.position.y = 0 
        gate_vis_msg.pose.position.z = 0 
        self.gate_vis_pub.publish(gate_vis_msg)
    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    # Instantiate and run the visualizer class
    visualizer = DroneEllipsoidVisualizer()
    visualizer.spin()
