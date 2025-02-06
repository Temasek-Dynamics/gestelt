#!/usr/bin/env python3
import sys
import os
import subprocess
import yaml
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append("../")
sys.path.append(subdirectory_path)

import rospy
from geometry_msgs.msg import  PoseStamped
from visualization_msgs.msg import Marker
class DroneEllipsoidVisualizer:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('drone_ellipsoid_vis', anonymous=True)

        # Publisher for the ellipsoid marker
        self.drone_ellipsoid_pub = rospy.Publisher(
            '/learning_agile_sim/drone_ellipsoid', Marker, queue_size=10
        )

        # Subscriber for the drone state
        self.drone_state_sub = rospy.Subscriber(
            '/mavros/local_position/pose', PoseStamped, self.drone_state_callback
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

        drone_ellipsoid.color.a = 0.5
        drone_ellipsoid.color.r = 0.0
        drone_ellipsoid.color.g = 1.0
        drone_ellipsoid.color.b = 0.0

        # Publish the marker
        self.drone_ellipsoid_pub.publish(drone_ellipsoid)

    def spin(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    # Instantiate and run the visualizer class
    visualizer = DroneEllipsoidVisualizer()
    visualizer.spin()
