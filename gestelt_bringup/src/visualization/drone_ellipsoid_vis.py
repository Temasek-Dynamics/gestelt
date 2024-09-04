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
from gestelt_msgs.msg import Goals,  CommanderState
from geometry_msgs.msg import  PoseStamped, TwistStamped, Point
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
import time
drone_ellispoid_pub = rospy.Publisher('/learning_agile_agent/drone_ellispoid', Marker, queue_size=10)
def drone_state_callback(msg):
    drone_ellispoid = Marker()
    drone_ellispoid.header.frame_id = "world"
    drone_ellispoid.header.stamp = rospy.Time.now()
    drone_ellispoid.ns = "drone_ellispoid"
    drone_ellispoid.id = 0
    drone_ellispoid.type = Marker.SPHERE
    drone_ellispoid.action = Marker.ADD
    drone_ellispoid.pose.position = msg.pose.position
    drone_ellispoid.pose.orientation = msg.pose.orientation
    drone_ellispoid.scale.x = 1.0
    drone_ellispoid.scale.y = 1.0
    drone_ellispoid.scale.z = 0.4   
    drone_ellispoid.color.a = 0.5
    drone_ellispoid.color.r = 0.0
    drone_ellispoid.color.g = 1.0
    drone_ellispoid.color.b = 0.0
    drone_ellispoid_pub.publish(drone_ellispoid)

drone_state_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, drone_state_callback)

def main():
    rospy.init_node('drone_ellispoid_vis', anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    main()