#!/usr/bin/env python3
import sys
import time
import numpy as np
import rospy
import pkg_resources
import yaml
import os
import rospkg
import yaml
from std_msgs.msg import Int8
from gestelt_msgs.msg import CommanderState, ExecTrajectory
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from enum import Enum
import roslib.packages
from std_msgs.msg import Bool
import tf2_ros
import threading
from std_msgs.msg import Int8
from signal import signal, SIGINT
import random

def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

class Republish(object):
    def __init__ (self):
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/vision_pose/pose",PoseStamped, self.poseCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/vrpn_client_node/warping/twist",TwistStamped, self.odomCb, queue_size = 10)

        self.drone_pose_pub_ = rospy.Publisher('/drone0/mavros/vision_pose/odom', Odometry, queue_size=5)

        self.drone_pos_msg = PoseStamped()

    def poseCb(self, msg):
        self.drone_pos_msg = msg

    def odomCb(self, msg):
        curr_nav_msgs = Odometry()
        curr_nav_msgs.header = msg.header
        curr_nav_msgs.twist.twist.linear.x = msg.twist.linear.x
        curr_nav_msgs.twist.twist.linear.y = msg.twist.linear.y
        curr_nav_msgs.twist.twist.linear.z = msg.twist.linear.z

        curr_nav_msgs.twist.twist.angular.x = msg.twist.angular.x
        curr_nav_msgs.twist.twist.angular.y = msg.twist.angular.y
        curr_nav_msgs.twist.twist.angular.z = msg.twist.angular.z

        curr_nav_msgs.pose.pose.position.x = self.drone_pos_msg.pose.position.x
        curr_nav_msgs.pose.pose.position.y = self.drone_pos_msg.pose.position.y
        curr_nav_msgs.pose.pose.position.z = self.drone_pos_msg.pose.position.z

        curr_nav_msgs.pose.pose.orientation.x = self.drone_pos_msg.pose.orientation.x
        curr_nav_msgs.pose.pose.orientation.y = self.drone_pos_msg.pose.orientation.y
        curr_nav_msgs.pose.pose.orientation.z = self.drone_pos_msg.pose.orientation.z
        curr_nav_msgs.pose.pose.orientation.w = self.drone_pos_msg.pose.orientation.w


        self.drone_pose_pub_.publish(curr_nav_msgs)


if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")
    rospy.init_node("republish1")
    republish = Republish()

    rospy.spin()