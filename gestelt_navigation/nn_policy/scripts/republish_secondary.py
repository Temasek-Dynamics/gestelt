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
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/pose",PoseStamped, self.poseCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/odom",Odometry, self.odomCb, queue_size = 10)

        self.drone_pose_pub_ = rospy.Publisher('/drone0/mavros/vision_pose/pose', PoseStamped, queue_size=5)
        self.drone_odom_pub_ = rospy.Publisher('/vrpn_client_node/warping/twist', TwistStamped, queue_size=5)

        self.drone_pos_msg = PoseStamped()

    def poseCb(self, msg):
        self.drone_pos_msg = msg
        curr_pose_msg = PoseStamped()
        curr_pose_msg.pose.position.x = msg.pose.position.x
        curr_pose_msg.pose.position.y = msg.pose.position.y
        curr_pose_msg.pose.position.z = msg.pose.position.z

        curr_pose_msg.pose.orientation.x = msg.pose.orientation.x
        curr_pose_msg.pose.orientation.y = msg.pose.orientation.y
        curr_pose_msg.pose.orientation.z = msg.pose.orientation.z
        curr_pose_msg.pose.orientation.w = msg.pose.orientation.w

        self.drone_pose_pub_.publish(curr_pose_msg)



    def odomCb(self, msg):
        curr_nav_msgs = TwistStamped()
        curr_nav_msgs.twist.linear.x = msg.twist.twist.linear.x
        curr_nav_msgs.twist.linear.y= msg.twist.twist.linear.y
        curr_nav_msgs.twist.linear.z = msg.twist.twist.linear.z

        curr_nav_msgs.twist.angular.x = msg.twist.twist.angular.x
        curr_nav_msgs.twist.angular.y= msg.twist.twist.angular.y
        curr_nav_msgs.twist.angular.z = msg.twist.twist.angular.z
        # curr_nav_msgs.header = msg.header


        self.drone_odom_pub_.publish(curr_nav_msgs)


if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")
    rospy.init_node("republish2")
    republish = Republish()

    rospy.spin()