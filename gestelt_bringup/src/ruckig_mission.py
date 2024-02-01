#!/usr/bin/env python3
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, Twist
from std_msgs.msg import Int8, Bool
import math
import time

waypoints_pub = rospy.Publisher('/planner/goals', Goals, queue_size=10)


def create_pose(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = -0.707
    pose.orientation.w = 0.707

    return pose
def create_accel(acc_x,acc_y,acc_z):
    acc = Accel()
    acc_mask = Bool()
    if acc_x !=None:
        acc.linear.x = acc_x
        acc.linear.y = acc_y
        acc.linear.z = acc_z
        acc_mask.data=False
    else:
        acc_mask.data=True
    return acc,acc_mask
def create_vel(vel_x,vel_y,vel_z):
    vel = Twist()
    vel_mask = Bool()
    if vel_x !=None:
        vel.linear.x = vel_x
        vel.linear.y = vel_y
        vel.linear.z = vel_z
        vel_mask.data=False
    else:
        vel_mask.data=True
    return vel,vel_mask

def pub_waypoints(waypoints,accels,vels):
    wp_msg = Goals()
    wp_pos_msg=PoseArray()
    wp_acc_msg=AccelStamped()
  

    wp_msg.header.frame_id = "world"
    # wp_msg.waypoints.header.frame_id = "world"
    wp_pos_msg.header.frame_id = "world"
    wp_acc_msg.header.frame_id = "world"

    wp_msg.waypoints = waypoints
    wp_msg.accelerations= [accel[0] for accel in accels]
    wp_msg.velocities= [vel[0] for vel in vels]

    wp_msg.accelerations_mask=[accel[1] for accel in accels]
    wp_msg.velocities_mask=[vel[1] for vel in vels]
    


    # for waypoints and acceleration vector visualization


    waypoints_pub.publish(wp_msg)
    rospy.loginfo("Waypoints published")

def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # hz 20hz

    

    print("tick!")
    rate.sleep()

    # Send waypoints to UAVs
    # frame is ENU
    print(f"Sending waypoints to UAVs")
    waypoints = []

  
    waypoints.append(create_pose(1,1.5,1.2)) # 3.0,2.0,3

    # the number of accelerations must be equal to the number of waypoints
    accel_list = []
    
    g=-9.81 #m/s^2
    f=0.3*(-g) #N
    angle=60
    angle_rad=math.radians(angle)

    

    accel_list.append(create_accel(-f*np.sin(angle_rad),0.0,g+f*np.cos(angle_rad)))

    
    # velocites constraint
    vel_list = []
    vel_list.append(create_vel(0.0,0.0,0.0))

    pub_waypoints(waypoints,accel_list,vel_list)
    rospy.spin()
if __name__ == '__main__':
    main()