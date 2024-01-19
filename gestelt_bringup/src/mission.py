#!/usr/bin/env python3

import rospy
import math
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand, PosGoals
from geometry_msgs.msg import Pose, Accel, PoseArray, Vector3
from std_msgs.msg import Int8
import numpy as np
import transforms3d as t3d
import time

# Publisher of server events to trigger change of states for trajectory server 
server_event_pub = rospy.Publisher('/traj_server/command', CommanderCommand, queue_size=10)
# Publisher of server events to trigger change of states for trajectory server 
waypoints_pub = rospy.Publisher('/planner/goals', Goals, queue_size=10)

# Dictionary of UAV states
server_states = {}

# Check if UAV has achived desired traj_server_state
def check_traj_server_states(des_traj_server_state):
    if len(server_states.items()) == 0:
        print("No Server states received!")
        return False
    
    for server_state in server_states.items():
        # print(f"{server_state[0]}: {des_traj_server_state}")
        if server_state[1].traj_server_state != des_traj_server_state:
            return False
    return True

def publishCommand(event_enum):
    commander_cmd = CommanderCommand()
    commander_cmd.command = event_enum

    server_event_pub.publish(commander_cmd)

def get_server_state_callback():
    msg = rospy.wait_for_message(f"/traj_server/state", CommanderState, timeout=5.0)
    server_states[str(msg.drone_id)] = msg
    # print("==================")
    # print(msg)
    # print("==================")

# def create_pose(x, y, z):
#     pose = Pose()
#     pose.position.x = x
#     pose.position.y = y
#     pose.position.z = z

#     pose.orientation.x = 0
#     pose.orientation.y = 0
#     pose.orientation.z = 0
#     pose.orientation.w = 1

#     return pose
# def create_accel(acc_x,acc_y,acc_z):
#     acc = Accel()
#     acc.linear.x = acc_x
#     acc.linear.y = acc_y
#     acc.linear.z = acc_z

#     return acc

def create_pose(x, y, z):
    pose_linear = Vector3()
    
    pose_linear.x = x
    pose_linear.y = y
    pose_linear.z = z
    
    # rotation_matrix = t3d.euler.euler2mat(roll, pitch, yaw, 'sxyz')
    # quaternion = t3d.quaternions.mat2quat(rotation_matrix)
    # cy = math.cos(yaw * 0.5)
    # sy = math.sin(yaw * 0.5)
    # cp = math.cos(pitch * 0.5)
    # sp = math.sin(pitch * 0.5)
    # cr = math.cos(roll * 0.5)
    # sr = math.sin(roll * 0.5)

    # q_w = cr * cp * cy + sr * sp * sy
    # q_x = sr * cp * cy - cr * sp * sy
    # q_y = cr * sp * cy + sr * cp * sy
    # q_z = cr * cp * sy - sr * sp * cy
    # pose.orientation.w = q_w
    # pose.orientation.x = q_x
    # pose.orientation.y = q_y
    # pose.orientation.z = q_z
    return pose_linear

def create_orientation(roll, pitch, yaw):
    pose_angular = Vector3()
    # pose_angular.x = math.radians(roll)
    # pose_angular.y = math.radians(pitch)
    # pose_angular.z = math.radians(yaw)
    pose_angular.x = roll
    pose_angular.y = pitch
    pose_angular.z = yaw
    return pose_angular

def create_vel_linear(x, y, z):
    vel_linear = Vector3()
    vel_linear.x = x
    vel_linear.y = y
    vel_linear.z = z
    return vel_linear

def create_vel_angular(x, y, z):
    vel_angular = Vector3()
    vel_angular.x = x
    vel_angular.y = y
    vel_angular.z = z
    return vel_angular
 
def pub_waypoints(waypoints_linear, waypoints_angular, waypoints_vel_linear, waypoints_vel_angular):
    wp_msg = Goals()
    wp_msg.header.frame_id = "world"
    wp_msg.waypoints_linear = waypoints_linear
    wp_msg.waypoints_angular = waypoints_angular
    wp_msg.waypoints_velocity_linear = waypoints_vel_linear
    wp_msg.waypoints_velocity_angular = waypoints_vel_angular
    wp_msg.velocity_mask = True    #set velocity_mask = False if want velocity constraint
    waypoints_pub.publish(wp_msg)


def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # 20hz
    degrees = 70
    g = 9.81
    HOVER_MODE = False
    MISSION_MODE = False

    while not rospy.is_shutdown():
        get_server_state_callback()

        if check_traj_server_states("MISSION"):
            MISSION_MODE = True
        if check_traj_server_states("HOVER"):
            HOVER_MODE = True
        
        if (MISSION_MODE):
            time.sleep(1)
            # Already in MISSION 
            break
        elif (not HOVER_MODE):
            # IDLE -> TAKE OFF -> HOVER
            print("Setting to HOVER mode!")
            publishCommand(CommanderCommand.TAKEOFF)
        elif (HOVER_MODE):
            # HOVER -> MISSION
            print("Setting to MISSION mode!")
            publishCommand(CommanderCommand.MISSION)

        print("tick!")
        rate.sleep()

    # Send waypoints to UAVs
    print(f"Sending waypoints to UAVs")
    waypoints_linear = []
    waypoints_angular = []
    waypoints_vel_linear = []
    waypoints_vel_angular = []
    waypoints_linear.append(create_pose(0.0, 3.0, 1.5))
    waypoints_linear.append(create_pose(2.0, 4.0, 1.5))
    
    waypoints_angular.append(create_orientation(0.0, 0.0, 40)) 
    waypoints_angular.append(create_orientation(0, 0, 0))
   
    waypoints_vel_linear.append(create_vel_linear(2.0, 0.0, 0.0))
    waypoints_vel_linear.append(create_vel_linear(0.0, 0.0, 0.0))

    waypoints_vel_angular.append(create_vel_angular(0.0, 0.0, 0.0))
    waypoints_vel_angular.append(create_vel_angular(0.0, 0.0, 0.0))


    pub_waypoints(waypoints_linear, waypoints_angular, waypoints_vel_linear, waypoints_vel_angular)
    rospy.spin()
if __name__ == '__main__':
    main()