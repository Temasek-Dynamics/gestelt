#!/usr/bin/env python3
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped
from std_msgs.msg import Int8
import math
import time
# Publisher of server events to trigger change of states for trajectory server 
server_event_pub = rospy.Publisher('/traj_server/command', CommanderCommand, queue_size=10)
# Publisher of server events to trigger change of states for trajectory server 
waypoints_pub = rospy.Publisher('/planner/goals', Goals, queue_size=10)

# for visualization
waypoints_pos_pub = rospy.Publisher('/planner/goals_pos', PoseArray, queue_size=10)
waypoints_acc_pub = rospy.Publisher('/planner/goals_acc', AccelStamped, queue_size=10)
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

def create_pose(x, y, z):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1

    return pose
def create_accel(acc_x,acc_y,acc_z):
    acc = Accel()
    acc.linear.x = acc_x
    acc.linear.y = acc_y
    acc.linear.z = acc_z
    
    return acc

def pub_waypoints(waypoints,accels):
    wp_msg = Goals()
    wp_pos_msg=PoseArray()
    wp_acc_msg=AccelStamped()
    
    wp_msg.header.frame_id = "world"
    # wp_msg.waypoints.header.frame_id = "world"
    wp_pos_msg.header.frame_id = "world"
    wp_acc_msg.header.frame_id = "world"

    wp_msg.waypoints = waypoints
    wp_pos_msg.poses = waypoints
    if len(accels)>0:
        wp_acc_msg.accel=accels[0]

    wp_msg.accelerations= accels
    waypoints_pub.publish(wp_msg)
    waypoints_pos_pub.publish(wp_pos_msg)
    waypoints_acc_pub.publish(wp_acc_msg)
def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # hz 20hz

    HOVER_MODE = False
    MISSION_MODE = False

    while not rospy.is_shutdown():
        get_server_state_callback()

        if check_traj_server_states("MISSION"):
            MISSION_MODE = True
        if check_traj_server_states("HOVER"):
            HOVER_MODE = True
        
        if (MISSION_MODE):
            # Already in MISSION 
            # time.sleep(5)
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
    # frame is ENU
    print(f"Sending waypoints to UAVs")
    waypoints = []

    waypoints.append(create_pose(0.0,1.0,1.5)) # 3.0,2.0,3

    waypoints.append(create_pose(0.0,-0.0,1.5)) # 3.0,2.0,3
    waypoints.append(create_pose(0.0,-2.0,1.5))# 5.0,2.0,3
    

    
    #FOR SLOW MOTION TEST
    # waypoints.append(create_pose(-0.5,-0.5,1.5)) # 0.0,-0.5,1
    # waypoints.append(create_pose(0.5,-0.5,1.5)) # 0.5,-0.5,1
    # waypoints.append(create_pose(0.5,0.5,0.5)) # 0.5,0.0,1
    # waypoints.append(create_pose(-0.5,0.5,0.5)) # 0.0,0.0,1
    # waypoints.append(create_pose(-0.5,-0.5,1.0)) # 0.0,-0.5,1
    # waypoints.append(create_pose(0.0,0.0,1.0)) # 0.0,0.0,1
    
    # the number of accelerations must be equal to the number of waypoints
    accel_list = []
    
    g=-9.81 #m/s^2
    f=0.3*(-g) #N
    angle=30
    
    angle_rad=math.radians(angle)

    
    # frame need to verify
    accel_list.append(create_accel(0.0,0.0,0.0))
    accel_list.append(create_accel(-f*np.sin(angle_rad),0.0,g+f*np.cos(angle_rad)))
    accel_list.append(create_accel(0.0,0.0,0.0))
    # accel_list.append(create_accel(0.0,0.0,0.0))
    # accel_list.append(create_accel(0.0,0.0,0.0))
    # accel_list.append(create_accel(0.0,0.0,0.0))
    # accel_list.append(create_accel(0.0,0.0,0.0))
    # accel_list.append(create_accel(0.0,0.0,0.0))
    pub_waypoints(waypoints,accel_list)
    rospy.spin()
if __name__ == '__main__':
    main()