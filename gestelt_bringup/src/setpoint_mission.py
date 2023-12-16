#!/usr/bin/env python3
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8
import math
import time

# Publisher of server events to trigger change of states for trajectory server 
server_event_pub = rospy.Publisher('/traj_server/command', CommanderCommand, queue_size=10)
# Publisher of server events to trigger change of states for trajectory server 
waypoints_pub = rospy.Publisher('/planner/goals', Goals, queue_size=10)

# Publisher for single setpoint
single_setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

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
    pub_freq = 25 # hz
    rate = rospy.Rate(pub_freq) # hz 20hz

    HOVER_MODE = False
    MISSION_MODE = False
    hover_count=0
    hover_duration=5*pub_freq # 5 seconds
    ramp_steps = 25*1
    t=0

    last_pos_x = 0
    last_pos_y = 0
    last_pos_z = 0
    while not rospy.is_shutdown():
        get_server_state_callback()

        if check_traj_server_states("MISSION"):
            MISSION_MODE = True
        if check_traj_server_states("HOVER"):
            HOVER_MODE = True
        
        if (MISSION_MODE):
            # Already in MISSION 
             # Send waypoints to UAVs
            # frame is ENU
            print(f"Sending single setpoint to UAVs")
            
            
            setpoint = PositionTarget()
            setpoint.header.stamp = rospy.Time.now()
            setpoint.header.frame_id = "world"
            setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            setpoint.type_mask = PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
            
            setpoint.velocity.x = 3
            setpoint.velocity.y = 3
            setpoint.velocity.z = 3
            last_pos_x +=setpoint.velocity.x*(1/pub_freq)
            last_pos_y +=setpoint.velocity.y*(1/pub_freq)
            last_pos_z +=setpoint.velocity.z*(1/pub_freq)
            
            setpoint.position.x = last_pos_x
            setpoint.position.y = last_pos_y
            setpoint.position.z = last_pos_z+1.5
           
            setpoint.acceleration_or_force.x = 0
            setpoint.acceleration_or_force.y = 0
            setpoint.acceleration_or_force.z = 0
            setpoint.yaw = 0
            setpoint.yaw_rate = 0
            single_setpoint_pub.publish(setpoint)


            if t < ramp_steps:
                t=t+1
        elif (not HOVER_MODE):
            # IDLE -> TAKE OFF -> HOVER
            print("Setting to HOVER mode!")
            publishCommand(CommanderCommand.TAKEOFF)
        elif (HOVER_MODE):
            # HOVER -> MISSION
            print("Setting to MISSION mode!")
            hover_count=hover_count+1
            if hover_count>hover_duration:
                publishCommand(CommanderCommand.MISSION)

        print("tick!")
        rate.sleep()

   
    rospy.spin()

if __name__ == '__main__':
    main()