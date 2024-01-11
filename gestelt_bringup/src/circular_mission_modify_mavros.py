#!/usr/bin/env python3
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, Twist
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8, Bool
from controller_msgs.msg import FlatTarget
import math
import time

# get ros params
takeoff_height = rospy.get_param("/takeoff_height", 1.2)


# Publisher of server events to trigger change of states for trajectory server 
server_event_pub = rospy.Publisher('/traj_server/command', CommanderCommand, queue_size=10)
# Publisher of server events to trigger change of states for trajectory server 
waypoints_pub = rospy.Publisher('/planner/goals', Goals, queue_size=10)

# Publisher for desired hover setpoint
hover_position_pub = rospy.Publisher('/planner/hover_position', Pose, queue_size=10)

# Publisher of circular trajectory setpoints(transfer to mavros setpoint)
circular_traj_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)


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
    wp_pos_msg.poses = waypoints
    if len(accels)>0:
        wp_acc_msg.accel=accels[1][0]
    
    rospy.loginfo("accels: %s",wp_msg.accelerations)
    rospy.loginfo("vels: %s",wp_msg.velocities)

    rospy.loginfo("accels mask: %s",wp_msg.accelerations_mask)
    rospy.loginfo("vels mask: %s",wp_msg.velocities_mask)

    waypoints_pub.publish(wp_msg)
    waypoints_pos_pub.publish(wp_pos_msg)
    waypoints_acc_pub.publish(wp_acc_msg)

def hover_position():
    
    hover_position = Pose()
    hover_position.position.x = 0.0
    hover_position.position.y = 1.8
    # z is the same as the takeoff height

    hover_position_pub.publish(hover_position)

def circular_raw_traj_cb(msg):
    circular_mavros_traj = PositionTarget()
    circular_mavros_traj.header.stamp = rospy.Time.now()
    circular_mavros_traj.header.frame_id = "world"
    circular_mavros_traj.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    circular_mavros_traj.position.x = msg.position.x
    circular_mavros_traj.position.y = msg.position.y
    circular_mavros_traj.position.z = msg.position.z
    circular_mavros_traj.velocity.x = msg.velocity.x
    circular_mavros_traj.velocity.y = msg.velocity.y
    circular_mavros_traj.velocity.z = msg.velocity.z
    circular_mavros_traj.acceleration_or_force.x = msg.acceleration.x
    circular_mavros_traj.acceleration_or_force.y = msg.acceleration.y
    circular_mavros_traj.acceleration_or_force.z = msg.acceleration.z
    
    circular_mavros_traj.type_mask = PositionTarget.IGNORE_YAW+PositionTarget.IGNORE_YAW_RATE

    circular_traj_pub.publish(circular_mavros_traj)

def main():
    """      if (!isExecutingMission()){
    logInfoThrottled("Waiting for mission", 5.0);
    // ROS_INFO("in waiting for mission");
    // execHover(); --> CLOSE THIS!!!, remember to open it for other missions
    """
    rospy.init_node('mission_startup', anonymous=True)
    pub_freq = 25 # hz
    rate = rospy.Rate(pub_freq) # hz 20hz

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
            time.sleep(5)
            break
        elif (not HOVER_MODE):
            # IDLE -> TAKE OFF -> HOVER
            hover_position()
            print("Setting to HOVER mode!")
            publishCommand(CommanderCommand.TAKEOFF)
        elif (HOVER_MODE):
            # HOVER -> desired position -> MISSION
            print("Setting to MISSION mode!")
            publishCommand(CommanderCommand.MISSION)


        print("tick!")
        rate.sleep()
    
    # Subscriber of the raw circular trajectory
    rospy.Subscriber('/reference/flatsetpoint', FlatTarget, callback=circular_raw_traj_cb)

    rospy.spin()
if __name__ == '__main__':
    main()