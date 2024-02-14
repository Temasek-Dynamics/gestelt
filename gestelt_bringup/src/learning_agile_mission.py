#!/usr/bin/env python3
import numpy as np
import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, Twist
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import Int8, Bool
import math
import time
import tf
# get ros params from rosparam server
is_simulation=rospy.get_param('mission/is_simulation', False)

# Publisher of server events to trigger change of states for trajectory server 
server_event_pub = rospy.Publisher('/traj_server/command', CommanderCommand, queue_size=10)
# Publisher of server events to trigger change of states for trajectory server 
waypoints_pub = rospy.Publisher('/planner/goals_learning_agile', Goals, queue_size=10)

# Publisher for desired hover setpoint
hover_position_pub = rospy.Publisher('/planner/hover_position', Pose, queue_size=10)

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

def transform_map_to_world():
    """
    this function calculates the transformation from map to world frame
    world frame is the initial position of the drone
    map frame is the origin of the map  
    Returns:
        trans: translation vector from map to world
        rot: rotation vector from map to world
    """
    # tf listener, for transformation from map to world(initialize at the drone position)
    tf_listener = tf.TransformListener()

    # in the simulation, the map frame is used to represent the abs world frame
    # in the real world, there is no map frame, the world frame is the abs world frame
    if is_simulation:
        while not rospy.is_shutdown():
            try:
        
                if tf_listener.canTransform("world", "map", rospy.Time(0)):
                    (trans, rot) = tf_listener.lookupTransform("world", "map", rospy.Time(0))
                    break 
                else:
                    rospy.sleep(0.04)
            except tf.TransformException as ex:
                rospy.logwarn("TransformException: {}".format(ex))
                rospy.sleep(0.04)
        return trans,rot
    else:
        return (0.0,0.0,0.0),(0.0,0.0,0.0,1.0)

def create_pose(x, y, z):
    pose = Pose()

    # transform waypoints from map to world
    trans,rot=transform_map_to_world()
    pose.position.x = x+trans[0]
    pose.position.y = y+trans[1]
    pose.position.z = z+trans[2]

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
    
    waypoints_pub.publish(wp_msg)
    waypoints_pos_pub.publish(wp_pos_msg)
    waypoints_acc_pub.publish(wp_acc_msg)

# def hover_position():
    
#      # transform waypoints from map to world
#     trans,rot=transform_map_to_world()

#     hover_position = Pose()
#     hover_position.position.x = 0.0+trans[0]
#     hover_position.position.y = 0.0+trans[1]
#     # z is the same as the takeoff height

#     hover_position_pub.publish(hover_position)


def main():
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
            time.sleep(1)
            break
        elif (not HOVER_MODE):
            # IDLE -> TAKE OFF -> HOVER
            # hover_position()
            print("Setting to HOVER mode!")
            publishCommand(CommanderCommand.TAKEOFF)
        elif (HOVER_MODE):
            # HOVER -> desired position -> MISSION
            print("Setting to MISSION mode!")
            publishCommand(CommanderCommand.MISSION)


        print("tick!")
        rate.sleep()
    ##--------------------- min snap trajectory---------------------##
        
    # Send waypoints to UAVs
    # frame is ENU
    print(f"Sending waypoints to UAVs")
    waypoints = []

    # side length 5m


    
    # world frame is the initial position of the drone
    # map frame is the origin of the map
    # waypoints are under the map frame, will be transformed to world frame
    
    # gate position
    waypoints.append(create_pose(1.2,-0.0,1.5)) # 3.0,2.0,3   2.0,-0.0,1.5
    
    # end position
    waypoints.append(create_pose(-0.0,-1.8,1.4))# 5.0,2.0,3  -2.0,-1.8,1.4

    #------------------------------------ for hovering test:-------------------------------------------#
    waypoints.append(create_pose(-0.0,0.0,1.3)) # 3.0,2.0,3   2.0,-0.0,1.5
    waypoints.append(create_pose(-0.0,0.0,1.3)) # 3.0,2.0,3   2.0,-0.0,1.5

    #--------------------------------------------------------------------------------------------------#
    # the number of accelerations must be equal to the number of waypoints
    accel_list = []
    accel_list.append(create_accel(None,None,None))
    accel_list.append(create_accel(None,None,None))
   
    # velocites constraint
    vel_list = []
    vel_list.append(create_vel(None,None,None))
    vel_list.append(create_vel(None,None,None))
   
    pub_waypoints(waypoints,accel_list,vel_list)
    ##--------------------- end of min snap trajectory--------------##

    # spending initial state to the learning agile node    
    rospy.spin()
if __name__ == '__main__':
    main()




##--------------------- min snap trajectory---------------------##
    
   