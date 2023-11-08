#!/usr/bin/env python3

import rospy
from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

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

def pub_waypoints(waypoints):
    wp_msg = Goals()
    wp_msg.waypoints.header.frame_id = "world"
    wp_msg.waypoints.poses = waypoints

    waypoints_pub.publish(wp_msg)

def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # 20hz

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
    waypoints = []
    waypoints.append(create_pose(1.0, 3.0, 2.0))
    pub_waypoints(waypoints)

if __name__ == '__main__':
    main()