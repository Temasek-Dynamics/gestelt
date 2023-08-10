#!/usr/bin/env python3

import rospy
from trajectory_server_msgs.msg import State, Waypoints
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

num_drones = 1

# Publisher of server events to trigger change of states for trajectory server 
waypoints_pub = rospy.Publisher('/waypoints', Waypoints, queue_size=10)

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
    wp_msg = Waypoints()
    wp_msg.waypoints.header.frame_id = "world"
    wp_msg.waypoints.poses = waypoints

    waypoints_pub.publish(wp_msg)

def get_server_state_callback():
    for drone_id in range(0, num_drones):
        msg = rospy.wait_for_message(f"/drone{drone_id}/server_state", State, timeout=5.0)
        server_states[str(msg.drone_id)] = msg
        # print("==================")
        # print(msg)
        # print("==================")

def main():
    rospy.init_node('mission_startup', anonymous=True)

    print(f"Collecting UAV states...")
    get_server_state_callback()

    print(f"Checking that all UAVs are in MISSION mode!")
    # Check that all vehicles are in mission state
    if not check_traj_server_states("MISSION"):
        print(f"Not all vehicles in MISSION mode. Aborting send waypoints!")
        return

    # Send waypoints to UAVs
    print(f"Sending waypoints to UAVs")
    waypoints = []
    # Square formation with length L
    length = 12
    d = length/2
    for i in range(10):
        waypoints.append(create_pose(d, 0, 1))
        waypoints.append(create_pose(d, d, 1))
        waypoints.append(create_pose(-d, d, 1))
        waypoints.append(create_pose(-d, -d, 1))
        waypoints.append(create_pose(d, -d, 1))
        waypoints.append(create_pose(0, 0, 1))
    pub_waypoints(waypoints)

if __name__ == '__main__':
    main()