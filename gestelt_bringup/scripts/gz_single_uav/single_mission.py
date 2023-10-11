#!/usr/bin/env python3

import rospy
from trajectory_server_msgs.msg import State, Waypoints
from geometry_msgs.msg import Pose
from std_msgs.msg import Int8

num_drones = 1

# Publisher of server events to trigger change of states for trajectory server 
server_event_pub = rospy.Publisher('/traj_server_event', Int8, queue_size=10)
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

def publish_server_event(event_enum):
    server_event_pub.publish(Int8(event_enum))

def get_server_state_callback():
    msg = rospy.wait_for_message(f"/drone0/server_state", State, timeout=5.0)
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
    wp_msg = Waypoints()
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
            publish_server_event(0)
        elif (HOVER_MODE):
            # HOVER -> MISSION
            print("Setting to MISSION mode!")
            publish_server_event(2)

        print("tick!")
        rate.sleep()

    # Send waypoints to UAVs
    print(f"Sending waypoints to UAVs")
    waypoints = []
    # Square formation with length L
    max_x = 1.5
    max_y = 1.5
    min_x = -1.5
    min_y = -1.5
    z = 1.0
    for i in range(10):
        waypoints.append(create_pose(max_x, min_y, z))
        waypoints.append(create_pose(max_x, max_y, z))
        waypoints.append(create_pose(min_x, max_y, z))
        waypoints.append(create_pose(min_x, min_y, z))
    waypoints.append(create_pose(0, 0, z))
    pub_waypoints(waypoints)

if __name__ == '__main__':
    main()