#!/usr/bin/env python3

import rospy
from trajectory_server_msgs.msg import State
from std_msgs.msg import Int8

num_drones = 2

# Publisher of server events to trigger change of states for trajectory server 
server_event_pub = rospy.Publisher('/traj_server_event', Int8, queue_size=10)

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
    for drone_id in range(0, num_drones):
        msg = rospy.wait_for_message(f"/drone{drone_id}/server_state", State, timeout=5.0)
        server_states[str(msg.drone_id)] = msg
        # print("==================")
        # print(msg)
        # print("==================")

def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # 20hz

    print("Setting to HOVER mode!")
    # Take off 
    while not rospy.is_shutdown():
        get_server_state_callback()
        if check_traj_server_states("HOVER"):
            break
        publish_server_event(0)
        print("tick!")
        rate.sleep()

    print("Setting to MISSION mode!")
    # Switch to mission mode
    while not rospy.is_shutdown():
        get_server_state_callback()
        if check_traj_server_states("MISSION"):
            break
        publish_server_event(2)
        print("tick!")
        rate.sleep()

if __name__ == '__main__':
    main()