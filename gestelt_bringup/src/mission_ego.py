#!/usr/bin/env python3

import rospy
from gestelt_msgs.msg import Command, CommanderState, Goals
from geometry_msgs.msg import Transform
from std_msgs.msg import Int8

num_drones = 1

# Publisher of server events to trigger change of states for trajectory server 
goals_pub = rospy.Publisher('/planner_adaptor/goals', Goals, queue_size=5)

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

def publish_server_cmd(event_enum):
    for drone_id in range(0, num_drones):
        traj_server_cmd_msg = Command()
        traj_server_cmd_msg.command = event_enum
        # Publisher of server events to trigger change of states for trajectory server 
        traj_server_cmd_pub = rospy.Publisher(f'/drone{drone_id}/traj_server/command', Command, queue_size=5)
        traj_server_cmd_pub.publish(traj_server_cmd_msg)

def get_server_state_callback():
    for drone_id in range(0, num_drones):
        msg = rospy.wait_for_message(f"/drone{drone_id}/traj_server/state", CommanderState, timeout=5.0)
        server_states[str(msg.drone_id)] = msg
        # print("==================")
        # print(msg)
        # print("==================")

def create_transform(x, y, z):
    pos = Transform()
    pos.translation.x = x
    pos.translation.y = y
    pos.translation.z = z

    pos.rotation.x = 0
    pos.rotation.y = 0
    pos.rotation.z = 0
    pos.rotation.w = 1

    return pos

def pub_goals(transform_wps):
    goals_msg = Goals()
    goals_msg.header.frame_id = "world"
    goals_msg.transforms = transform_wps

    goals_pub.publish(goals_msg)

def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # 20hz

    print("Setting to HOVER mode!")
    # Take off 
    while not rospy.is_shutdown():
        get_server_state_callback()
        if check_traj_server_states("HOVER"):
            break
        publish_server_cmd(0)
        print("tick!")
        rate.sleep()

    print("Setting to MISSION mode!")
    # Switch to mission mode
    while not rospy.is_shutdown():
        get_server_state_callback()
        if check_traj_server_states("MISSION"):
            break
        publish_server_cmd(2)
        print("tick!")
        rate.sleep()

    # Send waypoints to UAVs
    print(f"Sending waypoints to UAVs")
    transforms = []

    # length = 12
    # d = length/2
    # d = 10.5
    # for i in range(10):
    #     transforms.append(create_transform(d, 0, 1))
    #     transforms.append(create_transform(d, d, 1))
    #     transforms.append(create_transform(-d, d, 1))
    #     transforms.append(create_transform(-d, -d, 1))
    #     transforms.append(create_transform(d, -d, 1))
    #     transforms.append(create_transform(0, 0, 1))

    d = 1.5
    z = 0.75
    for i in range(10):
        transforms.append(create_transform(d, d, z))
        transforms.append(create_transform(-d, d, z))
        transforms.append(create_transform(-d, -d, z))
        transforms.append(create_transform(d, -d, z))
    transforms.append(create_transform(0, 0, z))

    pub_goals(transforms)

if __name__ == '__main__':
    main()