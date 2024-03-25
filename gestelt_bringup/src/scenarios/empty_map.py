#!/usr/bin/env python3

import rospy
from gestelt_msgs.msg import Command, CommanderState, Goals
from geometry_msgs.msg import Transform, PoseStamped
from std_msgs.msg import Int8

num_drones = 1

# Publisher of server events to trigger change of states for trajectory server 
goal_publishers = []
for i in range(0, num_drones):
    goal_publishers.append(rospy.Publisher(f'/drone{i}/planner_adaptor/goals', Goals, queue_size=2))

# single_goal_pub = rospy.Publisher('/drone0/planner/single_goal', PoseStamped, queue_size=2)

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

def create_pose(x, y, z):
    pos = PoseStamped()
    pos.pose.position.x = x
    pos.pose.position.y = y
    pos.pose.position.z = z

    pos.pose.orientation.x = 0
    pos.pose.orientation.y = 0
    pos.pose.orientation.z = 0
    pos.pose.orientation.w = 0

    return pos

def pub_goals(goals):
    if (len(goals) > num_drones):
        print(f"Failed to publish goals. No. of goals ({len(goals)}) > num_drones ({num_drones})")
        return False 

    for i in range(0, len(goals)):
        goals_msg = Goals()
        goals_msg.header.frame_id = "world"
        goals_msg.transforms = goals[i]

        goal_publishers[i].publish(goals_msg)

    return True
    

def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # 20hz

    if check_traj_server_states("MISSION"):
        print("Already in mission mode")
    else:
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

    goals_0 = []
    goals_0.append(create_transform(2.0, 2.0, 2.0))
    goals_0.append(create_transform(5.0, 5.0, 1.0))
    pub_goals([goals_0])

if __name__ == '__main__':
    main()