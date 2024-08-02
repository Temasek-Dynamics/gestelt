#!/usr/bin/env python3

import rospy
from gestelt_msgs.msg import Command, CommanderState, Goals
from geometry_msgs.msg import Transform, PoseStamped
from std_msgs.msg import Int32, Int8

num_drones = 3

# Publisher of swarm command
swarm_cmd_pub = rospy.Publisher(f'/traj_server/swarm_command', Int8, queue_size=5)

# Publisher of server events to trigger change of states for trajectory server 
goal_publishers = []
for i in range(0, num_drones):
    goal_publishers.append(rospy.Publisher(f'/drone{i}/planner_adaptor/goals', Goals, queue_size=2))

# single_goal_pub = rospy.Publisher('/drone0/planner/single_goal', PoseStamped, queue_size=2)

# Dictionary of UAV states
server_states = {}
current_task_ids = {} # Dict with keys representing drone_id and values representing the current task_id being executed

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

def publish_swarm_cmd(event_enum):
    msg = Int8()
    msg.data = event_enum
    # Publisher of server events to trigger change of states for trajectory server 
    swarm_cmd_pub.publish(msg)

def get_server_state_callback():
    for drone_id in range(0, num_drones):
        msg = rospy.wait_for_message(f"/drone{drone_id}/traj_server/state", CommanderState, timeout=5.0)
        server_states[str(msg.drone_id)] = msg

def get_current_task_id():
    """Get current task ID among all agents
    """
    for drone_id in range(0, num_drones):
        msg = rospy.wait_for_message(f"/drone{drone_id}/current_task_id", Int32, timeout=5.0)
        current_task_ids[str(drone_id)] = msg.data

# Check if UAV has completed task execution
def check_task_ids(des_task_id):
    if len(current_task_ids.items()) == 0:
        print("No task ids received!")
        return False
    
    for cur_task_id in current_task_ids.items():
        # print(f"{cur_task_id[0]}: {cur_task_id[1]}")
        if cur_task_id[1] != des_task_id:
            return False
    return True


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

def pub_goals(goals, task_id=0):
    if (len(goals) > num_drones):
        print(f"Failed to publish goals. No. of goals ({len(goals)}) > num_drones ({num_drones})")
        return False 

    for i in range(0, len(goals)):  # For each goal
        goals_msg = Goals()
        goals_msg.task_id = task_id
        goals_msg.header.frame_id = "world"
        goals_msg.transforms = goals[i]

        goal_publishers[i].publish(goals_msg)

    return True
    

def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate = rospy.Rate(5) # 20hz
    rate_wait_15s = rospy.Rate(1.0/15.0) 

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

    z = 1.25

    print("Publishing 1st set of goals ")

    goals_0 = [] # OG (1.6, -1.6) -> (-1.6, 1.6)  
    goals_1 = [] # OG (2.25, 0.0) -> (-2.25, 0.0) 
    goals_2 = [] # OG (1.6, 1.6) -> (-1.6, -1.6)  

    goals_0.append(create_transform(-1.6, 1.45, z))
    goals_1.append(create_transform(-2.25, 0.0, z+0.125))
    goals_2.append(create_transform(-1.6, -1.45, z-0.125))

    goals_0.append(create_transform(1.6, -1.45, z))
    goals_1.append(create_transform(2.25, 0.0, z+0.125))
    goals_2.append(create_transform(1.6, 1.45, z-0.125))

    pub_goals([goals_0, goals_1, goals_2])

    while not rospy.is_shutdown():
        get_current_task_id()
        if check_task_ids(1):
            break

    print("Publishing 2nd set of goals")
    pub_goals([goals_0, goals_1, goals_2])

    while not rospy.is_shutdown():
        get_current_task_id()
        if check_task_ids(2):
            break

    print("Publishing 3rd set of goals")
    pub_goals([goals_0, goals_1, goals_2])

    while not rospy.is_shutdown():
        get_current_task_id()
        if check_task_ids(3):
            break

    #################
    # ANTIPODAL SWAP
    #################

    print("Going to antipodal swap start positions")
    
    ap_start_0 = [] 
    ap_start_1 = []
    ap_start_2 = []

    ap_start_0.append(create_transform(2.0, -2.0, z))
    ap_start_1.append(create_transform(-2.15, -1.6, z+0.125))
    ap_start_2.append(create_transform(1.6, 2.15, z-0.125))

    pub_goals([ap_start_0, ap_start_1, ap_start_2])

    while not rospy.is_shutdown():
        get_current_task_id()
        if check_task_ids(4):
            break

    ap_swap_0 = [] 
    ap_swap_1 = []
    ap_swap_2 = []

    ap_swap_0.append(create_transform(-2.0, 2.0, z))
    ap_swap_1.append(create_transform(2.15, 1.6, z+0.125))
    ap_swap_2.append(create_transform(-1.6, -2.15, z-0.125))

    ap_swap_0.append(create_transform(2.0, -2.0, z))
    ap_swap_1.append(create_transform(-2.15, -1.6, z+0.125))
    ap_swap_2.append(create_transform(1.6, 2.15, z-0.125))

    print("Executing 1st antipodal swap")

    pub_goals([ap_swap_0, ap_swap_1, ap_swap_2])

    while not rospy.is_shutdown():
        get_current_task_id()
        if check_task_ids(5):
            break

    print("Executing 2nd antipodal swap")

    pub_goals([ap_swap_0, ap_swap_1, ap_swap_2])

    while not rospy.is_shutdown():
        get_current_task_id()
        if check_task_ids(6):
            break

    #################
    # GO HOME AND LAND
    #################

    home0 = [] # OG (1.6, -1.6) -> (-1.6, 1.6)  
    home1 = [] # OG (2.25, 0.0) -> (-2.25, 0.0) 
    home2 = [] # OG (1.6, 1.6) -> (-1.6, -1.6)  

    home0.append(create_transform(1.6, -1.45, z))
    home1.append(create_transform(2.25, 0.0, z+0.125))
    home2.append(create_transform(1.6, 1.45, z-0.125))

    pub_goals([home0, home1, home2])


    while not rospy.is_shutdown():
        get_current_task_id()
        if check_task_ids(7):
            break

    while not rospy.is_shutdown():
        publish_swarm_cmd(1) # Land 
        print("PUBLISHING COMMAND TO LAND DRONES")
        rospy.sleep(1)

    print("End of mission script")

if __name__ == '__main__':
    main()