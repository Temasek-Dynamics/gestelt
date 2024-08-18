#!/usr/bin/env python3

import rospy
from gestelt_msgs.msg import Command, CommanderState, Goals
from geometry_msgs.msg import Transform, PoseStamped
from std_msgs.msg import Int8

NUM_DRONES = 10

# Publisher of server events to trigger change of states for trajectory server 
goal_publishers = []
for i in range(0, NUM_DRONES):
    goal_publishers.append(rospy.Publisher(f'/drone{i}/goals', Goals, queue_size=5))

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


def pub_goals(goals):
    if (len(goals) > NUM_DRONES):
        print(f"Failed to publish goals. No. of goals ({len(goals)}) > NUM_DRONES ({NUM_DRONES})")
        return False 

    for i in range(0, len(goals)):
        goals_msg = Goals()
        goals_msg.header.frame_id = "world"
        goals_msg.transforms = goals[i]

        goal_publishers[i].publish(goals_msg)

    return True

def main():
    rospy.init_node('mission_startup', anonymous=True)
    wait_for_init = rospy.Rate(0.5) # 20hz

    wait_for_init.sleep()

    # Send waypoints to UAVs
    print(f"Sending waypoints to UAVs")

    a = 2.0
    b = 2.0
    z = 1.0 # Height

    goals_0 = []
    goals_1 = []
    goals_2 = []
    goals_3 = []
    goals_4 = []
    goals_5 = []
    goals_6 = []
    goals_7 = []
    goals_8 = []
    goals_9 = []

    goals_0.append(create_transform(-a-b, -a, z))
    goals_1.append(create_transform(-a-b, a, z))
    goals_2.append(create_transform(-a, a+b, z))
    goals_3.append(create_transform(a, a+b, z))

    goals_4.append(create_transform(a+b, a, z))
    goals_5.append(create_transform(a+b, -a, z))
    goals_6.append(create_transform(a, -a-b, z))
    goals_7.append(create_transform(-a, -a-b, z))

    goals_8.append(create_transform(-a-b, a+b, z))
    goals_9.append(create_transform(a+b, a+b, z))

    pub_goals([ goals_0, goals_1, goals_2, goals_3, 
                goals_4, goals_5, goals_6, goals_7,
                goals_8, goals_9])

if __name__ == '__main__':
    main()