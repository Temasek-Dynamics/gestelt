#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty 

dbg_start_pub = rospy.Publisher('/drone0/navigator/debug/plan_start', Pose, queue_size=5)
dbg_goal_pub = rospy.Publisher('/drone0/navigator/debug/plan_goal', Pose, queue_size=5)
plan_on_demand_pub = rospy.Publisher('/drone0/navigator/plan_on_demand', Empty, queue_size=5)

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

def main():
    rospy.init_node('mission_startup', anonymous=True)
    print(f"Sending waypoints to UAVs")
    rate = rospy.Rate(2) 

    ###############
    # Forest10x10
    ###############
    start = create_pose(-7.0, -7.0, 2.2)
    goal = create_pose(7.0, 7.0, 1.0)
    # start = create_pose(-5.5, -5.5, 1.0)
    # goal = create_pose(5.5, 5.5, 1.0)

    rate.sleep()
    dbg_start_pub.publish(start)
    rate.sleep()
    dbg_goal_pub.publish(goal)
    rate.sleep()
    plan_on_demand_pub.publish(Empty())

    print(f"Mission bubble planner script complete")

if __name__ == '__main__':
    main()