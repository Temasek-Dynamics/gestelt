#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty 

# Publisher of server events to trigger change of states for trajectory server 
dbg_start_esdf_free_pub = rospy.Publisher('/drone0/back_end_planner/debug/plan_start', Pose, queue_size=5)
dbg_goal_esdf_free_pub = rospy.Publisher('/drone0/back_end_planner/debug/plan_goal', Pose, queue_size=5)
plan_on_demand_esdf_free_pub = rospy.Publisher('/drone0/back_end_planner/plan_on_demand/esdf_free', Empty, queue_size=5)

dbg_start_sfc_pub = rospy.Publisher('/drone0/front_end_planner/debug/plan_start', Pose, queue_size=5)
dbg_goal_sfc_pub = rospy.Publisher('/drone0/front_end_planner/debug/plan_goal', Pose, queue_size=5)
plan_on_demand_sfc_pub = rospy.Publisher('/drone0/front_end_planner/plan_on_demand', Empty, queue_size=5)

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
    start = create_pose(0.0, 0.0, 1.0)
    goal = create_pose(5.5, 5.5, 1.0)
    # start = create_pose(-5.5, -5.5, 1.0)
    # goal = create_pose(5.5, 5.5, 1.0)

    ###############
    # Tunnel 2x2x10
    ###############
    # start = create_pose(-1.0, 0.0, 1.0)
    # goal = create_pose(22.0, 2.0, 1.0)

    ###############
    # Window 1x1, 4m side
    ###############
    # start = create_pose(-2.0, 4.5, 1.0)
    # goal = create_pose(2.0, 4.5, 1.0)

    # start = create_pose(-3.0, 1.0, 0.5)
    # goal = create_pose(3.0, 1.0, 0.5)

    # rate.sleep()
    # dbg_start_esdf_free_pub.publish(start)
    # rate.sleep()
    # dbg_goal_esdf_free_pub.publish(goal)
    # rate.sleep()
    # plan_on_demand_esdf_free_pub.publish(Empty())

    rate.sleep()
    dbg_start_sfc_pub.publish(start)
    rate.sleep()
    dbg_goal_sfc_pub.publish(goal)
    rate.sleep()
    plan_on_demand_sfc_pub.publish(Empty())

    print(f"Mission bubble planner script complete")

if __name__ == '__main__':
    main()