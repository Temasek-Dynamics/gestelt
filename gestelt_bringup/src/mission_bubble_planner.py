#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty 

# Publisher of server events to trigger change of states for trajectory server 
dbg_start_pub = rospy.Publisher('/drone0/debug/plan_start', Pose, queue_size=5)
dbg_goal_pub = rospy.Publisher('/drone0/debug/plan_goal', Pose, queue_size=5)
plan_on_demand_pub_ = rospy.Publisher('/drone0/plan_on_demand', Empty, queue_size=5)

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

    start = create_pose(0.0, 0.0, 1.0)
    goal = create_pose(10, 5.0, 1.0)

    rate.sleep()
    dbg_start_pub.publish(start)
    rate.sleep()
    dbg_goal_pub.publish(goal)  
    rate.sleep()
    plan_on_demand_pub_.publish(Empty())
    print(f"Script complete")

if __name__ == '__main__':
    main()