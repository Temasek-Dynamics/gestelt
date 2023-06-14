#!/usr/bin/env python3

import rospy
from trajectory_server_msgs.msg import State, Waypoints
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Int8, Empty

set_goal_pub = rospy.Publisher('/plan_set_goal', PoseStamped, queue_size=10)
set_start_pub = rospy.Publisher('/plan_set_start', PoseStamped, queue_size=10)
trigger_pub = rospy.Publisher('/trigger_plan', Empty, queue_size=10)

def create_posestamped(x, y, z):
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 1

    return pose

def main():
    rospy.init_node('test_global_planner', anonymous=True)

    start_pose = create_posestamped(0.0, 0.0, 1.0)
    goal_pose = create_posestamped(1.0, 1.0, 1.0)

    set_start_pub.publish(start_pose)
    set_goal_pub.publish(goal_pose)

    trigger_pub.publish(Empty())

    print(f"Requested for plan from global planner")

if __name__ == '__main__':
    main()