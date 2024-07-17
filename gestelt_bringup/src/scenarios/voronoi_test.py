#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped

start_dbg_pub = rospy.Publisher('/start_dbg', PoseStamped, queue_size=2)
goal_dbg_pub = rospy.Publisher('/goal_dbg', PoseStamped, queue_size=2)

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

def main():
    rospy.init_node('mission_startup', anonymous=True)
    rate_wait = rospy.Rate(0.5) 
    rate_pub_pause = rospy.Rate(5) # 0.2 seconds

    print(f"Sending waypoints to UAVs")

    rate_wait.sleep()

    start_dbg_pub.publish(create_pose(-7.0, -7.0, 1.1))
    rate_pub_pause.sleep()
    goal_dbg_pub.publish(create_pose(7.0, 7.0, 2.2))

if __name__ == '__main__':
    main()