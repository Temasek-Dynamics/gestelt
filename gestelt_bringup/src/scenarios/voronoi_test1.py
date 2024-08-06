#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
from gestelt_msgs.msg import PlanRequestDebug

start_dbg_pub = rospy.Publisher('/plan_request_dbg', PlanRequestDebug, queue_size=10, latch=True)

def create_pose(x, y, z):
    pos = Pose()
    pos.position.x = x
    pos.position.y = y
    pos.position.z = z

    pos.orientation.x = 0
    pos.orientation.y = 0
    pos.orientation.z = 0
    pos.orientation.w = 0

    return pos

def main():
    rospy.init_node('mission_startup', anonymous=True)
    pause_4s = rospy.Rate(0.25) # 2 seconds
    pause_short = rospy.Rate(2) # 0.5 seconds

    pause_4s.sleep()

    print(f"Sending waypoints to UAVs")

    plan_req_msg = PlanRequestDebug()
    plan_req_msg.start = create_pose(-7.0, -7.0, 2.2)
    plan_req_msg.goal = create_pose(7.0, 7.0, 1.0)
    plan_req_msg.agent_id = 0

    start_dbg_pub.publish(plan_req_msg)

    pause_short.sleep()

    plan_req_msg.start = create_pose(-6.5, -6.5, 2.2)
    plan_req_msg.goal = create_pose(7.0, 7.0, 1.0)
    plan_req_msg.agent_id = 1

    start_dbg_pub.publish(plan_req_msg)


if __name__ == '__main__':
    main()