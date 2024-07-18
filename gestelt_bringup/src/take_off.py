#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8

traj_server_cmd_pub = rospy.Publisher(f'/traj_server/swarm_command', Int8, queue_size=5)

def publish_server_cmd(event_enum):
    msg = Int8()
    msg.data = event_enum
    # Publisher of server events to trigger change of states for trajectory server 
    traj_server_cmd_pub.publish(msg)

def main():
    rospy.init_node('take_off', anonymous=True)

    while not rospy.is_shutdown():
        publish_server_cmd(0) # Take off 
        # publish_server_cmd(1) # Land 
        rospy.sleep(1)

if __name__ == '__main__':
    main()