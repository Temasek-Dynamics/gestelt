#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import time

class OdomDebug(object):

    def __init__(self):

        self.last_header_stamp = None
        self.last_arrival = None
        self.last_seq = None

        topic = "/drone0/mavros/local_position/odom"

        rospy.Subscriber(
            topic,
            Odometry,
            self.cb,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.loginfo("Listening on %s (tcp_nodelay=True)", topic)

    def cb(self, msg):

        arrival = rospy.Time.now()

        if self.last_header_stamp is not None:

            header_dt = (msg.header.stamp - self.last_header_stamp).to_sec()
            arrival_dt = (arrival - self.last_arrival).to_sec()

            seq_diff = msg.header.seq - self.last_seq

            stamp_age = (arrival - msg.header.stamp).to_sec()

            if (
                header_dt > 0.02 or
                arrival_dt > 0.02 or
                seq_diff != 1
            ):
                print("=" * 80)
                print("SEQ           : {} -> {}   (+{})".format(
                    self.last_seq,
                    msg.header.seq,
                    seq_diff
                ))
                print("HEADER DT     : {:.6f} s".format(header_dt))
                print("ARRIVAL DT    : {:.6f} s".format(arrival_dt))
                print("STAMP AGE     : {:.6f} s".format(stamp_age))
                print("HEADER STAMP  : {:.6f}".format(msg.header.stamp.to_sec()))
                print("ARRIVAL TIME  : {:.6f}".format(arrival.to_sec()))
                print("=" * 80)

        self.last_header_stamp = msg.header.stamp
        self.last_arrival = arrival
        self.last_seq = msg.header.seq


if __name__ == "__main__":

    rospy.init_node("odom_debug_tcp")

    OdomDebug()

    rospy.spin()