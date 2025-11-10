import rospkg
from nav_msgs.msg import Path
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np


class Listener(object):
    def __init__(self):
        self.pose_subscriber = rospy.Subscriber("/drone0/mavros/local_position/pose", PoseStamped, self.poseCB, queue_size=5)
        self.path_publisher = rospy.Publisher("/drone0/drone_path", Path, queue_size=5)
        self.path_timer = rospy.Timer(rospy.Duration(0.02), self.eventCB)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

    def poseCB(self, msg):
        self.pose = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.path_msg.poses.append(msg)


    def eventCB(self, event):
        self.path_publisher.publish(self.path_msg)



if __name__=="__main__":
    rospy.init_node("listener")
    listener = Listener()
    rospy.spin()