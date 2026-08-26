import rospy
from sensor_msgs.msg import Image as SImage
from cv_bridge import CvBridge, CvBridgeError

class ImageTest(object):
    def __init__(self):
        self._cv_bridge = CvBridge()
        self.depth_sub = rospy.Subscriber(f"/agent001/stereo_left_depth_no_agents", SImage, self.depthCallback, queue_size=10)

    def depth_norm(self, depth):
        depth = (((depth - 0)/ (255 - 0)) * 30 ) 
        return depth
    
    def depthCallback(self,msg):
        # self.depth_global_to_body = np.eye(4)
        # self.depth_body_to_global = np.eye(4)
        self.depth_unnorm = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth = self.depth_norm(self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough"))
        import matplotlib.pyplot as plt
        plt.imshow(self.depth)
        plt.show()
        print("Depth image received")


if __name__ == "__main__":
    rospy.init_node("test")
    imagetest = ImageTest()

    rospy.spin()