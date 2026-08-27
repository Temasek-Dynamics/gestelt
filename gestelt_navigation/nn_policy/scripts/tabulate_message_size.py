import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import AttitudeTarget
from quadrotor_msgs.msg import PositionCommand
from sensor_msgs.msg import Imu
import io  # use io.BytesIO in Python 3, cStringIO.StringIO in Python 2

# Create the message
msg = PositionCommand()
msg.header.frame_id = "map"
# msg.pose.position.x = 1.0
# msg.pose.orientation.w = 1.0

# Serialize using io.BytesIO
buf = io.BytesIO()
msg.serialize(buf)
serialized_data = buf.getvalue()

print(f"Serialized size: {len(serialized_data)} bytes")
