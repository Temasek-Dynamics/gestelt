import rosbag
import numpy as np
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion, quaternion_matrix
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# === User inputs ===
bag_path = "/home/yanrui/2025-09-25-15-52-40.bag"
drone_topic = "/drone0/mavros/local_position/pose"     # or /mavros/local_position/pose

# Window fixed pose
window_position = np.array([1.5, 1.0, 0.5])   # [x, y, z]
tilt_angle = np.deg2rad(60)                   # tilt about x-axis
window_size = [0.8, 0.3]                      # width × height

# === Drone trajectory ===
drone_positions = []
drone_orientations = []  # store quaternions

with rosbag.Bag(bag_path, "r") as bag:
    for topic, msg, t in bag.read_messages(topics=[drone_topic]):
        p = msg.pose.position
        q = msg.pose.orientation

        drone_positions.append([p.x, p.y, p.z])
        drone_orientations.append([q.x, q.y, q.z, q.w])

drone_positions = np.array(drone_positions)
drone_orientations = np.array(drone_orientations)

# === Build window rectangle ===
width, height = window_size
y = np.array([ -width/2,  width/2,  width/2, -width/2 ])
z = np.array([ -height/2, -height/2, height/2,  height/2 ])
x = np.zeros(4)

# Rotate around x-axis
R = np.array([[1, 0, 0],
              [0, np.cos(tilt_angle), -np.sin(tilt_angle)],
              [0, np.sin(tilt_angle),  np.cos(tilt_angle)]])
rect = np.vstack([x, y, z])   # 3×4
rect_rot = R @ rect

# Translate
X = rect_rot[0,:] + window_position[0]
Y = rect_rot[1,:] + window_position[1]
Z = rect_rot[2,:] + window_position[2]

# === Plot ===
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

# Drone trajectory (line)
ax.plot(drone_positions[:,0], drone_positions[:,1], drone_positions[:,2], label="Drone")

# Drone body axes (sampled every N poses)
N = 5   # step between poses to plot axes
axis_len = 0.1

for i in range(0, len(drone_positions), N):
    pos = drone_positions[i]
    quat = drone_orientations[i]

    # Convert quaternion to rotation matrix
    R = quaternion_matrix(quat)[:3, :3]

    # Body axes in world frame
    x_axis = R @ np.array([axis_len, 0, 0])
    y_axis = R @ np.array([0, axis_len, 0])
    z_axis = R @ np.array([0, 0, axis_len])

    # Plot quivers
    ax.quiver(pos[0], pos[1], pos[2], x_axis[0], x_axis[1], x_axis[2], color="r")
    ax.quiver(pos[0], pos[1], pos[2], y_axis[0], y_axis[1], y_axis[2], color="g")
    ax.quiver(pos[0], pos[1], pos[2], z_axis[0], z_axis[1], z_axis[2], color="b")

# Window plane as polygon
verts = [list(zip(X, Y, Z))]
ax.add_collection3d(Poly3DCollection(verts, color="red", alpha=0.5))

ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.legend()
plt.show()