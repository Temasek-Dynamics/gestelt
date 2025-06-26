
import numpy as np
import rosbag
import argparse
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion
plt.rcParams['font.family'] = 'Times New Roman'
# plt.rcParams['text.usetex'] = True  # 启用 LaTeX
def extract_pose_data(bag_path, topic_name):
    """ Extracts position and orientation data from a ROS bag file."""
    bag = rosbag.Bag(bag_path)
    times = []
    positions = []
    orientations = []

    for _, msg, t in bag.read_messages(topics=[topic_name]):
        times.append(t.to_sec())

        # Position
        pos=np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        positions.append(pos)

        # Orientation (convert quaternion to roll, pitch, yaw)
        q = msg.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        roll, pitch, yaw = euler_from_quaternion(quat)
        ori_euler = np.array([roll, pitch, yaw])
        orientations.append(ori_euler)
    bag.close()
    return np.array(times), np.array(positions), np.array(orientations)

def extract_vel_data(bag_path, topic_name):
    """ Extracts velocity data from a ROS bag file."""
    bag = rosbag.Bag(bag_path)
    times = []
    velocities = []

    for _, msg, t in bag.read_messages(topics=[topic_name]):
        times.append(t.to_sec())

        # Velocity
        vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])
        velocities.append(vel)

    bag.close()
    return np.array(times), np.array(velocities)

def extract_NN_weights(bag_path, topic_name):
    """ Extracts neural network weights from a ROS bag file."""
    bag = rosbag.Bag(bag_path)
    times = []
    weights_vector = []

    for _, msg, t in bag.read_messages(topics=[topic_name]):
        times.append(t.to_sec())
        for i in range(len(msg.weight_vector)):
            weight_vector = np.array(msg.weight_vector)
        weights_vector.append(weight_vector)

    bag.close()
    return np.array(times), np.array(weights_vector)

def preprocess(times,pos, vel, vy_lower, z_lower):
    """
    Resets the initial time to the point where y position drops below vy_lower.
    """
    for i, y in enumerate(vel[:, 1]):
        if y < vy_lower and pos[i, 2] > z_lower:
            t_init = times[i]
            print(f"[INFO] Resetting time using t_init = {t_init:.3f} (y = {y:.3f})")
            times = [t - t_init for t in times]
            return t_init


def plot_pose(
    axes,
    times,
    times_NN,
    t_init,
    positions, 
    orientations,
    NN_positions,
    NN_orientations
):
    # Position plot
    # plt.figure(figsize=(4, 3))
    mask_start = times[:] > t_init
    mask_end = times[:] < t_init + 2.5
    mask = mask_start & mask_end
    
    # shift times to start from t_init
    times = times - t_init
    axes[0,0].plot(times[mask], positions[mask, 0], label='x')
    axes[0,0].plot(times[mask], positions[mask, 1], label='y')
    axes[0,0].plot(times[mask], positions[mask, 2], label='z')
    
    # Plot the NN positions aa dashes
    mask_NN_start = times_NN[:] > t_init
    mask_NN_end = times_NN[:] < t_init + 2.5
    mask_NN = mask_NN_start & mask_NN_end   
    times_NN = times_NN - t_init
    axes[0,0].plot(times_NN[mask_NN], NN_positions[mask_NN, 0], '--', color='C0', label='NN x')
    axes[0,0].plot(times_NN[mask_NN], NN_positions[mask_NN, 1], '--', color='C1', label='NN y')
    axes[0,0].plot(times_NN[mask_NN], NN_positions[mask_NN, 2], '--', color='C2', label='NN z')
    #plt.title("Quadrotor actual Position VS Neural Network(NN) decided position (m)")
    # axes[0,0].set_xlabel("Time (s)")
    axes[0,0].set_ylabel("Position")
    axes[0,0].legend()
    axes[0,0].grid(True)
    # plt.tight_layout()
    # plt.savefig("diagram_position_plot.pdf", dpi=300)
    # plt.show()

    # Orientation plot
    # plt.figure(figsize=(4, 3))
    axes[0,1].plot(times[mask], orientations[mask, 0], label='roll')
    axes[0,1].plot(times[mask], orientations[mask, 1], label='pitch')
    axes[0,1].plot(times[mask], orientations[mask, 2], label='yaw')
    
    # Plot the NN orientations as dashes
    axes[0,1].plot(times_NN[mask_NN], NN_orientations[mask_NN, 0], '--', color='C0',  label='NN roll')
    axes[0,1].plot(times_NN[mask_NN], NN_orientations[mask_NN, 1], '--', color='C1',  label='NN pitch')
    axes[0,1].plot(times_NN[mask_NN], NN_orientations[mask_NN, 2], '--', color='C2',  label='NN yaw')
    
    #plt.title("Quadrotor actual orientation VS Neural Network(NN) decided orientation (rad)")
    # axes[0,1].set_xlabel(r"Time (s)")
    axes[0,1].set_ylabel("Euler Angle")
    axes[0,1].legend()
    axes[0,1].grid(True)
    # plt.tight_layout()
    # plt.savefig("diagram_orientation_plot.pdf", dpi=300)
    # plt.show()

def plot_vel(ax,times,t_init, velocities):
    """ Plot velocity data."""
    # plt.figure(figsize=(4, 3))
    mask_start = times[:] > t_init
    mask_end = times[:] < t_init + 2.5
    mask = mask_start & mask_end
    
    # shift times to start from t_init
    times = times - t_init
    ax.plot(times[mask], velocities[mask, 0], label='x_axis')
    ax.plot(times[mask], velocities[mask, 1], label='y_axis')
    ax.plot(times[mask], velocities[mask, 2], label='z_axis')
    #plt.title("Velocity (m/s)")
    # ax.set_xlabel("Time (s)")
    ax.set_ylabel("Velocity")
    ax.legend()
    ax.grid(True)
    # plt.tight_layout()
    # plt.savefig("diagram_velocity_plot.pdf", dpi=300)
    # plt.show()

def plot_NN_weights(axes, times, t_init, weights):   
    """ Plot neural network weights."""
    # plt.figure(figsize=(4, 3))
    mask_start = times[:] > t_init
    mask_end = times[:] < t_init + 2.5
    mask = mask_start & mask_end
    
    # shift times to start from t_init
    times = times - t_init
    axes[1,1].plot(times[mask], weights[mask, 0], label='x_axis')
    axes[1,1].plot(times[mask], weights[mask, 1], label='y_axis')
    axes[1,1].plot(times[mask], weights[mask, 2], label='z_axis')
    
    
    #plt.title("Neural Network decided goal reaching weights")
    # axes[1,1]..xlabel("Time (s)")
    axes[1,1].set_ylabel("goal reaching weight Value")
    axes[1,1].legend()
    axes[1,1].grid(True)
    # plt.tight_layout()
    # plt.savefig("diagram_NN_goal_weights_plot.pdf", dpi=300)
    
    # NN position reach weights
    # plt.figure(figsize=(4, 3))
    axes[2,0].plot(times[mask], weights[mask, 3], label='x_axis')
    axes[2,0].plot(times[mask], weights[mask, 4], label='y_axis')
    axes[2,0].plot(times[mask], weights[mask, 5], label='z_axis')
    
    
    #plt.title("Neural Network decided reference position reaching weights")
    axes[2,0].set_xlabel("Time (s)")
    axes[2,0].set_ylabel("reference position reaching Weight Value")
    axes[2,0].legend()
    axes[2,0].grid(True)
    # plt.tight_layout()
    # plt.savefig("diagram_NN_ref_position_weights_plot.pdf", dpi=300)
    
    # NN orientation weights
    # plt.figure(figsize=(4, 3))
    axes[2,1].plot(times[mask], weights[mask, 6], label='orientation weight')
    axes[2,1].plot(times[mask], weights[mask, 7], label='gamma')    
    
    #plt.title("Neural Network decided orientation reaching weights")
    axes[2,1].set_xlabel("Time (s)")
    axes[2,1].set_ylabel("Weight Value")
    axes[2,1].legend()
    axes[2,1].grid(True)
    # plt.tight_layout()
    # plt.savefig("diagram_NN_ori_weights_plot.pdf", dpi=300)
    # plt.show() 
    
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot geometry_msgs/Pose data from ROS bag")
    parser.add_argument("--bag_path",default="/home/tlab-uav/gestelt_ws/src/gestelt/gestelt_bringup/data/12_June_2025_good_for_paper/REAL_drone_2025-06-12-12-14-36_-1_rad_in_the_center_for_paper.bag",  help="Path to .bag file")
    parser.add_argument("--topic_pose", default="/mavros/local_position/pose", help="Pose topic, e.g. /pose or /odom.pose.pose")
    parser.add_argument("--topic_vel", default="/mavros/local_position/velocity_local", help="Pose topic, e.g. /pose or /odom.pose.pose")
    parser.add_argument("--topic_NN_pose", default="/visual/vis_NN_trav_pose", help="Pose topic, e.g. /pose or /odom.pose.pose")
    parser.add_argument("--topic_NN_weights", default="/learning_agile_sim/NN_output", help="Pose topic, e.g. /pose or /odom.pose.pose")
     
    args = parser.parse_args()

    print(f"Reading pose data from topic '{args.topic_pose}' in '{args.bag_path}'")
    times_pose, positions, orientations = extract_pose_data(args.bag_path, args.topic_pose)
    times_NN_pose, NN_positions, NN_orientations = extract_pose_data(args.bag_path, args.topic_NN_pose)
    times_vel, velocities = extract_vel_data(args.bag_path, args.topic_vel)
    times_weights, weights = extract_NN_weights(args.bag_path, args.topic_NN_weights)
    
    # Preprocess: reset time when y < threshold
    t_init = preprocess(times_pose, positions, velocities, vy_lower=-0.01, z_lower=1.19)
    
    # Plot [2,3] subplots
    plt,axes= plt.subplots(3, 2, figsize=(6, 6), sharex=True, sharey=False)
    plot_pose(axes, times_pose,times_NN_pose, t_init, positions, orientations, NN_positions, NN_orientations)
    plot_vel(axes[1,0],times_vel, t_init, velocities)
    plot_NN_weights(axes,times_weights, t_init, weights)
    plt.tight_layout()
    plt.savefig("diagram_all_plots.pdf", dpi=300)
    plt.show()
