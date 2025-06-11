import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
# Load the data
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

def rotation_vis(uav_traj=None,
                nn_output_list=None,
                des_tra_R_list=None,
                gate_pitch=None,
                t_tra_list=None,
                seprate_plot=False):

    if uav_traj is None:
        sim_file = os.path.join(current_dir, 'python_sim_result/uav_traj.npy')
        nn_output_file = os.path.join(current_dir, 'python_sim_result/nn_output_list.npy')
        des_tra_R_file = os.path.join(current_dir, 'python_sim_result/des_tra_R_list.npy')
        gate_pitch_file = os.path.join(current_dir, 'python_sim_result/Pitch.npy')
        
        uav_traj = np.load(sim_file)
        nn_output_list = np.load( nn_output_file)
        des_tra_R_list = np.load(des_tra_R_file)
        gate_pitch = np.load(gate_pitch_file)
    
    nn_output_list[0][3]=1
    # == convert drone state from quaternion to euler angles == ##
    # shaft quat from qw first to qx first
    uav_traj[:, 6:10] = uav_traj[:, [7, 8, 9, 6]]
    quat = R.from_quat(uav_traj[::5, 6:10])
    euler_drone = quat.as_euler('zyx', degrees=True)
    rot_vec = quat.as_rotvec()
    
    
    ## == covert nn output Rodrigues to euler angles == ##
    # quat_nn=R.from_quat(nn_output_list[:, 3:7])
    # euler_nn=quat_nn.as_euler('zyx', degrees=True)
    # rot_vec_nn = quat_nn.as_rotvec()
    

    ## == covert nn output rotation matrix to euler angles == ##
    r=R.from_matrix(des_tra_R_list[:, 0:9].reshape(-1,3,3))
    euler_nn=r.as_euler('zyx', degrees=True)
    
    ## == convert the gate pitch start from horizontal == ##
    gate_pitch[:] = np.degrees(gate_pitch[:])
  
    ## == traversing time == ##
    t_tra = np.where(t_tra_list < 0)[0][0]
    
    
    ## ==== plot nn and actual euler angles ==##
    plt.figure(figsize=(10, 5))
    plt.axvline(x=t_tra, color='r', linestyle='--', label='traverse time')

    plt.plot(euler_drone[:, 0], label='drone_Yaw')
    plt.plot(euler_drone[:, 1], label='drone_Pitch')
    plt.plot(euler_drone[:, 2], label='drone_Roll')

    plt.plot(euler_nn[:, 0], label='NN_Yaw')
    plt.plot(euler_nn[:, 1], label='NN_Pitch')
    plt.plot(euler_nn[:, 2], label='NN_Roll')
    

    plt.plot(gate_pitch[:] , label='Gate_Pitch')
    plt.xlabel('Time')
    plt.grid(True)

    ## == plot yaw == ##
    # plt.figure(figsize=(10, 5))
    # plt.plot(euler[:, 2], label='Yaw')
    # plt.plot(euler_nn[:, 1], label='NN_Yaw')
    # plt.xlabel('Time')
    # plt.grid(True)

    ## == plot as axis angle == ##
    # plt.figure(figsize=(10, 5))
    # plt.axvline(x=t_tra, color='r', linestyle='--', label='traverse time')
    # plt.plot(rot_vec[:, 2], label='x_rot_vec')
    # plt.plot(rot_vec[:, 1], label='y_rot_vec')

    # plt.plot(rot_vec_nn[:, 2], label='NN_x_rot_vec')
    # plt.plot(rot_vec_nn[:, 1], label='NN_y_rot_vec')

    ## == global plt settings == ##
    plt.legend()
    plt.grid(True)
    # plt.savefig('python_sim_result/euler.png')
    plt.figure()
    plt.plot(np.linalg.det(nn_output_list[:,3:12].reshape(-1,3,3)), label='9d_vector_determinant',color='r')
    plt.plot(np.linalg.det(des_tra_R_list[:, 0:9].reshape(-1,3,3)), label='SVD_result_determinant',color='b')
    plt.legend()
    plt.grid(True)
    
    
    # plt each element of nn_output_list[:,3:12]
    plt.figure()
    for i in range(9):
        plt.plot(nn_output_list[:, 3 + i], label=f'NN_output_{i}')
    plt.xlabel('Time')
    plt.ylabel('m elements')
    plt.title('NN Output m Elements')
    plt.grid(True)
    
    
   
    # if seprate_plot: 
    plt.show()    
    
    # 3d matrix row vectors
    animate_matrix_rows_3d(
        nn_output_list[::5,3:12], 
        des_tra_R_list[::5,0:9],
        save_path=os.path.join(current_dir, '../python_sim_result/nn_output_animation.gif')
    )
    
    return euler_nn



def animate_matrix_rows_3d(nn_outputs, des_Rs, interval=200, save_path=None):
    """
    Animate the row vectors of two sequences of 3x3 matrices in 3D.
    Args:
        nn_outputs: np.ndarray, shape (N, 9)
        des_Rs: np.ndarray, shape (N, 9)
        interval: int, delay between frames in ms
        save_path: str or None, if set, save the animation as mp4
    """
    assert nn_outputs.shape == des_Rs.shape
    N = nn_outputs.shape[0]

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    colors_m = ['r', 'g', 'b']
    colors_R = ['m', 'c', 'y']
    origin = np.zeros(3)

    # Prepare lines for updating
    lines = []
    for i in range(3):
        # NN output
        line_m, = ax.plot([0, 0], [0, 0], [0, 0], color=colors_m[i], label=f'NN row {i}')
        # SVD result
        line_R, = ax.plot([0, 0], [0, 0], [0, 0], color=colors_R[i], linestyle='dashed', label=f'SVD row {i}')
        lines.append((line_m, line_R))

    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("NN Output vs SVD Result (Animated)")
    ax.legend()

    def update(frame):
        m = nn_outputs[frame].reshape(3, 3)
        R = des_Rs[frame].reshape(3, 3)
        for i in range(3):
            # NN output vector
            vec_m = m[i]
            xs = np.array([origin[0], vec_m[0]])
            ys = np.array([origin[1], vec_m[1]])
            zs = np.array([origin[2], vec_m[2]])
            lines[i][0].set_data(xs, ys)
            lines[i][0].set_3d_properties(zs)
            # SVD result vector
            vec_R = R[i]
            xs_R = np.array([origin[0], vec_R[0]])
            ys_R = np.array([origin[1], vec_R[1]])
            zs_R = np.array([origin[2], vec_R[2]])
            lines[i][1].set_data(xs_R, ys_R)
            lines[i][1].set_3d_properties(zs_R)
        ax.set_title(f"Frame {frame+1}/{N}")
        return sum(lines, ())

    ani = FuncAnimation(fig, update, frames=N, interval=interval, blit=False)
    if save_path:
        ani.save(save_path, writer='pillow', fps=60)
    else:
        plt.show()

# Example usage:
# m = np.eye(3)
# R = np.array([[0,1,0],[1,0,0],[0,0,1]])
# plot_two_matrix_rows_3d(m, R)

# Example usage:
# mat = np.eye(3)
# plot_matrix_rows_3d(mat)

def plot_reward():
    reward_file = os.path.join(current_dir, 'training_data/mean_reward.npy')
    reward_data = np.load(reward_file)
    

    plt.figure(figsize=(10, 5))
    plt.plot(reward_data, label='Reward')
    plt.ylim(-30, 0)
    plt.xlabel('Episode')
    plt.ylabel('Reward')
    plt.title('Reward per Episode')
    plt.legend()
    plt.grid(True)
    plt.show()

if __name__ == '__main__':
    rotation_vis(seprate_plot=True)