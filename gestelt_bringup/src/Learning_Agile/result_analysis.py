import numpy as np
import os
import matplotlib.pyplot as plt
from quad_model import Quadrotor
from scipy.spatial.transform import Rotation as R
# Load the data
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

def python_sim_npy_parser(uav_traj=None,
                          nn_output_list=None,
                          des_tra_R_list=None,
                          gate_pitch=None,
                          seprate_plot=False):

    # if uav_traj is None:
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
    quat = R.from_quat(uav_traj[::5, 6:])
    euler_drone = quat.as_euler('zyx', degrees=True)
    rot_vec = quat.as_rotvec()
    
    
    ## == covert nn output Rodrigues to euler angles == ##
    # quat_nn=R.from_quat(nn_output_list[:, 3:7])
    # euler_nn=quat_nn.as_euler('zyx', degrees=True)
    # rot_vec_nn = quat_nn.as_rotvec()
    

    ## == covert nn output rotation matrix to euler angles == ##
    quat_nn=R.from_matrix(des_tra_R_list[:, 0:9].reshape(-1,3,3))
    euler_nn=quat_nn.as_euler('zyx', degrees=True)
    rot_vec_nn = quat_nn.as_rotvec()
    
    ## == convert the gate pitch start from horizontal == ##
    gate_pitch[:] = np.degrees(gate_pitch[:])
  
    ## == traversing time == ##
    t_tra = np.where(nn_output_list[:, -1] < 0)[0][0]
    
    
    ## ==== plot nn and actual euler angles ==##
    plt.figure(figsize=(10, 5))
    plt.axvline(x=t_tra, color='r', linestyle='--', label='traverse time')

    plt.plot(euler_drone[:, 0], label='drone_Roll')
    plt.plot(euler_drone[:, 1], label='drone_Pitch')

    plt.plot(euler_nn[:, 0], label='NN_Roll')
    plt.plot(euler_nn[:, 1], label='NN_Pitch')
    

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

    plt.figure()
    plt.plot(np.linalg.det(nn_output_list[:,3:12].reshape(-1,3,3)), label='9d_vector_determinant',color='r')
    plt.plot(np.linalg.det(des_tra_R_list[:, 0:9].reshape(-1,3,3)), label='SVD_result_determinant',color='b')
    plt.legend()
    plt.grid(True)
    if seprate_plot: 
        plt.show()    
    
    

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
    python_sim_npy_parser(seprate_plot=True)