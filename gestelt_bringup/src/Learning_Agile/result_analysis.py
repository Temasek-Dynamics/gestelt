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
                          gate_pitch=None,
                          seprate_plot=False):

    # if uav_traj is None:
    sim_file = os.path.join(current_dir, 'python_sim_result/uav_traj.npy')
    nn_output_file = os.path.join(current_dir, 'python_sim_result/nn_output_list.npy')
    gate_pitch_file = os.path.join(current_dir, 'python_sim_result/Pitch.npy')
    
    uav_traj = np.load(sim_file)
    nn_output_list = np.load( nn_output_file)
    gate_pitch = np.load(gate_pitch_file)
    
    # == convert the quaternion to euler angles == ##
    quat = R.from_quat(uav_traj[::5, 6:])
    euler = quat.as_euler('zyx', degrees=True)
    rot_vec = quat.as_rotvec()
    
    
    ## == covert nn output Rodrigues to euler angles == ##
    RP=R.from_rotvec(nn_output_list[:, 3:6])
    euler_nn=RP.as_euler('zyx', degrees=True)
    rot_vec_nn = RP.as_rotvec()
    
    
    ## == convert the gate pitch start from horizontal == ##
    gate_pitch[:] = np.degrees(gate_pitch[:])
  
    ## == traversing time == ##
    t_tra = np.where(nn_output_list[:, 6] < 0)[0][0]
    # plot the euler angles
    plt.figure(figsize=(10, 5))
    plt.axvline(x=t_tra, color='r', linestyle='--', label='traverse time')

    plt.plot(euler[:, 0], label='Roll')
    plt.plot(euler[:, 1], label='Pitch')

    plt.plot(euler_nn[:, 0], label='NN_Roll')
    plt.plot(euler_nn[:, 1], label='NN_Pitch')
    

    plt.plot(gate_pitch[:] , label='Gate_Pitch')
    # plt.plot(euler[:, 2], label='Yaw')
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
    # plt.plot(rot_vec[:, 0], label='x_rot_vec')
    # plt.plot(rot_vec[:, 1], label='y_rot_vec')

    # plt.plot(rot_vec_nn[:, 0], label='NN_x_rot_vec')
    # plt.plot(rot_vec_nn[:, 1], label='NN_y_rot_vec')
    plt.legend()
    

    if seprate_plot: 
        plt.show()    
    


def plot_reward():
    reward_file = os.path.join(current_dir, 'training_data/mean_reward.npy')
    reward_data = np.load(reward_file)
    # reward_data.resize((:,))
    # 绘制奖励图
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