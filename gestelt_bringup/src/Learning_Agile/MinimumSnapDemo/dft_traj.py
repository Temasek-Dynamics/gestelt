import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D 
import numpy as np
from .minsnap_traj import minimum_snap_traj, minimum_snap_traj_p2p, get_traj
from scipy.spatial.transform import Rotation


def plot_3D_xyz(traj):
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.scatter(traj[0], traj[1], traj[2],marker = 'x',color = 'red', s = 2 ,label = 'desire')
    ax.set_zlabel('Z', fontdict={'size': 15, 'color': 'red'})
    ax.set_ylabel('Y', fontdict={'size': 15, 'color': 'red'})
    ax.set_xlabel('X', fontdict={'size': 15, 'color': 'red'})
    plt.show()


def plot_1D_time(D3_traj,t_list):
    fig = plt.figure()
    plt.scatter(t_list[1:], D3_traj[0], marker = 'x', color = 'blue', s = 2, label = '0')
    plt.scatter(t_list[1:], D3_traj[1], marker = 'x', color = 'red', s = 2, label = '1')
    plt.scatter(t_list[1:], D3_traj[2], marker = 'x', color = 'green', s = 2, label = '2')
    plt.show()    


def differential_flatness_transform(p, v, a):
    psi = 0 # no psi plan
    g = np.array([0, 0, -9.8])
    x_c = np.array([np.cos(psi), np.sin(psi), 0])
    R_list = []
    for i in range(a.shape[1]):
        t = a[:,i] - g
        t_norm = np.linalg.norm(t)
        z_b = t/t_norm
        y_b = np.cross(z_b, x_c)
        x_b = np.cross(y_b, z_b)
        R = np.matrix([x_b, y_b, z_b])
        R_list.append(R)
    return R_list 

def simple_dft(a):
    psi = 0
    g = np.array([0, 0, -9.8])
    x_c = np.array([np.cos(psi), np.sin(psi), 0])
    t = a - g
    t_norm = np.linalg.norm(t)
    z_b = t/t_norm
    y_b = np.cross(z_b, x_c)
    x_b = np.cross(y_b, z_b)
    R = np.matrix([x_b, y_b, z_b])
    return R

def R_to_quat(R):
    """transform rotation matrix to quaternion q=(qw, qx, qy, qz)"""
    q=[]
    for i in range(len(R)):
        r=Rotation.from_matrix(R[i])
        q_i=r.as_quat()
        q_i = np.roll(q_i, 1) # roll the array to match the order (qw, qx, qy, qz)
        q.append(q_i)
        
    return q
   
def plot_trajectory_with_velocity_and_orientation(p, v, q):
    """
    绘制轨迹，颜色根据速度大小变化，同时显示姿态方向
    :param p: 位置列表，形状为 (3, N)
    :param v: 速度列表，形状为 (3, N)
    :param q: 四元数列表，形状为 (N, 4)
    """
    # 确保 p 和 v 是 NumPy 数组
    p = np.array(p[:3])
    v = np.array(v[:3])

    # ignore the last element of p and v for plotting
    p = p[:, :-1]
    v = v[:, :-1]
    # 计算速度的大小
    speed = np.linalg.norm(v, axis=0)
    print("speed:", speed)
    print("max speed:", np.max(speed))
    # 归一化速度到 [0, 1]，用于颜色映射
    norm_speed = (speed - np.min(speed)) / (np.max(speed) - np.min(speed))

    # 创建颜色映射（蓝色到红色）
    cmap = plt.cm.get_cmap("coolwarm")

    # 创建图形
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # 绘制轨迹，颜色根据速度变化
    for i in range(len(p[0]) - 1):
        ax.plot(
            p[0, i:i+2], p[1, i:i+2], p[2, i:i+2],
            color=cmap(norm_speed[i]), linewidth=2
        )

    # 绘制姿态（方向箭头）
    for i in range(0, len(q), max(1, len(q) // 50)):  # 每隔一定数量绘制一个箭头，避免过多箭头
        # 提取四元数并转换为旋转矩阵
        r = Rotation.from_quat([q[i][1], q[i][2], q[i][3], q[i][0]])  # (qx, qy, qz, qw)
        rot_matrix = r.as_matrix()

        # 提取箭头方向
        start = np.array([p[0][i], p[1][i], p[2][i]])  # 起点
        x_dir = rot_matrix[:, 0]  # x 轴方向
        y_dir = rot_matrix[:, 1]  # y 轴方向
        z_dir = rot_matrix[:, 2]  # z 轴方向

        # 绘制箭头
        ax.quiver(start[0], start[1], start[2], x_dir[0], x_dir[1], x_dir[2], color='red', length=0.2, alpha=0.2, normalize=True)
        ax.quiver(start[0], start[1], start[2], y_dir[0], y_dir[1], y_dir[2], color='green', length=0.2,alpha=0.2, normalize=True)
        ax.quiver(start[0], start[1], start[2], z_dir[0], z_dir[1], z_dir[2], color='blue', length=0.2,alpha=0.2, normalize=True)

    # 设置颜色条
    sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=np.min(speed), vmax=np.max(speed)))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, pad=0.1)
    cbar.set_label('Speed', rotation=270, labelpad=15)

    # 设置轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    ax.set_xlim([-3,3])
    ax.set_ylim([-3,3])
    ax.set_zlim([ 0,2])
    ax.set_title('Trajectory with Velocity-based Color and Orientation')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    way_points = np.array([[0.0, 1.8, 1.2, 0.0],\
                           [2.0, 0.0, 1.8, 0.0],\
                           [0.0,-1.8, 1.5, 0.0]]) # x,y,z,psi
    time_set = np.array([0,0.9,1.4])
    n_order = 5
    n_obj = 3
    v_i = [0,0,0,0]
    a_i = [0,0,0,0]
    v_e = [0,0,0,0]
    a_e = [0,0,0,0]
    sample_rate = 100
    Matrix_x, Matrix_y, Matrix_z = minimum_snap_traj_p2p(way_points, time_set, n_order, n_obj, v_i, a_i, v_e, a_e)
    p, v, a, t_list= get_traj(Matrix_x, Matrix_y, Matrix_z, time_set, sample_rate)
    R = differential_flatness_transform(np.array(p), np.array(v), np.array(a))
    q = R_to_quat(R)
    # plot_3D_xyz(p)
    # plot_1D_time(p,t_list)
    plot_trajectory_with_velocity_and_orientation(p, v, q)
    traj_data = {
        "t_list": np.array(t_list),
        "p": np.array(p).transpose(),
        "v": np.array(v).transpose(),
        "a": np.array(a).transpose(),
        "q": np.array(q),
        "R": np.array(R),
    }
    np.set_printoptions(precision=1, suppress=True)
    # print("t_list:", traj_data["t_list"])
    # print("p:", traj_data["p"]) 
    np.save("demo_traj.npy", traj_data)
    
    