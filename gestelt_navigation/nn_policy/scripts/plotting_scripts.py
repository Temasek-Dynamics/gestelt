import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.ticker as ticker


def gate_geometry(gate_angle_x, gate_center=np.array([1.5, 0.0, 1.0])):
    # --------------------------
    # Gate geometry (rotate about X)
    # --------------------------
    w_gate, h_gate = 1.0, 1.0

    gate_local = np.array([
        [0, -w_gate/2, -h_gate/2],
        [0,  w_gate/2, -h_gate/2],
        [0,  w_gate/2,  h_gate/2],
        [0, -w_gate/2,  h_gate/2],
        [0, -w_gate/2, -h_gate/2]  # close loop
    ])

    R_gate = np.array([
        [1, 0, 0],
        [0, np.cos(gate_angle_x), -np.sin(gate_angle_x)],
        [0, np.sin(gate_angle_x),  np.cos(gate_angle_x)]
    ])

    gate_world = (R_gate @ gate_local.T).T + gate_center  # shape (5,3)

    return gate_world

def set_axes_equal(ax):
    """Make axes of 3D plot have equal scale so spheres appear as spheres, cubes as cubes, etc."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot radius is half the max range of all axes
    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

def quat_to_rotmat(q):
    """
    q: (..., 4) quaternion in (w, x, y, z)
    returns: (..., 3, 3)
    """
    w, x, y, z = q[...,0], q[...,1], q[...,2], q[...,3]

    R = np.zeros(q.shape[:-1] + (3, 3))

    R[...,0,0] = 1 - 2*(y*y + z*z)
    R[...,0,1] = 2*(x*y - z*w)
    R[...,0,2] = 2*(x*z + y*w)

    R[...,1,0] = 2*(x*y + z*w)
    R[...,1,1] = 1 - 2*(x*x + z*z)
    R[...,1,2] = 2*(y*z - x*w)

    R[...,2,0] = 2*(x*z - y*w)
    R[...,2,1] = 2*(y*z + x*w)
    R[...,2,2] = 1 - 2*(x*x + y*y)

    return R

def quat_to_rotmat(q):
    # q = [x, y, z, w]
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])


def plot_spatial_plots(arr):
    T, N, _ = arr.shape
    plt.figure(figsize=(8,6))
    for i in range(N):
        plt.plot(arr[:, i, 0], arr[:, i, 1], linewidth=1, label=f'Traj {i+1}')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('All Trajectories (X vs Y)')
    plt.axis('equal')
    ax = plt.gca()
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1.5))
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.2))
    plt.grid(True)
    # plt.legend()  # optional, may be cluttered
    # plt.show()
    plt.savefig("trajectory_xy.png", dpi=300)  # optional: save figure

def plot_3D_plots(arr):
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')

    for i in range(N):
        ax.plot(arr[:, i, 0], arr[:, i, 1], arr[:, i, 2], label=f'Traj {i+1}', linewidth=1)
        # optional: mark start and end
        ax.scatter(arr[0, i, 0], arr[0, i, 1], arr[0, i, 2], color='green', s=20)
        ax.scatter(arr[-1, i, 0], arr[-1, i, 1], arr[-1, i, 2], color='red', s=20)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('All Trajectories (3D)')
    ax.yaxis.set_major_locator(ticker.MultipleLocator(0.2))
    # ax.legend()  # optional, may be cluttered for 20 trajectories

    ax.set_box_aspect([np.ptp(arr[:,:,0]), np.ptp(arr[:,:,1]), np.ptp(arr[:,:,2])])

    plt.show()
    plt.savefig("trajectory_3D.png", dpi=300)  # optional: save figure

def plot_gate_travesal(arr, att, gate_world, t_star=None,
                       pos_at_gate=None, pos_err=None,
                       vel_at_gate=None, vel_err=None,
                       euler_at_gate=None, x_dot=None, z_dot=None):
    T, N, _ = arr.shape
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=20, azim=180)

    for n in range(1):
        pos_n = arr[:, n, :]  # (T,3)
        ax.plot(pos_n[:,0], pos_n[:,1], pos_n[:,2], label=f'env {n}')

        if t_star is not None:
            ax.scatter(*pos_n[t_star], color='orange', s=80, zorder=5, label='gate crossing')

        for t in range(0, T, 3):
            R = quat_to_rotmat(att[t, n])
            origin = pos_n[t]
            ax.quiver(*origin, *(R @ np.array([1,0,0])), length=0.3, color='r')
            ax.quiver(*origin, *(R @ np.array([0,1,0])), length=0.3, color='g')
            ax.quiver(*origin, *(R @ np.array([0,0,1])), length=0.3, color='b')

    ax.plot(gate_world[:,0], gate_world[:,1], gate_world[:,2], 'k-', linewidth=2, label='gate')
    ax.set_xlim([1, 2])
    ax.set_ylim([-1, 1])
    ax.set_zlim([0, 2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_box_aspect([1,1,1])
    ax.legend()

    if pos_err is not None:
        p = np.round(pos_at_gate, 3)
        v = np.round(vel_at_gate, 3)
        e = np.round(euler_at_gate, 1)
        metrics_text = (f"Position  : [{p[0]}, {p[1]}, {p[2]}]  err={pos_err:.3f} m\n"
                        f"Velocity  : [{v[0]}, {v[1]}, {v[2]}]  err={vel_err:.3f} m/s\n"
                        f"Orient    : [{e[0]}, {e[1]}, {e[2]}] deg\n"
                        f"  dot(bx,gx)={x_dot:.4f}  dot(bz,gz)={z_dot:.4f}")
        ax.text2D(0.02, 0.95, metrics_text, transform=ax.transAxes, fontsize=9,
                  verticalalignment='top',
                  bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    plt.savefig("trajectory_orientation2.png", dpi=300)


def plot_metrics_timeseries(position_array, velocity_array, attitude_array, window_quat,
                            t_star, pos_at_gate, pos_err, vel_at_gate, vel_err,
                            euler_at_gate, x_dot, z_dot):
    T = position_array.shape[0]
    ts = np.arange(T)

    x_body = np.array([1.0, 0.0, 0.0])
    z_body = np.array([0.0, 0.0, 1.0])
    window_x_world = _rotate_vec_by_quat(x_body, window_quat)
    window_z_world = _rotate_vec_by_quat(z_body, window_quat)
    drone_x_all = _rotate_vec_by_quat_batch(x_body, attitude_array)  # (T,3)
    drone_z_all = _rotate_vec_by_quat_batch(z_body, attitude_array)  # (T,3)
    x_dots = drone_x_all @ window_x_world  # (T,)
    z_dots = drone_z_all @ window_z_world  # (T,)

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle('Gate Traversal Metrics', fontsize=13)

    ax = axes[0]
    for i, lbl in enumerate(['x', 'y', 'z']):
        val = pos_at_gate[i]
        ax.plot(ts, position_array[:, i], label=lbl)
        ax.scatter(t_star, val, color='orange', zorder=5)
        ax.annotate(f'{val:.3f}', (t_star, val), textcoords='offset points', xytext=(5, 4), fontsize=7)
    ax.axvline(t_star, color='orange', linestyle='--', label=f'crossing  err={pos_err:.3f} m')
    ax.set_ylabel('Position (m)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True)

    ax = axes[1]
    for i, lbl in enumerate(['vx', 'vy', 'vz']):
        val = vel_at_gate[i]
        ax.plot(ts, velocity_array[:, 3 + i], label=lbl)
        ax.scatter(t_star, val, color='orange', zorder=5)
        ax.annotate(f'{val:.3f}', (t_star, val), textcoords='offset points', xytext=(5, 4), fontsize=7)
    ax.axvline(t_star, color='orange', linestyle='--', label=f'crossing  err={vel_err:.3f} m/s')
    ax.set_ylabel('Velocity (m/s)')
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True)

    ax = axes[2]
    ax.plot(ts, x_dots, label='dot(bx, gx)')
    ax.plot(ts, z_dots, label='dot(bz, gz)')
    ax.scatter(t_star, x_dot, color='orange', zorder=5)
    ax.scatter(t_star, z_dot, color='orange', zorder=5)
    ax.annotate(f'{x_dot:.4f}', (t_star, x_dot), textcoords='offset points', xytext=(5,  4), fontsize=7)
    ax.annotate(f'{z_dot:.4f}', (t_star, z_dot), textcoords='offset points', xytext=(5, -10), fontsize=7)
    ax.axvline(t_star, color='orange', linestyle='--', label='crossing')
    ax.axhline(1.0, color='gray', linestyle=':', linewidth=0.8, label='perfect alignment')
    ax.set_ylabel('Axis alignment (dot product)')
    ax.set_xlabel('Timestep')
    ax.legend(loc='lower right', fontsize=8)
    ax.grid(True)

    plt.tight_layout()
    plt.savefig("gate_metrics_timeseries.png", dpi=300)

def _rotate_vec_by_quat(v, q):
    """Rotate vector v (3,) by quaternion q (4,) = [x,y,z,w]. Returns (3,)."""
    q_xyz = q[:3]
    w = q[3]
    t = 2.0 * np.cross(q_xyz, v)
    return v + w * t + np.cross(q_xyz, t)


def _rotate_vec_by_quat_batch(v, qs):
    """Rotate vector v (3,) by each quaternion in qs (T,4) = [x,y,z,w]. Returns (T,3)."""
    q_xyz = qs[:, :3]           # (T,3)
    w     = qs[:, 3:4]          # (T,1)
    t = 2.0 * np.cross(q_xyz, v[np.newaxis, :])   # (T,3)
    return v[np.newaxis, :] + w * t + np.cross(q_xyz, t)


def compute_gate_metrics(position_array, velocity_array, attitude_array, window_quat,
                         gate_center=np.array([1.5, 0.0, 1.0]),
                         target_vel=None):
    """
    position_array : (T, 3)  drone positions [x, y, z]
    velocity_array : (T, 6)  [ang_x, ang_y, ang_z, lin_x, lin_y, lin_z]
    attitude_array : (T, 4)  quaternion [x, y, z, w]
    window_quat    : (4,)    gate orientation quaternion [x, y, z, w]
    target_vel     : (3,)    desired velocity at gate crossing; if None, uses vy/vz deviation
    """
    from scipy.spatial.transform import Rotation as ScipyR

    dists = np.linalg.norm(position_array - gate_center, axis=1)
    t_star = int(np.argmin(dists))

    pos_at_gate = position_array[t_star]
    pos_err = dists[t_star]

    vel_at_gate = velocity_array[t_star, 3:6]
    if target_vel is not None:
        vel_err = np.linalg.norm(vel_at_gate - np.asarray(target_vel))
    else:
        vel_err = np.linalg.norm(vel_at_gate[1:])  # vy, vz deviation from forward-only passage

    q = attitude_array[t_star]  # [x, y, z, w]
    euler_at_gate = ScipyR.from_quat(q).as_euler('xyz', degrees=True)

    x_body = np.array([1.0, 0.0, 0.0])
    z_body = np.array([0.0, 0.0, 1.0])
    drone_x_world  = _rotate_vec_by_quat(x_body, q)
    drone_z_world  = _rotate_vec_by_quat(z_body, q)
    window_x_world = _rotate_vec_by_quat(x_body, window_quat)
    window_z_world = _rotate_vec_by_quat(z_body, window_quat)
    x_dot = float(np.dot(drone_x_world, window_x_world))
    z_dot = float(np.dot(drone_z_world, window_z_world))

    print(f"[Gate Metrics @ t={t_star}]")
    print(f"  Position    : {np.round(pos_at_gate, 3)}  |  err={pos_err:.4f} m")
    vel_err_label = "err(3D)" if target_vel is not None else "err(vy,vz)"
    print(f"  Velocity    : {np.round(vel_at_gate, 3)}  |  {vel_err_label}={vel_err:.4f} m/s")
    print(f"  Orientation : {np.round(euler_at_gate, 2)} deg  |  dot(bx,gx)={x_dot:.4f}  dot(bz,gz)={z_dot:.4f}")

    return t_star, pos_at_gate, pos_err, vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot


