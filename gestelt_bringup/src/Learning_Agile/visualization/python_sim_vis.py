from casadi import *
import casadi
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import math


from math import sqrt
from geometry.solid_geometry import dir_cosine

def get_quad_vert_pos(wing_len, state_traj):

    # thrust_position in body frame
    r1 = vertcat(wing_len*0.5/ sqrt(2) , wing_len*0.5/ sqrt(2) , 0)
    r2 = vertcat(-wing_len*0.5 / sqrt(2), wing_len*0.5 / sqrt(2), 0)
    r3 = vertcat(-wing_len*0.5 / sqrt(2), -wing_len*0.5 / sqrt(2), 0)
    r4 = vertcat(wing_len*0.5 / sqrt(2), -wing_len*0.5 / sqrt(2), 0)

    # r1 = vertcat(wing_len*0.5, 0, 0)
    # r2 = vertcat(0,-wing_len*0.5, 0)
    # r3 = vertcat(-wing_len*0.5,0, 0)
    # r4 = vertcat(0, wing_len*0.5, 0)
    # horizon
    horizon = np.size(state_traj, 0)
    position = np.zeros((horizon, 15))
    for t in range(horizon):
        # position of COM
        # state_traj [x,y,z,vx,vy,vz,qw,qx,qy,qz...]
        rc = state_traj[t, 0:3]
        # altitude of quaternion
        q = state_traj[t, 6:10]

        # direction cosine matrix from body to inertial
        CIB = np.transpose(dir_cosine(q).full())

        # position of each rotor in inertial frame
        r1_pos = rc + mtimes(CIB, r1).full().flatten()
        r2_pos = rc + mtimes(CIB, r2).full().flatten()
        r3_pos = rc + mtimes(CIB, r3).full().flatten()
        r4_pos = rc + mtimes(CIB, r4).full().flatten()

        # store
        position[t, 0:3] = rc
        position[t, 3:6] = r1_pos
        position[t, 6:9] = r2_pos
        position[t, 9:12] = r3_pos
        position[t, 12:15] = r4_pos

    return position

def get_NN_pose(NN_pos,NN_R):
    """the pose is the NN decided pose coordinate

    Args:
        NN_pos (_type_): a list of NN decided position
        NN_R (_type_): a list of NN decided rotation matrix
    """ 
    #1 create unit coordinate array, with the shape of (N,3)
    X=np.tile(np.array([1,0,0]),(np.size(NN_pos,0),1)) # shape is (N,3)
    Y=np.tile(np.array([0,1,0]),(np.size(NN_pos,0),1))
    Z=np.tile(np.array([0,0,1]),(np.size(NN_pos,0),1))

    # NN_pose shape is (N,3,3)
    NN_pose=np.stack((X,Y,Z),axis=1)
    #2 rotate the unit coordinate array by the rotation matrix
    NN_pose=np.matmul(NN_pose,NN_R.reshape(-1,3,3).transpose(0,2,1))

    #3. translate the rotated unit coordinate array by the position 
    NN_pose+=NN_pos.reshape(-1,1,3)
    
    

    return NN_pose
def traj_ani(ax, single_traj, predicted=False):
    
    cmap = plt.cm.get_cmap("coolwarm")  # blue to the red
    
    lines= []
    for i in range(len(single_traj) - 1):

        velocity = np.linalg.norm(single_traj[:, 3:6], axis=1)  
        norm_velocity = (velocity - np.min(velocity)) / (np.max(velocity) - np.min(velocity)+1e-5)  
        line,=ax.plot(single_traj[i:i+2, 0], single_traj[i:i+2, 1], single_traj[i:i+2, 2],
                color=cmap(norm_velocity[i]), 
                linewidth=1.0, 
                alpha=0.2 if predicted else 1.0,
                )
        lines.append(line)
    
    return lines
    
        

    
pred_lines = []
actual_lines = []
def play_animation(wing_len, state_traj, pred_traj_list, gate_traj1=None, gate_traj2=None,state_traj_ref=None,NN_pos=None,NN_R=None, dt=0.01, \
            point1 = None,point2 = None,point3 = None,point4 = None,save_option=0, title='UAV Maneuvering',\
                goal_pos=[0,0,0]):
        font1 = {
         'weight':'normal',
         'style':'normal', 'size':4}
        cm_2_inch = 2.54
        fig = plt.figure(figsize=(16/cm_2_inch,16*0.65/cm_2_inch),dpi=400)
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X (m)', labelpad=-13,**font1)
        ax.set_ylabel('Y (m)', labelpad=-13,**font1)
        ax.set_zlabel('Z (m)', labelpad=-13,**font1)
        ax.tick_params(axis='x',which='major',pad=-5)
        ax.tick_params(axis='y',which='major',pad=-5)
        ax.tick_params(axis='z',which='major',pad=-5)
        ax.set_zlim(-0, 3)
        ax.set_ylim(-1.5, 1.5)#9
        ax.set_xlim(-1.5, 1.5)#6


        # target landing point
        ax.plot([goal_pos[0]], [goal_pos[1]], [goal_pos[2]], c="r", marker="o",markersize=2)
        ax.view_init(2,-120)
        
        # plot gate
        if point1 is not None:
            ax.plot([point1[0],point2[0]],[point1[1],point2[1]],[point1[2],point2[2]],linewidth=1,color='red',linestyle='-')
            ax.plot([point2[0],point3[0]],[point2[1],point3[1]],[point2[2],point3[2]],linewidth=1,color='red',linestyle='-')
            ax.plot([point3[0],point4[0]],[point3[1],point4[1]],[point3[2],point4[2]],linewidth=1,color='red',linestyle='-')
            ax.plot([point4[0],point1[0]],[point4[1],point1[1]],[point4[2],point1[2]],linewidth=1,color='red',linestyle='-')
        # data
        position = get_quad_vert_pos(wing_len, state_traj)
        sim_horizon = np.size(position, 0)
        NN_pose = get_NN_pose(NN_pos,NN_R)
        if state_traj_ref is None:
            position_ref = get_quad_vert_pos(0, numpy.zeros_like(position))
        else:
            position_ref = get_quad_vert_pos(wing_len, state_traj_ref)

        
        velocity = np.linalg.norm(state_traj[:, 3:6], axis=1)  
        norm_velocity = (velocity - np.min(velocity)) / (np.max(velocity) - np.min(velocity))  # 归一化
        cmap = plt.cm.get_cmap("coolwarm")

        
        # add color bar
        sm = plt.cm.ScalarMappable(cmap=cmap, norm=plt.Normalize(vmin=np.min(velocity), vmax=np.max(velocity)))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax, pad=0.1)
        cbar.set_label('Speed', rotation=270, labelpad=15)
       

        ## animation
        # gate
        if gate_traj1 is not None:
            p1_x, p1_y, p1_z = gate_traj1[0, 0,:]
            p2_x, p2_y, p2_z = gate_traj1[0, 1,:]
            p3_x, p3_y, p3_z = gate_traj1[0, 2,:]
            p4_x, p4_y, p4_z = gate_traj1[0, 3,:]
            gate_l1, = ax.plot([p1_x,p2_x],[p1_y,p2_y],[p1_z,p2_z],linewidth=1,color='red',linestyle='-')
            gate_l2, = ax.plot([p2_x,p3_x],[p2_y,p3_y],[p2_z,p3_z],linewidth=1,color='red',linestyle='-')
            gate_l3, = ax.plot([p3_x,p4_x],[p3_y,p4_y],[p3_z,p4_z],linewidth=1,color='red',linestyle='-')
            gate_l4, = ax.plot([p4_x,p1_x],[p4_y,p1_y],[p4_z,p1_z],linewidth=1,color='red',linestyle='-')


        # quadrotor
        line_traj, = ax.plot(position[:1, 0], position[:1, 1], position[:1, 2],linewidth=0.5)
        c_x, c_y, c_z = position[0, 0:3]
        r1_x, r1_y, r1_z = position[0, 3:6]
        r2_x, r2_y, r2_z = position[0, 6:9]
        r3_x, r3_y, r3_z = position[0, 9:12]
        r4_x, r4_y, r4_z = position[0, 12:15]
        line_arm1, = ax.plot([c_x, r1_x], [c_y, r1_y], [c_z, r1_z], linewidth=1, color='red', marker='o', markersize=1)
        line_arm2, = ax.plot([c_x, r2_x], [c_y, r2_y], [c_z, r2_z], linewidth=1, color='blue', marker='o', markersize=1)
        line_arm3, = ax.plot([c_x, r3_x], [c_y, r3_y], [c_z, r3_z], linewidth=1, color='orange', marker='o', markersize=1)
        line_arm4, = ax.plot([c_x, r4_x], [c_y, r4_y], [c_z, r4_z], linewidth=1, color='green', marker='o', markersize=1)

        line_traj_ref, = ax.plot(position_ref[:1, 0], position_ref[:1, 1], position_ref[:1, 2], color='green', alpha=0.5)
        c_x_ref, c_y_ref, c_z_ref = position_ref[0, 0:3]
        r1_x_ref, r1_y_ref, r1_z_ref = position_ref[0, 3:6]
        r2_x_ref, r2_y_ref, r2_z_ref = position_ref[0, 6:9]
        r3_x_ref, r3_y_ref, r3_z_ref = position_ref[0, 9:12]
        r4_x_ref, r4_y_ref, r4_z_ref = position_ref[0, 12:15]

        ## NN pose
        NN_c_x,NN_c_y,NN_c_z=NN_pos[0,:]-position[0,0:3]
        NN_x_axis_x,NN_x_axis_y,NN_x_axis_z=NN_pose[0,0,:] + position[0,0:3]
        NN_y_axis_x,NN_y_axis_y,NN_y_axis_z=NN_pose[0,1,:] + position[0,0:3]
        NN_z_axis_x,NN_z_axis_y,NN_z_axis_z=NN_pose[0,2,:] + position[0,0:3]
        NN_pose_x_traj, = ax.plot([NN_c_x,NN_x_axis_x],[NN_c_y,NN_x_axis_y],[NN_c_z,NN_x_axis_z],linewidth=1,color='red',linestyle='--')
        NN_pose_y_traj, = ax.plot([NN_c_x,NN_y_axis_x],[NN_c_y,NN_y_axis_y],[NN_c_z,NN_y_axis_z],linewidth=1,color='blue',linestyle='--')
        NN_pose_z_traj, = ax.plot([NN_c_x,NN_z_axis_x],[NN_c_y,NN_z_axis_y],[NN_c_z,NN_z_axis_z],linewidth=1,color='green',linestyle='--')

        # time label
        time_template = 'time = %.2fs'
        time_text = ax.text2D(0.2, 0.7, "time", transform=ax.transAxes,**font1)

        # customize
        if state_traj_ref is not None:
            plt.legend([line_traj, line_traj_ref], ['learned', 'OC solver'], ncol=1, loc='best',
                       bbox_to_anchor=(0.35, 0.25, 0.5, 0.5))

        # pred_lines=traj_ani(ax, pred_traj_list[0])
        
        def update_traj(num):
            global pred_lines,actual_lines
            
            time_text.set_text(time_template % (num * dt))

            # trajectory
            # line_traj.set_data(position[:num, 0], position[:num, 1])
            # line_traj.set_3d_properties(position[:num, 2])
            # line_traj.set_color(colors[:num])

            # plot the predicted trajectory
            
            for line in pred_lines:
                line.remove()
            for line in actual_lines:
                line.remove()
                
            pred_lines = []
            actual_lines = []
        
            pred_lines = traj_ani(ax, pred_traj_list[num],predicted=True)
            if num >=1:
                actual_lines = traj_ani(ax, state_traj[:num])
            
            # uav
            c_x, c_y, c_z = position[num, 0:3]
            r1_x, r1_y, r1_z = position[num, 3:6]
            r2_x, r2_y, r2_z = position[num, 6:9]
            r3_x, r3_y, r3_z = position[num, 9:12]
            r4_x, r4_y, r4_z = position[num, 12:15]

            # NN output pose
            NN_c_x,NN_c_y,NN_c_z=NN_pos[num,:] + position[num,0:3]
            NN_x_axis_x,NN_x_axis_y,NN_x_axis_z=NN_pose[num,0,:] + position[num,0:3]
            NN_y_axis_x,NN_y_axis_y,NN_y_axis_z=NN_pose[num,1,:] + position[num,0:3]
            NN_z_axis_x,NN_z_axis_y,NN_z_axis_z=NN_pose[num,2,:] + position[num,0:3]
            
            NN_pose_x_traj.set_data_3d([NN_c_x,NN_x_axis_x],[NN_c_y,NN_x_axis_y],[NN_c_z,NN_x_axis_z])
            NN_pose_y_traj.set_data_3d([NN_c_x,NN_y_axis_x],[NN_c_y,NN_y_axis_y],[NN_c_z,NN_y_axis_z])
            NN_pose_z_traj.set_data_3d([NN_c_x,NN_z_axis_x],[NN_c_y,NN_z_axis_y],[NN_c_z,NN_z_axis_z])



            line_arm1.set_data_3d([c_x, r1_x], [c_y, r1_y],[c_z, r1_z])
            #line_arm1.set_3d_properties()

            line_arm2.set_data_3d([c_x, r2_x], [c_y, r2_y],[c_z, r2_z])
            #line_arm2.set_3d_properties()

            line_arm3.set_data_3d([c_x, r3_x], [c_y, r3_y],[c_z, r3_z])
            #line_arm3.set_3d_properties()

            line_arm4.set_data_3d([c_x, r4_x], [c_y, r4_y],[c_z, r4_z])
            #line_arm4.set_3d_properties()

            # trajectory ref
            nu=sim_horizon-1
            line_traj_ref.set_data_3d(position_ref[:nu, 0], position_ref[:nu, 1],position_ref[:nu, 2])
            #line_traj_ref.set_3d_properties()

            # uav ref
            c_x_ref, c_y_ref, c_z_ref = position_ref[nu, 0:3]
            r1_x_ref, r1_y_ref, r1_z_ref = position_ref[nu, 3:6]
            r2_x_ref, r2_y_ref, r2_z_ref = position_ref[nu, 6:9]
            r3_x_ref, r3_y_ref, r3_z_ref = position_ref[nu, 9:12]
            r4_x_ref, r4_y_ref, r4_z_ref = position_ref[nu, 12:15]

            ## plot moving gate
            if gate_traj1 is not None:
                p1_x, p1_y, p1_z = gate_traj1[num, 0,:]
                p2_x, p2_y, p2_z = gate_traj1[num, 1,:]
                p3_x, p3_y, p3_z = gate_traj1[num, 2,:]
                p4_x, p4_y, p4_z = gate_traj1[num, 3,:]       

                gate_l1.set_data_3d([p1_x,p2_x],[p1_y,p2_y],[p1_z,p2_z])
                gate_l2.set_data_3d([p2_x,p3_x],[p2_y,p3_y],[p2_z,p3_z]) 
                gate_l3.set_data_3d([p3_x,p4_x],[p3_y,p4_y],[p3_z,p4_z]) 
                gate_l4.set_data_3d([p4_x,p1_x],[p4_y,p1_y],[p4_z,p1_z])




                return line_traj,gate_l1,gate_l2,gate_l3,gate_l4,line_arm1, line_arm2, line_arm3, line_arm4, \
                        NN_pose_x_traj,NN_pose_y_traj,NN_pose_z_traj,\
                    line_traj_ref, time_text
                                            #, line_arm1_ref, line_arm2_ref, line_arm3_ref, line_arm4_ref
            return line_traj, line_arm1, line_arm2, line_arm3, line_arm4, \
                NN_pose_x_traj,NN_pose_y_traj,NN_pose_z_traj,\
                line_traj_ref, time_text #, line_arm1_ref, line_arm2_ref, line_arm3_ref, line_arm4_ref, time_text
     

        frames=np.arange(0,500)
        ani = animation.FuncAnimation(fig,update_traj,frames, interval=1, blit=True, repeat=True)
        # # 在图上添加一个按钮
        ax_button = plt.axes([0.1, 0.9, 0.1, 0.05])  # 按钮位置 ([left, bottom, width, height])
        button = Button(ax_button, 'Pause/Start')

        paused = False
        def toggle_pause(event):
            nonlocal paused
            if paused:
                ani.event_source.start()
            else:
                ani.event_source.stop()
                plt.show()
            paused = not paused
        button.on_clicked(toggle_pause)

        if save_option != 0:
            Writer = animation.writers['ffmpeg']
            writer = Writer(fps=10, metadata=dict(artist='Me'), bitrate=-1)
            ani.save('case2'+title + '.mp4', writer=writer, dpi=300)
            print('save_success')
        plt.tight_layout()
        plt.show()

def plot_position(axs, state_traj, dt=0.1, label_prefix=""):
    """
    axs: a list (or array) of 3 Axes objects
    state_traj: your state trajectory array
    dt: time step
    label_prefix: optional string to label these subplots, e.g. name
    """
    N = len(state_traj[:, 0])
    x = np.arange(0, N * dt, dt)

    axs[0].plot(x, state_traj[:, 0])
    axs[0].set_title(f"{label_prefix} x-position")

    axs[1].plot(x, state_traj[:, 1])
    axs[1].set_title(f"{label_prefix} y-position")

    axs[2].plot(x, state_traj[:, 2])
    axs[2].set_title(f"{label_prefix} z-position")

    for ax in axs:
        ax.grid(True, linestyle="--", alpha=0.6)
    
def plot_velocity(axs, state_traj, dt=0.1):
    """
    axs: a list (or array) of 3 Axes objects
    state_traj: your state trajectory array
    dt: time step
    """
    N = len(state_traj[:, 0])
    x = np.arange(0, N * dt, dt)

    axs[0].plot(x, state_traj[:, 3])
    axs[0].set_title("x-velocity")

    axs[1].plot(x, state_traj[:, 4])
    axs[1].set_title("y-velocity")

    axs[2].plot(x, state_traj[:, 5])
    axs[2].set_title("z-velocity")

    for ax in axs:
        ax.grid(True, linestyle="--", alpha=0.6)

def plot_quaternions(
    axs,                # a list or array of 4 Axes objects
    state_traj, 
    dt=0.1, 
    label_prefix=""
):
    """
    Plots q0, q1, q2, q3 vs time on the provided 4 Axes:
        axs[0] -> q0
        axs[1] -> q1
        axs[2] -> q2
        axs[3] -> q3
    """
    N = len(state_traj[:, 0])
    t = np.arange(0, N * dt, dt)

    axs[0].plot(t, state_traj[:, 6])
    axs[0].set_title(f"{label_prefix} q0")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("q0")

    axs[1].plot(t, state_traj[:, 7])
    axs[1].set_title(f"{label_prefix} q1")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("q1")

    axs[2].plot(t, state_traj[:, 8])
    axs[2].set_title(f"{label_prefix} q2")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("q2")

    axs[3].plot(t, state_traj[:, 9])
    axs[3].set_title(f"{label_prefix} q3")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel("q3")

    for ax in axs:
        ax.grid(True, linestyle='--', alpha=0.6)

def plot_quaternions_norm(
    ax,              # a single Axes object
    state_traj, 
    dt=0.1, 
    label_prefix=""
):
    """
    Plots the norm of (q0, q1, q2, q3) vs time on a single Axis.
    """
    N = len(state_traj[:, 0])
    t = np.arange(0, N * dt, dt)
    q_norm = np.linalg.norm(state_traj[:, 6:10], axis=1)

    ax.plot(t, q_norm)
    ax.set_title(f"{label_prefix} Quaternion Norm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("|q|")
    ax.grid(True, linestyle='--', alpha=0.6)

def plot_angularrate(
    ax,               # a single Axes object
    control_traj, 
    dt=0.01, 
    label_prefix=""
):
    """
    Plots angular rates (w1, w2, w3) vs time on a single Axis.
    """
    N = len(control_traj[:, 0])
    t = np.arange(0, N * dt, dt)

    ax.plot(t, control_traj[:, 0], label="w1",color='red')
    ax.plot(t, control_traj[:, 1], label="w2",color='blue')
    ax.plot(t, control_traj[:, 2], label="w3",color='green')

    ax.set_title(f"{label_prefix} Angular Rates")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Angular Rate (rad/s)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

def plot_thrust(
    ax,               # a single Axes object
    control_traj, 
    dt=0.1, 
    label_prefix=""
):
    """
    Plots collective thrust (control_traj[:,0]) vs time on a single Axis.
    """
    N = len(control_traj[:, 0])
    t = np.arange(0, N * dt, dt)

    ax.plot(t, control_traj[:, 0], label="u1")
    ax.set_title(f"{label_prefix} Collective Thrust")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Thrust (N)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

def plot_T(
    ax,               # a single Axes object
    control_traj, 
    dt=0.1, 
    label_prefix="Single Rotor Thrust"
):
    """
    Plots T0, T1, T2, T3 (4 lines) vs time on a single Axis.
    """
    N = len(control_traj[:, 0])
    t = np.arange(0, N * dt, dt)

    ax.plot(t, control_traj[:, 0], label="T0")
    ax.plot(t, control_traj[:, 1], label="T1")
    ax.plot(t, control_traj[:, 2], label="T2")
    ax.plot(t, control_traj[:, 3], label="T3")

    ax.set_title(f"{label_prefix} vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Thrust (N)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

def plot_M(
    ax,               # a single Axes object
    control_traj, 
    dt=0.1, 
    label_prefix="Torque"
):
    """
    Plots Mx, My, Mz (columns 1,2,3) vs time on a single Axis.
    Note: The code suggests control_traj[:,0] might be something else (like T0?), 
    so torque could be columns 1..3.
    """
    N = len(control_traj[:, 0])
    t = np.arange(0, N * dt, dt)

    # The original code used control_traj[:,1], control_traj[:,2], control_traj[:,3]
    ax.plot(t, control_traj[:, 1], label="Mx")
    ax.plot(t, control_traj[:, 2], label="My")
    ax.plot(t, control_traj[:, 3], label="Mz")

    ax.set_title(f"{label_prefix} vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Torque (Nm)")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

def plot_scalar(
    ax, 
    scalar, 
    scalar_name="scalar", 
    label_prefix=""
):
    """
    Plots a scalar (1D array) vs its index or time on a single Axis.
    """
    # If you have a known dt, you can do t = np.arange(len(scalar)) * dt
    ax.plot(scalar, label=scalar_name)
    ax.set_title(f"{label_prefix} {scalar_name} vs Time")
    ax.set_xlabel("Time Index")
    ax.set_ylabel(scalar_name)
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

def plot_weights(
    ax, 
    tra_pos_weights,
    tra_att_weights, 
    dt=0.1, 
    label_prefix=""
):
    """
    Plots the weights of the neural network vs time on a single Axis.
    """


    ax.plot(tra_pos_weights, label="traverse_position_weight")
    ax.plot(tra_att_weights, label="traverse_attitude_weight")

    ax.set_title(f"{label_prefix} Weights vs Time")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Weight Value")
    ax.grid(True, linestyle='--', alpha=0.6)
    ax.legend()

def plot_3D_traj(
                wing_len,
                uav_height,
                state_traj,
                gate_traj,
                TRAIN_VIS=False,
                tra_node=None,
                NN_pos=None,
                NN_R=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    position = get_quad_vert_pos(wing_len, state_traj)

    NN_pose = get_NN_pose(NN_pos,NN_R)

    for i in range(np.size(state_traj,0)):

        if TRAIN_VIS:
            p1_x, p1_y, p1_z = gate_traj[0,:]
            p2_x, p2_y, p2_z = gate_traj[1,:]
            p3_x, p3_y, p3_z = gate_traj[2,:]
            p4_x, p4_y, p4_z = gate_traj[3,:]

        else:
            p1_x, p1_y, p1_z = gate_traj[i, 0,:]
            p2_x, p2_y, p2_z = gate_traj[i, 1,:]
            p3_x, p3_y, p3_z = gate_traj[i, 2,:]
            p4_x, p4_y, p4_z = gate_traj[i, 3,:]
        
        c_x, c_y, c_z = position[i,0:3]
        r1_x, r1_y, r1_z = position[i,3:6]
        r2_x, r2_y, r2_z = position[i,6:9]
        r3_x, r3_y, r3_z = position[i,9:12]
        r4_x, r4_y, r4_z = position[i,12:15]
        
        #  calculate the distance between the quadrotor and the gate
        gate_center = np.array([(p1_x+p2_x+p3_x+p4_x)/4,(p1_y+p2_y+p3_y+p4_y)/4,(p1_z+p2_z+p3_z+p4_z)/4])
        quadrotor_center = np.array([c_x,c_y,c_z])

        distance = np.linalg.norm(gate_center-quadrotor_center)
        
        condition_test=distance <= 0.5
        condition_train=i==tra_node
        condition=condition_test
        if TRAIN_VIS:
            condition=condition_train
        if condition:
            plot_alpha = 1
            

            ## plot the drone ellipsoid
            
            # rotation of the drone
            q = state_traj[i,6:10]
            R = transpose(dir_cosine(q)) # body frame to world frame
            
            # Create a grid of u, v values (parametric angles)
            u = np.linspace(0, 2 * np.pi, 10)
            v = np.linspace(0, np.pi, 10)

            # Parametric equations for the ellipsoid
            x = wing_len/2 * np.outer(np.cos(u), np.sin(v))
            y = (wing_len/2) * np.outer(np.sin(u), np.sin(v))
            z = uav_height * np.outer(np.ones(np.size(u)), np.cos(v))

            points_3d = np.array([x.flatten(), y.flatten(), z.flatten()])
            # Apply rotation matrix
            rotated_points = np.dot(R, points_3d)
            x = np.reshape(rotated_points[0, :], x.shape)
            y = np.reshape(rotated_points[1, :], y.shape)
            z = np.reshape(rotated_points[2, :], z.shape)
            
            ax.plot_surface(x + c_x, y + c_y, z + c_z, color='b', alpha=0.05)
        else:
            plot_alpha = 0.1*(i/np.size(state_traj,0))+0.1
        gate_l1, = ax.plot([p1_x,p2_x],[p1_y,p2_y],[p1_z,p2_z],linewidth=1,color='orangered',linestyle='-',alpha=plot_alpha)
        gate_l2, = ax.plot([p2_x,p3_x],[p2_y,p3_y],[p2_z,p3_z],linewidth=1,color='orangered',linestyle='-',alpha=plot_alpha)
        gate_l3, = ax.plot([p3_x,p4_x],[p3_y,p4_y],[p3_z,p4_z],linewidth=1,color='orangered',linestyle='-',alpha=plot_alpha)
        gate_l4, = ax.plot([p4_x,p1_x],[p4_y,p1_y],[p4_z,p1_z],linewidth=1,color='orangered',linestyle='-',alpha=plot_alpha)
        
        ## NN pose
        NN_c_x,NN_c_y,NN_c_z=NN_pos[i,:]+position[i,0:3]
        NN_x_axis_x,NN_x_axis_y,NN_x_axis_z=NN_pose[i,0,:]+position[i,0:3]
        NN_y_axis_x,NN_y_axis_y,NN_y_axis_z=NN_pose[i,1,:]+position[i,0:3]
        NN_z_axis_x,NN_z_axis_y,NN_z_axis_z=NN_pose[i,2,:]+position[i,0:3]
        NN_pose_x_traj, = ax.plot([NN_c_x,NN_x_axis_x],[NN_c_y,NN_x_axis_y],[NN_c_z,NN_x_axis_z],linewidth=1,color='red',linestyle='--',alpha=plot_alpha)
        NN_pose_y_traj, = ax.plot([NN_c_x,NN_y_axis_x],[NN_c_y,NN_y_axis_y],[NN_c_z,NN_y_axis_z],linewidth=1,color='blue',linestyle='--',alpha=plot_alpha)
        NN_pose_z_traj, = ax.plot([NN_c_x,NN_z_axis_x],[NN_c_y,NN_z_axis_y],[NN_c_z,NN_z_axis_z],linewidth=1,color='green',linestyle='--',alpha=plot_alpha)

        # if TRAIN_VIS:
        #     for i in range(4):
        #         ax.scatter(gate_traj[i+4,0],gate_traj[i+4,1],gate_traj[i+4,2],c='b',marker='o',s=10)

        line_arm1, = ax.plot([c_x, r1_x], [c_y, r1_y], [c_z, r1_z], linewidth=4, color='crimson', marker='o', markersize=1,alpha=plot_alpha)
        line_arm2, = ax.plot([c_x, r2_x], [c_y, r2_y], [c_z, r2_z], linewidth=4, color='lightskyblue', marker='o', markersize=1,alpha=plot_alpha)
        line_arm3, = ax.plot([c_x, r3_x], [c_y, r3_y], [c_z, r3_z], linewidth=4, color='peachpuff', marker='o', markersize=1,alpha=plot_alpha)
        line_arm4, = ax.plot([c_x, r4_x], [c_y, r4_y], [c_z, r4_z], linewidth=4, color='darkseagreen', marker='o', markersize=1,alpha=plot_alpha)
        
        # set the axes limits
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])
    plt.tight_layout()   


def plot_mc_traj(state_traj_batch:list,
                 gate_traj_batch:list,
                 failed_batch:list,
                 failed_state_batch:list):
    """plot the position trajectory of the states in the batch,
    with the failed states marked in red"""
    """
    Args:
    state_traj_batch: list of numpy arrays, each of shape (N, 16)
    failed_batch: list of bool
    """
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    for i, state_traj in enumerate(state_traj_batch):
        if failed_batch[i]:
            color = 'r'
        else:
            color = 'b'
        ax.plot(state_traj[:,0], state_traj[:,1], state_traj[:,2], color=color)
        
        ## plot the gate
        gate_traj = gate_traj_batch[i]
        p1_x, p1_y, p1_z = gate_traj[i, 0,:]
        p2_x, p2_y, p2_z = gate_traj[i, 1,:]
        p3_x, p3_y, p3_z = gate_traj[i, 2,:]
        p4_x, p4_y, p4_z = gate_traj[i, 3,:]
        plot_alpha = 0.5
        gate_l1, = ax.plot([p1_x,p2_x],[p1_y,p2_y],[p1_z,p2_z],linewidth=1,color='green',linestyle='-',alpha=plot_alpha)
        gate_l2, = ax.plot([p2_x,p3_x],[p2_y,p3_y],[p2_z,p3_z],linewidth=1,color='green',linestyle='-',alpha=plot_alpha)
        gate_l3, = ax.plot([p3_x,p4_x],[p3_y,p4_y],[p3_z,p4_z],linewidth=1,color='green',linestyle='-',alpha=plot_alpha)
        gate_l4, = ax.plot([p4_x,p1_x],[p4_y,p1_y],[p4_z,p1_z],linewidth=1,color='green',linestyle='-',alpha=plot_alpha)
    for i, failed_state in enumerate(failed_state_batch):
        ax.scatter(failed_state[0], failed_state[1], failed_state[2], color='r')
    # set the axes limits
    ax.set_xlim([-2, 2])
    ax.set_ylim([-2, 2])
    ax.set_zlim([-2, 2])

    plt.tight_layout()    
    plt.show()


# 生成一个单位球
def plot_unit_sphere(ax):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 50)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones(np.size(u)), np.cos(v))
    ax.plot_surface(x, y, z, color='c', alpha=0.1, edgecolor='k')

# 生成一个随机 3x3 旋转矩阵（正交矩阵）
def random_rotation_matrix():
    U, _, Vt = np.linalg.svd(np.random.randn(3, 3))  # SVD 保证正交
    return U @ Vt  # 确保行列式为 1（即纯旋转）

# 画旋转矩阵
def plot_rotation_matrix(ax, R):
    origin = np.array([[0, 0, 0]]).T  # 原点
    colors = ['r', 'g', 'b']  # x, y, z 轴的颜色
    labels = ['X', 'Y', 'Z']

    for i in range(3):  # 画 3 个基向量
        ax.quiver(*origin, *R[:, i], color=colors[i], label=f'{labels[i]}-axis')

def show_rotation_matrix():
    # 创建 3D 图像
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')

    # 画单位球
    plot_unit_sphere(ax)

    # 生成并绘制旋转矩阵
    R = random_rotation_matrix()
    plot_rotation_matrix(ax, R)

    # 设定坐标轴范围
    ax.set_xlim([-1, 1])
    ax.set_ylim([-1, 1])
    ax.set_zlim([-1, 1])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3×3 Rotation Matrix on Unit Sphere')
    ax.legend()

    plt.show()