from casadi import *
import casadi
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
import math


from math import sqrt
from solid_geometry import dir_cosine

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

def play_animation( wing_len, state_traj, gate_traj1=None, gate_traj2=None,state_traj_ref=None,NN_pos=None,NN_R=None, dt=0.01, \
            point1 = None,point2 = None,point3 = None,point4 = None,save_option=0, title='UAV Maneuvering',\
                goal_pos=[0,0,0]):
        font1 = {'family':'Times New Roman',
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
        # ax.set_title(title, pad=20, fontsize=15)
        # for t in ax.xaxis.get_major_ticks(): 
        #     t.label.set_font('Times New Roman') 
        #     t.label.set_fontsize(7)
        # for t in ax.yaxis.get_major_ticks(): 
        #     t.label.set_font('Times New Roman') 
        #     t.label.set_fontsize(7)
        # for t in ax.zaxis.get_major_ticks(): 
        #     t.label.set_font('Times New Roman') 
        #     t.label.set_fontsize(7)

        # target landing point
        ax.plot([goal_pos[0]], [goal_pos[1]], [goal_pos[2]], c="r", marker="o",markersize=2)
        ax.view_init(2,-120)
        #plot the final state
        #final_position = get_final_position(wing_len=wing_len)
        #c_x, c_y, c_z = final_position[0:3]
        #r1_x, r1_y, r1_z = final_position[3:6]
        #r2_x, r2_y, r2_z = final_position[6:9]
        #r3_x, r3_y, r3_z = final_position[9:12]
        #r4_x, r4_y, r4_z = final_position[12:15]
        #line_arm1, = ax.plot([c_x, r1_x], [c_y, r1_y], [c_z, r1_z], linewidth=2, color='grey', marker='o', markersize=3)
        #line_arm2, = ax.plot([c_x, r2_x], [c_y, r2_y], [c_z, r2_z], linewidth=2, color='grey', marker='o', markersize=3)
        #line_arm3, = ax.plot([c_x, r3_x], [c_y, r3_y], [c_z, r3_z], linewidth=2, color='grey', marker='o', markersize=3)
        #line_arm4, = ax.plot([c_x, r4_x], [c_y, r4_y], [c_z, r4_z], linewidth=2, color='grey', marker='o', markersize=3)
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

        ## plot the process of moving window and quadrotor
        #for i in range(10):
        #    a = i*6
        #    b = 0.9-0.1*i
        #    c = (b,b,b)
        #    c_x, c_y, c_z = position[a,0:3]
        #    r1_x, r1_y, r1_z = position[a,3:6]
        #    r2_x, r2_y, r2_z = position[a,6:9]
        #    r3_x, r3_y, r3_z = position[a,9:12]
        #    r4_x, r4_y, r4_z = position[a,12:15]
        #    line_arm1, = ax.plot([c_x, r1_x], [c_y, r1_y], [c_z, r1_z], linewidth=2, color=c, marker='o', markersize=3)
        #    line_arm2, = ax.plot([c_x, r2_x], [c_y, r2_y], [c_z, r2_z], linewidth=2, color=c, marker='o', markersize=3)
        #    line_arm3, = ax.plot([c_x, r3_x], [c_y, r3_y], [c_z, r3_z], linewidth=2, color=c, marker='o', markersize=3)
        #    line_arm4, = ax.plot([c_x, r4_x], [c_y, r4_y], [c_z, r4_z], linewidth=2, color=c, marker='o', markersize=3)

        #    p1_x, p1_y, p1_z = gate_traj1[a, 0,:]
        #    p2_x, p2_y, p2_z = gate_traj1[a, 1,:]
        #    p3_x, p3_y, p3_z = gate_traj1[a, 2,:]
        #    p4_x, p4_y, p4_z = gate_traj1[a, 3,:]
        #    gate_l1, = ax.plot([p1_x,p2_x],[p1_y,p2_y],[p1_z,p2_z],linewidth=1,color=c,linestyle='--')
        #    gate_l2, = ax.plot([p2_x,p3_x],[p2_y,p3_y],[p2_z,p3_z],linewidth=1,color=c,linestyle='--')
        #    gate_l3, = ax.plot([p3_x,p4_x],[p3_y,p4_y],[p3_z,p4_z],linewidth=1,color=c,linestyle='--')
        #    gate_l4, = ax.plot([p4_x,p1_x],[p4_y,p1_y],[p4_z,p1_z],linewidth=1,color=c,linestyle='--')
        

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

            #p1_xa, p1_ya, p1_za = gate_traj2[0, 0,:]
            #p2_xa, p2_ya, p2_za = gate_traj2[0, 1,:]
            #p3_xa, p3_ya, p3_za = gate_traj2[0, 2,:]
            #p4_xa, p4_ya, p4_za = gate_traj2[0, 3,:]
            #gate_l1a, = ax.plot([p1_xa,p2_xa],[p1_ya,p2_ya],[p1_za,p2_za],linewidth=1,color='red',linestyle='--')
            #gate_l2a, = ax.plot([p2_xa,p3_xa],[p2_ya,p3_ya],[p2_za,p3_za],linewidth=1,color='red',linestyle='--')
            #gate_l3a, = ax.plot([p3_xa,p4_xa],[p3_ya,p4_ya],[p3_za,p4_za],linewidth=1,color='red',linestyle='--')
            #gate_l4a, = ax.plot([p4_xa,p1_xa],[p4_ya,p1_ya],[p4_za,p1_za],linewidth=1,color='red',linestyle='--')    

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
        # line_arm1_ref, = ax.plot([c_x_ref, r1_x_ref], [c_y_ref, r1_y_ref], [c_z_ref, r1_z_ref], linewidth=2,
        #                          color='green', marker='o', markersize=3, alpha=0.7)
        # line_arm2_ref, = ax.plot([c_x_ref, r2_x_ref], [c_y_ref, r2_y_ref], [c_z_ref, r2_z_ref], linewidth=2,
        #                          color='green', marker='o', markersize=3, alpha=0.7)
        # line_arm3_ref, = ax.plot([c_x_ref, r3_x_ref], [c_y_ref, r3_y_ref], [c_z_ref, r3_z_ref], linewidth=2,
        #                          color='green', marker='o', markersize=3, alpha=0.7)
        # line_arm4_ref, = ax.plot([c_x_ref, r4_x_ref], [c_y_ref, r4_y_ref], [c_z_ref, r4_z_ref], linewidth=2,
        #                          color='green', marker='o', markersize=3, alpha=0.7)

        ## NN pose
        NN_c_x,NN_c_y,NN_c_z=NN_pos[0,:]
        NN_x_axis_x,NN_x_axis_y,NN_x_axis_z=NN_pose[0,0,:]
        NN_y_axis_x,NN_y_axis_y,NN_y_axis_z=NN_pose[0,1,:]
        NN_z_axis_x,NN_z_axis_y,NN_z_axis_z=NN_pose[0,2,:]
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

        def update_traj(num):
            # customize
            time_text.set_text(time_template % (num * dt))

            # trajectory
            line_traj.set_data(position[:num, 0], position[:num, 1])
            line_traj.set_3d_properties(position[:num, 2])


            # uav
            c_x, c_y, c_z = position[num, 0:3]
            r1_x, r1_y, r1_z = position[num, 3:6]
            r2_x, r2_y, r2_z = position[num, 6:9]
            r3_x, r3_y, r3_z = position[num, 9:12]
            r4_x, r4_y, r4_z = position[num, 12:15]

            # NN output pose
            NN_c_x,NN_c_y,NN_c_z=NN_pos[num,:]
            NN_x_axis_x,NN_x_axis_y,NN_x_axis_z=NN_pose[num,0,:]
            NN_y_axis_x,NN_y_axis_y,NN_y_axis_z=NN_pose[num,1,:]
            NN_z_axis_x,NN_z_axis_y,NN_z_axis_z=NN_pose[num,2,:]
            
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

            # line_arm1_ref.set_data_3d([c_x_ref, r1_x_ref], [c_y_ref, r1_y_ref],[c_z_ref, r1_z_ref])
            # #line_arm1_ref.set_3d_properties()

            # line_arm2_ref.set_data_3d([c_x_ref, r2_x_ref], [c_y_ref, r2_y_ref],[c_z_ref, r2_z_ref])
            # #line_arm2_ref.set_3d_properties()

            # line_arm3_ref.set_data_3d([c_x_ref, r3_x_ref], [c_y_ref, r3_y_ref],[c_z_ref, r3_z_ref])
            # #line_arm3_ref.set_3d_properties()

            # line_arm4_ref.set_data_3d([c_x_ref, r4_x_ref], [c_y_ref, r4_y_ref],[c_z_ref, r4_z_ref])
            #line_arm4_ref.set_3d_properties()

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


                #p1_xa, p1_ya, p1_za = gate_traj2[num, 0,:]
                #p2_xa, p2_ya, p2_za = gate_traj2[num, 1,:]
                #p3_xa, p3_ya, p3_za = gate_traj2[num, 2,:]
                #p4_xa, p4_ya, p4_za = gate_traj2[num, 3,:]       

                #gate_l1a.set_data_3d([p1_xa,p2_xa],[p1_ya,p2_ya],[p1_za,p2_za])
                #gate_l2a.set_data_3d([p2_xa,p3_xa],[p2_ya,p3_ya],[p2_za,p3_za]) 
                #gate_l3a.set_data_3d([p3_xa,p4_xa],[p3_ya,p4_ya],[p3_za,p4_za]) 
                #gate_l4a.set_data_3d([p4_xa,p1_xa],[p4_ya,p1_ya],[p4_za,p1_za])




                return line_traj,gate_l1,gate_l2,gate_l3,gate_l4,line_arm1, line_arm2, line_arm3, line_arm4, \
                        NN_pose_x_traj,NN_pose_y_traj,NN_pose_z_traj,\
                    line_traj_ref, time_text
                                            #, line_arm1_ref, line_arm2_ref, line_arm3_ref, line_arm4_ref
            return line_traj, line_arm1, line_arm2, line_arm3, line_arm4, \
                NN_pose_x_traj,NN_pose_y_traj,NN_pose_z_traj,\
                line_traj_ref, time_text #, line_arm1_ref, line_arm2_ref, line_arm3_ref, line_arm4_ref, time_text
     

        frames=np.arange(0,500)
        ani = animation.FuncAnimation(fig,update_traj,frames, interval=1, blit=True)
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

def plot_position(state_traj,name,dt = 0.1):
    fig, axs = plt.subplots(3)
    fig.suptitle(f'{name}+position vs t')
    N = len(state_traj[:,0])
    x = np.arange(0,N*dt,dt)
    axs[0].plot(x,state_traj[:,0])
    axs[1].plot(x,state_traj[:,1])
    axs[2].plot(x,state_traj[:,2])
    plt.savefig(f'./python_sim_result/{name}+position.png')
    # plt.show()
    
def plot_velocity(state_traj,dt = 0.1):
    fig, axs = plt.subplots(3)
    fig.suptitle('velocity vs t')
    N = len(state_traj[:,0])
    x = np.arange(0,N*dt,dt)
    axs[0].plot(x,state_traj[:,3])
    axs[1].plot(x,state_traj[:,4])
    axs[2].plot(x,state_traj[:,5])
    plt.savefig('./python_sim_result/velocity.png')
    # plt.show()

def plot_quaternions(state_traj,dt = 0.1,save=True):
    fig, axs = plt.subplots(4)
    fig.suptitle('quaternions vs t')
    N = len(state_traj[:,0])
    x = np.arange(0,N*dt,dt)
    axs[0].plot(x,state_traj[:,6])
    axs[1].plot(x,state_traj[:,7])
    axs[2].plot(x,state_traj[:,8])
    axs[3].plot(x,state_traj[:,9])
    
    if save:
        plt.savefig('./python_sim_result/quaternions.png')
    # plt.show()
def plot_quaternions_norm(state_traj,dt = 0.1,save=True):
    fig, axs = plt.subplots(1)
    fig.suptitle('MPC last predicted status quaternions norm vs each MPC t')
    N = len(state_traj[:,0])
    x = np.arange(0,N*dt,dt)
    norm = np.linalg.norm(state_traj[:,6:10],axis=1)
    axs.plot(x,norm)
    if save:
        plt.savefig('./python_sim_result/quaternions_norm.png')
    # plt.show()
def plot_angularrate(control_traj,dt = 0.01):
    plt.figure() 
    plt.title('angularrate vs time')
    N = len(control_traj[:,0])
    x = np.arange(0,N*dt,dt)
    plt.plot(x,control_traj[:,0],color = 'b', label = 'w1')
    plt.plot(x,control_traj[:,1],color = 'r', label = 'w2')
    plt.plot(x,control_traj[:,2],color = 'y', label = 'w3')
    plt.xlabel('t')
    plt.ylabel('w')
    plt.grid(True,color='0.6',dashes=(2,2,1,1))
    plt.legend()
    plt.savefig('./python_sim_result/angularrate.png')
    # plt.show()
    

def plot_thrust(control_traj,dt = 0.1):
    plt.figure() 
    N = int(len(control_traj[:,0]))
    x = np.arange(0,round(N*dt,1),dt)
    plt.plot(x,control_traj[:,0],color = 'b', label = 'u1')
    # plt.plot(x,control_traj[:,1],color = 'r', label = 'u2')
    # plt.plot(x,control_traj[:,2],color = 'y', label = 'u3')
    # plt.plot(x,control_traj[:,3],color = 'g', label = 'u4')
    plt.title('collective thrust vs time (N)')
    plt.ylim([0,10])
    plt.xlabel('t')
    plt.ylabel('u')
    plt.grid(True,color='0.6',dashes=(2,2,1,1))
    plt.legend()
    plt.savefig('./python_sim_result/thrust.png')
    # plt.show()
    
            
def plot_T(control_traj,dt = 0.1,name="Single rotor thrust"):
    plt.figure()
    N = int(len(control_traj[:,0]))
    x = np.arange(0,N*dt,dt)
    plt.plot(x,control_traj[:,0],color = 'b', label = 'T0')  
    plt.plot(x,control_traj[:,1],color = 'g', label = 'T1')   
    plt.plot(x,control_traj[:,2],color = 'r', label = 'T2')   
    plt.plot(x,control_traj[:,3],color = 'y', label = 'T3')   
    plt.title(f'{name} vs time')
    # plt.ylim([0,20])
    plt.xlabel('t')
    plt.ylabel(name)
    plt.grid(True,color='0.6',dashes=(2,2,1,1))
    plt.legend()
    plt.savefig(f'./python_sim_result/{name}.png')
    # plt.show()
    

def plot_M(control_traj,dt = 0.1):
    plt.figure() 
    N = int(len(control_traj[:,0]))
    x = np.arange(0,round(N*dt,1),dt)
    plt.plot(x,control_traj[:,1],color = 'r', label = 'Mx')
    plt.plot(x,control_traj[:,2],color = 'y', label = 'My')
    plt.plot(x,control_traj[:,3],color = 'g', label = 'Mz')
    plt.title('torque vs time')
    plt.xlabel('t')
    plt.ylabel('Torque')
    plt.grid(True,color='0.6',dashes=(2,2,1,1))
    plt.legend()
    plt.savefig('./python_sim_result/input_M.png')
    # plt.show()
    

def plot_scalar(scalar, scalar_name):
    plt.figure() 
    plt.plot(scalar)
    plt.title(f'{scalar_name} vs time')
    plt.xlabel('t')
    plt.ylabel(scalar_name)
    plt.grid(True,color='0.6',dashes=(2,2,1,1))
    plt.legend()
    plt.savefig(f'./python_sim_result/{scalar_name}.png')
    # plt.show()

    
def plot_3D_traj(
                    wing_len,
                    uav_height,
                    state_traj,
                    gate_traj,
                    TRAIN_VIS=False,
                    tra_node=None):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    position = get_quad_vert_pos(wing_len, state_traj)

    

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
            plot_alpha = 0.1
        gate_l1, = ax.plot([p1_x,p2_x],[p1_y,p2_y],[p1_z,p2_z],linewidth=1,color='red',linestyle='-',alpha=plot_alpha)
        gate_l2, = ax.plot([p2_x,p3_x],[p2_y,p3_y],[p2_z,p3_z],linewidth=1,color='red',linestyle='-',alpha=plot_alpha)
        gate_l3, = ax.plot([p3_x,p4_x],[p3_y,p4_y],[p3_z,p4_z],linewidth=1,color='red',linestyle='-',alpha=plot_alpha)
        gate_l4, = ax.plot([p4_x,p1_x],[p4_y,p1_y],[p4_z,p1_z],linewidth=1,color='red',linestyle='-',alpha=plot_alpha)
        
        # if TRAIN_VIS:
        #     for i in range(4):
        #         ax.scatter(gate_traj[i+4,0],gate_traj[i+4,1],gate_traj[i+4,2],c='b',marker='o',s=10)

        line_arm1, = ax.plot([c_x, r1_x], [c_y, r1_y], [c_z, r1_z], linewidth=1, color='red', marker='o', markersize=1,alpha=plot_alpha)
        line_arm2, = ax.plot([c_x, r2_x], [c_y, r2_y], [c_z, r2_z], linewidth=1, color='blue', marker='o', markersize=1,alpha=plot_alpha)
        line_arm3, = ax.plot([c_x, r3_x], [c_y, r3_y], [c_z, r3_z], linewidth=1, color='orange', marker='o', markersize=1,alpha=plot_alpha)
        line_arm4, = ax.plot([c_x, r4_x], [c_y, r4_y], [c_z, r4_z], linewidth=1, color='green', marker='o', markersize=1,alpha=plot_alpha)
        
        # set the axes limits
        ax.set_xlim([-2, 2])
        ax.set_ylim([-2, 2])
        ax.set_zlim([-2, 2])
    plt.show()    

