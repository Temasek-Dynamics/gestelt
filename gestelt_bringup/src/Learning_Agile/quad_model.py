##this file is to generate model of quadrotor

from casadi import *
import casadi
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math
from scipy.spatial.transform import Rotation as R
from solid_geometry import norm
from math import sqrt
from solid_geometry import *
from matplotlib.patches import Ellipse
# quadrotor (UAV) environment
class Quadrotor:
    def __init__(self, project_name='my UAV'):
        self.project_name = 'my uav'

        # define the state of the quadrotor
        rx, ry, rz = SX.sym('rx'), SX.sym('ry'), SX.sym('rz')
        self.r_I = vertcat(rx, ry, rz)
        vx, vy, vz = SX.sym('vx'), SX.sym('vy'), SX.sym('vz')
        self.v_I = vertcat(vx, vy, vz)
        # quaternions attitude of B w.r.t. I
        q0, q1, q2, q3 = SX.sym('q0'), SX.sym('q1'), SX.sym('q2'), SX.sym('q3')
        self.q = vertcat(q0, q1, q2, q3)
       
        # quad state
        self.quad_state=vertcat(self.r_I,self.v_I,self.q)
        
        # define the quadrotor input
        f1, f2, f3, f4 = SX.sym('f1'), SX.sym('f2'), SX.sym('f3'), SX.sym('f4')
        self.T_B = vertcat(f1, f2, f3, f4)
        wx, wy, wz = SX.sym('wx'), SX.sym('wy'), SX.sym('wz')
        wx_last, wy_last, wz_last = SX.sym('wx_last'), SX.sym('wy_last'), SX.sym('wz_last')
        self.ang_rate_B = vertcat(wx, wy, wz)
        self.ang_rate_B_last = vertcat(wx_last, wy_last, wz_last)
   
        # total thrust in body frame
        self.thrust_mag=SX.sym('thrust')
        self.thrust_mag_last=SX.sym('thrust_last')
 

        # define desire traverse pose and time
        self.des_tra_r_I = vertcat(SX.sym('des_tra_rx'), SX.sym('des_tra_ry'), SX.sym('des_tra_rz'))
        self.des_tra_rodi_param=vertcat(SX.sym('des_tra_rodi_param0'),SX.sym('des_tra_rodi_param1'),SX.sym('des_tra_rodi_param2'))
        self.des_tra_q = vertcat(SX.sym('des_tra_q0'), SX.sym('des_tra_q1'), SX.sym('des_tra_q2'), SX.sym('des_tra_q3'))
        self.des_t_tra = SX.sym('des_t_tra')
        self.t_node = SX.sym('t_node')

        # define desired goal state
        self.goal_r_I  = vertcat(SX.sym('des_goal_rx'), SX.sym('des_goal_ry'), SX.sym('des_goal_rz'))
        self.goal_v_I = vertcat(SX.sym('des_goal_vx'), SX.sym('des_goal_vy'), SX.sym('des_goal_vz'))
        self.goal_q = vertcat(SX.sym('des_goal_q0'), SX.sym('des_goal_q1'), SX.sym('des_goal_q2'), SX.sym('des_goal_q3'))
        self.goal_w_B= vertcat(SX.sym('des_goal_wx'), SX.sym('des_goal_wy'), SX.sym('des_goal_wz'))
        
        self.goal_state=vertcat(self.goal_r_I,self.goal_v_I,self.goal_q)#,self.goal_w_B)

        ###################################################################
        ###-----------ellipse drone collision detection-----------------###
        ###################################################################
        self.uav_radius = casadi.SX.sym('uav_radius',1)
        self.uav_height = casadi.SX.sym('uav_height',1)
        
        
        
    
    def initDyn(self, Jx=None, Jy=None, Jz=None, mass=None, l=None, c=None):
        # global parameter
        g = 9.81
        # parameters settings
        parameter = []
        if Jx is None:
            self.Jx = SX.sym('Jx')
            parameter += [self.Jx]
        else:
            self.Jx = Jx

        if Jy is None:
            self.Jy = SX.sym('Jy')
            parameter += [self.Jy]
        else:
            self.Jy = Jy

        if Jz is None:
            self.Jz = SX.sym('Jz')
            parameter += [self.Jz]
        else:
            self.Jz = Jz

        if mass is None:
            self.mass = SX.sym('mass')
            parameter += [self.mass]
        else:
            self.mass = mass

        if l is None:
            self.l = SX.sym('l')
            parameter += [self.l]
        else:
            self.l = l

        if c is None:
            self.c = SX.sym('c')
            parameter += [self.c]
        else:
            self.c = c

        self.dyn_auxvar = vcat(parameter)

        # Angular moment of inertia
        self.J_B = diag(vertcat(self.Jx, self.Jy, self.Jz))
        # Gravity
        self.g_I = vertcat(0, 0, -g)
        # Mass of rocket, assume is little changed during the landing process
        self.m = self.mass

        
        # thrust = self.T_B[0] + self.T_B[1] + self.T_B[2] + self.T_B[3]
        self.thrust_B_vec = vertcat(0, 0, self.thrust_mag)
        # total moment M in body frame
        Mx = -self.T_B[1] * self.l / 2 + self.T_B[3] * self.l / 2
        My = -self.T_B[0] * self.l / 2 + self.T_B[2] * self.l / 2
        Mz = (self.T_B[0] - self.T_B[1] + self.T_B[2] - self.T_B[3]) * self.c

        self.u_m = np.array([
            [1,1,1,1],
            [0,-self.l/2,0,self.l/2],
            [-self.l/2,0,self.l/2,0],
            [self.c,-self.c,self.c,-self.c]
        ])
        self.M_B = vertcat(Mx, My, Mz)

        Mx = self.T_B[0] * sqrt(2)*self.l / 4 -self.T_B[1] * sqrt(2)*self.l / 4 - self.T_B[2] * sqrt(2)*self.l / 4 + self.T_B[3] * sqrt(2)*self.l / 4
        My = self.T_B[0] * sqrt(2)*self.l / 4 +self.T_B[1] * sqrt(2)*self.l / 4 - self.T_B[2] * sqrt(2)*self.l / 4 - self.T_B[3] * sqrt(2)*self.l / 4
        Mz = (self.T_B[0] + self.T_B[1] - self.T_B[2] - self.T_B[3]) * self.c


        # cosine directional matrix
        C_B_I = dir_cosine(self.q)  # inertial to body
        C_I_B = transpose(C_B_I)  # body to inertial

        # Newton's law
        dr_I = self.v_I
        dv_I = 1 / self.m * mtimes(C_I_B, self.thrust_B_vec) + self.g_I
        
        # Euler's law
        dq = 1 / 2 * mtimes(self.omega(self.ang_rate_B), self.q)
        
        dw = mtimes(inv(self.J_B), self.M_B - mtimes(mtimes(self.skew(self.ang_rate_B), self.J_B), self.ang_rate_B))
        
        # state
        self.X = vertcat(self.r_I, self.v_I, self.q)#, self.ang_rate_B)
        
        # input
        # self.U = self.T_B
        self.U=vertcat(self.thrust_mag,self.ang_rate_B)
        self.Ulast = vertcat(self.thrust_mag_last,self.ang_rate_B_last)

        # dynamics
        self.f = vertcat(dr_I, dv_I, dq)#, dw)

    def initCost(self, wrt=None, wqt=None,max_tra_w=0,gamma=0,
                 wrp=None, wvp=None, wqp=None,
                wrf=None, wvf=None, wqf=None, 
                wwt=None, wwt_z=None,wthrust=0.5,wInputDiff=10):
        #traverse
        parameter = []
        if wrt is None:
            self.wrt = SX.sym('wrt')
            parameter += [self.wr]
        else:
            self.wrt = wrt

        if wqt is None:
            self.wqt = SX.sym('wqt')
            parameter += [self.wq]
        else:
            self.wqt = wqt

        #path
        if wrf is None:
            self.wrp = SX.sym('wrp')
            parameter += [self.wrp]
        else:
            self.wrp = wrp
        
        if wvp is None:
            self.wvp = SX.sym('wvp')
            parameter += [self.wvp]
        else:
            self.wvp = wvp
        
        if wqp is None:
            self.wqp = SX.sym('wqp')
            parameter += [self.wqp]
        else:
            self.wqp = wqp
        # Terminal cost
        if wrf is None:
            self.wrf = SX.sym('wrf')
            parameter += [self.wrf]
        else:
            self.wrf = wrf
        
        if wvf is None:
            self.wvf = SX.sym('wvf')
            parameter += [self.wvf]
        else:
            self.wvf = wvf
        
        if wqf is None:
            self.wqf = SX.sym('wqf')
            parameter += [self.wqf]
        else:
            self.wqf = wqf
        
        if wwt is None:
            self.wwt = SX.sym('wwt')
            parameter += [self.wwt]
        else:
            self.wwt = wwt

        if wwt_z is None:
            self.wwt_z = SX.sym('wwt_z')
            parameter += [self.wwt_z]
        else:
            self.wwt_z = wwt_z

        #thrust
        if wthrust is None:
            self.wthrust = SX.sym('wthrust')
            parameter += [self.wthrust]
        else:
            self.wthrust = wthrust


        # traversing manually set params
        if max_tra_w is None:
            self.max_tra_w = SX.sym('max_tra_w')
            parameter += [self.max_tra_w]
        else:
            self.max_tra_w = max_tra_w
        
        if gamma is None:
            self.gamma = SX.sym('gamma')
            parameter += [self.gamma]
        else:
            self.gamma = gamma
        
        if wInputDiff is None:
            self.wInputDiff = SX.sym('wInputDiff')
            parameter += [self.wInputDiff]
        else:
            self.wInputDiff = wInputDiff
        
        self.cost_auxvar = vcat(parameter)

        ## goal cost
        # goal position in the world frame
        # self.goal_r_I is the external variable of the acados
        self.cost_r_I_g = dot(self.r_I - self.goal_r_I, self.r_I - self.goal_r_I)

        # goal velocity
        # self.goal_v_I is the external variable of the acados
        self.cost_v_I_g = dot(self.v_I - self.goal_v_I, self.v_I - self.goal_v_I)

        # final attitude error
        # self.goal_q = toQuaternion(goal_atti[0],goal_atti[1])
        goal_R_B_I = dir_cosine(self.goal_q)
        R_B_I = dir_cosine(self.q)
        self.cost_q_g = trace(np.identity(3) - mtimes(transpose(goal_R_B_I), R_B_I))
        # self.cost_q_g = 2-sqrt(1+trace(mtimes(transpose(goal_R_B_I), R_B_I)))
        ## angular velocity cost
        self.goal_w_B = [0, 0, 0]
        self.cost_ang_rate_B = dot(self.ang_rate_B[0:2] - self.goal_w_B[0:2], self.ang_rate_B[0:2] - self.goal_w_B[0:2])
        self.cost_ang_rate_B_z = dot(self.ang_rate_B[2] - self.goal_w_B[2], self.ang_rate_B[2] - self.goal_w_B[2])
        self.cost_thrust = dot(self.thrust_mag, self.thrust_mag) 


        self.input_cost = self.wthrust * self.cost_thrust \
                         + self.wwt  * self.cost_ang_rate_B \
                         + self.wwt_z* self.cost_ang_rate_B_z
        ## input difference cost
        # self.input_diff_cost = self.wInputDiff*dot(self.U - self.Ulast, self.U - self.Ulast)
        
        ## the final (goal) cost
        self.goal_cost =  self.wrp * self.cost_r_I_g \
                        + self.wvp * self.cost_v_I_g \
                        + self.wqp * self.cost_q_g \
                     
        
        self.final_cost =  self.wrf * self.cost_r_I_g\
                         + self.wvf * self.cost_v_I_g\
                         + self.wqf * self.cost_q_g
        
    def vee_map(self,mat):
        return ca.vertcat(mat[2, 1], mat[0, 2], mat[1, 0])
    
    def init_TraCost(self): # transforming Rodrigues to Quaternion is shown in mpc_update function
        ## traverse cost
        # traverse position in the world frame
        """   
        acados solver external variables:   
        self.t_node
        self.des_t_r_I
        self.des_t_tra
        self.des_tra_rodi_param 

        trav_auxvar: (hyperparameters for PDP analytics gradient objects)
        self.des_t_r_I
        self.des_t_tra
        self.des_tra_rodi_param 

        """
        # replaced by symbolic variables: des_tra_r_I, des_tra_q
        tra_atti = Rd2Rp_casadi(self.des_tra_rodi_param)
        self.des_tra_q=toQuaternion_casadi(tra_atti[0],tra_atti[1])

        self.cost_r_I_t = dot(self.r_I - self.des_tra_r_I, self.r_I - self.des_tra_r_I)

        # traverse attitude error
        tra_R_B_I = dir_cosine(self.des_tra_q)
        R_B_I = dir_cosine(self.q)
        self.cost_q_t = trace(np.identity(3) - mtimes(transpose(tra_R_B_I), R_B_I))
        # self.cost_q_t = 2-sqrt(1+trace(mtimes(transpose(tra_R_B_I), R_B_I)))



        # weight = max_tra_w*casadi.exp(-gamma*(dt*i-t_tra)**2) #gamma should increase as the flight duration decreases
        self.tra_cost = self.max_tra_w * casadi.exp(-self.gamma*(self.t_node-self.des_t_tra)**2) * (self.wrt * self.cost_r_I_t + self.wqt * self.cost_q_t)
         
                    
        
        ## set traverse pose as the auxiliary variables (hyperparameters)
        self.trav_auxvar = vertcat(self.des_tra_r_I, self.des_tra_rodi_param,self.des_t_tra)

    def setDyn(self, dt):       

        # self.dyn = casadi.Function('f',[self.X, self.U],[self.f])
        self.dyn = self.X + dt * self.f
        self.dyn_fn = casadi.Function('dynamics', [self.X, self.U], [self.dyn])

        #M = 4
        #DT = dt/4
        #X0 = casadi.SX.sym("X", self.X.numel())
        #U = casadi.SX.sym("U", self.U.numel())
        # #
        #X = X0
        #for _ in range(M):
            # --------- RK4------------
        #    k1 =DT*self.dyn(X, U)
        #    k2 =DT*self.dyn(X+0.5*k1, U)
        #    k3 =DT*self.dyn(X+0.5*k2, U)
        #    k4 =DT*self.dyn(X+k3, U)
            #
        #    X = X + (k1 + 2*k2 + 2*k3 + k4)/6        
        # Fold
        #self.dyn_fn = casadi.Function('dyn', [X0, U], [X])

    ## below is for animation (demo)
    def get_quadrotor_position(self, wing_len, state_traj):

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
    
    def get_quadrotor_position_tensor(self, wing_len, state_traj):
        # thrust_position in body frame
        r1 = torch.tensor([wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)),
                        wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)), 0.0], dtype=state_traj.dtype)
        r2 = torch.tensor([-wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)),
                        wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)), 0.0], dtype=state_traj.dtype)
        r3 = torch.tensor([-wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)),
                        -wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)), 0.0], dtype=state_traj.dtype)
        r4 = torch.tensor([wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)),
                        -wing_len * 0.5 / torch.sqrt(torch.tensor(2.0)), 0.0], dtype=state_traj.dtype)

        # horizon
        horizon = state_traj.shape[0]
        position = torch.zeros((horizon, 15), dtype=state_traj.dtype)
        
        for t in range(horizon):
            # position of COM
            rc = state_traj[t, 0:3]
            # altitude of quaternion
            q = state_traj[t, 6:10]

            # direction cosine matrix from body to inertial
            CIB = self.dir_cosine_tensor(q).transpose(0, 1)

            # position of each rotor in inertial frame
            r1_pos = rc + torch.matmul(CIB, r1)
            r2_pos = rc + torch.matmul(CIB, r2)
            r3_pos = rc + torch.matmul(CIB, r3)
            r4_pos = rc + torch.matmul(CIB, r4)

            # store
            position[t, 0:3] = rc
            position[t, 3:6] = r1_pos
            position[t, 6:9] = r2_pos
            position[t, 9:12] = r3_pos
            position[t, 12:15] = r4_pos

        return position

    def get_final_position(self,wing_len, p= None,q = None):
        p = self.tra_r_I
        q = self.tra_q
        r1 = vertcat(wing_len*0.5 / sqrt(2), wing_len*0.5 / sqrt(2), 0)
        r2 = vertcat(-wing_len*0.5 / sqrt(2), wing_len*0.5 / sqrt(2), 0)
        r3 = vertcat(-wing_len*0.5 / sqrt(2), -wing_len*0.5 / sqrt(2), 0)
        r4 = vertcat(wing_len*0.5 / sqrt(2), -wing_len*0.5 / sqrt(2), 0)

        # r1 = vertcat(wing_len*0.5, 0, 0)
        # r2 = vertcat(0,-wing_len*0.5, 0)
        # r3 = vertcat(-wing_len*0.5,0, 0)
        # r4 = vertcat(0, wing_len*0.5, 0)

        CIB = np.transpose(dir_cosine(q).full())
 
        r1_pos = p + mtimes(CIB, r1).full().flatten()   
        r2_pos = p + mtimes(CIB, r2).full().flatten()
        r3_pos = p + mtimes(CIB, r3).full().flatten()
        r4_pos = p + mtimes(CIB, r4).full().flatten()

        position = np.zeros(15)
        position[0:3] = p
        position[3:6] = r1_pos
        position[6:9] = r2_pos
        position[9:12] = r3_pos
        position[12:15] = r4_pos

        return position

    

    def play_animation(self, wing_len, state_traj, gate_traj1=None, gate_traj2=None,state_traj_ref=None, dt=0.01, \
            point1 = None,point2 = None,point3 = None,point4 = None,save_option=0, title='UAV Maneuvering',\
                goal_pos=[0,0,0]):
        font1 = {'family':'Times New Roman',
         'weight':'normal',
         'style':'normal', 'size':7}
        cm_2_inch = 2.54
        fig = plt.figure(figsize=(8/cm_2_inch,8*0.65/cm_2_inch),dpi=400)
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X (m)', labelpad=-13,**font1)
        ax.set_ylabel('Y (m)', labelpad=-13,**font1)
        ax.set_zlabel('Z (m)', labelpad=-13,**font1)
        ax.tick_params(axis='x',which='major',pad=-5)
        ax.tick_params(axis='y',which='major',pad=-5)
        ax.tick_params(axis='z',which='major',pad=-5)
        ax.set_zlim(-1, 3)
        ax.set_ylim(-2, 2)#9
        ax.set_xlim(-2, 2)#6
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
        ax.view_init(25,-150)
        #plot the final state
        #final_position = self.get_final_position(wing_len=wing_len)
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
        position = self.get_quadrotor_position(wing_len, state_traj)
        sim_horizon = np.size(position, 0)

        if state_traj_ref is None:
            position_ref = self.get_quadrotor_position(0, numpy.zeros_like(position))
        else:
            position_ref = self.get_quadrotor_position(wing_len, state_traj_ref)

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
                    line_traj_ref, time_text
                                            #, line_arm1_ref, line_arm2_ref, line_arm3_ref, line_arm4_ref
            return line_traj, line_arm1, line_arm2, line_arm3, line_arm4, \
                line_traj_ref, time_text #, line_arm1_ref, line_arm2_ref, line_arm3_ref, line_arm4_ref, time_text
     

        frames=np.arange(0,500)
        ani = animation.FuncAnimation(fig,update_traj,frames, interval=1, blit=True)

        if save_option != 0:
            Writer = animation.writers['ffmpeg']
            writer = Writer(fps=10, metadata=dict(artist='Me'), bitrate=-1)
            ani.save('case2'+title + '.mp4', writer=writer, dpi=300)
            print('save_success')

        plt.show()

    def plot_position(self,state_traj,name,dt = 0.1):
        fig, axs = plt.subplots(3)
        fig.suptitle(f'{name}+position vs t')
        N = len(state_traj[:,0])
        x = np.arange(0,N*dt,dt)
        axs[0].plot(x,state_traj[:,0])
        axs[1].plot(x,state_traj[:,1])
        axs[2].plot(x,state_traj[:,2])
        plt.savefig(f'./python_sim_result/{name}+position.png')
        # plt.show()
        
    def plot_velocity(self,state_traj,dt = 0.1):
        fig, axs = plt.subplots(3)
        fig.suptitle('velocity vs t')
        N = len(state_traj[:,0])
        x = np.arange(0,N*dt,dt)
        axs[0].plot(x,state_traj[:,3])
        axs[1].plot(x,state_traj[:,4])
        axs[2].plot(x,state_traj[:,5])
        plt.savefig('./python_sim_result/velocity.png')
        # plt.show()

    def plot_quaternions(self,state_traj,dt = 0.1,save=True):
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
    
    def plot_angularrate(self,state_traj,dt = 0.01):
        plt.figure() 
        plt.title('angularrate vs time')
        N = len(state_traj[:,0])
        x = np.arange(0,N*dt,dt)
        plt.plot(x,state_traj[:,1],color = 'b', label = 'w1')
        plt.plot(x,state_traj[:,2],color = 'r', label = 'w2')
        plt.plot(x,state_traj[:,3],color = 'y', label = 'w3')
        plt.xlabel('t')
        plt.ylabel('w')
        plt.grid(True,color='0.6',dashes=(2,2,1,1))
        plt.legend()
        plt.savefig('./python_sim_result/angularrate.png')
        # plt.show()
        

    def plot_thrust(self,control_traj,dt = 0.1):
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
        
    def plot_trav_weight(self,trav_weight):
        plt.figure() 
        plt.plot(trav_weight)
        plt.title('trav_weight vs time')
        plt.xlabel('t')
        plt.ylabel('trav_weight')
        plt.grid(True,color='0.6',dashes=(2,2,1,1))
        plt.legend()
        plt.savefig('./python_sim_result/trav_weight.png')
        # plt.show()

    def plot_solving_time(self,solving_time):
        plt.figure()    
        plt.plot(solving_time)
        plt.title('mpc solving time at the main loop')
        plt.savefig('./python_sim_result/solving_time.png') 
        # plt.show()
               
    def plot_T(self,control_traj,dt = 0.1):
        N = int(len(control_traj[:,0]))
        x = np.arange(0,round(N*dt,1),dt)
        plt.plot(x,control_traj[:,0],color = 'b', label = 'T')
        plt.title('input vs time')
        plt.ylim([0,20])
        plt.xlabel('t')
        plt.ylabel('T')
        plt.grid(True,color='0.6',dashes=(2,2,1,1))
        plt.legend()
        plt.savefig('./python_sim_result/input_T.png')
        # plt.show()
        
    
    def plot_M(self,control_traj,dt = 0.1):
        N = int(len(control_traj[:,0]))
        x = np.arange(0,round(N*dt,1),dt)
        plt.plot(x,control_traj[:,1],color = 'r', label = 'Mx')
        plt.plot(x,control_traj[:,2],color = 'y', label = 'My')
        plt.plot(x,control_traj[:,3],color = 'g', label = 'Mz')
        plt.title('input vs time')
        plt.ylim([0,1])
        plt.xlabel('t')
        plt.ylabel('T')
        plt.grid(True,color='0.6',dashes=(2,2,1,1))
        plt.legend()
        plt.savefig('./input_M.png')
        plt.show()
        

    def plot_trav_time(self,trav_time):
        plt.figure() 
        plt.plot(trav_time)
        plt.title('trav_time vs time')
        plt.xlabel('t')
        plt.ylabel('trav_time')
        plt.grid(True,color='0.6',dashes=(2,2,1,1))
        plt.legend()
        plt.savefig('./python_sim_result/trav_time.png')
        # plt.show()

    def plot_3D_traj(self,
                     wing_len,
                     uav_height,
                     state_traj,
                     gate_traj,
                     TRAIN_VIS=False,
                     tra_node=None):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        position = self.get_quadrotor_position(wing_len, state_traj)

        

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
    
    
    # def dir_cosine(self, q): # world frame to body frame
    #     C_B_I = vertcat(
    #         horzcat(1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])),
    #         horzcat(2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])),
    #         horzcat(2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
    #     )
    #     return C_B_I

    def dir_cosine_tensor(self, q):
        # World frame to body frame direction cosine matrix
        C_B_I = torch.stack([
            torch.stack([1 - 2 * (q[2] ** 2 + q[3] ** 2), 2 * (q[1] * q[2] + q[0] * q[3]), 2 * (q[1] * q[3] - q[0] * q[2])]),
            torch.stack([2 * (q[1] * q[2] - q[0] * q[3]), 1 - 2 * (q[1] ** 2 + q[3] ** 2), 2 * (q[2] * q[3] + q[0] * q[1])]),
            torch.stack([2 * (q[1] * q[3] + q[0] * q[2]), 2 * (q[2] * q[3] - q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2)])
        ])
        return C_B_I
    def skew(self, v):
        v_cross = vertcat(
            horzcat(0, -v[2], v[1]),
            horzcat(v[2], 0, -v[0]),
            horzcat(-v[1], v[0], 0)
        )
        return v_cross

    def omega(self, w):
        omeg = vertcat(
            horzcat(0, -w[0], -w[1], -w[2]),
            horzcat(w[0], 0, w[2], -w[1]),
            horzcat(w[1], -w[2], 0, w[0]),
            horzcat(w[2], w[1], -w[0], 0)
        )
        return omeg

    def quaternion_mul(self, p, q):
        return vertcat(p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
                       p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
                       p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
                       p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]
                       )

## define the class of the gate (kinematics)
class Gate:
    ## using 12 coordinates to define a gate
    def __init__(self, gate_point = None):
        self.gate_point = gate_point

        ##obtain the position (centroid)
        self.centroid = np.array([np.mean(self.gate_point[:,0]),np.mean(self.gate_point[:,1]),np.mean(self.gate_point[:,2])])

        ## obtain the orientation using the unit vector in the world frame
        az = norm(np.array([0,0,1]))
        ay = norm(np.cross(self.gate_point[1]-self.gate_point[0],self.gate_point[2]-self.gate_point[1]))
        ax = np.cross(ay,az)
        self.ay = ay
        self.I_G = np.array([ax,ay,az]).T # rotaton matrix from the world frame to the gap-attached frame

    ## rotate an angle around y axis of thw window
    def rotate_y(self,angle):
        ## define the rotation matrix to rotate
        rotation = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        gate_point = self.gate_point - np.array([self.centroid,self.centroid,self.centroid,self.centroid])
        for i in range(4):
            [gate_point[i,0],gate_point[i,2]] = np.matmul(rotation,np.array([gate_point[i,0],gate_point[i,2]]))
        self.gate_point = gate_point + np.array([self.centroid,self.centroid,self.centroid,self.centroid])

        ## update the orientation and the position
        self.centroid = np.array([np.mean(self.gate_point[:,0]),np.mean(self.gate_point[:,1]),np.mean(self.gate_point[:,2])])
        az = norm(np.array([0,0,1]))
        ay = norm(np.cross(self.gate_point[1]-self.gate_point[0],self.gate_point[2]-self.gate_point[1]))
        ax = np.cross(ay,az)
        self.ay = ay
        self.I_G = np.array([ax,ay,az]) # rotation matrix from gate frame to world frame

    ## rotate an angle around z axis of thw window
    def rotate(self,angle):
        ## define the rotation matrix to rotate
        rotation = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        gate_point = self.gate_point - np.array([self.centroid,self.centroid,self.centroid,self.centroid])
        for i in range(4):
            gate_point[i,0:2] = np.matmul(rotation,gate_point[i,0:2])
        self.gate_point = gate_point + np.array([self.centroid,self.centroid,self.centroid,self.centroid])

        ## update the orientation and the position
        self.centroid = np.array([np.mean(self.gate_point[:,0]),np.mean(self.gate_point[:,1]),np.mean(self.gate_point[:,2])])
        az = norm(np.array([0,0,1]))
        ay = norm(np.cross(self.gate_point[1]-self.gate_point[0],self.gate_point[2]-self.gate_point[1]))
        ax = np.cross(ay,az)
        self.ay = ay
        self.I_G = np.array([ax,ay,az])

    ## translate the gate in world frame
    def translate(self,displace):
        self.gate_point = self.gate_point + np.array([displace,displace,displace,displace])
        self.centroid = np.array([np.mean(self.gate_point[:,0]),np.mean(self.gate_point[:,1]),np.mean(self.gate_point[:,2])])

        ## update the orientation and the positio
        az = norm(np.array([0,0,1]))
        ay = norm(np.cross(self.gate_point[1]-self.gate_point[0],self.gate_point[2]-self.gate_point[1]))
        ax = np.cross(ay,az)
        self.ay = ay
        self.I_G = np.array([ax,ay,az]) # this is a rotation matrix from gate frame to inertial frame, which is an identity matrix.

    ## 'out' means return the 12 coordinates of the gate
    def translate_out(self,displace):
        return self.gate_point + np.array([displace,displace,displace,displace])

    def rotate_y_out(self,angle):
        rotation = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        gate_point = self.gate_point - np.array([self.centroid,self.centroid,self.centroid,self.centroid])
        for i in range(4):
            [gate_point[i,0],gate_point[i,2]] = np.matmul(rotation,np.array([gate_point[i,0],gate_point[i,2]]))
        gate_point = gate_point + np.array([self.centroid,self.centroid,self.centroid,self.centroid])
        return gate_point

    def rotate_out(self,angle):
        rotation = np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
        gate_point = self.gate_point - np.array([self.centroid,self.centroid,self.centroid,self.centroid])
        for i in range(4):
            gate_point[i,0:2] = np.matmul(rotation,gate_point[i,0:2])
        gate_point = gate_point + np.array([self.centroid,self.centroid,self.centroid,self.centroid])
        return gate_point

    ## given time horizon T and time interval dt, return a sequence of position representing the random move of the gate
    # def random_move(self, T = 4, dt = 0.01):
    #     gate_point = self.gate_point
    #     move = [gate_point]
    #     ## initial random velocity
    #     velo = np.random.normal(0,0.2,size=2)
    #     for i in range(int(T/dt)):
    #         ## random acceleration
    #         accel = np.random.normal(0,2,size=2)
    #         ## integration
    #         velo += dt*accel
    #         velocity = np.clip(np.array([velo[0],0,velo[1]]),-0.4,0.4)
    #         for j in range(4):
    #             gate_point[j] += dt * velocity
    #         move = np.concatenate((move,[gate_point]),axis=0)
    #     return move
    
    ## given constant velocity and angular velocity around y axis, return a sequence of position representing the random move of the gate 
    def move(self, T = 5, dt = 0.01, v = [0,0,0], w = 0):
        gate_point = self.gate_point
        gate_points_list = [gate_point]
        velo = np.array(v) 
        V    = [velo]
        
        # define the rotation matrix
        rotation = np.array([[math.cos(dt*w),-math.sin(dt*w)],[math.sin(dt*w),math.cos(dt*w)]])
        for i in range(int(T/dt)):
            v_noise = np.clip(np.random.normal(0,0.1,3),-0.1,0.1)
            centroid = np.array([np.mean(gate_point[:,0]),np.mean(gate_point[:,1]),np.mean(gate_point[:,2])])
            gate_pointx = gate_point - np.array([centroid,centroid,centroid,centroid]) # coordinates in the window body frame
            # rotation about the y axis
            for i in range(4):
                [gate_pointx[i,0],gate_pointx[i,2]] = np.matmul(rotation,np.array([gate_pointx[i,0],gate_pointx[i,2]]))
            gate_point = gate_pointx + np.array([centroid,centroid,centroid,centroid])
            # translation
            for j in range(4):
                gate_point[j] += dt * (velo+v_noise)
            gate_points_list = np.concatenate((gate_points_list,[gate_point]),axis=0)
            V    = np.concatenate((V,[velo+v_noise]),axis=0)
        return gate_points_list, V
    
    ## transform the state in world frame to the state in window frame
    def transform(self, inertial_state):
        outputs = np.zeros(10)
        ## position
        outputs[0:3] = np.matmul(self.I_G, inertial_state[0:3] - self.centroid) # relative position, the future gap is viewed to be static
        ## velocity
        outputs[3:6] = np.matmul(self.I_G, inertial_state[3:6])
        # ## angular velocity
        # outputs[10:13] = inertial_state[10:13]
        ## attitude
        quat = np.zeros(4)
        quat[0:3] = inertial_state[7:10]
        quat[3] = inertial_state[6]
        r1 = R.from_quat(quat)
        # attitude transformation
        r2 = R.from_matrix(np.matmul(self.I_G,r1.as_matrix()))
        quat_out = np.array(r2.as_quat())
        outputs[6] = quat_out[3]
        outputs[7:10] = quat_out[0:3]
        return outputs

    ## transform the final point in world frame to the point in window frame
    def t_final(self, final_point):
        return np.matmul(self.I_G, final_point - self.centroid)
def Rd2Rp(tra_ang):
    theta = 2*math.atan(magni(tra_ang))
    vector = norm(tra_ang+np.array([1e-8,0,0]))
    return [theta,vector]


def Rd2Rp_casadi(tra_ang):
    theta = 2*casadi.atan(magni_casadi(tra_ang))
    vector = tra_ang+np.array([1e-8,0,0])/ca.norm_2(tra_ang+np.array([1e-8,0,0]))
    return [theta,vector]

def toQuaternion(angle, dir):
    if type(dir) == list:
        dir = numpy.array(dir)
    dir = dir / numpy.linalg.norm(dir)
    quat = numpy.zeros(4)
    quat[0] = math.cos(angle / 2)
    quat[1:] = math.sin(angle / 2) * dir
    return quat.tolist()


def toQuaternion_casadi(angle, dir):
    
    dir = dir / casadi.norm_2(dir)
    quat = casadi.SX.zeros(4)
    quat[0] = casadi.cos(angle / 2)
    quat[1:] = casadi.sin(angle / 2) * dir
    return quat


# normalized verctor
def normalizeVec(vec):
    if type(vec) == list:
        vec = np.array(vec)
    vec = vec / np.linalg.norm(vec)
    return vec


def quaternion_conj(q):
    conj_q = q
    conj_q[1] = -q[1]
    conj_q[2] = -q[2]
    conj_q[3] = -q[3]
    return conj_q