##this file is to generate model of quadrotor

from casadi import *
import casadi
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
from geometry.solid_geometry import norm
from math import sqrt

from config import mission_cfg
from geometry.solid_geometry import dir_cosine, SVD, magni, magni_casadi
# quadrotor (UAV) environment
class QuadrotorDynamic:
    """Only the dynamics, support different state and input dimension.

    Returns:
        
    """
    def __init__(self,ctl_mode:int=0,config=None):
        self.dt=config['learning_agile']['dt']
        self.ctl_mode=ctl_mode
        # define the state of the quadrotor
        rx, ry, rz = SX.sym('rx'), SX.sym('ry'), SX.sym('rz')
        cur_rx, cur_ry, cur_rz = SX.sym('cur_rx'), SX.sym('cur_ry'), SX.sym('cur_rz')
        self.r_I = vertcat(rx, ry, rz)
        self.cur_r_I = vertcat(cur_rx, cur_ry, cur_rz)
        vx, vy, vz = SX.sym('vx'), SX.sym('vy'), SX.sym('vz')
        self.v_I = vertcat(vx, vy, vz)

        # quaternions attitude of B w.r.t. I
        q0, q1, q2, q3 = SX.sym('q0'), SX.sym('q1'), SX.sym('q2'), SX.sym('q3')
        self.q = vertcat(q0, q1, q2, q3)

        # body rate
        wx, wy, wz = SX.sym('wx'), SX.sym('wy'), SX.sym('wz')
        self.ang_rate_B = vertcat(wx, wy, wz)

        if self.ctl_mode==0 or self.ctl_mode==2:
            # collective thrust in body frame 
            self.col_thrust_mag=SX.sym('thrust')
        elif self.ctl_mode==1 or self.ctl_mode==3:   
            # single motor thrust
            f1, f2, f3, f4 = SX.sym('f1'), SX.sym('f2'), SX.sym('f3'), SX.sym('f4')
            self.T_B = vertcat(f1, f2, f3, f4)
            df1, df2, df3, df4 = SX.sym('df1'), SX.sym('df2'), SX.sym('df3'), SX.sym('df4')
            self.delta_T_B = vertcat(df1, df2, df3, df4)
            
        # body torque
        Mx, My, Mz = SX.sym('Mx'), SX.sym('My'), SX.sym('Mz')
        self.M_B = vertcat(Mx, My, Mz)


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
       
        self.m = self.mass

        
    
        if self.ctl_mode==0 or self.ctl_mode==2:   
            self.thrust_B_vec = vertcat(0, 0, self.col_thrust_mag)
        elif self.ctl_mode==1 or self.ctl_mode==3:
            self.thrust_B_vec = vertcat(0, 0, self.T_B[0] + self.T_B[1] + self.T_B[2] + self.T_B[3])

            ###############################
            ###############################
            #^                             ^
            #| 2 G###################### 0 | Y
            #   \          X          /   # 
            #     \        ^        /     #
            #       \      |      /       #
            #         \    |    /         #
            #           \  |  /           #
            #             \ /             #
            # Y <--------- O              #                 
            #             / \             #
            #           /     \           #
            #         /         \         #
            #       /             \       #
            #     /                 \     #
            #|   /                     \   |
            #v 1 ####################### 3 v B
            #R
            # total moment M in body frame

            Mx = -self.T_B[0] * sqrt(2)*self.l / 4 +self.T_B[1] * sqrt(2)*self.l / 4 + self.T_B[2] * sqrt(2)*self.l / 4 - self.T_B[3] * sqrt(2)*self.l / 4
            My = -self.T_B[0] * sqrt(2)*self.l / 4 -self.T_B[1] * sqrt(2)*self.l / 4 + self.T_B[2] * sqrt(2)*self.l / 4 + self.T_B[3] * sqrt(2)*self.l / 4
            Mz = (-self.T_B[0] - self.T_B[1] + self.T_B[2] + self.T_B[3]) * self.c
            self.M_B = vertcat(Mx, My, Mz)
        # else:
        
        
        # cosine directional matrix
        C_B_I = dir_cosine(self.q)  # inertial to body
        C_I_B = transpose(C_B_I)  # body to inertial

        # Newton's law
        self.dr_I = self.v_I
        self.dv_I = 1 / self.m * mtimes(C_I_B, self.thrust_B_vec) + self.g_I
        
        # Euler's law
        self.dq = 1 / 2 * mtimes(omega(self.ang_rate_B), self.q)
        self.dw = mtimes(inv(self.J_B), self.M_B - mtimes(mtimes(skew(self.ang_rate_B), self.J_B), self.ang_rate_B))

        if self.ctl_mode==3:
            self.dT_B = self.T_B + self.delta_T_B*self.dt
        
        # self.u_m = np.array([
        #     [1,1,1,1],
        #     [0,-self.l/2,0,self.l/2],
        #     [-self.l/2,0,self.l/2,0],
        #     [self.c,-self.c,self.c,-self.c]
        # ])

class CostBase:
    """the Base cost definition for the MPC,only declare the symbolic reference and weight of the cost function.
    """

    def __init__(self,ctl_mode:int=0,config=None):
        self.ctl_mode=ctl_mode
        self.drone_mass=config['drone']['mass']
        # define desire traverse pose and time
        self.des_tra_r_I = vertcat(SX.sym('des_tra_rx'), SX.sym('des_tra_ry'), SX.sym('des_tra_rz'))
        self.des_tra_r_B = vertcat(SX.sym('des_tra_rx_B'), SX.sym('des_tra_ry_B'), SX.sym('des_tra_rz_B'))
        self.des_tra_rodi_param=vertcat(SX.sym('des_tra_rodi_param0'),SX.sym('des_tra_rodi_param1'),SX.sym('des_tra_rodi_param2'))

        ##==traverse pose 9D vector == ##
        self.des_tra_m = vertcat(SX.sym('des_tra_m0'),SX.sym('des_tra_m1'),SX.sym('des_tra_m2'),\
                                SX.sym('des_tra_m3'),SX.sym('des_tra_m4'),SX.sym('des_tra_m5'),\
                                SX.sym('des_tra_m6'),SX.sym('des_tra_m7'),SX.sym('des_tra_m8'))
        
        self.des_tra_R=vertcat(SX.sym('des_tra_R0'),SX.sym('des_tra_R1'),SX.sym('des_tra_R2'),\
                              SX.sym('des_tra_R3'),SX.sym('des_tra_R4'),SX.sym('des_tra_R5'),\
                              SX.sym('des_tra_R6'),SX.sym('des_tra_R7'),SX.sym('des_tra_R8'))
        
        self.des_tra_roll_m = vertcat(SX.sym('des_tra_roll_m0'),SX.sym('des_tra_roll_m1'),\
                                      SX.sym('des_tra_roll_m2'),SX.sym('des_tra_roll_m3'))
        
        self.des_tra_pitch_m = vertcat(SX.sym('des_tra_pitch_m0'),SX.sym('des_tra_pitch_m1'),\
                                        SX.sym('des_tra_pitch_m2'),SX.sym('des_tra_pitch_m3'))
        
        self.des_tra_q = vertcat(SX.sym('des_tra_q0'), SX.sym('des_tra_q1'), SX.sym('des_tra_q2'), SX.sym('des_tra_q3'))
        self.des_t_tra = SX.sym('des_t_tra')
        self.t_node = SX.sym('t_node')
        self.tra_throttle = SX.sym('tra_throttle')
        # self.traverse_weight_span = SX.sym('traverse_weight_span ')
        # define desired goal state
        self.goal_r_I  = vertcat(SX.sym('des_goal_rx'), SX.sym('des_goal_ry'), SX.sym('des_goal_rz'))
        self.goal_r_B  = vertcat(SX.sym('des_goal_rx_B'), SX.sym('des_goal_ry_B'), SX.sym('des_goal_rz_B'))
        self.goal_v_I = vertcat(SX.sym('des_goal_vx'), SX.sym('des_goal_vy'), SX.sym('des_goal_vz'))
        self.goal_q = vertcat(SX.sym('des_goal_q0'), SX.sym('des_goal_q1'), SX.sym('des_goal_q2'), SX.sym('des_goal_q3'))
        self.goal_w_B= vertcat(SX.sym('des_goal_wx'), SX.sym('des_goal_wy'), SX.sym('des_goal_wz'))
        self.goal_T_B= vertcat(SX.sym('des_goal_T0'), SX.sym('des_goal_T1'), SX.sym('des_goal_T2'), SX.sym('des_goal_T3'))
    def init_weight(self, wrt=None, wqt=None,max_tra_w=None,traverse_weight_span=None,
                 wrp=None, wvp=None, wqp=None,
                wrf=None, wvf=None, wqf=None, 
                wwt=None, wwt_z=None,w_tra_throttle=None, wthrust=None,wdthrust=None,wm=None):
        
        """
        If the weight value is None, it means this value is a learnable parameter
        """
        #traverse
        parameter = []
        if wrt is None:
            self.wrt = vertcat(SX.sym('wrtx'),SX.sym('wrty'),SX.sym('wrtz'))
        else:
            self.wrt = wrt

        if wqt is None:
            self.wqt = SX.sym('wqt')
        else:
            self.wqt = wqt

        #path
        if wrp is None:
            self.wrp = vertcat(SX.sym('wrpx'),SX.sym('wrpy'),SX.sym('wrpz'))
        else:
            self.wrp = wrp

        
        if wvp is None:
            self.wvp = SX.sym('wvp')
        else:
            self.wvp = wvp
        
        if wqp is None:
            self.wqp = SX.sym('wqp')
        else:
            self.wqp = wqp
        # Terminal cost
        if wrf is None:
            self.wrf = SX.sym('wrf')
        else:
            self.wrf = wrf
        
        if wvf is None:
            self.wvf = SX.sym('wvf')
        else:
            self.wvf = wvf
        
        if wqf is None:
            self.wqf = SX.sym('wqf')
        else:
            self.wqf = wqf
        
        if wwt is None:
            self.wwt = SX.sym('wwt')
        else:
            self.wwt = wwt

        if wwt_z is None:
            self.wwt_z = SX.sym('wwt_z')
        else:
            self.wwt_z = wwt_z

        #thrust
        if wthrust is None:
            self.wthrust = SX.sym('wthrust')
            parameter += [self.wthrust]
        else:
            self.wthrust = wthrust
        #thrust
        if wdthrust is None:
            self.wdthrust = SX.sym('wdthrust')
            parameter += [self.wdthrust]
        else:
            self.wdthrust = wdthrust

        if w_tra_throttle is None:
            self.w_tra_throttle = SX.sym('tra_throttle')
            parameter += [self.w_tra_throttle]
        else:
            self.w_tra_throttle = w_tra_throttle
        # torque weight
        if wm is None:
            self.wm = SX.sym('wthrust')
            parameter += [self.wm]
        else:
            self.wm = wm

        # traversing manually set params
        if max_tra_w is None:
            self.max_tra_w = SX.sym('max_tra_w')
            parameter += [self.max_tra_w]
        else:
            self.max_tra_w = max_tra_w
        
        if traverse_weight_span is None:
            self.traverse_weight_span = SX.sym('gamma')
            parameter += [self.traverse_weight_span]
        else:
            self.traverse_weight_span = traverse_weight_span
          
        self.cost_auxvar = vcat(parameter)
    
    def path_error(self,quad_dyn:QuadrotorDynamic=None):
        """path error includes:
        1   before passing, the position error to the gate
            
        2.  after passing, the full state error is to the goal"""
        # traverse position error 
        self.des_tra_r_I = self.des_tra_r_B + quad_dyn.cur_r_I
        self.e_r_I_t = quad_dyn.r_I - self.des_tra_r_I
       
        ## goal cost
        # goal position in the world frame
        self.goal_r_I = self.goal_r_B + quad_dyn.cur_r_I
        self.e_r_I= quad_dyn.r_I - self.goal_r_I
        self.dot_e_r_I = dot(self.e_r_I,self.e_r_I)
       
        # goal velocity
        self.e_v_I = quad_dyn.v_I - self.goal_v_I
        self.dot_e_v_I = dot(quad_dyn.v_I - self.goal_v_I, quad_dyn.v_I - self.goal_v_I)

        # final attitude error
        goal_R_B_I = dir_cosine(self.goal_q)
        R_B_I = dir_cosine(quad_dyn.q)

        ## squared Chordal distance
        self.e_q_g = casadi.norm_fro(goal_R_B_I-R_B_I)**2

        ## angular velocity cost
        if self.ctl_mode==0:
            self.goal_w_B = [0, 0, 0]
            weight = self.drone_mass*9.81
            self.cost_col_thrust = dot(quad_dyn.col_thrust_mag-weight,quad_dyn.col_thrust_mag-weight)
        elif self.ctl_mode==1 or self.ctl_mode==3:
            self.cost_SRT = dot(quad_dyn.T_B ,quad_dyn.T_B)
            self.cost_dSRT=dot(quad_dyn.delta_T_B,quad_dyn.delta_T_B)
        elif self.ctl_mode==2:
            self.cost_col_thrust = dot(quad_dyn.col_thrust_mag,quad_dyn.col_thrust_mag)
            self.cost_torque = dot(quad_dyn.M_B,quad_dyn.M_B)
        self.cost_ang_rate_B = dot(quad_dyn.ang_rate_B[0:2] - self.goal_w_B[0:2], quad_dyn.ang_rate_B[0:2] - self.goal_w_B[0:2])
        self.cost_ang_rate_B_z = dot(quad_dyn.ang_rate_B[2] - self.goal_w_B[2], quad_dyn.ang_rate_B[2] - self.goal_w_B[2])
       

    def traverse_error(self,quad_dyn:QuadrotorDynamic=None, options=None):
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
        
        svd= SVD()
        self.trav_auxvar = vertcat(self.des_tra_r_B, self.des_tra_m, self.wrp, self.wrt, self.wqt, self.traverse_weight_span) 
        self.tra_R_B_I= svd.SVD_M_to_SO3_ca(self.des_tra_m)
       
        ## =========== traverse cost =========##
        

        # attitude error
        R_B_I = dir_cosine(quad_dyn.q)

        ## squared Chordal distance
        self.cost_q_t = casadi.norm_fro(self.tra_R_B_I-R_B_I)**2

        ## traverse thrust cost
        self.cost_tra_throttle = dot(quad_dyn.col_thrust_mag-self.tra_throttle*mission_cfg['learning_agile']['single_motor_max_thrust']*4,\
                                     quad_dyn.col_thrust_mag-self.tra_throttle*mission_cfg['learning_agile']['single_motor_max_thrust']*4)
       
class QuadrotorCTBRCtl:
    """
    quadrotor model andd cost for Collective Thrust and Body Rate (CTBR) MPC
    """
    def __init__(self,options,config):
        self.options=options
        self.quad_dyn = QuadrotorDynamic(ctl_mode=0,config=config)
        self.cost_base = CostBase(ctl_mode=0,config=config)
        

    def init_model(self):    
        # state
        self.X = vertcat(self.quad_dyn.r_I, self.quad_dyn.v_I, self.quad_dyn.q)
        
        # input
        self.U = vertcat(self.quad_dyn.col_thrust_mag,self.quad_dyn.ang_rate_B)

        # dynamics
        self.f = vertcat(self.quad_dyn.dr_I, self.quad_dyn.dv_I, self.quad_dyn.dq)


    def init_cost(self): 
        self.cost_base.traverse_error(self.quad_dyn,self.options)
        self.cost_base.path_error(self.quad_dyn)
        self.goal_state=vertcat(self.cost_base.goal_r_B,self.cost_base.goal_v_I,self.cost_base.goal_q)

        tanh_wrt = self.cost_base.wrt * (0.5 * (1+casadi.tanh(1000*(self.cost_base.des_t_tra - self.cost_base.t_node))))
        tanh_wrp = self.cost_base.wrp * (0.5 * (1+casadi.tanh(1000*(self.cost_base.t_node - self.cost_base.des_t_tra))))
        tanh_wvp = self.cost_base.wvp * (0.5 * (1+casadi.tanh(1000*(self.cost_base.t_node - self.cost_base.des_t_tra))))
        tanh_wqp = self.cost_base.wqp * (0.5 * (1+casadi.tanh(1000*(self.cost_base.t_node - self.cost_base.des_t_tra))))
        
        cost_r_I_t = self.cost_base.e_r_I_t.T @ casadi.diag(tanh_wrt) @ self.cost_base.e_r_I_t
        cost_r_I_g =   self.cost_base.e_r_I.T @ casadi.diag(tanh_wrp) @ self.cost_base.e_r_I
        cost_v_I_g =   self.cost_base.e_v_I.T @ casadi.diag(tanh_wvp) @ self.cost_base.e_v_I
        cost_q_g   =   self.cost_base.e_q_g.T @ casadi.diag(tanh_wqp) @ self.cost_base.e_q_g
        
        

        ## the path cost to the first to the gate, then to the goal
        self.path_cost = self.cost_base.wwt * self.cost_base.cost_ang_rate_B +  \
                         self.cost_base.wwt_z * self.cost_base.cost_ang_rate_B_z +  \
                         self.cost_base.wthrust* self.cost_base.cost_col_thrust + \
                         cost_r_I_g  + \
                         cost_v_I_g  + \
                         cost_q_g    + \
                         cost_r_I_t 
                        
        # the final cost
        self.final_cost = self.cost_base.wrf * self.cost_base.dot_e_r_I\
                        + self.cost_base.wvf * self.cost_base.dot_e_v_I\
                        + self.cost_base.wqf * self.cost_base.e_q_g        
  
    
    def init_traCost(self): # transforming Rodrigues to Quaternion is shown in mpc_update function
        # cost_r_I_t = self.cost_base.e_r_I_t.T @ casadi.diag(self.cost_base.wrt) @ self.cost_base.e_r_I_t
        self.tra_cost = self.cost_base.max_tra_w * \
                        casadi.exp(-self.cost_base.traverse_weight_span*(self.cost_base.t_node-self.cost_base.des_t_tra)**2) \
                        * (self.cost_base.wqt * self.cost_base.cost_q_t) # + cost_r_I_t)
                        #  + self.cost_base.w_tra_throttle * self.cost_base.cost_tra_throttle)
        
    def set_bound_value(self, config):
        self.col_thrust_ub = config['learning_agile']['single_motor_max_thrust']*4*config['learning_agile']['throttle_upper_bound']
        self.col_thrust_lb = config['learning_agile']['single_motor_max_thrust']*4*config['learning_agile']['throttle_lower_bound']

        self.ang_rate_b_xy = config['learning_agile']['angular_vel_bound_xy']
        self.ang_rate_b_z = config['learning_agile']['angular_vel_bound_z']

        sc= 1 #1e2
        self.pos_b   = config['learning_agile']['pos_bound'] # in each axis
        self.pos_lb_z = config['learning_agile']['pos_lb_z']
        self.pos_ub_z = config['learning_agile']['pos_ub_z']
        self.vel_b   = config['learning_agile']['linear_vel_bound'] #0.5 # in each axis

        self.state_lb = [-self.pos_b,-self.pos_b,-self.pos_b,-self.vel_b,-self.vel_b,-self.vel_b,-sc,-sc,-sc,-sc]
        self.state_ub = [self.pos_b,self.pos_b,self.pos_b,self.vel_b,self.vel_b,self.vel_b,sc,sc,sc,sc]
        self.control_lb = [self.col_thrust_lb,-self.ang_rate_b_xy,-self.ang_rate_b_xy,-self.ang_rate_b_z]
        self.control_ub = [self.col_thrust_ub,self.ang_rate_b_xy,self.ang_rate_b_xy,self.ang_rate_b_z]
        
    def init_constraint(self):
        """
        this function constructs the constraints for the safe PDP
        """
        thrust_ub_inequ=self.quad_dyn.col_thrust_mag-self.col_thrust_ub
        thrust_lb_inequ=self.col_thrust_lb-self.quad_dyn.col_thrust_mag
        ang_rate_ub_inequ=self.quad_dyn.ang_rate_B[0:2]-self.ang_rate_b_xy
        ang_rate_lb_inequ=self.ang_rate_b_xy-self.quad_dyn.ang_rate_B[0:2]
        ang_rate_ub_z_inequ=self.quad_dyn.ang_rate_B[2]-self.ang_rate_b_z
        ang_rate_lb_z_inequ=self.ang_rate_b_z-self.quad_dyn.ang_rate_B[2]
        
        pos_ub_z_inequ=self.quad_dyn.r_I[2]-self.pos_ub_z
        pos_lb_z_inequ=self.pos_lb_z-self.quad_dyn.r_I[2]
        self.path_inequ_cstr=vcat([thrust_ub_inequ,thrust_lb_inequ, \
                                   pos_ub_z_inequ,pos_lb_z_inequ,\
                                   ang_rate_ub_inequ, ang_rate_lb_inequ,\
                                   ang_rate_ub_z_inequ,ang_rate_lb_z_inequ])
    
        self.final_inequ_cstr=vcat([pos_ub_z_inequ,pos_lb_z_inequ])
        
class QuadrotorSRTCtl:
    """
    quadrotor model andd cost for Single rotor thrust MPC
    """
    def __init__(self,options,config):
        self.options=options
        self.quad_dyn = QuadrotorDynamic(ctl_mode=1,config=config)
        self.cost_base = CostBase(ctl_mode=1,config=config)


    def init_model(self):    
        # state
        self.X = vertcat(self.quad_dyn.r_I, self.quad_dyn.v_I, self.quad_dyn.q, self.quad_dyn.ang_rate_B)  
        
        # input
        self.U=self.quad_dyn.T_B

        # dynamics
        self.f = vertcat(self.quad_dyn.dr_I, self.quad_dyn.dv_I, self.quad_dyn.dq, self.quad_dyn.dw)

    def init_cost(self): 
        self.cost_base.path_error(self.quad_dyn)
        self.goal_state=vertcat(self.cost_base.goal_r_I,self.cost_base.goal_v_I,self.cost_base.goal_q,self.cost_base.goal_w_B)
        ## the path cost to the goal
        self.path_cost = self.cost_base.wrp * self.cost_base.cost_r_I_g \
                       + self.cost_base.wvp * self.cost_base.cost_v_I_g \
                       + self.cost_base.wqp * self.cost_base.cost_q_g \
                       + self.cost_base.wwt * self.cost_base.cost_ang_rate_B \
                       + self.cost_base.wwt_z * self.cost_base.cost_ang_rate_B_z\
                       + self.cost_base.wthrust* self.cost_base.cost_SRT

        
        # the final cost
        self.final_cost = self.cost_base.wrf * self.cost_base.cost_r_I_g\
                        + self.cost_base.wvf * self.cost_base.cost_v_I_g\
                        + self.cost_base.wqf * self.cost_base.cost_q_g\
                        + self.cost_base.wwt * self.cost_base.cost_ang_rate_B \
                        + self.cost_base.wwt_z * self.cost_base.cost_ang_rate_B_z       
  
    
    def init_traCost(self): # transforming Rodrigues to Quaternion is shown in mpc_update function
        self.cost_base.traverse_error(self.quad_dyn,self.options)
        self.tra_cost = self.cost_base.max_tra_w * \
                        casadi.exp(-self.cost_base.traverse_weight_span*(self.cost_base.t_node-self.cost_base.des_t_tra)**2) \
                        * (self.cost_base.wrt * self.cost_base.cost_r_I_t + self.cost_base.wqt * self.cost_base.cost_q_t)
         
    def set_bound_value(self, config):
        self.sing_thrust_ub = config['learning_agile']['single_motor_max_thrust']*config['learning_agile']['throttle_upper_bound']
        self.sing_thrust_lb = config['learning_agile']['single_motor_max_thrust']*config['learning_agile']['throttle_lower_bound']

        self.ang_rate_b_xy = config['learning_agile']['angular_vel_bound_xy']
        self.ang_rate_b_z = config['learning_agile']['angular_vel_bound_z']

        sc= 1 #1e2
        self.pos_b   = config['learning_agile']['pos_bound'] # in each axis
        self.pos_lb_z = config['learning_agile']['pos_lb_z']
        self.pos_ub_z = config['learning_agile']['pos_ub_z']
        self.vel_b   = config['learning_agile']['linear_vel_bound'] #0.5 # in each axis

        self.state_lb = [-self.pos_b,-self.pos_b,self.pos_lb_z ,-self.vel_b,-self.vel_b,-self.vel_b,-sc,-sc,-sc,-sc,-self.ang_rate_b_xy,-self.ang_rate_b_xy,-self.ang_rate_b_z]
        self.state_ub = [self.pos_b,self.pos_b,self.pos_ub_z,self.vel_b,self.vel_b,self.vel_b,sc,sc,sc,sc,self.ang_rate_b_xy,self.ang_rate_b_xy,self.ang_rate_b_z]
        self.control_lb = [self.sing_thrust_lb]*4
        self.control_ub = [self.sing_thrust_ub]*4
        
    def init_constraint(self):
        thrust_ub_inequ=self.quad_dyn.T_B-self.control_ub
        thrust_lb_inequ=self.control_lb-self.quad_dyn.T_B
        pos_ub_z_inequ=self.quad_dyn.r_I[2]-self.pos_ub_z
        pos_lb_z_inequ=self.pos_lb_z-self.quad_dyn.r_I[2]
        self.path_inequ_cstr=vcat([thrust_ub_inequ,thrust_lb_inequ,pos_ub_z_inequ,pos_lb_z_inequ])
        self.final_inequ_cstr=vcat([pos_ub_z_inequ,pos_lb_z_inequ])

class QuadrotorWrenchCtl:
    """
    quadrotor model andd cost for force and torque (Wrench) MPC
    """
    def __init__(self,options,config):
        self.options=options
        self.quad_dyn = QuadrotorDynamic(ctl_mode=2,config=config)
        self.cost_base = CostBase(ctl_mode=2,config=config)


    def init_model(self):    
        # state
        self.X = vertcat(self.quad_dyn.r_I, self.quad_dyn.v_I, self.quad_dyn.q, self.quad_dyn.ang_rate_B)  
        
        # input
        self.U=vertcat(self.quad_dyn.col_thrust_mag,self.quad_dyn.M_B)

        # dynamics
        self.f = vertcat(self.quad_dyn.dr_I, self.quad_dyn.dv_I, self.quad_dyn.dq, self.quad_dyn.dw)

    def init_cost(self): 
        self.cost_base.path_error(self.quad_dyn)
        self.goal_state=vertcat(self.cost_base.goal_r_I,self.cost_base.goal_v_I,self.cost_base.goal_q,self.cost_base.goal_w_B)
        ## the path cost to the goal
        self.path_cost = self.cost_base.wrp * self.cost_base.cost_r_I_g \
                       + self.cost_base.wvp * self.cost_base.cost_v_I_g \
                       + self.cost_base.wqp * self.cost_base.cost_q_g \
                       + self.cost_base.wwt * self.cost_base.cost_ang_rate_B \
                       + self.cost_base.wwt_z * self.cost_base.cost_ang_rate_B_z\
                       + self.cost_base.wthrust * self.cost_base.cost_col_thrust \
                       + self.cost_base.wm * self.cost_base.cost_torque

        
        # the final cost
        self.final_cost = self.cost_base.wrf * self.cost_base.cost_r_I_g\
                        + self.cost_base.wvf * self.cost_base.cost_v_I_g\
                        + self.cost_base.wqf * self.cost_base.cost_q_g\
                        + self.cost_base.wwt * self.cost_base.cost_ang_rate_B \
                        + self.cost_base.wwt_z * self.cost_base.cost_ang_rate_B_z       
  
    
    def init_traCost(self): # transforming Rodrigues to Quaternion is shown in mpc_update function
        self.cost_base.traverse_error(self.quad_dyn,self.options)
        self.tra_cost = self.cost_base.max_tra_w * \
                        casadi.exp(-self.cost_base.traverse_weight_span*(self.cost_base.t_node-self.cost_base.des_t_tra)**2) \
                        * (self.cost_base.wrt * self.cost_base.cost_r_I_t + self.cost_base.wqt * self.cost_base.cost_q_t)
         
    def set_bound_value(self, config):
        self.col_thrust_ub = config['learning_agile']['single_motor_max_thrust']*4*config['learning_agile']['throttle_upper_bound']
        self.col_thrust_lb = config['learning_agile']['single_motor_max_thrust']*4*config['learning_agile']['throttle_lower_bound']
        self.sing_axis_torque_ub = self.col_thrust_ub*config['drone']['diagonal_axis_dist']/4

        self.ang_rate_b_xy = config['learning_agile']['angular_vel_bound_xy']
        self.ang_rate_b_z = config['learning_agile']['angular_vel_bound_z']

        sc= 1 #1e2
        self.pos_b   = config['learning_agile']['pos_bound'] # in each axis
        self.pos_lb_z = config['learning_agile']['pos_lb_z']
        self.pos_ub_z = config['learning_agile']['pos_ub_z']
        self.vel_b   = config['learning_agile']['linear_vel_bound'] #0.5 # in each axis

        self.state_lb = [-self.pos_b,-self.pos_b,self.pos_lb_z ,-self.vel_b,-self.vel_b,-self.vel_b,-sc,-sc,-sc,-sc,-self.ang_rate_b_xy,-self.ang_rate_b_xy,-self.ang_rate_b_z]
        self.state_ub = [self.pos_b,self.pos_b,self.pos_ub_z,self.vel_b,self.vel_b,self.vel_b,sc,sc,sc,sc,self.ang_rate_b_xy,self.ang_rate_b_xy,self.ang_rate_b_z]
        self.control_lb = [self.col_thrust_lb,-10,-10,-10]
        self.control_ub = [self.col_thrust_ub, 10, 10, 10] 

        
    def init_constraint(self):
        thrust_ub_inequ=self.U-self.control_ub
        thrust_lb_inequ=self.control_lb-self.U
        pos_ub_z_inequ=self.quad_dyn.r_I[2]-self.pos_ub_z
        pos_lb_z_inequ=self.pos_lb_z-self.quad_dyn.r_I[2]
        self.path_inequ_cstr=vcat([thrust_ub_inequ,thrust_lb_inequ,pos_ub_z_inequ,pos_lb_z_inequ])
        self.final_inequ_cstr=vcat([pos_ub_z_inequ,pos_lb_z_inequ])


class QuadrotorAugmentedSRTCtl:
    """
    Augmented quadrotor model andd cost for Single rotor thrust MPC, 
    the control input is the df1,df2,df3,df4
    following the paper: "Model Predictive Contouring Control for  Time-Optimal Quadrotor Flight"
    """
    def __init__(self,options,config):
        self.options=options
        self.quad_dyn = QuadrotorDynamic(ctl_mode=3,config=config)
        self.cost_base = CostBase(ctl_mode=3,config=config)


    def init_model(self):    
        # state
        self.X = vertcat(self.quad_dyn.r_I, self.quad_dyn.v_I, self.quad_dyn.q, self.quad_dyn.ang_rate_B,self.quad_dyn.T_B)
        
        # input
        self.U=self.quad_dyn.delta_T_B

        # dynamics
        self.f = vertcat(self.quad_dyn.dr_I, self.quad_dyn.dv_I, self.quad_dyn.dq, self.quad_dyn.dw,self.quad_dyn.dT_B)

    def init_cost(self): 
        self.cost_base.path_error(self.quad_dyn)
        self.goal_state=vertcat(self.cost_base.goal_r_I,self.cost_base.goal_v_I,self.cost_base.goal_q,self.cost_base.goal_w_B,self.cost_base.goal_T_B)
        ## the path cost to the goal
        self.path_cost = self.cost_base.wrp * self.cost_base.cost_r_I_g \
                       + self.cost_base.wvp * self.cost_base.cost_v_I_g \
                       + self.cost_base.wqp * self.cost_base.cost_q_g \
                       + self.cost_base.wwt * self.cost_base.cost_ang_rate_B \
                       + self.cost_base.wwt_z * self.cost_base.cost_ang_rate_B_z\
                       + self.cost_base.wthrust* self.cost_base.cost_SRT\
                       + self.cost_base.wdthrust* self.cost_base.cost_dSRT

        
        # the final cost
        self.final_cost = self.cost_base.wrf * self.cost_base.cost_r_I_g\
                        + self.cost_base.wvf * self.cost_base.cost_v_I_g\
                        + self.cost_base.wqf * self.cost_base.cost_q_g\
                        + self.cost_base.wwt * self.cost_base.cost_ang_rate_B \
                        + self.cost_base.wwt_z * self.cost_base.cost_ang_rate_B_z \
                        + self.cost_base.wthrust* self.cost_base.cost_SRT       
  
    
    def init_traCost(self): # transforming Rodrigues to Quaternion is shown in mpc_update function
        self.cost_base.traverse_error(self.quad_dyn,self.options)
        self.tra_cost = self.cost_base.max_tra_w * \
                        casadi.exp(-self.cost_base.traverse_weight_span*(self.cost_base.t_node-self.cost_base.des_t_tra)**2) \
                        * (self.cost_base.wrt * self.cost_base.cost_r_I_t + self.cost_base.wqt * self.cost_base.cost_q_t)
         
    def set_bound_value(self, config):
        self.sing_thrust_ub = config['learning_agile']['single_motor_max_thrust']*config['learning_agile']['throttle_upper_bound']
        self.sing_thrust_lb = config['learning_agile']['single_motor_max_thrust']*config['learning_agile']['throttle_lower_bound']

        self.ang_rate_b_xy = config['learning_agile']['angular_vel_bound_xy']
        self.ang_rate_b_z = config['learning_agile']['angular_vel_bound_z']

        sc= 1 #1e2
        self.pos_b   = config['learning_agile']['pos_bound'] # in each axis
        self.pos_lb_z = config['learning_agile']['pos_lb_z']
        self.pos_ub_z = config['learning_agile']['pos_ub_z']
        self.vel_b   = config['learning_agile']['linear_vel_bound'] #0.5 # in each axis

        self.state_lb = [-self.pos_b,-self.pos_b,self.pos_lb_z ,-self.vel_b,-self.vel_b,-self.vel_b,-sc,-sc,-sc,-sc,-self.ang_rate_b_xy,-self.ang_rate_b_xy,-self.ang_rate_b_z\
                         ,self.sing_thrust_lb,self.sing_thrust_lb,self.sing_thrust_lb,self.sing_thrust_lb]
        self.state_ub = [self.pos_b,self.pos_b,self.pos_ub_z,self.vel_b,self.vel_b,self.vel_b,sc,sc,sc,sc,self.ang_rate_b_xy,self.ang_rate_b_xy,self.ang_rate_b_z\
                         ,self.sing_thrust_ub,self.sing_thrust_ub,self.sing_thrust_ub,self.sing_thrust_ub]

        self.control_lb = [-100,-100,-100,-100]
        self.control_ub = [100, 100, 100, 100]
        
    def init_constraint(self):
        thrust_ub_inequ=self.quad_dyn.T_B-self.control_ub
        thrust_lb_inequ=self.control_lb-self.quad_dyn.T_B
        pos_ub_z_inequ=self.quad_dyn.r_I[2]-self.pos_ub_z
        pos_lb_z_inequ=self.pos_lb_z-self.quad_dyn.r_I[2]
        self.path_inequ_cstr=vcat([thrust_ub_inequ,thrust_lb_inequ,pos_ub_z_inequ,pos_lb_z_inequ])
        self.final_inequ_cstr=vcat([pos_ub_z_inequ,pos_lb_z_inequ])
    
# def skew(v):
#     v_cross = vertcat(
#         horzcat(0, -v[2], v[1]),
#         horzcat(v[2], 0, -v[0]),
#         horzcat(-v[1], v[0], 0)
#     )
#     return v_cross

def omega(w):
    omeg = vertcat(
        horzcat(0, -w[0], -w[1], -w[2]),
        horzcat(w[0], 0, w[2], -w[1]),
        horzcat(w[1], -w[2], 0, w[0]),
        horzcat(w[2], w[1], -w[0], 0)
    )
    return omeg

# def quaternion_mul(p, q):
#     return vertcat(p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3],
#                     p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2],
#                     p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1],
#                     p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0]
#                     )

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
        rotation = np.array([[math.cos(angle),math.sin(angle)],[-math.sin(angle),math.cos(angle)]])
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
        # from the body frame to the world frame
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

def get_gate_points(gate_center,gate_length,gate_width):
    return np.array([[gate_center[0]-gate_length/2, gate_center[1], gate_center[2]+gate_width/2],
                     [gate_center[0]+gate_length/2, gate_center[1], gate_center[2]+gate_width/2],
                     [gate_center[0]+gate_length/2, gate_center[1], gate_center[2]-gate_width/2],
                     [gate_center[0]-gate_length/2, gate_center[1], gate_center[2]-gate_width/2]])

def Rd2Rp(tra_ang):
    theta = 2*math.atan(magni(tra_ang))
    vector = norm(tra_ang+np.array([1e-8,0,0]))
    return [theta,vector]


def Rd2Rp_casadi(tra_ang):
    theta = 2*casadi.atan(magni_casadi(tra_ang))
    vector = tra_ang+np.array([1e-8,0,0])/casadi.norm_2(tra_ang+np.array([1e-8,0,0]))
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