## this file is a package for policy search for quadrotor

from quad_OC import OCSys
from scipy.spatial.transform import Rotation as R
from math import cos, pi, sin, sqrt, tan
from quad_model import *
from casadi import *
import scipy.io as sio
import numpy as np
import time
from solid_geometry import *
def Rd2Rp(tra_ang):
    theta = 2*math.atan(magni(tra_ang))
    vector = norm(tra_ang+np.array([1e-8,0,0]))
    return [theta,vector]

class run_quad:
    def __init__(self,config_dict, 
                USE_PREV_SOLVER = False):
        
       

        ######################################################
        #######------------ UAV PARAM----------------#########
        ######################################################
        ## definition 
        self.winglen = 1.5

        # --------------------------- create model1 ----------------------------------------
        self.uav1 = Quadrotor()
        # jx, jy, jz = 0.0023, 0.0023, 0.004
        # self.uav1.initDyn(Jx=0.0023,Jy=0.0023,Jz=0.004,mass=0.5,l=0.35,c=0.0245) # hb quadrotor

        # c is the force constant, l is the arm length
        self.uav1.initDyn(Jx=0.000392,Jy=0.000405,Jz=0.000639,mass=0.248,l=0.1650,c=3.383e-07 ) # NUSWARM quadrotor

        ######################################################
        #######------------ MPC PARAM----------------#########
        ######################################################

        # MPC prediction step, and prediction horizon
        self.horizon = config_dict['learning_agile']['horizon']
        self.dt=config_dict['learning_agile']['dt']
        # initialize the cost function with symbolic variables
        self.max_tra_w = config_dict['learning_agile']['max_traverse_weight']

    
        # --------------------------- create PDP object1 ----------------------------------------
        # create a pdp object
        self.uavoc1 = OCSys(config_dict)
        self.uavoc1.setAuxvarVariable()
        sc= 5 #1e20
        pos_b   = 5 # in each axis
        vel_b   = config_dict['learning_agile']['linear_vel_bound'] #0.5 # in each axis
        wc   = pi/2 #pi
        tw   = 1.22
        t2w  = 2
        # set symbolic functions for the MPC solver
        self.uavoc1.setStateVariable(self.uav1.X,state_lb=[-pos_b,-pos_b,0.3,
                                                           -vel_b,-vel_b,-vel_b,
                                                           -sc,-sc,-sc,-sc],\
                                     state_ub=[pos_b,pos_b,2,
                                               vel_b,vel_b,vel_b,
                                               sc,sc,sc,sc]) 
        
        self.thrust_ub = config_dict['learning_agile']['single_motor_max_thrust']*4*config_dict['learning_agile']['throttle_upper_bound']
        self.thrust_lb = config_dict['learning_agile']['single_motor_max_thrust']*4*config_dict['learning_agile']['throttle_lower_bound']
        ang_rate_b = config_dict['learning_agile']['angular_vel_bound']

        self.uavoc1.setControlVariable(self.uav1.U,
                                       control_lb=[self.thrust_lb ,-ang_rate_b,-ang_rate_b,-ang_rate_b],\
                                       control_ub= [self.thrust_ub,ang_rate_b,ang_rate_b,ang_rate_b]) # thrust-to-weight = 4:1

        self.uavoc1.setDyn(self.uav1.f,self.dt)
      

        # wrt: ,gate traverse position cost
        # wqt: gate traverse attitude cost
        # wthrust: thrust cost
        # wwt: input angular velocity cost
        # wrf: final position cost
        # wvf: final velocity cost
        # wqf: final attitude cost
        # wwf: final angular velocity cost
        self.uav1.initCost(wrt=5,wqt=8,wthrust=1,wwt=0,wrf=1,wvf=1,wqf=1) # wthrust = 0.1
        self.uav1.init_TraCost()
        self.uavoc1.setInputCost(self.uav1.input_cost)
        self.uavoc1.setPathCost(self.uav1.goal_cost,goal_state_sym=self.uav1.goal_state_sym)
        self.uavoc1.setTraCost(self.uav1.tra_cost,
                               self.uav1.des_tra_r_I,
                               self.uav1.des_tra_q)
        
        self.uavoc1.setFinalCost(self.uav1.final_cost,goal_state_sym=self.uav1.goal_state_sym)

        # initialize the mpc solver
        # self.uavoc1.ocSolverInit(horizon=self.horizon,dt=self.dt)
        self.uavoc1.AcadosOcSolverInit(horizon=self.horizon,
                                       dt=self.dt,
                                       USE_PREV_SOLVER=USE_PREV_SOLVER)
    
    def init_state_and_mission(self,
                goal_pos = [0, 8, 0],
                goal_ori= toQuaternion(0.0,[3,3,5]),
                
                ini_r=[0,-8,0],
                ini_v_I = [0.0, 0.0, 0.0], 
                ini_q = toQuaternion(0.0,[3,3,5])):  
        # goal
        self.goal_pos = goal_pos
        self.goal_ori = goal_ori
        # initial
        if type(ini_r) is not list:
            ini_r = ini_r.tolist()
        self.ini_r = ini_r
        self.ini_v_I = ini_v_I 
        self.ini_q = ini_q
        self.ini_w =  [0.0, 0.0, 0.0]
        self.ini_state = self.ini_r + self.ini_v_I + self.ini_q# + self.ini_w
        
    

    # initialize the narrow window
    def init_obstacle(self,gate_point):
        self.point1 = gate_point[0:3]
        self.point2 = gate_point[3:6]
        self.point3 = gate_point[6:9]
        self.point4 = gate_point[9:12]        
        self.obstacle1 = obstacle(self.point1,self.point2,self.point3,self.point4)
    
    def R_from_MPC(self,tra_pos=None,tra_ang=None,t_tra = 3, Ulast = None):

        # round the traversal time, keep to one decimal place
        # t_tra = round(t_tra,1)

        Ulast=np.array([2,0.0,0.0,0.0])

        # obtain solution of trajectory
        self.mpc_update(self.ini_state, 
                       Ulast, 
                       tra_pos, 
                       tra_ang, 
                       t_tra)
        
        # state_traj [x,y,z,vx,vy,vz,qw,qx,qy,qz]
        state_traj = self.sol1['state_traj_opt']

        self.traj = self.uav1.get_quadrotor_position(wing_len = self.winglen, state_traj = state_traj)
        # calculate trajectory reward
        self.collision = 0
        self.path = 0
        ## detect whether there is collision
        self.co = 0
        for c in range(4):
            self.collision += self.obstacle1.collis_det(self.traj[:,3*(c+1):3*(c+2)],self.horizon)
            self.co += self.obstacle1.co 
        for p in range(4):
            self.path += np.dot(self.traj[self.horizon-1-p,0:3]-self.goal_pos, self.traj[self.horizon-1-p,0:3]-self.goal_pos)
        reward = 1000 * self.collision - 0.5 * self.path + 100
        return reward
    # --------------------------- solution and learning----------------------------------------
    ##solution and demo
    def sol_gradient(self,tra_pos =None,tra_ang=None,t_tra=None,Ulast=None):
        """
        receive the decision variables from DNN1, do the MPC, then calculate d_reward/d_z
        """
        
        tra_ang = np.array(tra_ang)
        tra_pos = np.array(tra_pos)

        # run the MPC to execute plan and execute based on the high-level variables
        # j is the reward
        j = self.R_from_MPC(tra_pos,
                            tra_ang,
                            t_tra)
        ## fixed perturbation to calculate the gradient
        delta = 1e-3

        # drdx,drdy,drdz,drda,drdb,drdc=0
        drdx = np.clip(self.R_from_MPC(tra_pos+[delta,0,0],tra_ang, t_tra,Ulast) - j,-0.5,0.5)*0.1
        drdy = np.clip(self.R_from_MPC(tra_pos+[0,delta,0],tra_ang, t_tra,Ulast) - j,-0.5,0.5)*0.1
        drdz = np.clip(self.R_from_MPC(tra_pos+[0,0,delta],tra_ang, t_tra,Ulast) - j,-0.5,0.5)*0.1
        drda = np.clip(self.R_from_MPC(tra_pos,tra_ang+[delta,0,0], t_tra,Ulast) - j,-0.5,0.5)*(1/(500*tra_ang[0]**2+5))
        drdb = np.clip(self.R_from_MPC(tra_pos,tra_ang+[0,delta,0], t_tra,Ulast) - j,-0.5,0.5)*(1/(500*tra_ang[1]**2+5))
        drdc = np.clip(self.R_from_MPC(tra_pos,tra_ang+[0,0,delta], t_tra,Ulast) - j,-0.5,0.5)*(1/(500*tra_ang[2]**2+5))
        # drdt =0
        if((self.R_from_MPC(tra_pos,tra_ang,t_tra-0.1)-j)>2):
            drdt = -0.05
        if((self.R_from_MPC(tra_pos,tra_ang,t_tra+0.1)-j)>2):
            drdt = 0.05
        ## return gradient and reward (for deep learning)
        return np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,j])


    def optimize(self, t):
        tra_pos = self.obstacle1.centroid
        tra_posx = self.obstacle1.centroid[0]
        tra_posy = self.obstacle1.centroid[1]
        tra_posz = self.obstacle1.centroid[2]
        tra_a = 0
        tra_b = 0
        tra_c = 0
        tra_ang = np.array([tra_a,tra_b,tra_c])
        ## fixed perturbation to calculate the gradient
        for k in range(200):
            j = self.R_from_MPC (tra_pos,tra_ang,t)
            drdx = np.clip(self.R_from_MPC(tra_pos+[0.001,0,0],tra_ang=tra_ang, t=t) - j,-0.5,0.5)
            drdy = np.clip(self.R_from_MPC(tra_pos+[0,0.001,0],tra_ang=tra_ang, t=t) - j,-0.5,0.5)
            drdz = np.clip(self.R_from_MPC(tra_pos+[0,0,0.001],tra_ang=tra_ang, t=t) - j,-0.5,0.5)
            drda = np.clip(self.R_from_MPC(tra_pos,tra_ang=tra_ang+[0.001,0,0], t=t) - j,-0.5,0.5)
            drdb = np.clip(self.R_from_MPC(tra_pos,tra_ang=tra_ang+[0,0.001,0], t=t) - j,-0.5,0.5)
            drdc = np.clip(self.R_from_MPC(tra_pos,tra_ang=tra_ang+[0,0,0.001], t=t) - j,-0.5,0.5)
            #drdt = np.clip(self.R_from_MPC(tra_pos,tra_ang,t-0.1)-j,-10,10)
            # update
            tra_posx += 0.1*drdx
            tra_posy += 0.1*drdy
            tra_posz += 0.1*drdz
            tra_a += (1/(500*tra_a**2+5))*drda
            tra_b += (1/(500*tra_b**2+5))*drdb
            tra_c += (1/(500*tra_c**2+5))*drdc
            if((self.R_from_MPC(tra_pos,tra_ang,t-0.1)-j)>2):
                t = t-0.1
            if((self.R_from_MPC(tra_pos,tra_ang,t+0.1)-j)>2):
                t = t+0.1
            t = round(t,1)
            tra_pos = np.array([tra_posx,tra_posy,tra_posz])
            tra_ang = np.array([tra_a,tra_b,tra_c])
            ## display the process
            print(str(j)+str('  ')+str(tra_pos)+str('  ')+str(tra_ang)+str('  ')+str(t)+str('  ')+str(k))
        return [t,tra_posx,tra_posy,tra_posz,tra_a, tra_b,tra_c, j,self.collision,self.path]

    ## use random perturbations to calculate the gradient and update(not recommonded)
    def LSFD(self,t):
        tra_posx = self.obstacle1.centroid[0]
        tra_posy = self.obstacle1.centroid[1]
        tra_posz = self.obstacle1.centroid[2]
        tra_a = 0
        tra_b = 0
        tra_c = 0
        current_para = np.array([tra_posx,tra_posy,tra_posz,tra_a,tra_b,tra_c])
        lr = np.array([2e-4,2e-4,2e-4,5e-5,5e-5,5e-5])
        for k in range(50):
            j = self.R_from_MPC(current_para[0:3],current_para[3:6],t)
            # calculate derivatives
            c = []
            f = []
            for i in range(24):
                dx = sample(0.001)
                dr = self.R_from_MPC (current_para[0:3]+dx[0:3],current_para[3:6]+dx[3:6],t)-j
                c += [dx]
                f += [dr]
            # update
            cm = np.array(c)
            fm = np.array(f)
            a = np.matmul(np.linalg.inv(np.matmul(cm.T,cm)),cm.T)
            drdx = np.matmul(a,fm)
            current_para = current_para + lr * drdx
            j = self.R_from_MPC(current_para[0:3],current_para[3:6],t)
            if((self.R_from_MPC(current_para[0:3],current_para[3:6],t+0.1)-j)>20):
                t = t + 0.1
            else:
                if((self.R_from_MPC(current_para[0:3],current_para[3:6],t-0.1)-j)>20):
                    t = t - 0.1
            t = round(t,1) 
            print(str(t)+str('  ')+str(drdx)+str('  ')+str(k))
        return [current_para, j,self.collision,self.path]        

    ## play the animation for one set of high-level paramters of such a scenario
    def play_ani(self, tra_pos=None,tra_ang=None, t = 3,Ulast = None):
        tra_atti = Rd2Rp(tra_ang)
        self.uav1.init_TraCost(tra_pos,tra_atti)
        self.uavoc1.setTraCost(self.uav1.tra_cost,t)
        ## obtain the trajectory
        self.sol1 = self.uavoc1.ocSolver(horizon=self.horizon,dt=self.dt,Ulast=Ulast)
        state_traj1 = self.sol1['state_traj_opt']
        traj = self.uav1.get_quadrotor_position(wing_len = self.winglen, state_traj = state_traj1)
        ## plot the animation
        self.uav1.play_animation(wing_len = self.winglen, state_traj = state_traj1,dt=self.dt, point1 = self.point1,\
            point2 = self.point2, point3 = self.point3, point4 = self.point4)
    
    ## given initial state, control command, high-level parameters, obtain the first control command of the quadrotor
    def mpc_update(self, current_state, Ulast ,tra_pos, tra_ang, t_tra):
    
        # # initialize the NLP problem
        # # self.uav1.init_TraCost(tra_pos,tra_atti)
        # # self.uavoc1.setTraCost(self.uav1.tra_cost,
        # #                        tra_pos,
        # #                        tra_atti,
        # #                        t_tra)

    
        ##----- cause the different bewteen the python and the gazebo--###
        tra_atti = Rd2Rp(tra_ang)

        tra_q=toQuaternion(tra_atti[0],tra_atti[1])

        current_state_control = np.concatenate((current_state,Ulast))
       
        # self.sol1 = self.uavoc1.ocSolver(current_state_control=current_state_control,t_tra=t)
        self.sol1,weight_vis,NO_SOLUTION_FLAG = self.uavoc1.AcadosOcSolver(current_state_control=current_state_control,
                                                goal_pos=self.goal_pos,
                                                goal_ori=self.goal_ori,
                                                tra_pos=tra_pos,
                                                tra_q=tra_q,
                                                t_tra=t_tra,
                                                max_tra_w = self.max_tra_w)
        
        # return control, pos_vel_cmd
        return self.sol1,weight_vis,NO_SOLUTION_FLAG

## sample the perturbation (only for random perturbations)
def sample(deviation):
    act = np.random.normal(0,deviation,size=6)
    return act