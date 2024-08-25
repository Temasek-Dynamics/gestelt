## this file is a package for policy search for quadrotor

from quad_OC import OCSys,LQR
from scipy.spatial.transform import Rotation as R
from math import cos, pi, sin, sqrt, tan
from quad_model import *
from casadi import *
import scipy.io as sio
import numpy as np
import time
from solid_geometry import *
import torch

class run_quad:
    def __init__(self,config_dict,
                SQP_RTI_OPTION=True, 
                USE_PREV_SOLVER = False,
                PDP_GRADIENT=False):
        
       

        ######################################################
        #######------------ UAV PARAM----------------#########
        ######################################################
        ## definition 
        self.wing_len = 1.5 
        self.uav_height = 0.4
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
       
        sc= 1 #1e2
        pos_b   = config_dict['learning_agile']['pos_bound'] # in each axis
        vel_b   = config_dict['learning_agile']['linear_vel_bound'] #0.5 # in each axis
     
        # set symbolic functions for the MPC solver
        self.uavoc1.setStateVariable(self.uav1.X,state_lb=[-pos_b,-pos_b,-pos_b,
                                                           -vel_b,-vel_b,-vel_b,
                                                           -sc,-sc,-sc,-sc],\
                                     state_ub=[pos_b,pos_b,pos_b,
                                               vel_b,vel_b,vel_b,
                                               sc,sc,sc,sc]) 
        
        self.thrust_ub = config_dict['learning_agile']['single_motor_max_thrust']*4*config_dict['learning_agile']['throttle_upper_bound']
        self.thrust_lb = config_dict['learning_agile']['single_motor_max_thrust']*4*config_dict['learning_agile']['throttle_lower_bound']
        ang_rate_b = config_dict['learning_agile']['angular_vel_bound']
        self.uavoc1.setAuxvarVariable()
        self.uavoc1.setControlVariable(self.uav1.U,
                                       control_lb=[self.thrust_lb ,-ang_rate_b,-ang_rate_b,-ang_rate_b],\
                                       control_ub= [self.thrust_ub,ang_rate_b,ang_rate_b,ang_rate_b]) # thrust-to-weight = 4:1

        self.uavoc1.setDyn(self.uav1.f,self.dt)
      

        # wrt: ,gate traverse position cost
        # wqt: gate traverse attitude cost
        # wthrust: input thrust cost
        # wwt: input angular velocity cost
        # wrf: final position cost
        # wvf: final velocity cost
        # wqf: final attitude cost
        # wwf: final angular velocity cost

        ## initialize the cost function
        self.uav1.initCost(wrt=config_dict['learning_agile']['wrt'],
                           wqt=config_dict['learning_agile']['wqt'],
                           wthrust=config_dict['learning_agile']['wthrust'],
                           wwt=config_dict['learning_agile']['wwt'],
                           wrf=config_dict['learning_agile']['wrf'],
                           wvf=config_dict['learning_agile']['wvf'],
                           wqf=config_dict['learning_agile']['wqf'],
                           max_tra_w=config_dict['learning_agile']['max_traverse_weight'],
                           gamma=config_dict['learning_agile']['traverse_weight_span'],
                            wInputDiff=config_dict['learning_agile']['wInputDiff']
                           ) 
        self.uav1.init_TraCost()

        ## set the symbolic cost function to the solver
        self.uavoc1.setInputCost(self.uav1.input_cost)
        
        self.uavoc1.setInputDiffCost(self.uav1.Ulast,
                                           self.uav1.input_diff_cost)

        self.uavoc1.setPathCost(self.uav1.goal_cost,goal_state=self.uav1.goal_state)
        self.uavoc1.setTraCost(self.uav1.tra_cost,
                               self.uav1.trav_auxvar,
                               self.uav1.t_node
                              )
        
        self.uavoc1.setFinalCost(self.uav1.final_cost,goal_state=self.uav1.goal_state)

        # initialize the mpc solver
        # self.uavoc1.ocSolverInit(horizon=self.horizon,dt=self.dt)
        self.uavoc1.AcadosOcSolverInit(horizon=self.horizon,
                                       dt=self.dt,
                                       SQP_RTI_OPTION=SQP_RTI_OPTION,
                                       USE_PREV_SOLVER=USE_PREV_SOLVER)

        ###################################################################
        ###------------ PDP auxiliary control system----------------#######
        ###################################################################
        # define the auxilary control system symbolic functions
        self.PDP_GRADIENT = PDP_GRADIENT
        if self.PDP_GRADIENT:
            self.uavoc1.diffPMP()
            self.lqr_solver = LQR()

       
        
    def init_state_and_mission(self,
                goal_pos = [0, 8, 0],
                goal_ori= toQuaternion(0.0,[0,0,1]),
                
                ini_r=[0,-8,0],
                ini_v_I = [0.0, 0.0, 0.0], 
                ini_q = toQuaternion(0.0,[0,0,1])):  
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
        
    def update_goal_pos(self,goal_pos):
        self.goal_pos = goal_pos
    

    # initialize the narrow window
    def init_obstacle(self,gate_point,gate_pitch):
        self.gate_corners = gate_point
        self.gate_quat = toQuaternion(-gate_pitch,[0,1,0])
        self.point1 = gate_point[0:3]
        self.point2 = gate_point[3:6]
        self.point3 = gate_point[6:9]
        self.point4 = gate_point[9:12]        
        self.obstacle1 = obstacle(self.point1,self.point2,self.point3,self.point4)
        self.obstacle1_torch=obstacle_torch(self.point1,self.point2,self.point3,self.point4)


    def R_from_MPC(self,tra_pos=None,tra_ang=None,t_tra = 3, Ulast_value = None):
       
        # round the traversal time, keep to one decimal place
        # t_tra = round(t_tra,1)

    
        # obtain solution of trajectory
        NO_SOLUTION_FLAG = False
        self.sol1,NO_SOLUTION_FLAG =self.mpc_update(self.ini_state, 
                       Ulast_value, 
                       tra_pos, 
                       tra_ang, 
                       t_tra)
        
        # state_traj [x,y,z,vx,vy,vz,qw,qx,qy,qz]
        state_traj = self.sol1['state_traj_opt']
        # get the quadrotor both center and edges position trajectory
        self.traj = self.uav1.get_quadrotor_position(wing_len = self.wing_len, state_traj = state_traj)

        
        ELLIPSOID_COLLISION_CHECK = False
        if not self.AUTO_DIFF:
            # calculate trajectory reward
            self.collision = 0
            self.path = 0
            ## detect whether there is collision
            self.co = 0

            
            for c in range(4):
                self.collision += self.obstacle1.collis_det(self.traj[:,3*(c+1):3*(c+2)],self.horizon)
                self.co += self.obstacle1.co 

            ## calculate the path cost
            # check the drone centroid position error with the goal position
            for p in range(4):
                self.path += np.dot(self.traj[self.horizon-1-p,0:3]-self.goal_pos, self.traj[self.horizon-1-p,0:3]-self.goal_pos)
            
            # the sign of the collision is already negative
            # pitch angle reward temproally be here
            pitch_reward =  0 * 0.5 * tra_ang[1]**2
            self.drdpitch = 0 * tra_ang[1]
            
            reward_origin = 1000 * self.collision - 0.5 * self.path + 100 + 10 * pitch_reward


            #############################################################
            ##-----------------ellipse collision check-----------------##
            #############################################################
            # initialize the drone ellipse
            self.obstacle1.reward_calc_sym(self.uav1,
                                            quad_height=self.uav_height,
                                            quad_radius=self.wing_len/2,
                                            alpha=5,
                                            beta=10,
                                            Q_tra=5,
                                            w_goal=0.1)    
            
                                            # alpha=5,
                                            # beta=10,
                                            # Q_tra=1,
                                            # safe_margin=0.0,
                                            # w_goal=0.1)    
            
    
            # reward,self.drdstate_traj,gate_check_points=self.obstacle1.reward_calc_value(state_traj,
            #                                     self.gate_corners,
            #                                     goal_pos=self.goal_pos,
            #                                     vert_traj=self.traj[:,0:3],
            #                                     horizon=self.horizon)
            gate_check_points = np.zeros((4,3))
            for i in range(4):
                if i < 4:
                        ## load four corners first
                        gate_check_points[i,:] = self.gate_corners[i*3:i*3+3]

            if sys.gettrace() is not None:  # In debug mode
                des_node_tra=int(np.round(t_tra*(10)))
                self.uav1.plot_3D_traj(wing_len=1.5,
                                    uav_height=self.uav_height,
                                        state_traj=state_traj,
                                        gate_traj=gate_check_points,
                                        TRAIN_VIS=True,
                                        tra_node=des_node_tra)

            self.obstacle1.reward_calc_differentiable_collision(state_traj,
                                                                self.gate_corners,
                                                                gate_quat=self.gate_quat,
                                                                vert_traj=self.traj[:,0:3],
                                                                horizon=self.horizon)
            
           
            
            return reward #+ pitch_reward
        
        
        else:   
            self.state_traj_tensor = torch.tensor(state_traj,requires_grad=True)
            self.traj_tensor = self.uav1.get_quadrotor_position_tensor(wing_len = self.wing_len, state_traj = self.state_traj_tensor )
            self.collision_tensor = torch.tensor(0.0,  requires_grad=True)
            self.co_tensor = torch.tensor(0.0,  requires_grad=True)
            self.path_tensor = torch.tensor(0.0,  requires_grad=True)
            self.pitch_reward_tensor = torch.tensor(0.0,  requires_grad=True)
            self.reward_tensor = torch.tensor(0.0,  requires_grad=True)
            self.goal_pos_tensor = torch.tensor(self.goal_pos,requires_grad=True)
            
            for c in range(4):
                self.collision_tensor = torch.add(self.collision_tensor,self.obstacle1_torch.collis_det_torch(self.traj_tensor[:,3*(c+1):3*(c+2)],self.horizon))
                # self.collision_tensor += self.obstacle1_torch.collis_det_torch(self.traj_tensor[:,3*(c+1):3*(c+2)],self.horizon)
                # self.co_tensor += self.obstacle1_torch.co


            # Calculate the path value
            for p in range(4):
                diff = self.traj_tensor[self.horizon-p, 0:3] - self.goal_pos_tensor
                self.path_tensor = torch.add(self.path_tensor, torch.dot(diff, diff))
                # self.path_tensor += torch.dot(diff, diff)
            

            
            # Calculate pitch angle reward
            # self.tra_ang_tensor = torch.tensor(tra_ang, requires_grad=True)
            # pitch_reward_tensor = torch.abs(self.tra_ang_tensor[1])

            # Calculate the reward
            # The sign of the collision is already negative
            reward_tensor = 1000 * self.collision_tensor - 0.5 * self.path_tensor + 100 #+ 10 * pitch_reward_tensor
            
            if not self.PDP_GRADIENT: 
                reward_tensor.detach().numpy()
            # Return the reward
            return reward_tensor

    # --------------------------- solution and learning----------------------------------------
    ##solution and demo
    def sol_gradient(self,tra_pos =None,tra_ang=None,t_tra=None,Ulast_value=None, AUTO_DIFF = False):
        """
        receive the decision variables from DNN1, do the MPC, then calculate d_reward/d_z
        """
        self.AUTO_DIFF = AUTO_DIFF
        tra_ang = np.array(tra_ang)
        tra_pos = np.array(tra_pos)

        # run the MPC to execute plan and execute based on the high-level variables
        # j is the reward
        j = self.R_from_MPC(tra_pos,
                            tra_ang,
                            t_tra,
                            Ulast_value)
        
       
        ## fixed perturbation to calculate the gradient
        delta = 1e-3

        
        if not self.PDP_GRADIENT:    
            drdx,drdy,drdz,drda,drdb,drdc=0
        drdx = np.clip(self.R_from_MPC(tra_pos+[delta,0,0],tra_ang, t_tra,Ulast_value) - j,-0.5,0.5)*0.1
        drdy = np.clip(self.R_from_MPC(tra_pos+[0,delta,0],tra_ang, t_tra,Ulast_value) - j,-0.5,0.5)*0.1
        drdz = np.clip(self.R_from_MPC(tra_pos+[0,0,delta],tra_ang, t_tra,Ulast_value) - j,-0.5,0.5)*0.1
        drda = np.clip(self.R_from_MPC(tra_pos,tra_ang+[delta,0,0], t_tra,Ulast_value) - j,-0.5,0.5)*(1/(500*tra_ang[0]**2+5))
        drdb = np.clip(self.R_from_MPC(tra_pos,tra_ang+[0,delta,0], t_tra,Ulast_value) - j,-0.5,0.5)*(1/(500*tra_ang[1]**2+5))
        drdc = np.clip(self.R_from_MPC(tra_pos,tra_ang+[0,0,delta], t_tra,Ulast_value) - j,-0.5,0.5)*(1/(500*tra_ang[2]**2+5))
        drdt =0
        if((self.R_from_MPC(tra_pos,tra_ang,t_tra-0.1,Ulast_value)-j)>2):
            drdt = -0.05
        if((self.R_from_MPC(tra_pos,tra_ang,t_tra+0.1,Ulast_value)-j)>2):
            drdt = 0.05
        ## return gradient and reward (for deep learning)
        
        if sys.gettrace is not None:
            print("finite diff:",np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,j]))
        # return np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,j])
        
        ########################################################################
        #=======================SYMBOLIC GRADIENT+PDP===========================
        ########################################################################
        # else:


        ###################################################################
        ###----- Set mpc external variables VALUE to diffPMP--------#######
        ###################################################################
        ## goal state, Ulast value, t_node is set in the mpc_update function,
        ## need to be given here
    
    
        ## set the traverse hyperparameters value (auxvar) here
        trav_auxvar = np.array([tra_pos[0],tra_pos[1],tra_pos[2],tra_ang[0],tra_ang[1],tra_ang[2],t_tra])
        goal_state_value=np.concatenate((self.goal_pos,np.zeros(3),self.goal_ori))  

        
        ## using LQR solver to solve the auxilary control system to get the analytical gradient
        # set values to the auxilary control system symbolic functions 
        aux_sys = self.uavoc1.getAuxSys(state_traj_opt=self.sol1['state_traj_opt'],
                                        control_traj_opt=self.sol1['control_traj_opt'],
                                        costate_traj_opt=self.sol1['costate_traj_opt'],
                                        goal_state_value=goal_state_value,
                                        Ulast_value=Ulast_value,
                                        auxvar_value=trav_auxvar)
        
        # set values to the LQR solver
        self.lqr_solver.setDyn(dynF=aux_sys['dynF'], dynG=aux_sys['dynG'], dynE=aux_sys['dynE'])
        self.lqr_solver.setPathCost(Hxx=aux_sys['Hxx'], Huu=aux_sys['Huu'], Hxu=aux_sys['Hxu'], Hux=aux_sys['Hux'],
                                Hxe=aux_sys['Hxe'], Hue=aux_sys['Hue'])
        self.lqr_solver.setFinalCost(hxx=aux_sys['hxx'], hxe=aux_sys['hxe'])


        ## solve the auxilary control system and get the analytical gradient
        aux_sol=self.lqr_solver.lqrSolver(np.zeros((self.uavoc1.n_state, self.uavoc1.n_trav_auxvar)), self.horizon)
        

        # take solution of the auxiliary control system
        
        # which is the dtrajectory/dtraverse_auxvar 
        dstate_trajdp = aux_sol['state_traj_opt'] #(n_node,n_state,n_trav_auxvar)
        dinput_trajdp = aux_sol['control_traj_opt'] 
        
        dstate_trajdp = np.array(dstate_trajdp)
        
        
        if  AUTO_DIFF:
            # acquire dreward/dtrajectory
            j.backward()
            drdstate_traj=self.state_traj_tensor.grad #(n_node,15)     
            drdstate_traj=drdstate_traj.numpy().reshape(self.horizon+1,1,self.uavoc1.n_state)
            
            
        drdstate_traj=self.drdstate_traj.reshape(self.horizon+1,1,self.uavoc1.n_state)
        
        # normalize the gradient
        norm_drdstate_traj = drdstate_traj/np.linalg.norm(drdstate_traj)    
        norm_dstate_trajdp = dstate_trajdp/np.linalg.norm(dstate_trajdp)
        drdp=np.zeros(7)
        for i in range(self.horizon):
            drdp += np.matmul(norm_drdstate_traj[i,:,:],norm_dstate_trajdp[i,:,:]).reshape(7)

        drdp += np.matmul(norm_drdstate_traj[self.horizon,:,:],norm_dstate_trajdp[self.horizon,:,:]).reshape(7)    

        drdx = drdp[0]
        drdy = drdp[1]
        drdz = drdp[2]
        drda = drdp[3]
        drdb = drdp[4]
        drdc = drdp[5]
        drdt = drdp[6]
        # j = j.detach().numpy()
        if sys.gettrace() is not None:
            print("analytic grad:",np.array([-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,j]))
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
        traj = self.uav1.get_quadrotor_position(wing_len = self.wing_len, state_traj = state_traj1)
        ## plot the animation
        self.uav1.play_animation(wing_len = self.wing_len, state_traj = state_traj1,dt=self.dt, point1 = self.point1,\
            point2 = self.point2, point3 = self.point3, point4 = self.point4)
    
    ## given initial state, control command, high-level parameters, obtain the first control command of the quadrotor
    def mpc_update(self, 
                   current_state, 
                   Ulast ,
                   tra_pos, 
                   tra_ang, 
                   t_tra):
    
        # # initialize the NLP problem
        # # self.uav1.init_TraCost(tra_pos,tra_atti)
        # # self.uavoc1.setTraCost(self.uav1.tra_cost,
        # #                        tra_pos,
        # #                        tra_atti,
        # #                        t_tra)

    
        ##----- cause the different bewteen the python and the gazebo--###
        # tra_atti = Rd2Rp(tra_ang)

        # tra_q=toQuaternion(tra_atti[0],tra_atti[1])

        current_state_control = np.concatenate((current_state,Ulast))
       
        # self.sol1 = self.uavoc1.ocSolver(current_state_control=current_state_control,t_tra=t)
        self.sol1,NO_SOLUTION_FLAG = self.uavoc1.AcadosOcSolver(current_state_control=current_state_control,
                                                goal_pos=self.goal_pos,
                                                goal_ori=self.goal_ori,
                                                tra_pos=tra_pos,
                                                tra_ang=tra_ang,
                                                t_tra=t_tra,
                                                max_tra_w = self.max_tra_w)
        
        # return control, pos_vel_cmd
        return self.sol1,NO_SOLUTION_FLAG

## sample the perturbation (only for random perturbations)
def sample(deviation):
    act = np.random.normal(0,deviation,size=6)
    return act