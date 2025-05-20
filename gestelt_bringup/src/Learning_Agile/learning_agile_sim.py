#!/usr/bin/env python3

## this file is for traversing moving narrow window
import os
import subprocess
import time
import argparse
import ray


import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R
import torch 
import math
import matplotlib.pyplot as plt

from quad_model import toQuaternion, Gate, Rd2Rp, get_gate_points
from visualization.python_sim_vis import play_animation, plot_position, plot_velocity, plot_scalar, plot_thrust, plot_angularrate, plot_3D_traj,plot_M,plot_T,plot_3axis_weights
from quad_policy import PlanFwdBwdWrapper, get_obs, manual_set_z_forward
from quad_nn import nn_sample, network
from quad_moving import binary_search_solver,input_cal
from visualization.result_analysis import rotation_vis
from geometry.solid_geometry import magni, pitch_from_gate, verify_SVD_ca
from misc.misc import str2bool 
from config import train_cfg, mission_cfg, current_dir

device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# device=torch.device('cpu')

input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size'] 
class MovingGate():
    def __init__(self, 
                env_init_set,
                gate_center,
                gate_length):
        
        # initialize the gate1, with the initial gate position
        # env_init_set[7]: gate width
        
        ###############################################
        ###############################################
        ##################gate length##################    z
        # 0------------------------------------------1     ^     y
        # |                   ^z                     |     |   /
        # |<-gate width       |                      |     | /
        # |                   *--> x                 |     *-------> x
        # 3------------------------------------------2
        ###############################################
        ###############################################
        gate_width = env_init_set[7]
        gate_point_no_pitch = get_gate_points(gate_center,gate_length,gate_width)
        
        self.gate = Gate(gate_point_no_pitch)
        
        # add the pitch angle to the gate
        gate_init_euler = R.from_matrix(env_init_set[8:17].reshape(3,3)).as_euler('zyx')
        self.gate_init_pitch = gate_init_euler[1]
        self.gate.rotate_y(self.gate_init_pitch)


    
    def set_vel(
        self,
        dt,
        gate_v,
        gate_w,
        python_sim_time
        ):
        
        self.v=gate_v
        self.w=gate_w
        
        # pre calculate gate points for future T durations
        self.gate_points_list, self.V = self.gate.move(T = python_sim_time, v = gate_v ,w = gate_w ,dt = dt)


class LearningAgileSim():
    def __init__(self,
                 python_sim_time,
                 mission_cfg:dict=None,
                 train_cfg:dict=None,
                 model_file=None,
                 dyn_step=0.002,
                 options:dict=None) -> None:
        self.options=options
        
        self.sim_time=python_sim_time
    
        # drone state
        if mission_cfg["ctl_mode"] == 0:
            self.state = np.zeros(10)
        elif mission_cfg["ctl_mode"] == 1 or mission_cfg["ctl_mode"] == 2:
            self.state = np.zeros(13)
        elif mission_cfg["ctl_mode"] == 3:
            self.state = np.zeros(17)

        # load the configuration file
        self.config_dict = mission_cfg
        self.train_cfg = train_cfg    
        if not self.options['MANUAL_SET_POSE_TEST']:
            # load trained DNN2 model
            if model_file is not None:
                self.model = network(
                    train_cfg['model']['input_size'], 
                    train_cfg['model']['hidden_size'], 
                    train_cfg['model']['hidden_size'],
                    weights_vector_length=train_cfg['model']['weights_vector_length'],
                    activation=train_cfg['model']['activation']
                ).to(device)


                if options['MC_EVALUATION']:
                    self.model.load_state_dict(torch.load(model_file,map_location='cpu'))
                    
                else:
                    self.model.load_state_dict(torch.load(model_file))
                # self.model = torch.load(model_file, map_location=device)
                self.model.eval()   

        ##-------------------- planning variables --------------------------##
        self.planner = PlanFwdBwdWrapper(self.config_dict,self.options)
        self.u=np.zeros(4)
        self.last_u=np.zeros(4)
        self.state_n = []
        self.control_n = [self.u.tolist()]
        
        # set the dynamics step of the python sim (Explict Euler, ERK4)
        self.dyn_step=dyn_step
        self.planner.uavoc.AcadosSimIntegratorInit(self.dyn_step,options['USE_PREV_SOLVER'])
        self.integrator=self.planner.uavoc.acados_integrator

        self.Ttra    = []
        self.T       = []
        self.NN_T_tra = []
        self.Time    = []
        self.Pitch   = []
        self.i       = 0
        self.solving_time = []
        self.tra_weight_list = []   
        self.verify_tra_R_list = []
        self.pred_traj_list = []
        # trajectory pos_vel_att_cmd
        self.pos_vel_att_cmd=np.zeros(len(self.state))
        self.pos_vel_att_cmd[6:10] = [1,0,0,0]
        self.pos_vel_att_cmd_n = [self.pos_vel_att_cmd]
        self.last_gate_points = np.zeros((4,3))
        
        ##==================NN params=======================##
        self.input_size = train_cfg['model']['input_size']
        self.output_size = train_cfg['model']['output_size']
    
    def generate_mission(self,i=train_cfg['training']['num_epochs'],TEST=True):
        """
        receive the ini_pos,end point defined in the mission file

        """
        
        
        # env_init_set[0:3]: drone initial position
        # env_init_set[3:6]: drone final position
        # env_init_set[6]: drone initial yaw
        # env_init_set[7]: gate width (randomly set)
        # env_init_set[8]: gate pitch angle (randomly set)
      
    
        
        ini_yaw=np.array(self.config_dict['mission']['initial_ori_euler'])[0]
        self.goal_yaw=np.array(self.config_dict['mission']['goal_ori_euler'])[0]
        
        self.gate_center=np.array(self.config_dict['mission']['gate_position'])
        # self.gate_center += np.random.uniform(-0.1,0.1,3)
        # self.gate_ori_RP=np.array(self.config_dict['mission']['gate_ori_RP'])
        gate_ori_euler=np.array(self.config_dict['mission']['gate_ori_euler'])
        self.gate_ori_9d=R.from_euler('zyx',gate_ori_euler).as_matrix().flatten()

        self.t_tra_abs=self.config_dict['t_tra_abs']
        
        self.env_init_set = nn_sample(cur_epoch=i,TEST=TEST)
        # if self.options['MANUAL_SET_POSE_TEST']:
            # ini_pos=self.config_dict['mission']['initial_position']
            # end_pos=np.array(self.config_dict['mission']['goal_position'])
        #     self.env_init_set[0:3]=ini_pos
        #     self.env_init_set[3:6]=end_pos
        self.env_init_set[6]=ini_yaw # drone_init_yaw
        self.final_point = self.env_init_set[3:6]
    

        ## ===== send mission to the quadrotor mpc solver ======== ##
        ini_q=toQuaternion(self.env_init_set[6],[0,0,1])
        final_q=toQuaternion(self.goal_yaw,[0,0,1])
          

        self.planner.init_state_and_mission(goal_pos=self.env_init_set[3:6],
                              goal_ori=final_q,
                              ini_r=self.env_init_set[0:3].tolist(),
                              ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                              ini_q=ini_q,)
        
        if self.config_dict['ctl_mode'] != 3:
            self.u=self.planner.hover_u
            self.last_u=self.u
            
        

    def prepare_gate(self):
        
        """
        this function is to initialize a gate with pitch angle, and set the gate velocity
        """
        
        ##---------------------gate initialization ------------------------##
        gate_length = self.config_dict['gate']['length'] 
        gate_v = np.array(self.config_dict['gate']['linear_vel'])
        gate_w = self.config_dict['gate']['angular_vel'] 

        if self.options['STATE_2_MOVING_GATE']:
            gate_w = np.random.normal(gate_w,0.1)
            judge = np.random.normal(0,1)
            if judge<0:
                gate_w = -gate_w
           
        ## ================ gate initialization ================== ##
        self.moving_gate = MovingGate(
            self.env_init_set,
            gate_center=self.gate_center,
            gate_length=gate_length
        )

        self.moving_gate.set_vel(dt=self.dyn_step,gate_v=gate_v,gate_w=gate_w,python_sim_time=self.sim_time)
        self.gate_points_list = self.moving_gate.gate_points_list
        self.gate_t_i = Gate(self.gate_points_list[0])

    def log_NN_IO_for_RM(self,gate_pitch,out,des_tra_R):
        """
        record the NN output raw 9D vector and converted Rotation Matrix
        """
        self.NN_T_tra = np.concatenate((self.NN_T_tra,[out[-1]]),axis = 0)
        self.nn_output_list=np.concatenate((self.nn_output_list,[out]),axis = 0)
        self.des_tra_R_list = np.concatenate((self.des_tra_R_list,[des_tra_R]),axis = 0)
        self.wrp_list = np.concatenate((self.wrp_list, out[-8:-5].reshape(1, 3)), axis=0)
        self.wrt_list = np.concatenate((self.wrt_list, out[-5:-2].reshape(1, 3)), axis=0)
        self.wqt_list = np.concatenate((self.wqt_list,[out[-2]]),axis = 0)
        self.gamma_list = np.concatenate((self.gamma_list,[out[-1]]),axis = 0)
        self.Pitch = np.concatenate((self.Pitch,[gate_pitch]),axis = 0) 


    def close_loop_NN_forward(self):
        
        obs, self.gate_pitch, self.last_gate_points  = get_obs( 
                self.last_gate_points,
                self.i,
                self.input_size,
                self.state,
                self.final_point,
                self.gate_t_i
        )
        nn_output = self.model(torch.tensor(obs.reshape([1,-1]), dtype=torch.float).to(device))[0]
        out = nn_output.to('cpu').data.numpy()

        if self.options['COMPARISON']:
            out[0:3]=self.gate_center
            out[3:12]=self.gate_ori_9d
        
        verify_tra_R,_=verify_SVD_ca(out[3:12])
        self.log_NN_IO_for_RM(self.gate_pitch,out,verify_tra_R.flatten()) 
        self.verify_tra_R_list.append(verify_tra_R)
        return out 
    


    def forward(self,python_sim_data_dir=None, \
                STAB_TEST=False):
        """
        python simulation

        """
        
        self.state = self.planner.ini_state # state= feedback from pybullet, 13-by-1, 3 position, 3 velocity (world frame), 4 quaternion, 3 angular rate
        self.state_n = [self.state]
        self.failed_state = []
        self.Time = [0]
        self.nn_output_list = [np.zeros(output_size)] # 3 position, 4 quaternion, 1 traversal time
        self.des_tra_R_list = [np.zeros(9)] # 3x3 rotation matrix(in flat form)
        self.wrp_list = np.empty((0, 3))  # 3×0 矩阵，以便沿 axis=1 连接
        self.wrt_list = np.empty((0, 3))
        self.wqt_list = []
        self.gamma_list = []
        trav_auxvar_value = np.zeros(output_size)
        self.last_gate_points =(self.gate_points_list[0]-self.state[0:3])

        STAB_FAILED = False
        for self.i in range(self.sim_time*(int(1/self.dyn_step))): # 5s, 500 Hz
            
            self.Time = np.concatenate((self.Time,[self.i*self.dyn_step]),axis = 0)
            if (self.i%5)==0: # control frequency = 100 hz  
                
                if self.options['MANUAL_SET_POSE_TEST']:
                    gate_pitch,trav_auxvar_value,verify_tra_R = manual_set_z_forward(
                        cur_pos=self.state[0:3],
                        gate_center=self.gate_center,
                        gate_ori_9d=self.gate_ori_9d
                    )
                    self.log_NN_IO_for_RM(gate_pitch,trav_auxvar_value,verify_tra_R.flatten()) 


                else:
                    ## NN decision
                    self.gate_t_i = Gate(self.gate_points_list[self.i])
                    trav_auxvar_value = self.close_loop_NN_forward()

                
                des_t_tra= self.t_tra_abs - self.i * self.dyn_step
                self.NN_T_tra = np.concatenate((self.NN_T_tra,[des_t_tra]),axis = 0)
                # print('des_t_tra=',des_t_tra)
                
                
                if self.i == 0:
                    gate_ori_euler=np.array([0,1.2,0])
                    zero_gate_ori_9d=R.from_euler('zyx',gate_ori_euler).as_matrix().flatten()
                    _,manual_auxvar_value,_ = manual_set_z_forward(
                        cur_pos=self.state[0:3],
                        gate_center=self.gate_center,
                        gate_ori_9d=zero_gate_ori_9d
                    )
                    for i in range(2):
                        init_solution, NO_SOLUTION_FLAG  = self.planner.conser_mpc_as_init_guess(
                        cur_state=self.state,
                        trav_auxvar_value=manual_auxvar_value,
                        last_u=self.last_u,
                        des_t_tra=des_t_tra, 
                        first_iter=(self.i==0)
                        )
                else:
                    init_solution = None
                    
                t_comp = time.time()
                cmd_solution,NO_SOLUTION_FLAG  = self.planner.mpc_update(
                    cur_state=self.state,
                    trav_auxvar_value=trav_auxvar_value,
                    last_u=self.last_u,
                    des_t_tra=des_t_tra, 
                    first_iter=(self.i==0),
                    init_guess=init_solution,
                )
                if NO_SOLUTION_FLAG:
                    STAB_FAILED = True
                    self.failed_state.append(self.state)
                    print('No solution found')
                    print('traverse_auxvar_value=',trav_auxvar_value)
                    
                # print('solving time at main=',time.time()-t_comp)
                self.solving_time.append(time.time()- t_comp)
                self.u=cmd_solution['control_traj_opt'][0,:].tolist()
                self.last_u = cmd_solution['control_traj_opt'][0,:]
                self.pos_vel_att_cmd=cmd_solution['state_traj_opt'][1,:] #self.config_dict['learning_agile']['horizon']
                self.pred_traj_list.append(cmd_solution['state_traj_opt'][:,:])
                # self.tra_weight_list.append(weight_vis)
           

            ########################################################
            ###================= state update====================###
            ########################################################

            ###===================Explict Euler(obsolete) or ERK4====================###
            # self.state = np.array(self.planner.uavoc.dyn_fn_disc(self.state, self.u)).reshape(10) # Yixiao's simulation environment ('uav.dyn_fn'), replaced by pybullet
            
            
            ##================= acados integrator IRK========================###
            self.integrator.set('x',np.array(self.state))

            
            self.integrator.set('u',np.array(self.u))
            # self.integrator.set('p',np.zeros(18))
            status_sim = self.integrator.solve()
            if status_sim != 0:
                raise Exception('acados integrator returned status {}. Exiting.'.format(status_sim))

            
            self.state = self.integrator.get('x')
            self.state_n = np.concatenate((self.state_n,[self.state]),axis = 0)
            self.control_n = np.concatenate((self.control_n,[self.u]),axis = 0)
            self.pos_vel_att_cmd_n = np.concatenate((self.pos_vel_att_cmd_n,[self.pos_vel_att_cmd]),axis = 0)
    
            
        print('MPC finished')   
        
        if STAB_TEST:
            FAILED=STAB_FAILED
        else:               
            FAILED=self.planner.final_traj_eval(self.state_n[::10,:],self.gate_points_list[::10,:,:])
            print('FAILED=',FAILED)

            if self.options['VISUALIZE']:
                self.visualize()

            if self.options['SAVE_SIM']:
                self.save(python_sim_data_dir)


        return {'FAILED':FAILED,
                'failed_state':self.failed_state}

        
    def visualize(self):
        play_animation(wing_len=self.planner.wing_len,
                        gate_traj1=self.gate_points_list[::5,:,:],
                        state_traj=self.state_n[::5,:],
                        pred_traj_list=self.pred_traj_list,
                        goal_pos=self.final_point.tolist(),
                        NN_pos=self.nn_output_list[:,0:3],
                        NN_R=self.des_tra_R_list,
                        dt=0.01,
                        save_option=0)
            
        # save the data, not show it
        fig, axes = plt.subplots(4, 3, figsize=(12, 8),dpi=100)  
        plot_position(axes[0,0], self.state_n, dt=0.1, label_prefix='drone_actual_position')
        plot_position(axes[0,1], self.nn_output_list, dt=0.1, label_prefix='NN2_output_position')
        plot_scalar(axes[0,2],self.solving_time,scalar_name='MPC_solving_time')
        plot_scalar(axes[2,2],self.wqt_list,scalar_name='traverse_orientation_weight')
        
        plot_velocity([axes[1,0], axes[1,1], axes[1,2]],self.state_n)
       
       
        if self.config_dict['ctl_mode'] == 0:
            plot_angularrate(axes[2,0], self.control_n[:,1:])
            plot_thrust(axes[2,1], self.control_n)
            
        elif self.config_dict['ctl_mode'] == 1:# SRT
            plot_angularrate(axes[2,0], self.state_n[:,10:13])
            plot_T(axes[2,1], self.control_n)
        elif self.config_dict['ctl_mode'] == 2:#wrench
            plot_angularrate(axes[2,0],self.state_n[:,10:13])
            plot_M(axes[2,1], self.control_n)
            plot_thrust(axes[2,2],self.control_n)
        elif self.config_dict['ctl_mode'] == 3:
            plot_angularrate(axes[2,0],self.state_n[:,10:13])
            plot_T(axes[2,1], self.state_n[:,13:17],name='single_rotor_thrust')
            plot_T(axes[2,2],self.control_n,name='single_rotor_thrust_differencce')

        
        plot_3axis_weights(axes[3,0],self.wrp_list,name='wrp')
        plot_3axis_weights(axes[3,1],self.wrt_list, name='wrt')
        plot_scalar(axes[3,2],self.gamma_list,scalar_name='traverse_weight_span')
        
        
        
        fig.tight_layout()
        plt.savefig("./python_sim_result/combined_results.png")
        plt.show()

        
        plot_3D_traj(
            wing_len=self.planner.wing_len,
            uav_height=self.planner.uav_height/2,
            state_traj=self.state_n[::20,:],
            gate_traj=self.gate_points_list[::20,:,:],
            NN_pos=self.nn_output_list[:,0:3],
            NN_R=self.des_tra_R_list[:,:]
        )
        
        self.euler_nn=rotation_vis(
            uav_traj=self.state_n,
            nn_output_list=self.nn_output_list,
            des_tra_R_list=self.des_tra_R_list,
            t_tra_list=self.NN_T_tra,
            gate_pitch=self.Pitch
        )  
    

    def save(self,python_sim_data_dir):
        """
        save the data
        """

        if self.options['SAVE_CSV']:
            from misc.misc import save_mpc_ctl_csv, save_state_csv,save_nn_decision_csv
            save_state_csv(self.Time,self.state_n,python_sim_data_dir)
            save_mpc_ctl_csv(self.Time,self.control_n,python_sim_data_dir)
            nn_decision=np.concatenate((self.nn_output_list[:,:3],self.euler_nn,self.nn_output_list[:,-2:]),axis=1)
            save_nn_decision_csv(self.Time,nn_decision,python_sim_data_dir)

        np.save(os.path.join(python_sim_data_dir,'gate_points_list_traj'),self.gate_points_list)
        np.save(os.path.join(python_sim_data_dir,'uav_traj'),self.state_n)
        np.save(os.path.join(python_sim_data_dir,'uav_ctrl'),self.control_n)
        np.save(os.path.join(python_sim_data_dir,'abs_tra_time'),self.Ttra)
        np.save(os.path.join(python_sim_data_dir,'tra_time'),self.NN_T_tra)
        np.save(os.path.join(python_sim_data_dir,'Time'),self.Time)
        np.save(os.path.join(python_sim_data_dir,'Pitch'),self.Pitch)
        np.save(os.path.join(python_sim_data_dir,'solving_time'),self.solving_time)
        np.save(os.path.join(python_sim_data_dir,'nn_output_list'),self.nn_output_list)
        np.save(os.path.join(python_sim_data_dir,'des_tra_R_list'),self.des_tra_R_list)

def parse_options():
    parser = argparse.ArgumentParser(description="Options for the program.")

    parser.add_argument('--MPC_BACKWARD', type=str2bool, default=False, help='Enable or disable MPC_BACKWARD.')
    parser.add_argument('--USE_PREV_SOLVER', type=str2bool, default=False, help='Enable or disable USE_PREV_SOLVER.')
    parser.add_argument('--PDP_GRADIENT', type=str2bool, default=False, help='Enable or disable PDP_GRADIENT.')
    parser.add_argument('--SQP_RTI_OPTION', type=str2bool, default=True, help='SQP or the DDP')
    parser.add_argument('--MANUAL_SET_POSE_TEST', type=str2bool, default=False, help='Enable or disable MANUAL_SET_POSE_TEST.')
    parser.add_argument('--CLOSE_LOOP_MODEL', type=str2bool, default=True, help='Enable or disable CLOSE_LOOP_MODEL.')
    parser.add_argument('--JAX_SVD', type=str2bool, default=False, help='Enable or disable JAX_SVD.')
    parser.add_argument('--CLOSE_LOOP_TRAINING', type=str2bool, default=False, help='Enable or disable CLOSE_LOOP_TRAINING.')
    parser.add_argument('--VISUALIZE', type=str2bool, default=True, help='Enable or disable VISUALIZE.')
    parser.add_argument('--STATE_2_MOVING_GATE', type=str2bool, default=False, help='Enable or disable STATE_2_MOVING_GATE.')
    parser.add_argument('--SAVE_SIM', type=str2bool, default=True, help='Enable or disable SAVE_SIM.')
    parser.add_argument('--SAVE_CSV', type=str2bool, default=True, help='Enable or disable save sim data in the csv format.')
    parser.add_argument('--COMPARISON',  type=str2bool, default=False, help='Compare the training results with other methods')
    parser.add_argument('--MC_EVALUATION',  type=str2bool, default=False, help='Compare the training results with other methods')
    parser.add_argument('--MULTI_COLLISION_POINT_CHECK',  type=str2bool, default=False, help='multiple collision point check for the training')
    args = parser.parse_args()
    return vars(args)  # Return options as a dictionary  

# @ray.remote     
def eval_sim_interface(mission_cfg=None,
                 train_cfg=None,
                 options=None,
                 model_file=None,
                 python_sim_data_dir=None,
                 INTRAIN=False,
                 STAB_TEST=False):
    """
    test the success rate, evaluate the real executed trajectory
    """

    
    if INTRAIN:
        options['VISUALIZE']=False
        options['SAVE_SIM']=False
        options['USE_PREV_SOLVER']=True
    
    # create the learning agile agent
    # problem definition
    # the dyn_step is the simulation step in the simulation environment
    # for the acados ERK integrator, the step is (integral step)/4 =0.025s
    learning_agile_sim=LearningAgileSim(python_sim_time=5,
                                        mission_cfg=mission_cfg,
                                        train_cfg=train_cfg,
                                        model_file=model_file,
                                        dyn_step=0.002,
                                        options=options)
    
    

    
    #####==============load env config ====================#######
    learning_agile_sim.generate_mission(TEST=True)
    learning_agile_sim.prepare_gate()
    
    #####============== Solve the problem ====================#######
    # solve the problem
    forward_return=learning_agile_sim.forward(python_sim_data_dir,STAB_TEST=STAB_TEST)
    out={'FAILED':forward_return['FAILED'],
        'failed_state':forward_return['failed_state'],         
        'state_traj':learning_agile_sim.state_n,
        'gate_traj':learning_agile_sim.gate_points_list
        }
    return out

def main():
    ray.init()
    python_sim_data_dir = os.path.join(current_dir, 'python_sim_result')
    options=parse_options()
    print("Parsed Options:", options)

    if options['CLOSE_LOOP_MODEL']:
        model_name = mission_cfg['NN_model_name']#'NN2_imitate_1.pth' #'NN_close_2.pth'
        model_file=os.path.join(current_dir,model_name)
    else:   
        model_name = '20241031-142733-PDP-Trial 1, shrink the gate from [1.2,0.56] to [1.0, 0.4]/NN2_imitate_1.pth' 
        model_file=os.path.join(current_dir, f'training_data/NN_model/',model_name)
    
    if options['MC_EVALUATION']:
        eval_sim_interface.remote(mission_cfg,
                    train_cfg,
                    options,
                    model_file,
                    python_sim_data_dir,
                    INTRAIN=False)
    else:
        eval_sim_interface(mission_cfg,
                    train_cfg,
                    options,
                    model_file,
                    python_sim_data_dir,
                    INTRAIN=False)
    
    

    # every time after reconstruct the solver, need to catkin build the MPC wrapper to 
    # relink the shared library
    shell_script="""catkin build mpc_ros_wrapper"""

    # run the shell script
    subprocess.run(shell_script,shell=True,check=False)

if __name__ == '__main__':
    main()
   

   # def gate_state_search(self):

    #     """
    #     depricated.
    #     estimate the gate pose, using binary search
    #     t_tra_abs: the absolute traversal time w.r.t the mission start time
    #     t_tra_rel: the relative traversal time w.r.t the current time

    #     """


        
    #     if self.options['MANUAL_SET_POSE_TEST'] or self.options['COMPARISON']:
    #         self.gate_t_i = Gate(self.gate_points_list[0])

    #         # self.t_tra_abs is manually set
    #         self.t_tra_rel=self.t_tra_abs-self.i*self.dyn_step

    #     else:

    #         self.gate_t_i = Gate(self.gate_points_list[self.i])
    #         # print('gate_t_i.centroid=',self.gate_t_i.centroid)
    #         ## binary search for the traversal time
    #         ## to set the drone state under the gate frame, for the NN2 input
    #         self.t_tra_rel = binary_search_solver(self.model,device,self.state,self.final_point,self.gate_t_i,self.moving_gate.V[self.i],self.moving_gate.w)
    #         self.t_tra_abs = self.t_tra_rel+self.i*self.dyn_step

    
            
    #         # print('step',self.i,'tranversal time W.R.T current=',t,'gap_pitch=',gap_pitch*180/pi)
    #         # print('step',self.i,'abs_tranversal time W.R.T mission=',t_tra)
            
        

    #         ## obtain the future traversal window state w.r.t current time-step gate_t_i
    #         self.gate_t_i.translate(self.t_tra_rel*self.moving_gate.V[self.i])
    #         self.gate_t_i.rotate_y(self.t_tra_rel*self.moving_gate.w)
    #         # print('rotation matrix I_G=',gate_t_i.I_G)
            
    #     self.Ttra= np.concatenate((self.Ttra,[self.t_tra_abs]),axis = 0)
    #     self.T = np.concatenate((self.T,[self.t_tra_rel]),axis = 0)
    # def log_NN_IO_for_RP(self,nn2_inputs,out):
    #     """
    #     record the NN output Rodrigues parameters, convert it to quaternion
    #     """
    #     atti = Rd2Rp(out[3:6])
    #     quat_nn=toQuaternion(atti[0],atti[1])
        
    #     out_as_quat=np.concatenate((out[0:3],np.array(quat_nn),out[6].reshape([1,])),axis = 0)
        
    #     self.NN_T_tra = np.concatenate((self.NN_T_tra,[out[6]]),axis = 0)
    #     self.nn_output_list=np.concatenate((self.nn_output_list,[out_as_quat]),axis = 0)
    #     self.Pitch = np.concatenate((self.Pitch,[nn2_inputs[output_size]]),axis = 0) 

    # def imiate_NN_forward(self):
        
    #     nn2_inputs,gate_pitch = input_cal(self.state,self.final_point,self.gate_t_i)
       
    #     # NN2 OUTPUT the traversal time and pose
    #     out = self.model(torch.tensor(nn2_inputs, dtype=torch.float).to(device)).to('cpu')
    #     out = out.data.numpy()
        
    #     verify_tra_R=verify_SVD_ca(out[3:12])
    #     self.log_NN_IO_for_RM(gate_pitch,out,verify_tra_R.flatten())       
    #     return out
    