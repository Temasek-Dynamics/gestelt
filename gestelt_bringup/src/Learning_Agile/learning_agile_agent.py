#!/usr/bin/env python3

## this file is for traversing moving narrow window
import sys
import os
import subprocess
import yaml
from scipy.spatial.transform import Rotation as R
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append("../")
sys.path.append(subdirectory_path)


from typing import Any
from quad_model import *
from quad_policy import *
from quad_nn import *
from quad_moving import *

# ros
import numpy as np
import rospy
# from gestelt_msgs.msg import CommanderState, Goals, CommanderCommand
from geometry_msgs.msg import Pose, Accel,PoseArray,AccelStamped, TwistStamped, PoseStamped,Quaternion,Vector3
from mavros_msgs.msg import PositionTarget, AttitudeTarget
from std_msgs.msg import Int8, Bool,Float32
import math
import time
import tf


# load the DNN2 model
abs_dir=os.path.dirname(os.path.abspath(__file__))
print(abs_dir)
FILE = os.path.join(abs_dir, 'nn3_1.pth')
# FILE ="nn3_1.pth"



class MovingGate():
    def __init__(self, env_init_set):
        
        # initialize the gate1, with the initial gate position
        # env_init_set[7]: gate width
        gate_point_no_pitch = np.array([[-env_init_set[7]/2,-0,1],[env_init_set[7]/2,-0,1],[env_init_set[7]/2,-0,-1],[-env_init_set[7]/2,-0,-1]])
        self.gate1 = gate(gate_point_no_pitch)
        self.gate_init_p = env_init_set[8]
        
        # add the pitch angle to the gate1
        self.gate1.rotate_y(self.gate_init_p)

        # generate new four corners of the gate1
        self.gate_point = self.gate1.gate_point

        # update the gate1 with the initial pitch angle
        self.gate1 = gate(self.gate_point)
        
    
    def let_gate_move(self,dt):
    
        ## define the kinematics of the narrow window
        # gate linear velocity
        self.v =np.array([0,0.0,0.0]) #np.array([1,0.3,0.4])
        
        # gate pitch angular velocity
        self.w = 0 #pi/2
        self.gate_move, self.V = self.gate1.move(v = self.v ,w = self.w,dt=dt)

        # self.gate_move=np.zeros([1,4,3])
        # self.V=np.zeros([1,3])
        # return self.w
    
class LearningAgileAgent():
    def __init__(self,python_sim_time,yaml_file) -> None:

        self.sim_time=python_sim_time
    
        # drone state
        self.state = np.zeros(10)


        # load the configuration file
        with open(yaml_file, 'r', encoding='utf-8') as file:
            self.config_dict = yaml.safe_load(file)

        # load trained DNN2 model
        self.model = torch.load(FILE)
    

        ##-------------------- planning variables --------------------------##
        # MPC prediction step, and prediction horizon
        self.dt=self.config_dict['learning_agile']['dt']
        self.horizon=self.config_dict['learning_agile']['horizon'] #(T/dt)

        self.u = np.array([2,0.0,0.0,0.0])
        self.tm = [0,0,0,0]
        self.state_n = []
        self.control_n = [self.u.tolist()]
        self.control_tm = [self.tm]
        
        self.hl_para = [0,0,0,0,0,0,0]
        self.hl_variable = [self.hl_para]
        


        
        self.Ttra    = []
        self.T       = []
        self.Time    = []
        self.Pitch   = []
        self.i       = 0
        self.solving_time = []
        self.tra_weight_list = []   
        # trajectory pos_vel_att_cmd
        self.pos_vel_att_cmd=np.zeros(10)
        self.pos_vel_att_cmd_n = [self.pos_vel_att_cmd]

        # FIXME Temporally set the traversal time here, for both python and gazebo simulation
        
    
    def receive_mission_states(self,
                                STATIC_GATE_TEST,
                                ini_pos,
                                end_pos,
                                
                                ini_yaw,  
                                goal_yaw,
                                
                                gate_center=np.array([0,0,1.5]),
                                gate_ori_euler=np.array([0,0,0]),
                                
                                t_tra_abs=1,
                                max_tra_w=0,

                                ):
        """
        receive the ini_pos,end point defined in the mission file

        """
        self.STATIC_GATE_TEST=STATIC_GATE_TEST
        
        # env_init_set[0:3]: drone initial position
        # env_init_set[3:6]: drone final position
        # env_init_set[6]: drone initial yaw
        # env_init_set[7]: gate width (randomly set)
        # env_init_set[8]: gate pitch angle (randomly set)
        self.env_init_set = nn_sample()
        self.env_init_set[0:3]=ini_pos
        self.env_init_set[3:6]=end_pos
        self.env_init_set[6]=ini_yaw # drone_init_yaw
        # self.env_init_set[8]=pi/2 # gate pitch angle

        ##---------------------gate initialization ------------------------##
        self.moving_gate = MovingGate(self.env_init_set)
        self.gate_point = self.moving_gate.gate_point
        

        self.final_point = end_pos
        self.goal_yaw = goal_yaw
        
        self.gate_center = gate_center
        self.gate_ori_euler = gate_ori_euler
        self.t_tra_abs =t_tra_abs
        self.max_tra_w=max_tra_w

        print('start_point=',self.env_init_set[0:3])
        print('final_point=',self.env_init_set[3:6])

    def problem_definition(self,gazebo_sim=False,dyn_step=0.002):
        """
        initial traversal problem

        """
        ini_q=R.from_euler('xyz',[0,0,self.env_init_set[6]]).as_quat()
        ini_q=np.roll(ini_q,1)
        
        final_q=R.from_euler('xyz',[0,0,self.goal_yaw]).as_quat()
        final_q=np.roll(final_q,1)
        
          

        self.quad1 = run_quad(self.config_dict,
                              goal_pos=self.env_init_set[3:6].tolist(),
                              goal_ori=final_q.tolist(),
                              
                              ini_r=self.env_init_set[0:3].tolist(),
                              ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                              ini_q=ini_q.tolist(),
                              
                              gazebo_sim=gazebo_sim,
                              dt=self.dt)
        

        self.quad1.init_obstacle(self.gate_point.reshape(12))

        # set the dynamics step of the python sim
        self.dyn_step=dyn_step
        self.quad1.uav1.setDyn(self.dyn_step)

        ##---------------------gate initialization ------------------------##
        self.moving_gate.let_gate_move(dt=self.dyn_step)
        self.gate_move = self.moving_gate.gate_move
        self.gate_n = gate(self.gate_move[0])

        ##-------------- initial guess of the traversal time---------------##
        self.t_guess = magni(self.gate_n.centroid-self.state[0:3])/3
        
    
    def gate_state_estimation(self):

        """
        estimate the gate pose, using binary search
        t_tra_abs: the absolute traversal time w.r.t the mission start time
        t_tra_rel: the relative traversal time w.r.t the current time

        """


        
        if self.STATIC_GATE_TEST:
            self.gate_n = gate(self.gate_move[0])

            # self.t_tra_abs is manually set
            self.t_tra_rel=self.t_tra_abs-self.i*self.dyn_step

        else:

            self.gate_n = gate(self.gate_move[self.i])
        
            ## binary search for the traversal time
            ## to set the drone state under the gate frame, for the NN2 input
            self.t_tra_rel = binary_search_solver(self.model,self.state,self.final_point,self.gate_n,self.moving_gate.V[self.i],self.moving_gate.w)
            self.t_tra_abs = self.t_tra_rel+self.i*0.01

    
            
            # print('step',self.i,'tranversal time W.R.T current=',t,'gap_pitch=',gap_pitch*180/pi)
            # print('step',self.i,'abs_tranversal time W.R.T mission=',t_tra)
            
        

            ## obtain the future traversal window state
            self.gate_n.translate(self.t_tra_rel*self.moving_gate.V[0])
            self.gate_n.rotate_y(self.t_tra_rel*self.moving_gate.w)
            # print('rotation matrix I_G=',gate_n.I_G)
            
        self.Ttra= np.concatenate((self.Ttra,[self.t_tra_abs]),axis = 0)
        self.T = np.concatenate((self.T,[self.t_tra_rel]),axis = 0)
        
        self.i += 1
        # return self.t_tra_abs,self.gate_n.centroid


    def solve_problem(self):
        """
        python simulation

        """
        
        self.state = self.quad1.ini_state # state= feedback from pybullet, 13-by-1, 3 position, 3 velocity (world frame), 4 quaternion, 3 angular rate
        self.state_n = [self.state]
        for self.i in range(self.sim_time*(int(1/self.dyn_step))): # 5s, 500 Hz
            
            
            self.gap_pitch = self.moving_gate.gate_init_p + self.moving_gate.w*self.i*self.dyn_step
            self.Time = np.concatenate((self.Time,[self.i*self.dyn_step]),axis = 0)
            self.Pitch = np.concatenate((self.Pitch,[self.gap_pitch]),axis = 0)      
            
            if (self.i%5)==0: # control frequency = 100 hz
                # decision variable is updated in 100 hz
                self.gate_state_estimation()
                
                
                nn_mpc_inputs = np.zeros(18)
                if self.STATIC_GATE_TEST:
                    nn_mpc_inputs[0:10] = self.state 
                    nn_mpc_inputs[10:13] = self.final_point
                    

                    # manually set the traversal time and pose
                    out=np.zeros(7)
                    out[0:3]=self.gate_center
                    out[3:6]=self.gate_ori_euler # Rodrigues parameters

                    # relative traversal time
                    out[6]=self.t_tra_rel
                else:
                    # drone state under the predicted gate frame(based on the binary search)
                    nn_mpc_inputs[0:10] = self.gate_n.transform(self.state)
                    nn_mpc_inputs[10:13] = self.gate_n.t_final(self.final_point)
                    nn_mpc_inputs[16] = magni(self.gate_n.gate_point[0,:]-self.gate_n.gate_point[1,:]) # gate width
                    nn_mpc_inputs[17] = atan((self.gate_n.gate_point[0,2]-self.gate_n.gate_point[1,2])/(self.gate_n.gate_point[0,0]-self.gate_n.gate_point[1,0])) # compute the actual gate pitch ange in real-time

                    # NN2 OUTPUT the traversal time and pose
                    out = self.model(nn_mpc_inputs).data.numpy()

                

                t_comp = time.time()
                
                cmd_solution,weight_vis,NO_SOLUTION_FLAG  = self.quad1.get_input(nn_mpc_inputs[0:10],
                                                    self.u,
                                                    out[0:3],
                                                    out[3:6],
                                                    out[6],
                                                    max_tra_w=self.max_tra_w) # control input 4-by-1 thrusts to pybullet
                
                print('solving time at main=',time.time()-t_comp)
                self.solving_time.append(time.time()- t_comp)
                self.u=cmd_solution['control_traj_opt'][0,:].tolist()
                self.pos_vel_att_cmd=cmd_solution['state_traj_opt'][0,:]
                self.tra_weight_list.append(weight_vis)
            
            self.state = np.array(self.quad1.uav1.dyn_fn(self.state, self.u)).reshape(10) # Yixiao's simulation environment ('uav1.dyn_fn'), replaced by pybullet
            self.state_n = np.concatenate((self.state_n,[self.state]),axis = 0)
            self.control_n = np.concatenate((self.control_n,[self.u]),axis = 0)
            self.pos_vel_att_cmd_n = np.concatenate((self.pos_vel_att_cmd_n,[self.pos_vel_att_cmd]),axis = 0)
            u_m = self.quad1.uav1.u_m
            u1 = np.reshape(self.u,(4,1))
            tm = np.matmul(u_m,u1)
            tm = np.reshape(tm,4)
            # control_tm = np.concatenate((control_tm,[tm]),axis = 0)
            # self.hl_variable = np.concatenate((self.hl_variable,[out]),axis=0)       
            
        print('MPC finished')   
        np.save('gate_move_traj',self.gate_move)
        np.save('uav_traj',self.state_n)
        np.save('uav_ctrl',self.control_n)
        np.save('abs_tra_time',self.Ttra)
        np.save('tra_time',self.T)
        np.save('Time',self.Time)
        np.save('Pitch',self.Pitch)
        np.save('HL_Variable',self.hl_variable)
        np.save('solving_time',self.solving_time)
        self.quad1.uav1.play_animation(wing_len=1.5,
                                       gate_traj1=self.gate_move[::5,:,:],
                                       state_traj=self.state_n[::5,:],
                                       goal_pos=self.final_point.tolist(),
                                       dt=self.dyn_step)
        
        # save the data, not show it
        self.quad1.uav1.plot_thrust(self.control_n)
        self.quad1.uav1.plot_angularrate(self.control_n)
        self.quad1.uav1.plot_position(self.state_n)
        self.quad1.uav1.plot_velocity(self.state_n)
        self.quad1.uav1.plot_quaternions(self.state_n)
        self.quad1.uav1.plot_trav_weight(self.tra_weight_list)
       
        self.quad1.uav1.plot_solving_time(self.solving_time)

        # self.quad1.uav1.plot_T(control_tm)
        # self.quad1.uav1.plot_M(control_tm)
    
        
def main():
    # yaml file dir#
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')

    ## --------------for single planning part test-------------------------------##.
    # create the learning agile agent
    learing_agile_agent=LearningAgileAgent(python_sim_time=5,yaml_file=yaml_file)


    #------------------------------set the mission--------------------------------------#
    config_dict = learing_agile_agent.config_dict
    learing_agile_agent.receive_mission_states( STATIC_GATE_TEST=True,
                                                ini_pos=np.array(config_dict['mission']['initial_position']),
                                                end_pos=np.array(config_dict['mission']['goal_position']),
                                                
                                                ini_yaw=np.array(config_dict['mission']['initial_ori_euler'])[2],
                                                goal_yaw=np.array(config_dict['mission']['goal_ori_euler'])[2],
                                                
                                                gate_center=np.array(config_dict['mission']['gate_position']),
                                                gate_ori_euler=np.array(config_dict['mission']['gate_ori_euler']),
                                                
                                                t_tra_abs=config_dict['learning_agile']['traverse_time'],
                                                max_tra_w=config_dict['learning_agile']['max_traverse_weight'])

    # #------------------------------------------------------------------------------#
    # problem definition
    # the dyn_step is the simulation step in the simulation environment
    # for the acados ERK integrator, the step is (integral step)/4 =0.025s
    learing_agile_agent.problem_definition(dyn_step=0.002)

    # solve the problem
    learing_agile_agent.solve_problem()

    # every time after reconstruct the solver, need to catkin build the MPC wrapper to 
    # relink the shared library
    shell_script="""
    catkin build mpc_ros_wrapper
    """

    # run the shell script
    subprocess.run(shell_script,shell=True)

if __name__ == '__main__':
    main()
   