#!/usr/bin/env python3

## this file is for traversing moving narrow window
import sys
import os
import subprocess
import yaml
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

# add to sys.path
sys.path.append("../")
sys.path.append(subdirectory_path)


from quad_model import *
from quad_policy import *
from quad_nn import *
from quad_moving import *
from result_analysis import *
import numpy as np
import time
from solid_geometry import *


device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')


class MovingGate():
    def __init__(self, env_init_set,
                        gate_cen_h,
                        gate_length):
        
        # initialize the gate1, with the initial gate position
        # env_init_set[7]: gate width
        gate_width = env_init_set[7]
        gate_point_no_pitch = np.array([[-gate_length/2, 0, gate_cen_h+gate_width/2],
                                        [ gate_length/2, 0, gate_cen_h+gate_width/2],
                                        [ gate_length/2, 0, gate_cen_h-gate_width/2],
                                        [-gate_length/2, 0, gate_cen_h-gate_width/2]])
        
        self.gate = Gate(gate_point_no_pitch)
        
        # add the pitch angle to the gate
        gate_init_euler = R.from_matrix(env_init_set[8:17].reshape(3,3)).as_euler('zyx')
        self.gate_init_pitch = gate_init_euler[1]
        # self.gate_init_pitch = 1.2
        self.gate.rotate_y(self.gate_init_pitch)


    
    def set_vel(self,
                dt,
                gate_v,
                gate_w):
        
        self.v=gate_v
        self.w=gate_w
        
        # pre calculate gate points for future T durations
        self.gate_points_list, self.V = self.gate.move(T = 8, v = gate_v ,w = gate_w ,dt = dt)

    
class LearningAgileAgent():
    def __init__(self,python_sim_time,
                 yaml_file,
                 model_file=None,
                 dyn_step=0.002,
                 options:dict=None) -> None:
        self.options=options
        
        self.sim_time=python_sim_time
    
        # drone state
        self.state = np.zeros(10)


        # load the configuration file
        with open(yaml_file, 'r', encoding='utf-8') as file:
            self.config_dict = yaml.safe_load(file)

        if not self.options['STATIC_GATE_TEST']:
            # load trained DNN2 model
            if model_file is not None:
                self.model = torch.load(model_file)
    

        ##-------------------- planning variables --------------------------##

        self.u = np.array([2,0.0,0.0,0.0])
        self.tm = [0,0,0,0]
        self.state_n = []
        self.control_n = [self.u.tolist()]
        self.control_tm = [self.tm]
        
        self.hl_para = [0,0,0,0,0,0,0]
        self.hl_variable = [self.hl_para]
        

        self.planner = PlanFwdBwdWrapper(self.config_dict,self.options)
        
 
        # set the dynamics step of the python sim (Explict Euler, ERK4)
        self.dyn_step=dyn_step
        self.planner.uav1.setDyn(self.dyn_step)
        self.planner.uavoc1.AcadosSimIntegratorInit(self.dyn_step)
        self.integrator=self.planner.uavoc1.acados_integrator

        self.Ttra    = []
        self.T       = []
        self.NN_T_tra = []
        self.Time    = []
        self.Pitch   = []
        self.i       = 0
        self.solving_time = []
        self.tra_weight_list = []   
        # trajectory pos_vel_att_cmd
        self.pos_vel_att_cmd=np.zeros(10)
        self.pos_vel_att_cmd[6:10] = [1,0,0,0]
        self.pos_vel_att_cmd_n = [self.pos_vel_att_cmd]

       
    
    def generate_mission(self):
        """
        receive the ini_pos,end point defined in the mission file

        """
        
        
        # env_init_set[0:3]: drone initial position
        # env_init_set[3:6]: drone final position
        # env_init_set[6]: drone initial yaw
        # env_init_set[7]: gate width (randomly set)
        # env_init_set[8]: gate pitch angle (randomly set)
      
        ini_pos=self.config_dict['mission']['initial_position']
        end_pos=np.array(self.config_dict['mission']['goal_position'])
        
        ini_yaw=np.array(self.config_dict['mission']['initial_ori_euler'])[2]
        self.goal_yaw=np.array(self.config_dict['mission']['goal_ori_euler'])[2]
        
        self.gate_center=np.array(self.config_dict['mission']['gate_position'])
        self.gate_ori_RP=np.array(self.config_dict['mission']['gate_ori_RP'])
        
        self.t_tra_abs=self.config_dict['learning_agile']['traverse_time']
        
        self.env_init_set = nn_sample()
        if self.options['STATIC_GATE_TEST']:
            self.env_init_set[0:3]=ini_pos
            self.env_init_set[3:6]=end_pos
        self.env_init_set[6]=ini_yaw # drone_init_yaw
        self.final_point = self.env_init_set[3:6]
    

        # print('start_point=',self.env_init_set[0:3])
        # print('final_point=',self.env_init_set[3:6])

        ## ===== send mission to the quadrotor mpc solver ======== ##
        ini_q=toQuaternion(self.env_init_set[6],[0,0,1])
        final_q=toQuaternion(self.goal_yaw,[0,0,1])
          

        self.planner.init_state_and_mission(goal_pos=self.env_init_set[3:6].tolist(),
                              goal_ori=final_q,
                              
                              ini_r=self.env_init_set[0:3].tolist(),
                              ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                              ini_q=ini_q,)

        
       

    def prepare_gate(self):
        
        ##---------------------gate initialization ------------------------##
        gate_length = self.config_dict['gate']['length'] 
        gate_v = np.array(self.config_dict['gate']['linear_vel'])
        gate_w = self.config_dict['gate']['angular_vel'] 
        ## ================ gate initialization ================== ##
        self.moving_gate = MovingGate(self.env_init_set,
                                      gate_cen_h=1.2,
                                      gate_length=gate_length)

        self.moving_gate.set_vel(dt=self.dyn_step,gate_v=gate_v,gate_w=gate_w)
        self.gate_points_list = self.moving_gate.gate_points_list
        self.gate_t_i = Gate(self.gate_points_list[0])

        
    
    def gate_state_estimation(self):

        """
        estimate the gate pose, using binary search
        t_tra_abs: the absolute traversal time w.r.t the mission start time
        t_tra_rel: the relative traversal time w.r.t the current time

        """


        
        if self.options['STATIC_GATE_TEST']:
            self.gate_t_i = Gate(self.gate_points_list[0])

            # self.t_tra_abs is manually set
            self.t_tra_rel=self.t_tra_abs-self.i*self.dyn_step

        else:

            self.gate_t_i = Gate(self.gate_points_list[self.i])
            # print('gate_t_i.centroid=',self.gate_t_i.centroid)
            ## binary search for the traversal time
            ## to set the drone state under the gate frame, for the NN2 input
            self.t_tra_rel = binary_search_solver(self.model,device,self.state,self.final_point,self.gate_t_i,self.moving_gate.V[self.i],self.moving_gate.w)
            self.t_tra_abs = self.t_tra_rel+self.i*self.dyn_step

    
            
            # print('step',self.i,'tranversal time W.R.T current=',t,'gap_pitch=',gap_pitch*180/pi)
            # print('step',self.i,'abs_tranversal time W.R.T mission=',t_tra)
            
        

            ## obtain the future traversal window state w.r.t current time-step gate_t_i
            self.gate_t_i.translate(self.t_tra_rel*self.moving_gate.V[self.i])
            self.gate_t_i.rotate_y(self.t_tra_rel*self.moving_gate.w)
            # print('rotation matrix I_G=',gate_t_i.I_G)
            
        self.Ttra= np.concatenate((self.Ttra,[self.t_tra_abs]),axis = 0)
        self.T = np.concatenate((self.T,[self.t_tra_rel]),axis = 0)
        
        
       
    def log_NN_IO_for_RP(self,nn2_inputs,out):
        """
        record the NN output Rodrigues parameters, convert it to quaternion
        """
        atti = Rd2Rp(out[3:6])
        quat_nn=toQuaternion(atti[0],atti[1])
        
        out_as_quat=np.concatenate((out[0:3],np.array(quat_nn),out[6].reshape([1,])),axis = 0)
        
        self.NN_T_tra = np.concatenate((self.NN_T_tra,[out[6]]),axis = 0)
        self.nn_output_list=np.concatenate((self.nn_output_list,[out_as_quat]),axis = 0)
        self.Pitch = np.concatenate((self.Pitch,[nn2_inputs[14]]),axis = 0) 

    def log_NN_IO_for_RM(self,gate_pitch,out,des_tra_R):
        """
        record the NN output raw 9D vector and converted Rotation Matrix
        """
        self.NN_T_tra = np.concatenate((self.NN_T_tra,[out[6]]),axis = 0)
        self.nn_output_list=np.concatenate((self.nn_output_list,[out]),axis = 0)
        self.des_tra_R_list = np.concatenate((self.des_tra_R_list,[des_tra_R]),axis = 0)
        self.Pitch = np.concatenate((self.Pitch,[gate_pitch]),axis = 0) 

    def close_loop_model_forward(self):

        nn2_inputs = np.zeros(18)
        # drone state under the predicted gate frame(based on the binary search)
        nn2_inputs[0:10] = self.gate_t_i.transform(self.state)
        nn2_inputs[10:13] = self.gate_t_i.t_final(self.final_point)
        # position of the gate
        nn2_inputs[13:16] = self.gate_t_i.centroid
        # width of the gate
        nn2_inputs[16] = magni(self.gate_t_i.gate_point[0,:]-self.gate_t_i.gate_point[1,:]) # gate width
        # pitch angle of the gate
        nn2_inputs[17] = atan((self.gate_t_i.gate_point[0,2]-self.gate_t_i.gate_point[1,2])/(self.gate_t_i.gate_point[0,0]-self.gate_t_i.gate_point[1,0])) # compute the actual gate pitch ange in real-time
        
        # NN2 OUTPUT the traversal time and pose
        out = self.model(torch.tensor(nn2_inputs, dtype=torch.float).to(device)).to('cpu')
        out = out.data.numpy()

        self.log_NN_IO(nn2_inputs,out)
        return out 
    
    def imitate_model_forward(self):
        
        nn2_inputs,gate_pitch = input_cal(self.state,self.final_point,self.gate_t_i)
       
        # NN2 OUTPUT the traversal time and pose
        out = self.model(torch.tensor(nn2_inputs, dtype=torch.float).to(device)).to('cpu')
        out = out.data.numpy()
        
        verify_tra_R=verify_SVD_casadi(out[3:12])
        self.log_NN_IO_for_RM(gate_pitch,out,verify_tra_R.flatten())       
        return out
    

    def forward_sim(self,python_sim_data_folder):
        """
        python simulation

        """
        
        self.state = self.planner.ini_state # state= feedback from pybullet, 13-by-1, 3 position, 3 velocity (world frame), 4 quaternion, 3 angular rate
        self.state_n = [self.state]
        self.nn_output_list = [np.zeros(13)] # 3 position, 4 quaternion, 1 traversal time
        self.des_tra_R_list = [np.zeros(9)] # 3x3 rotation matrix(in flat form)
        for self.i in range(self.sim_time*(int(1/self.dyn_step))): # 5s, 500 Hz
            
            self.Time = np.concatenate((self.Time,[self.i*self.dyn_step]),axis = 0)
            
            if not self.options['CLOSE_LOOP_MODEL']:
                if (self.i%25)==0: # estimation frequency = 20 hz 
                    # decision variable is updated in 20 hz
                    self.gate_state_estimation()

            if (self.i%5)==0: # control frequency = 100 hz  
                
                if self.options['STATIC_GATE_TEST']:
                    self.gate_state_estimation()
                    nn2_inputs = np.zeros(23)
                    nn2_inputs[0:10] = self.state 
                    nn2_inputs[10:13] = self.final_point
                    

                    # manually set the traversal time and pose
                    out=np.zeros(13)
                    out[0:3]=self.gate_center
                    # out[3:6]=self.gate_ori_RP # Rodrigues parameters
                    # out[3:12]=np.array([[0.7603140,  0.0000000, -0.6495557],
                    #                     [0.0000000,  1.0000000,  0.0000000],
                    #                     [0.6495557,  0.0000000,  0.7603140]]).flatten() # 3x3 rotation matrix(in flat form)
                    out[3:12]=np.array([[0.0007963,  0.0000000, -0.9999997],
                                        [0.0000000,  1.0000000,  0.0000000],
                                        [0.9999997,  0.0000000,  0.0007963]]).flatten()
                    print("="*50)
                    print("NN pose det before SVD",np.linalg.det(out[3:12].reshape(3,3)))
                    
                    des_tra_pos=out[0:3]

                    

                    if self.options['JAX_SVD']:
                        ### SVD through JAX
                        des_tra_R=SVD_M_to_SO3(out[3:12]).flatten() # 9D vector to 3x3 rotation matrix(in flat form)
                        print("NN pose det after SVD",np.linalg.det(des_tra_R.reshape(3,3)))
                        # relative traversal time
                        out[12]=self.t_tra_rel
                        self.log_NN_IO_for_RM(out,des_tra_R,gate_pitch=0) 
                    else:
                        ### SVD through CasADi
                        des_tra_m=out[3:12]

                        # relative traversal time
                        out[12]=self.t_tra_rel
                        verify_tra_R=verify_SVD_casadi(out[3:12])
                        gate_pitch=0
                        self.log_NN_IO_for_RM(gate_pitch,out,verify_tra_R.flatten())       
                else:
                    
                    if self.options['CLOSE_LOOP_MODEL']:
                        out = self.close_loop_model_forward()
                    else:
                        out = self.imitate_model_forward()
                    des_tra_pos=self.gate_t_i.centroid+out[0:3]
                    des_tra_m=out[3:12]
                t_comp = time.time()
                
                
                    
                if self.options['JAX_SVD']:
                    cmd_solution,NO_SOLUTION_FLAG  = self.planner.mpc_update(self.state,
                                                        des_tra_pos,
                                                        des_tra_R,#des_tra_R, SVD JAX output
                                                        out[12]) # control input 4-by-1 thrusts to pybullet
                else:
                    cmd_solution,NO_SOLUTION_FLAG  = self.planner.mpc_update(self.state,
                                                        des_tra_pos,
                                                        des_tra_m,#des_tra_m, 9D vector
                                                        out[12])
                
                print('solving time at main=',time.time()-t_comp)
                self.solving_time.append(time.time()- t_comp)
                self.u=cmd_solution['control_traj_opt'][0,:].tolist()
                self.pos_vel_att_cmd=cmd_solution['state_traj_opt'][1,:] #self.config_dict['learning_agile']['horizon']
                # self.tra_weight_list.append(weight_vis)
            
                ##=== for integrator test===##
                # test_u=np.array([2.6,0,0,0])
                # if self.i>=500:
                #     test_u=np.array([4.6,160,0,0])
                

            ########################################################
            ###================= state update====================###
            ########################################################

            ###===================Explict Euler(obsolete) or ERK4====================###
            # self.state = np.array(self.planner.uav1.dyn_fn(self.state, test_u)).reshape(10) # Yixiao's simulation environment ('uav1.dyn_fn'), replaced by pybullet
            
            
            ##================= acados integrator IRK========================###
            self.integrator.set('x',np.array(self.state))

            
            self.integrator.set('u',np.array(self.u))
            # self.integrator.set('p',np.zeros(18))
            status_sim = self.integrator.solve()
            if status_sim != 0:
                raise Exception('acados integrator returned status {}. Exiting.'.format(status_sim))

            
            self.state = self.integrator.get('x')


            # re-normalize the quaternion
            # self.state[6:10] = self.state[6:10]jnp.linalg.det(U),jnp.linalg.det(Vh))/np.linalg.norm(self.state[6:10])


            self.state_n = np.concatenate((self.state_n,[self.state]),axis = 0)
            self.control_n = np.concatenate((self.control_n,[self.u]),axis = 0)
            self.pos_vel_att_cmd_n = np.concatenate((self.pos_vel_att_cmd_n,[self.pos_vel_att_cmd]),axis = 0)
            u_m = self.planner.uav1.u_m
            u1 = np.reshape(self.u,(4,1))
            tm = np.matmul(u_m,u1)
            tm = np.reshape(tm,4)
            # control_tm = np.concatenate((control_tm,[tm]),axis = 0)
            # self.hl_variable = np.concatenate((self.hl_variable,[out]),axis=0)       
            
        print('MPC finished')   
        np.save(os.path.join(python_sim_data_folder,'gate_points_list_traj'),self.gate_points_list)
        np.save(os.path.join(python_sim_data_folder,'uav_traj'),self.state_n)
        np.save(os.path.join(python_sim_data_folder,'uav_ctrl'),self.control_n)
        np.save(os.path.join(python_sim_data_folder,'abs_tra_time'),self.Ttra)
        np.save(os.path.join(python_sim_data_folder,'tra_time'),self.NN_T_tra)
        np.save(os.path.join(python_sim_data_folder,'Time'),self.Time)
        np.save(os.path.join(python_sim_data_folder,'Pitch'),self.Pitch)
        np.save(os.path.join(python_sim_data_folder,'HL_Variable'),self.hl_variable)
        np.save(os.path.join(python_sim_data_folder,'solving_time'),self.solving_time)
        np.save(os.path.join(python_sim_data_folder,'nn_output_list'),self.nn_output_list)
        np.save(os.path.join(python_sim_data_folder,'des_tra_R_list'),self.des_tra_R_list)
        self.planner.uav1.play_animation(wing_len=self.planner.wing_len,
                                       gate_traj1=self.gate_points_list[::5,:,:],
                                       state_traj=self.state_n[::5,:],
                                       goal_pos=self.final_point.tolist(),
                                       dt=0.01)
        
        # save the data, not show it
        if not self.options['STATIC_GATE_TEST']:
            self.planner.uav1.plot_position(self.nn_output_list,name='NN2_output')

            if self.options['CLOSE_LOOP_MODEL']:
                self.planner.uav1.plot_trav_time(self.NN_T_tra) # pure NN close loop traversal time
            else:
                self.planner.uav1.plot_trav_time(self.T) # Binary search traversal time
        self.planner.uav1.plot_thrust(self.control_n)
        self.planner.uav1.plot_angularrate(self.control_n)
        self.planner.uav1.plot_position(self.state_n,name='drone_actual')
        self.planner.uav1.plot_velocity(self.state_n)
        self.planner.uav1.plot_quaternions(self.state_n)

        # self.planner.uav1.plot_quaternions_norm(self.state_n)
        self.planner.uav1.plot_quaternions_norm(self.pos_vel_att_cmd_n)
        # self.planner.uav1.plot_trav_weight(self.tra_weight_list)

        self.planner.uav1.plot_solving_time(self.solving_time)
        python_sim_npy_parser(uav_traj=self.state_n,
                              nn_output_list=self.nn_output_list,
                              des_tra_R_list=self.des_tra_R_list,
                              gate_pitch=self.Pitch)
        self.planner.uav1.plot_3D_traj(wing_len=self.planner.wing_len,
                                    uav_height=self.planner.uav_height/2,
                                    state_traj=self.state_n[::50,:],
                                    gate_traj=self.gate_points_list[::50,:,:])


        # self.planner.uav1.plot_T(control_tm)
        # self.planner.uav1.plot_M(control_tm)
    
        
def main():
    # yaml file dir#
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    python_sim_data_folder = os.path.join(current_dir, 'python_sim_result')
    

    ########################################################################
    #####---------------------- TEST option -------------------------#######
    ########################################################################
    options={}
    options['MPC_BACKWARD']=False
    options['USE_PREV_SOLVER']=False
    options['PDP_GRADIENT']=False
    options['SQP_RTI_OPTION']=True
    options['STATIC_GATE_TEST']=False
    options['CLOSE_LOOP_MODEL']= False
    options['JAX_SVD']=False

    if options['CLOSE_LOOP_MODEL']:
        model_name = 'NN_close_0.pth'#'NN2_imitate_1.pth' #'NN_close_2.pth'
    else:   
        model_name = '20241018-093741-PDP-Trial_1/NN2_imitate_1.pth'


    model_file=os.path.join(current_dir, f'training_data/NN_model/',model_name)
    
    
    # create the learning agile agent
    # problem definition
    # the dyn_step is the simulation step in the simulation environment
    # for the acados ERK integrator, the step is (integral step)/4 =0.025s
    learing_agile_agent=LearningAgileAgent(python_sim_time=5,
                                           yaml_file=yaml_file,
                                           model_file=model_file,
                                           dyn_step=0.002,
                                            options=options)
    
    

    
    #####==============load env config ====================#######
    learing_agile_agent.generate_mission()
    learing_agile_agent.prepare_gate()
    
    #####============== Solve the problem ====================#######
    # solve the problem
    learing_agile_agent.forward_sim(python_sim_data_folder)

    # every time after reconstruct the solver, need to catkin build the MPC wrapper to 
    # relink the shared library
    shell_script="""catkin build mpc_ros_wrapper"""

    # run the shell script
    subprocess.run(shell_script,shell=True)

if __name__ == '__main__':
    main()
   