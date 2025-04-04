import numpy as np
import torch
import os
# import cProfile
from collections import deque

from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

from geometry.solid_geometry import magni,pitch_from_gate,recover_euler_from_9d,verify_SVD_ca
from learning_agile_sim import LearningAgileSim, Gate,get_obs

from config import mission_cfg, train_cfg,current_dir

## this options is for debugging
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
options = {}
options['MPC_BACKWARD']=True
options['USE_PREV_SOLVER']=False
options['PDP_GRADIENT']= True
options['SQP_RTI_OPTION']=True
options['JAX_SVD']=False
options['MANUAL_SET_POSE_TEST']=False
options['ORIGIN_REWARD']=False
options['CLOSE_LOOP_TRAINING']=True
options['TRAINING']=False
options['DEBUG']=False
options['BACKWARD']=True
options['STATE_2_MOVING_GATE']=False
class LearningAgileBase:
    """
    this class is responsible for wrap the single episode for training,
    take the MPC predicted first state as the drone real state to update,
    include forward, penalty, backward, 
    return the penalty and Gradient of this episode
    penalty: [L_0, L_1, L_2, ..., L_H] for each step's prediction
    Gradient: [p_L_0_p_w, p_L_1_p_w, p_L_2_p_w, ..., p_L_H_p_w] for each step's prediction
    """
    def __init__(self,
                 mission_cfg:dict,
                 train_cfg:dict,
                 options:dict):
        self.mission_cfg = mission_cfg
        self.train_cfg = train_cfg

        self.learning_agile_sim = LearningAgileSim(python_sim_time=self.mission_cfg['mission_period'],
                                                    mission_cfg=self.mission_cfg,
                                                    train_cfg=self.train_cfg,
                                                    dyn_step=0.002,
                                                    options=options)
        self.planner = self.learning_agile_sim.planner
      
        self.input_size = train_cfg['model']['input_size']
        self.output_size = train_cfg['model']['output_size']
        self.history_obs = deque(maxlen=5)
        self.control=np.zeros(4)
        self.reg_control=0
        self.reg_det=0
        self.dreg_dz = np.zeros([1,self.output_size])
        self.last_u=np.zeros(4)
        self.np_nn_out=np.zeros(self.output_size)
        self.i=0
        
    def load_model(self,model_folder):
        ##== load the pre-trained model ==##
        FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        self.model = torch.load(FILE).to(device)

    def reset(self,cur_epoch: int=0):
        #== random generate the env and set to the mpc solver
        self.learning_agile_sim.generate_mission(cur_epoch,
                                                 TEST=self.mission_cfg['FIX_GATE_PITCH_TEST'])
        
        

        #== reset the gate
        self.learning_agile_sim.prepare_gate()
        self.gate_points_list = self.learning_agile_sim.gate_points_list
        
        #== reset the gradient
        self.init_gradient()
        
        #== reset the state
        self.state_traj=[]
        self.state = self.planner.ini_state
        self.state_n = np.array([self.state])

        #== reset the obs
        self.gate_step_and_obs(0)
 

    def gate_step_and_obs(self,i):
        self.i = i
        ## == gate forward === ##
        gate_t_i = Gate(self.gate_points_list[i])

        self.obs,_ = get_obs(self.history_obs,
                            self.i,
                            self.input_size,
                            self.state,
                            self.learning_agile_sim.final_point,
                            gate_t_i)
        return self.obs

    def get_NN_decision(self,obs):
        # NN output the traversal time and pose
        return self.model(torch.tensor(obs, dtype=torch.float).unsqueeze(0).to(device))[0]
    
    def get_NN_decision_debug(self):
        # manually set the traversal time and pose
        np_nn_out=np.zeros(self.output_size)
        np_nn_out[0:3]=[0,0,0]
        np_nn_out[3:12]=np.array([[0.0007963,  0.0000000, -0.9999997],
                                  [0.0000000,  1.0000000,  0.0000000],
                                  [0.9999997,  0.0000000,  0.0007963]]).flatten()
        # np_nn_out[3:12]=np.eye(3).flatten()
        t_tra_abs=1.5
        np_nn_out[12]=t_tra_abs-self.i*0.1
        return np_nn_out
    
    def init_gradient(self):
        self.L_i = []
        self.p_L_i_p_X_traj_i = []
        self.p_X_traj_i_p_x_i = []
        self.p_X_traj_i_p_z_i = []
        self.p_L_i_p_z_i = []
        self.p_z_i_p_w = []
        self.p_L_i_p_w = []

        self.p_L_i_p_x_i={}
        self.p_L_i_p_z_last={}
    
    
    def step(self,nn_out=None):

    
        if nn_out is None:
            ## if training, nn_out comes from the model for a batch of episodes
            if options['DEBUG']:
                nn_out = self.get_NN_decision_debug()
            else:  
                nn_out = self.get_NN_decision(self.obs)       

        self.np_nn_out = nn_out.to('cpu').data.numpy()
        self.t_tra_rel = self.np_nn_out[-1]
        
        ## == MPC forward === ##        
        cmd_solution,NO_SOLUTION_FLAG = self.planner.mpc_update(cur_state=self.state,
                                                                trav_auxvar_value=self.np_nn_out,
                                                                last_u=self.last_u,
                                                                first_iter=(self.i==0))
        if NO_SOLUTION_FLAG:
            print('No solution found')
            print('traverse_auxvar_value=',self.np_nn_out)
            

        self.pred_st_traj = cmd_solution['state_traj_opt']
        self.control_traj = cmd_solution['control_traj_opt']
        

        ## === state update === ##
        self.state = cmd_solution['state_traj_opt'][1,:]
        self.control = cmd_solution['control_traj_opt'][0,:]
        self.last_u = cmd_solution['control_traj_opt'][0,:]
        ## === actual trajectory === ##
        self.state_traj.append(self.state)
        self.state_n = np.concatenate((self.state_n,[self.state]),axis = 0)
       
        ## === initial gate obstacle based on current NN prediction === ##
        pred_t_i = self.i + self.t_tra_rel*(1/self.mission_cfg['learning_agile']['dt'])
        gate_t_pred = Gate(self.gate_points_list[int(pred_t_i)])
        self.planner.init_obstacle(gate_t_pred)

        ## === MPC backward === ##
        self.planner.PDP_grad(self.np_nn_out)
    
    # def get_reg_euler(self):
    #     """regularize the output of the neural network

    #     """
    #     euler_nn,deuler_dm = recover_euler_from_9d(self.np_nn_out)
    #     reg_euler=self.mission_cfg['penalty']['euler_reg_w']*np.linalg.norm(euler_nn)
    #     dreg_deuler = self.mission_cfg['penalty']['euler_reg_w']*(euler_nn/reg_euler).reshape(1,-1)
    #     self.dreg_dz[:,3:12]=np.matmul(dreg_deuler,deuler_dm).flatten()
    # def get_reg_m(self):
    #     """
    #     regularize the output of the neural network, 
    #     The NN output reference should be as close as the current drone state

    #     """
    #     # the NN output R
    #     des_tra_R,dR_dm=verify_SVD_ca(self.np_nn_out[3:12])

    #     # the current drone R
    #     r=R.from_quat(np.roll(self.state[6:10],-1)) # w,x,y,z -> x,y,z,w
    #     cur_drone_R=r.as_matrix()

    #     self.reg_m=self.mission_cfg['penalty']['m_reg_w']*np.trace(np.eye(3)-np.dot(des_tra_R,cur_drone_R.T))
    #     dreg_dR = self.mission_cfg['penalty']['m_reg_w']*(des_tra_R.T)
    #     self.dreg_dz[:,3:12]=np.matmul(dreg_dR.flatten(),dR_dm).flatten()

        
    def get_reg_det(self):
        """
        regularize the output of the neural network m, the determinant of the m should be close to 1

        """
        m=self.np_nn_out[3:12].reshape(3,3)
        self.reg_det=self.mission_cfg['penalty']['det_reg_w']*(np.linalg.det(m)-1)**2
        adj_m_T = np.linalg.det(m)*np.linalg.inv(m).T
        dreg_det_dm = self.mission_cfg['penalty']['det_reg_w']*2*(np.linalg.det(m)-1)*adj_m_T
        self.dreg_dz[:,3:12]=dreg_det_dm.flatten()


    def get_reg_control(self):
        ## acquire p_X_traj_i/p_z_i
        
        self.reg_control = self.mission_cfg['penalty']['control_reg_w']*np.dot(self.control-self.planner.hover_u,self.control-self.planner.hover_u)
        dreg_dcontrol = self.mission_cfg['penalty']['control_reg_w']*2*(self.control-self.planner.hover_u)
        self.dreg_dz[:,:]=np.matmul(dreg_dcontrol,self.planner.d_input_traj_d_z[0,:,:]).flatten()
           
    def backward_per_step(self,dyn_decay=0.9):
        """
        get the gradient of the penalty w.r.t. the NN output, 
        store the gradient per step in a list
        H: close loop horizon
        N: prediction horizon
        """
        
        ## acquire p_X_traj_i/p_x_i
        cur_p_X_traj_i_p_x_i = np.ones([self.planner.horizon+1,len(self.state),len(self.state)])
        
        for j in range(1,self.planner.horizon+1):
            cur_p_X_traj_i_p_x_i[j,:,:] = self.planner.uavoc.dfx_fn(self.pred_st_traj[j-1],self.control_traj[j-1]).toarray() * cur_p_X_traj_i_p_x_i[j-1,:,:]
        
        self.p_X_traj_i_p_x_i.append(cur_p_X_traj_i_p_x_i)
        
        
        # append size N * 10 * 13
        self.p_X_traj_i_p_z_i.append(self.planner.d_st_traj_d_z[:,:,:])
       
        ## 13 * 1
        # self.p_L_i_p_z_i.append(np.einsum('bij,bjk->ik',self.p_L_i_p_X_traj_i[self.i-2],self.p_X_traj_i_p_z_i[self.i-2]))
        ## z_i:
        self.p_L_i_p_z_i.append(np.einsum('bij,bjk->ik',self.p_L_i_p_X_traj_i[self.i-2],self.p_X_traj_i_p_z_i[self.i-2])) #+self.dreg_dz
        
        # if self.i > 10:
        #     print('debug')
        
        # # Backpropagate through the last one time-step
        # if self.i > 2:
        #     self.p_L_i_p_x_i[f'{self.i}-2'] = np.einsum('bij,bjk->ik', self.p_L_i_p_X_traj_i[self.i-2],self.p_X_traj_i_p_x_i[self.i-2])
        #     self.p_L_i_p_z_last[f'{self.i}-3'] = self.p_L_i_p_x_i[f'{self.i}-2'] @ self.p_X_traj_i_p_z_i[self.i-3][1, :, :]

        #     self.p_L_i_p_z_i[self.i-3] += self.p_L_i_p_z_last[f'{self.i}-3']
        
        

        # if self.i > 3:       
        #     # Backpropagate through the last second time-step     
        #     self.p_L_i_p_x_i[f'{self.i}-3'] = dyn_decay * self.p_L_i_p_x_i[f'{self.i}-2'] @ self.p_X_traj_i_p_x_i[self.i-3][1, :, :]
        #     self.p_L_i_p_z_last[f'{self.i}-4'] =  self.p_L_i_p_x_i[f'{self.i}-3'] @ self.p_X_traj_i_p_z_i[self.i-4][1, :, :]

        #     self.p_L_i_p_z_i[self.i-4] += self.p_L_i_p_z_last[f'{self.i}-4']

        #     # # # Backpropagate through all last time-steps
        #     for k in range(4, self.i):
        #         self.p_L_i_p_x_i[f'{self.i}-{k}'] = dyn_decay * self.p_L_i_p_x_i[f'{self.i}-{k-1}'] @ self.p_X_traj_i_p_x_i[self.i-k][1, :, :]
        #         self.p_L_i_p_z_last[f'{self.i}-{k+1}'] = self.p_L_i_p_x_i[f'{self.i}-{k}'] @ self.p_X_traj_i_p_z_i[self.i-k-1][1, :, :]

        #         self.p_L_i_p_z_i[self.i-k-1] += self.p_L_i_p_z_last[f'{self.i}-{k+1}']
                

        #         if k == 10:
        #             # BPTT k = 5 last four time-steps
                    # break



    @property
    def drone_state(self):
        return self.state
    
    @property
    def penalty(self):
        return np.array([sum(self.L_i)])/mission_cfg['learning_agile']['horizon']
    
    @property
    def p_L_p_z(self):
        p_L_p_z = np.array(self.p_L_i_p_z_i)
        return p_L_p_z/mission_cfg['learning_agile']['horizon']

   
# def get_penalty(base:LearningAgileBase):
#     """
#     calculate the penalty of MPC solution
#     """
#     L_i = np.array(base.planner.get_penalty(base.pred_st_traj)[0])
#     p_L_i_p_X_traj_i = (base.planner.get_penalty(base.pred_st_traj)[1])

#     return [L_i, p_L_i_p_X_traj_i]

# def run_single_episode(base,nn_out=None):
#     base.reset()
#     if options['TRAINING']:
#         base.nn_out = nn_out

#     for i in range(base.train_cfg['training']['close_loop_horizon']):
#         base.gate_step_and_obs(i)
#         base.step()

#         if i > 0: 
#             # skip the first step since the first prediction of the SQP_RTI is initial guess
#             penalty_and_gradient=get_penalty(base)
#             base.L_i.append(penalty_and_gradient[0])
#             base.p_L_i_p_X_traj_i.append(penalty_and_gradient[1])

#             if options['BACKWARD']:
#                 base.backward_per_step()    


# def run_debug(planner,state_n,final_point,gate_points_list):
    
#     planner.uav1.play_animation(wing_len=planner.wing_len,
#                                 gate_traj1=gate_points_list[:,:,:],
#                                 state_traj=np.array(state_n[:,:]),
#                                 goal_pos=final_point.tolist(),
#                                 dt=0.1)
    
def vis_gradient_norm(p_L_p_z_batch:np.array):
    p_L_p_z_norm = np.linalg.norm(p_L_p_z_batch,axis=2)
    p_L_p_z_norm = p_L_p_z_norm.mean(axis=0)
    ## plot the gradient norm
    fig, ax = plt.subplots()
    t=range(len(p_L_p_z_norm))
    ax.plot(t,p_L_p_z_norm)
    ax.set(xlabel='time step', ylabel='gradient norm',
           title='Gradient Norm')
    ax.grid()
    plt.show()

# if __name__ == "__main__":
#     training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
#     model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))


#     # cProfile.run("base=LearningAgileBase(mission_cfg,train_cfg,options)")
#     # cProfile.run("base.load_model(model_folder)")
#     # cProfile.run("base.run_single_episode(DEBUG=True,BACKWARD=False)")
#     base=LearningAgileBase(mission_cfg,train_cfg,options)
#     base.load_model(model_folder)
#     run_single_episode(base)
#     print(base.penalty)
#     print(base.p_L_p_z)

#     vis_gradient_norm(base.p_L_p_z)
    # run_debug(base.learning_agile_sim.planner,
    #               base.state_n,
    #               base.learning_agile_sim.final_point,
    #               base.learning_agile_sim.gate_points_list)