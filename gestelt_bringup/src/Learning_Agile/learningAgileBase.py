import numpy as np
import torch
import os
import cProfile
from collections import deque
from math import atan
from scipy.spatial.transform import Rotation as R


from solid_geometry import magni
from learning_agile_sim import LearningAgileSim, Gate

from config import mission_cfg, train_cfg,current_dir

## this options is for debugging
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
options = {}
options['MPC_BACKWARD']=True
options['USE_PREV_SOLVER']=False
options['PDP_GRADIENT']= True
options['SQP_RTI_OPTION']=True
options['JAX_SVD']=False
options['STATIC_GATE_TEST']=False
options['ORIGIN_REWARD']=False
options['CLOSE_LOOP_TRAINING']=True
options['TRAINING']=False
options['DEBUG']=False
options['BACKWARD']=True
class LearningAgileBase:
    """
    this class is responsible for wrap the single episode,
    include forward, reward, backward, 
    return the Reward and Gradient of this episode
    Reward: [R_0, R_1, R_2, ..., R_H] for each step's prediction
    Gradient: [p_R_0_p_w, p_R_1_p_w, p_R_2_p_w, ..., p_R_H_p_w] for each step's prediction
    """
    def __init__(self,
                 mission_cfg:dict,
                 train_cfg:dict,
                 options:dict):
        self.mission_cfg = mission_cfg
        self.train_cfg = train_cfg

        self.gate_v = np.array(self.mission_cfg['gate']['linear_vel'])
        self.gate_w = mission_cfg['gate']['angular_vel'] 
        self.NN_freq = mission_cfg['NN2_freq']
        self.learning_agile_sim = LearningAgileSim(python_sim_time=5,
                                                    mission_cfg=self.mission_cfg,
                                                    dyn_step=1/self.NN_freq,
                                                    options=options)
        self.planner = self.learning_agile_sim.planner
        ## keep history five states, RING BUFFER
        self.input_size = 26
        self.output_size = 13

        
        self.history_obs = deque(maxlen=5)

    def load_model(self,model_folder):
        ##== load the pre-trained model ==##
        FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        self.model = torch.load(FILE).to(device)

    def reset(self):
        #== random generate the env and set to the mpc solver
        self.learning_agile_sim.generate_mission()
        
        self.state = self.planner.ini_state

        #== reset the gate
        self.learning_agile_sim.prepare_gate()
        self.gate_points_list = self.learning_agile_sim.gate_points_list
        
        #== reset the obs
        
        
        self.init_gradient()
        self.state_traj=[]
        self.state_n = np.array([self.state])

        obs=self.get_obs(0)
        return obs



    def get_obs(self,i):
        self.i = i
        ## == gate forward === ##
        gate_t_i = Gate(self.gate_points_list[i])
        gate_pitch = atan((gate_t_i.gate_point[0,2]-gate_t_i.gate_point[1,2])/(gate_t_i.gate_point[0,0]-gate_t_i.gate_point[1,0])) # compute the actual gate pitch ange in real-time
        
        ##==calculate the gate RM
        rot=R.from_euler('zyx',[0,gate_pitch,0])

        immed_obs=np.zeros(self.input_size)
        immed_obs[0:10]=self.state
        immed_obs[10:13]=self.learning_agile_sim.final_point

        # position of the gate,# width of the gate,# pitch angle of the gate
        immed_obs[13:16] = gate_t_i.centroid
        immed_obs[16] = magni(gate_t_i.gate_point[0,:]-gate_t_i.gate_point[3,:]) # gate width
        immed_obs[17:26]=rot.as_matrix().flatten()

        if i == 0:
            for i in range(5):
                self.history_obs.append(immed_obs)
        else:
            self.history_obs.append(immed_obs)
        
        self.obs=np.array(self.history_obs)
        self.planner.init_obstacle(gate_t_i.gate_point[:,:].reshape(12),gate_pitch)

        return self.obs

    def get_NN_decision(self,obs):
        # NN output the traversal time and pose
        nn_out = self.model(torch.tensor(obs, dtype=torch.float).unsqueeze(0).to(device))[0]
        
        
        return nn_out
    
    def get_NN_decision_debug(self,obs):
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
        self.R_i = []
        self.p_R_i_p_X_traj_i = []
        self.p_X_traj_i_p_x_i = []
        self.p_X_traj_i_p_z_i = []
        self.p_R_i_p_z_i = []
        self.p_z_i_p_w = []
        self.p_R_i_p_w = []

    
    
    def step(self,nn_out=None):

    
        if nn_out is None:
            ## if training, self.nn_out comes from the model for a batch of episodes
            if options['DEBUG']:
                self.nn_out = self.get_NN_decision_debug(self.obs)
            else:  
                self.nn_out = self.get_NN_decision(self.obs)       
        else:
            ## from the model for a batch of episodes
            self.nn_out = nn_out

        self.np_nn_out = self.nn_out.to('cpu').data.numpy()
        ## == MPC forward === ##
        cmd_solution,NO_SOLUTION_FLAG = self.planner.mpc_update(self.state,
                                            self.np_nn_out[0:3],
                                            self.np_nn_out[3:12],
                                            self.np_nn_out[-1]) # control input 4-by-1 thrusts to pybullet
        
       
        ## record the gradient and step reward
        # log_train_IO(self.writer,nn_input,self.np_nn_out.reshape(output_size),self.i+self.training_horizon*global_step)
        # log_gradient(self.writer,grads_list[0,:].reshape(output_size+1),global_step)

        self.pred_st_traj = cmd_solution['state_traj_opt']
        self.control_traj = cmd_solution['control_traj_opt']
        

        ## === state update === ##
        self.state = cmd_solution['state_traj_opt'][1,:]
        self.u = cmd_solution['control_traj_opt'][0,:].tolist()

        ## === actual trajectory === ##
        self.state_traj.append(self.state)
        self.state_n = np.concatenate((self.state_n,[self.state]),axis = 0)
       
    
    
    def backward_per_step(self):
        """
        get the gradient of the reward w.r.t. the NN output, 
        store the gradient per step in a list
        H: close loop horizon
        N: prediction horizon
        """

        # append N * 1* 10
        ## this has been done in the get_reward function
        # self.p_R_i_p_X_traj_i.append(self.planner.d_R_d_st_traj[:,:,:])

        
        ## acquire p_X_traj_i/p_x_i
        # cur_p_X_traj_i_p_x_i = np.ones([self.planner.horizon+1,10,10])
        
        # for j in range(self.planner.horizon+1):
        #     if j == 1:
        #         cur_p_X_traj_i_p_x_i[j,:,:] = self.dyn_decay * self.planner.uavoc1.dfx_fn(self.pred_st_traj[j-1],self.control_traj[j-1]).toarray()    
            
        #     if j > 1:
        #         cur_p_X_traj_i_p_x_i[j,:,:] = self.dyn_decay * self.planner.uavoc1.dfx_fn(self.pred_st_traj[j-1],self.control_traj[j-1]).toarray() * cur_p_X_traj_i_p_x_i[j-1,:,:]
        
        # self.p_X_traj_i_p_x_i.append(torch.tensor(cur_p_X_traj_i_p_x_i, dtype=torch.float).to(self.device))
        
        ## acquire p_X_traj_i/p_z_i
        self.planner.PDP_grad(self.np_nn_out[0:3],self.np_nn_out[3:12],self.np_nn_out[-1])
        # append size N * 10 * 13
        self.p_X_traj_i_p_z_i.append(self.planner.d_st_traj_d_z[:,:,:])

        ## acquire p_z_i/p_w (w is the weight of the NN)
        # append size 13 * 1
        # self.p_z_i_p_w.append(self.nn_out.unsqueeze(1))
        self.p_z_i_p_w.append(self.np_nn_out.reshape(13,1))
        ## 13 * 1
        self.p_R_i_p_z_i.append(np.einsum('bij,bjk->ik',self.p_R_i_p_X_traj_i[self.i-2],self.p_X_traj_i_p_z_i[self.i-2]))
        
        ## acquire p_R_i/p_w
        # append size 1 * 1
        # self.p_R_i_p_w.append(np.matmul(p_R_i_p_z_i,self.p_z_i_p_w[self.i-1]))

        # print(self.p_R_i_p_X_traj_i[self.i-1].shape)
        # print(self.p_X_traj_i_p_z_i[self.i-1].shape)
        # print(self.p_z_i_p_w[self.i-1].shape)
        # print(p_R_i_p_z.shape)
    

    # def run_single_step(self,nn_output):
    #     self.step(nn_output)
    #     self.get_reward()
    #     if options['BACKWARD']:
    #         self.backward_per_step()

    @property
    def drone_state(self):
        return self.state
    
    @property
    def reward(self):
        return np.array([sum(self.R_i)])
    
    @property
    def p_R_p_z(self):
        p_R_p_z = np.sum(np.array(self.p_R_i_p_z_i),axis=0)/(self.train_cfg['training']['close_loop_horizon']*1000)

        return p_R_p_z
   
def get_reward(base):
    """
    calculate the reward of MPC solution
    """
    R_i = np.array(base.planner.get_reward(base.pred_st_traj)[0])
    p_R_i_p_X_traj_i = (base.planner.get_reward(base.pred_st_traj)[1])

    return [R_i, p_R_i_p_X_traj_i]

def run_single_episode(base,nn_out=None):
    base.reset()
    if options['TRAINING']:
        base.nn_out = nn_out

    for i in range(base.train_cfg['training']['close_loop_horizon']):
        base.get_obs(i)
        base.step()

        if i > 0: 
            # skip the first step since the first prediction of the SQP_RTI is initial guess
            reward_and_gradient=get_reward(base)
            base.R_i.append(reward_and_gradient[0])
            base.p_R_i_p_X_traj_i.append(reward_and_gradient[1])

            if options['BACKWARD']:
                base.backward_per_step()    


def run_debug(planner,state_n,final_point,gate_points_list):
    
    planner.uav1.play_animation(wing_len=planner.wing_len,
                                gate_traj1=gate_points_list[:,:,:],
                                state_traj=np.array(state_n[:,:]),
                                goal_pos=final_point.tolist(),
                                dt=0.1)
    
if __name__ == "__main__":
    training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
    model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))


    # cProfile.run("base=LearningAgileBase(mission_cfg,train_cfg,options)")
    # cProfile.run("base.load_model(model_folder)")
    # cProfile.run("base.run_single_episode(DEBUG=True,BACKWARD=False)")
    base=LearningAgileBase(mission_cfg,train_cfg,options)
    base.load_model(model_folder)
    run_single_episode(base)
    print(base.reward)
    print(base.p_R_p_z)
    # run_debug(base.learning_agile_sim.planner,
    #               base.state_n,
    #               base.learning_agile_sim.final_point,
    #               base.learning_agile_sim.gate_points_list)