## this file is for deep learning
"""
here, DNN1 is trained for static gate, random drone static initial position and orientation
DNN1 should generate single one open loop MPC decision variables. supervised by finite policy gradient
"""
from quad_nn import *
from quad_model import *
from quad_policy import *
from multiprocessing import Process, Array
import rospy
import yaml
from logger_config import LoggerConfig
import logging
from tqdm import tqdm
from torch.utils.tensorboard import SummaryWriter
import datetime
import os
from learning_agile_agent import MovingGate, LearningAgileAgent

# for multiprocessing, obtain the gradient

"""
finite policy gradient,
output is the decision variables z. [x,y,z,a,b,c,t_traverse]
[a,b,c] is the theta*k, k is the rotation axis
"""

def log_train_in_ouputs(writer,inputs,outputs,global_step):

    
    writer.add_scalar('gate_pitch', inputs[8], global_step)
  
    writer.add_scalar('x_tra', outputs[0], global_step)
    writer.add_scalar('y_tra', outputs[1], global_step)
    writer.add_scalar('z_tra', outputs[2], global_step)
    writer.add_scalar('Rx_tra', outputs[3], global_step)
    writer.add_scalar('Ry_tra', outputs[4], global_step)
    writer.add_scalar('Rz_tra', outputs[5], global_step)
    writer.add_scalar('t_tra', outputs[6], global_step)
    

def log_gradient(writer,gra,global_step):
    writer.add_scalar('drdx', gra[0], global_step)
    writer.add_scalar('drdy', gra[1], global_step)
    writer.add_scalar('drdz', gra[2], global_step)
    writer.add_scalar('drda', gra[3], global_step)
    writer.add_scalar('drdb', gra[4], global_step)
    writer.add_scalar('drdc', gra[5], global_step)
    writer.add_scalar('drdt', gra[6], global_step)
    writer.add_scalar('step_reward', gra[7], global_step)


class DirectCloseLoop():
    def __init__(self, mission_yaml, train_yaml) -> None:
        ## =================load parameters from yaml======================##
        # load the configuration file
        self.mission_yaml = mission_yaml
        with open(mission_yaml, 'r', encoding='utf-8') as file:
            self.config_dict = yaml.safe_load(file)
        
        with open(train_yaml, 'r', encoding='utf-8') as file:
            self.train_cfg_dict = yaml.safe_load(file)

        self.gate_v = np.array(self.config_dict['gate']['linear_vel'])
        self.gate_w = self.config_dict['gate']['angular_vel'] 
        self.NN_freq = self.config_dict['NN2_freq']

        ##====================NN initialization ========================##
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        self.learning_agile_agent = LearningAgileAgent(self.config_dict,
                                                       self.mission_yaml,
                                                       USE_PREV_SOLVER=False,
                                                       dyn_step=1/self.NN_freq,
                                                       PDP_GRADIENT=self.train_cfg_dict['training']['PDP_GRADIENT'],
                                                       SQP_RTI_OPTION=False)
        self.init_log()
    def init_log(self):
        log_dir = os.path.join(current_dir, "NN_direct_close_loop_training_logs")
        current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        PDP_GRADIENT = self.train_cfg_dict['training']['PDP_GRADIENT']
        if PDP_GRADIENT:
            method_name = 'PDP'
        else:
            method_name = 'FD'

        file_dir = os.path.join(log_dir, f"train-{current_time}-{method_name}")
        self.writer = SummaryWriter(log_dir=file_dir)
        logger_config=LoggerConfig("NN_direct_close_loop_training_logs")


        ##== save the yaml file in the log folder ==##
        logger_config.log_yaml_data(self.config_dict)
        logger_config.log_yaml_data(self.train_cfg_dict)


    def init_gradient(self):
        self.R_i = 0
        self.p_R_i_p_X_traj_i =[]
        self.p_x_next_p_x_i = []
        self.p_x_i_p_z_i = []
        self.p_z_i_p_w = []

    
    def init_train(self,model_folder):
        input_size = 18 
        hidden_size = 128 
        output_size = 7
        self.training_horizon = self.train_cfg_dict['training']['horizon']
        learning_rate = self.train_cfg_dict['training']['learning_rate']
    

        ##== load the pre-trained model ==##
        FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        self.model = torch.load(FILE).to(self.device)

         # Loss and optimizer
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)  

    def reset(self):
        
        #== random generate the env and set to the mpc solver
        self.learning_agile_agent.receive_mission_states(STATIC_GATE_TEST=False)
        self.quad = self.learning_agile_agent.quad
        self.state = self.quad.ini_state

        #== reset the gate
        self.learning_agile_agent.prepare_gate()
        self.i = 0
        self.gate_t_i = self.learning_agile_agent.gate_t_i
        self.gate_points_list = self.learning_agile_agent.gate_points_list
        

        self.init_gradient()
        self.state_traj=[]

    def step(self,global_step):
        """
        1. gate forward
        2. NN forward
        3. MPC forward
        4. state update
        """
        ## == gate forward === ##
        self.gate_t_i = gate(self.gate_points_list[self.i])
        
        ## MPC update step is 0.1s
        if (self.i % 2) == 0:
            
            ## == NN forward === ##
            nn_input=np.zeros(18)
            nn_input[0:10]=self.state
            nn_input[10:13]=self.learning_agile_agent.final_point


            # position of the gate
            nn_input[13:16] = self.gate_t_i.centroid
            # width of the gate
            nn_input[16] = magni(self.gate_t_i.gate_point[0,:]-self.gate_t_i.gate_point[1,:]) # gate width
            # pitch angle of the gate
            nn_input[17] = atan((self.gate_t_i.gate_point[0,2]-self.gate_t_i.gate_point[1,2])/(self.gate_t_i.gate_point[0,0]-self.gate_t_i.gate_point[1,0])) # compute the actual gate pitch ange in real-time
            
            # NN output the traversal time and pose
            self.nn_output = self.model(torch.tensor(nn_input, dtype=torch.float).to(self.device))
            self.np_nn_output = self.nn_output.to('cpu').data.numpy()
            
                
            ## == MPC forward === ##
            cmd_solution,NO_SOLUTION_FLAG = self.quad.mpc_update(self.state,
                                                self.np_nn_output[0:3],
                                                self.np_nn_output[3:6],
                                                self.np_nn_output[6]) # control input 4-by-1 thrusts to pybullet
            
            log_train_in_ouputs(self.writer,nn_input,self.np_nn_output,self.i+2*self.training_horizon*global_step)
        
            self.u = cmd_solution['control_traj_opt'][0,:].tolist()
            
           

            ## === state update === ##
            self.state = cmd_solution['state_traj_opt'][1,:]
            
            self.state_traj.append(self.state)
        

    
    def backward(self):
        
        ## acquire the gradient p_R_i/p_X_traj_i
        self.R_i=self.quad.R_from_MPC_pred(self.np_nn_output[0:3],self.np_nn_output[3:6],self.np_nn_output[6])

        # size mpc_horizon x 10
        self.p_R_i_p_X_traj_i.append(torch.tensor(self.quad.d_R_d_st_traj[:,:,:], dtype=torch.float).to(self.device))
        

        ## acquire the gradient p_X_traj_i/p_x_i
        
        ## acquire the gradient p_x_i+1/p_x_i
        # size 10x10
        self.p_x_next_p_x_i.append(torch.tensor(self.quad.uavoc1.dfx_fn(self.state,self.u).toarray(), dtype=torch.float).to(self.device))
        
        ## acquire the gradient p_x_i/p_z_i
        self.quad.PDP_grad(self.np_nn_output[0:3],self.np_nn_output[3:6],self.np_nn_output[6])
        # size 10x7
        self.p_x_i_p_z_i.append(torch.tensor(self.quad.d_st_traj_d_z[1,:,:], dtype=torch.float).to(self.device))

        ## acquire the gradient p_z_i/p_w (w is the weight of the NN)
        # size 7x1
        self.p_z_i_p_w.append(self.nn_output)
       


    def train_one_step(self,global_step):

        """ 
        the drone will execute the whole trajectory and then update the weights of the NN
        """
        R=0
        d_R_d_w=torch.zeros(1).to(self.device)

        ## for each node in the trajectory
        for self.i in range(2*self.training_horizon):
            self.step(global_step)
            self.backward()
            
            R += self.R_i
            d_R_i_d_w = torch.zeros(1).to(self.device)
    
            ## each node is related to all history nodes NN decisions
            for k in range(0,self.i):
                p_R_i_p_w = self.p_z_i_p_w[k] # size 7x1
                p_R_i_p_w = torch.matmul(self.p_x_i_p_z_i[k],p_R_i_p_w) # 10x7 * 7x1 = 10x1
                j=k

                ## propagate the gradient from the k-th history node to the i-th current node
                while j < self.i:
                    p_R_i_p_w = 0.94 * torch.matmul(self.p_x_next_p_x_i[j] , p_R_i_p_w) # 10x10 * 10x1 = 10x1
                    j+=1   

                p_R_i_p_w = torch.matmul(self.p_R_i_p_X_traj_i[self.i] , p_R_i_p_w) #1x10 * 10x1 = 1x1
                d_R_i_d_w += p_R_i_p_w
            
            d_R_d_w += -d_R_i_d_w
            self.i += 1
        d_R_d_w = d_R_d_w/(self.training_horizon*100)
        
        self.optimizer.zero_grad()
        d_R_d_w.backward()
        self.optimizer.step()

        ##== log the gradient and reward ==##
        self.writer.add_scalar('step_reward', R, global_step)
        self.writer.add_scalar('gradient', d_R_d_w, global_step)

    def train_loop(self,model_folder):
        num_epochs = self.train_cfg_dict['training']['num_epochs']
        steps_per_epoch = self.train_cfg_dict['training']['steps_per_epoch']
        global_step = 0
        for epoch in range(num_epochs):

            with tqdm(total=steps_per_epoch, desc=f'Epoch {epoch+1}/{num_epochs}', unit='epoch') as pbar:

                for cur_step in range(steps_per_epoch):
                    self.reset()
                    self.train_one_step(global_step)
                   
                    pbar.set_postfix({'step': cur_step})
                    pbar.update(1)

                    global_step += 1
            
            
            torch.save(self.model,  model_folder+"/NN_close_"+str(epoch)+".pth")
def main():
    # yaml file dir#
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    mission_yaml = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    train_yaml = os.path.join(conf_folder, 'training_params.yaml')
    training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
    model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))

    direct_close_loop=DirectCloseLoop(mission_yaml, train_yaml)
    direct_close_loop.init_train(model_folder)
    direct_close_loop.train_loop(model_folder)
    
if __name__ == '__main__':
    main()