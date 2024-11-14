## this file is for deep learning
"""
here, DNN1 is trained for static gate, random drone static initial position and orientation
DNN1 should generate single one open loop MPC decision variables. supervised by finite policy gradient
"""
from quad_policy import *
from quad_nn import *
from quad_model import *
from collections import deque
from multiprocessing import Process, Array
import rospy
import yaml
from logger_misc import LoggerConfig
import logging
from tqdm import tqdm
from torch.utils.tensorboard import SummaryWriter
import datetime
import os
from learning_agile_sim import MovingGate, LearningAgileSim
from deep_learning import log_train_IO, log_gradient

# for multiprocessing, obtain

"""
finite policy gradient,
output is the decision variables z. [x,y,z,a,b,c,t_traverse]
[a,b,c] is the theta*k, k is the rotation axis
"""
input_size = 26 # current drone state (10), goal position (3), gate position(3), gate width(1) and orientation(9)
output_size = 13  # #tra_pos(3), tra_9D_orientation(9), traversing_time(1)


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
        # self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.device = torch.device('cpu')
        options={}
        options['MPC_BACKWARD']=True
        options['USE_PREV_SOLVER']=False
        options['PDP_GRADIENT']= True
        options['SQP_RTI_OPTION']=False
        options['JAX_SVD']=False
        options['STATIC_GATE_TEST']=False
        options['ORIGIN_REWARD']=False
        options['CLOSE_LOOP_TRAINING']=True
        self.learning_agile_sim = LearningAgileSim(python_sim_time=5,
                                                       yaml_file=self.mission_yaml,
                                                       dyn_step=1/self.NN_freq,
                                                       options=options)
        # self.learning_agile_sim = LearningAgileSim(self.config_dict,
        #                                                self.mission_yaml,
        #                                                USE_PREV_SOLVER=False,
        #                                                dyn_step=1/self.NN_freq,
        #                                                PDP_GRADIENT=self.train_cfg_dict['training']['PDP_GRADIENT'],
        #                                                SQP_RTI_OPTION=False)
        self.init_log()

        ## keep history five states, RING BUFFER
        self.history_state = deque(maxlen=5)
    def init_log(self):
        log_dir = os.path.join(current_dir, "NN_direct_close_loop_training_logs")

        PDP_GRADIENT = self.train_cfg_dict['training']['PDP_GRADIENT']
        if PDP_GRADIENT:
            method_name = 'PDP'
        else:
            method_name = 'FD'

        current_day = datetime.datetime.now().strftime("%Y-%m-%d")
        current_time = datetime.datetime.now().strftime("%H%M%S")
        training_notes = "Trial 4, 5 steps, no differentiable physic"
        file_dir = os.path.join(log_dir,current_day, f"train-{current_time}-{method_name}-{training_notes}")
        self.writer = SummaryWriter(log_dir=file_dir)
        logger_config=LoggerConfig("NN_direct_close_loop_training_logs")


        ##== save the yaml file in the log folder ==##
        logger_config.log_yaml_data(self.config_dict)
        logger_config.log_yaml_data(self.train_cfg_dict)


    
    def init_train(self,model_folder):
        self.training_horizon = self.train_cfg_dict['training']['horizon']
        learning_rate = self.train_cfg_dict['training']['learning_rate']
        self.dyn_decay = self.train_cfg_dict['training']['dyn_decay']

        ##== load the pre-trained model ==##
        FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        self.model = torch.load(FILE).to(self.device)

         # Loss and optimizer
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=learning_rate)  
        # learning rate scheduler
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=5, gamma=0.9)
        
    def reset(self):
        
        #== random generate the env and set to the mpc solver
        self.learning_agile_sim.generate_mission()
        self.planner = self.learning_agile_sim.planner
        self.state = self.planner.ini_state

        #== reset the gate
        self.learning_agile_sim.prepare_gate()
        self.i = 0
        self.gate_t_i = self.learning_agile_sim.gate_t_i
        self.gate_points_list = self.learning_agile_sim.gate_points_list
        

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
        self.gate_t_i = Gate(self.gate_points_list[self.i])
        
        
            
        ## == NN forward === ##
        nn_input=np.zeros(input_size)
        nn_input[0:10]=self.state
        nn_input[10:13]=self.learning_agile_sim.final_point


        # position of the gate
        nn_input[13:16] = self.gate_t_i.centroid
        # width of the gate
        nn_input[16] = magni(self.gate_t_i.gate_point[0,:]-self.gate_t_i.gate_point[3,:]) # gate width
        # pitch angle of the gate
        gate_pitch = atan((self.gate_t_i.gate_point[0,2]-self.gate_t_i.gate_point[1,2])/(self.gate_t_i.gate_point[0,0]-self.gate_t_i.gate_point[1,0])) # compute the actual gate pitch ange in real-time
        
        self.planner.init_obstacle(self.gate_t_i.gate_point[:,:].reshape(12),gate_pitch)
        
        ##==calculate the gate RM
        rot=R.from_euler('zyx',[0,gate_pitch,0])
        nn_input[17:26]=rot.as_matrix().flatten()

        if self.i == 0:
            for i in range(5):
                self.history_state.append(nn_input)
        else:
            self.history_state.append(nn_input)
        
        full_input=np.array(self.history_state)
        # NN output the traversal time and pose
        self.nn_output = self.model(torch.tensor(full_input, dtype=torch.float).to(self.device))[0]
        self.np_nn_out = self.nn_output.to('cpu').data.numpy()
        
            
        ## == MPC forward === ##
        cmd_solution,NO_SOLUTION_FLAG = self.planner.mpc_update(self.state,
                                            self.np_nn_out[0:3],
                                            self.np_nn_out[3:12],
                                            self.np_nn_out[-1]) # control input 4-by-1 thrusts to pybullet
        
       
        ## record the gradient and step reward
        log_train_IO(self.writer,nn_input,self.np_nn_out.reshape(output_size),self.i+self.training_horizon*global_step)
        # log_gradient(self.writer,grads_list[0,:].reshape(output_size+1),global_step)

        self.pred_st_traj = cmd_solution['state_traj_opt']
        self.control_traj = cmd_solution['control_traj_opt']
        

        ## === state update === ##
        self.state = cmd_solution['state_traj_opt'][1,:]
        self.u = cmd_solution['control_traj_opt'][0,:].tolist()

        ## === actual trajectory === ##
        self.state_traj.append(self.state)
        
    
    def init_gradient(self):
        self.R_i = 0
        self.p_R_i_p_X_traj_i = []
        self.p_X_traj_i_p_x_i = []
        self.p_X_traj_i_p_z_i = []
        self.p_z_i_p_w = []

    
    def backward(self):
        
        ## acquire p_R_i/p_X_traj_i
        self.R_i=self.planner.MPC_and_R(self.np_nn_out[0:3],self.np_nn_out[3:12],self.np_nn_out[-1])

        # H * 1* 10
        self.p_R_i_p_X_traj_i.append(torch.tensor(self.planner.d_R_d_st_traj[:,:,:], dtype=torch.float).to(self.device))

        
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
        # size H * 10 * 7
        self.p_X_traj_i_p_z_i.append(torch.tensor(self.planner.d_st_traj_d_z[:,:,:], dtype=torch.float).to(self.device))

        ## acquire p_z_i/p_w (w is the weight of the NN)
        # size 7 x 1
        self.p_z_i_p_w.append(self.nn_output.unsqueeze(1))
       
    def train_one_step_no_differentiable_physic(self,global_step):

        """ 
        the drone will execute the whole trajectory and then update the weights of the NN
        """
        R=0
        d_R_d_w=torch.zeros(1).to(self.device)

        ## for each node in the trajectory
        for self.i in range(self.training_horizon):
            self.step(global_step)
            self.backward()
            
            R += self.R_i
            d_R_i_d_w = torch.zeros(1).to(self.device)
            
        
            d_R_i_d_z = torch.einsum('bij,bjk->ik',self.p_R_i_p_X_traj_i[self.i],self.p_X_traj_i_p_z_i[self.i])
            d_R_i_d_w = torch.matmul(d_R_i_d_z,self.p_z_i_p_w[self.i])
           
            d_R_d_w += d_R_i_d_w[0]    
            self.i += 1
        
        return R,d_R_d_w
        
        
    def train_one_step(self,global_step):

        """ 
        the drone will execute the whole trajectory and then update the weights of the NN
        """
        R=0
        d_R_d_w=torch.zeros(1).to(self.device)

        ## for each node in the trajectory
        for self.i in range(self.training_horizon):
            self.step(global_step)
            self.backward()
            
            R += self.R_i
            d_R_i_d_w = torch.zeros(1).to(self.device)

            ### == assemble the gradient of the loss function == ###
            ## each node is related to all history nodes NN decisions
            for k in range(0,self.i+1):
                p_z_k_p_w = self.p_z_i_p_w[k] # size 7

                if k == self.i:
                    p_X_traj_p_w = torch.matmul(self.p_X_traj_i_p_z_i[k],p_z_k_p_w) # H x 10 x 7 * 7 x 1 = H x 10 x 1
                
                else:
                    
                    ## propagate from the k-th history node to the i-th current node
                    # p_x_k+1/p_z_k
                    p_x_k_next_p_w = torch.matmul(self.p_X_traj_i_p_z_i[k][1] , p_z_k_p_w) # 10 x 7 * 7 x 1 = 10 x 1
                    
                    j=k
                    while j < self.i-1: 
                        j += 1
                        # p_x_j+2/p_x_j+1
                        p_x_k_next_p_w  = self.dyn_decay * torch.matmul(self.p_X_traj_i_p_x_i[j][1] , p_x_k_next_p_w ) #10 x 10 * 10 x 1 = 10 x 1
                        
                    
                    #p_X_traj_i/p_x_i
                    p_X_traj_p_w = torch.matmul(self.p_X_traj_i_p_x_i[self.i] , p_x_k_next_p_w) #H x 10 x 10 * 10 x 1 = H x 10 x 1      
                
                # p_R_i/p_X_traj_i
                p_R_i_p_w = torch.matmul(self.p_R_i_p_X_traj_i[self.i], p_X_traj_p_w) #H x 1 x 10* H x 10 x 1 = 1x1
                
                d_R_i_d_w += p_R_i_p_w.sum()
            
            d_R_d_w += d_R_i_d_w
            self.i += 1

        d_R_d_w = d_R_d_w/(self.training_horizon*20000)
        
        self.optimizer.zero_grad()
        d_R_d_w.backward()
        self.optimizer.step()

        ##== log and reward ==##
        self.writer.add_scalar('step_reward', R, global_step)
        self.writer.add_scalar('loss', d_R_d_w, global_step)


    

    def train_loop(self,save_folder,batch_size=20):
        num_epochs = self.train_cfg_dict['training']['num_epochs']
        steps_per_epoch = self.train_cfg_dict['training']['steps_per_epoch']
        global_step = 0
        for epoch in range(num_epochs):

            with tqdm(total=steps_per_epoch, desc=f'Epoch {epoch+1}/{num_epochs}', unit='epoch') as pbar:

                for cur_step in range(steps_per_epoch):
                    for k in range(batch_size):
                        self.reset()
                        R,d_R_d_w=self.train_one_step_no_differentiable_physic(global_step)

                    d_R_d_w = d_R_d_w/(batch_size*self.training_horizon*5000)
                    d_R_d_w =torch.clamp(d_R_d_w,-0.01,0.01)
                    R = R/(batch_size*self.training_horizon)
                    self.optimizer.zero_grad()
                    d_R_d_w.backward()
                    self.optimizer.step()

                    ##== log and reward ==##
                    self.writer.add_scalar('step_reward', R, global_step)
                    self.writer.add_scalar('loss', d_R_d_w, global_step)

                    pbar.set_postfix({'step': cur_step})
                    pbar.update(1)

                    global_step += 1
            
            self.scheduler.step()
            # if epoch % 2 == 0:  
            torch.save(self.model,  save_folder+"/NN_close_"+str(epoch)+".pth")
def main():
    # yaml file dir#
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    mission_yaml = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    train_yaml = os.path.join(conf_folder, 'training_params.yaml')
    training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
    model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
    current_day = datetime.datetime.now().strftime("%Y-%m-%d")
    current_time = datetime.datetime.now().strftime("%H%M%S")
    today_folder=os.path.join(model_folder,f"{current_day}-close_loop")
    
    # if today_folder not in os.listdir(model_folder):
    #     os.mkdir(today_folder)
    
    saved_folder=os.path.join(today_folder,f"{current_time}")
    if saved_folder not in os.listdir(today_folder):
        os.mkdir(saved_folder)
    direct_close_loop=DirectCloseLoop(mission_yaml, train_yaml)
    direct_close_loop.init_train(model_folder)
    direct_close_loop.train_loop(saved_folder)
    
if __name__ == '__main__':
    main()