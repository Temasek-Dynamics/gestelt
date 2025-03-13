import numpy as np
import torch
import os
from multiprocessing import Process, Queue
from tqdm import tqdm

from torch.utils.tensorboard import SummaryWriter

from learningAgileBase import LearningAgileBase,vis_gradient_norm
from config import mission_cfg,train_cfg,current_dir,setup_training_directories
from logger_misc import log_drone_state,log_train_IO,log_gradient
from mc_evaluation import mc_evaluation
from geometry.solid_geometry import magni

folder_dict=setup_training_directories()
trained_model_folder=folder_dict['trained_model_folder']
log_folder=folder_dict['log_folder']

## this options is for close loop training
training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
writer = SummaryWriter(log_dir=log_folder)
checkpoint_trained_model_folder=os.path.abspath(os.path.join(current_dir,'training_results/'))
options = {}
options['MPC_BACKWARD']=True
options['USE_PREV_SOLVER']=False
options['PDP_GRADIENT']= True
options['SQP_RTI_OPTION']=True
options['JAX_SVD']=False
options['MANUAL_SET_POSE_TEST']=False
options['ORIGIN_penalty']=False
options['CLOSE_LOOP_TRAINING']=True
options['TRAINING']=True
options['DEBUG']=False
options['BACKWARD']=True
options['MULTI_PROCESSES']=True
options['TRAIN_FROM_CHECKPOINT']=False
options['STATE_2_MOVING_GATE']=False

class LearningAgileAPG:
    """
    APG: Analytical Policy Gradient
    this class is responsible for running episodes batches in multi-process manner,
    collect gradients in a batch and update the network
    """
    def __init__(self,
                 mission_cfg:dict,
                 train_cfg:dict,
                 options:dict):
        self.mission_cfg = mission_cfg
        self.train_cfg = train_cfg
        self.batch_size = train_cfg['training']['batch_size']
        self.penalty_batch=torch.zeros(1)
        self.gradient_batch=torch.zeros(1)
        self.success_rate=0
        self.global_step = 0
        self.episodes = []
        self.reg=0
        for _ in range(self.batch_size):
            self.episodes.append(LearningAgileBase(mission_cfg=self.mission_cfg,
                                                 train_cfg=self.train_cfg,
                                                 options=options))

        
    def init_train(self,model_folder,checkpoint_trained_model_folder):
        # self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.device = torch.device('cpu')

        if options['TRAIN_FROM_CHECKPOINT'] or options['STATE_2_MOVING_GATE']:
            FILE = os.path.join(checkpoint_trained_model_folder, "new_format/2025-01-31/11-40-49/trained_model/NN_close_1900.pth")

            self.learning_rate = self.train_cfg['training']['learning_rate']#*0.9**(300/self.train_cfg['training']['lr_decay_num_epochs'])
        else:
            FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        self.model = torch.load(FILE).to(self.device)

        
        self.learning_rate = self.train_cfg['training']['learning_rate']
        self.dyn_decay = self.train_cfg['training']['dyn_decay']
        lr_gamma = self.train_cfg['training']['lr_gamma']
        lr_decay_num_epochs = self.train_cfg['training']['lr_decay_num_epochs']

        # Loss and optimizer
        # self.optimizer = torch.optim.Adam([{'params': self.model.position_head.parameters(), 'weight_decay': 0.00},  
        #                                    {'params': self.model.orientation_head.parameters(), 'weight_decay': 0.00}, 
        #                                    {'params': self.model.traverse_time_head.parameters(), 'weight_decay': 0.00},
        #                                    {'params': self.model.weights_head.parameters(), 'weight_decay': 0.00} ],\
        #                                   lr=self.learning_rate)  #,weight_decay=0.01
        
        if mission_cfg['LBFGS']:
            self.optimizer = torch.optim.LBFGS(self.model.parameters(), lr=self.learning_rate)
        else:
            self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)  

        # learning rate scheduler
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=lr_decay_num_epochs, gamma=lr_gamma)
        # self.scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(self.optimizer,T_max=50,eta_min=self.train_cfg['training']['eta_min'])

    def get_penalty_episodes(self, i:int,
                            episode:LearningAgileBase,
                            R_Grad_queue:Queue):
        L_i = np.array(episode.planner.get_penalty(episode.pred_st_traj,real_state_i=i,success_rate=self.success_rate)[0])
        p_L_i_p_X_traj_i = (episode.planner.get_penalty(episode.pred_st_traj,real_state_i=i,success_rate=self.success_rate)[1])

        R_Grad_queue.put([L_i, p_L_i_p_X_traj_i])
    def smooth_loss(self,outputs_batch,prev_outputs_batch):
        return 1 * torch.norm(outputs_batch - prev_outputs_batch, p=2)
    
    def update_network(self):
        self.optimizer.zero_grad()
        self.loss.backward()
        self.optimizer.step()
        self.scheduler.step()
    
    def update_network_lbfgs(self):
        self.optimizer.step(self.closure)

    def closure(self):

        self.optimizer.zero_grad()
        self.loss = self.model.loss_close_loop(self.outputs_stack.to(self.device), self.p_L_p_z_batch, self.device)
        torch.autograd.set_detect_anomaly(True)
        self.loss.backward(retain_graph=True)

        return  self.loss

    def train_one_epoch(self,cur_epoch:int,GRAD_VIS:bool=False):
        """
        run the episodes in parallel, collect the penalty and gradient and update the network
        0. reset all the episodes
        for i in range(close_loop_horizon):
            1. get observations for every episode[i]
            2. model forward in a batch
            3. step for every episode
            4. model backward in a batch

        Args:
            episodes (list): list of episodes
        """

    
        processes = []
        R_Grad_queue=Queue()
   

        
        penalty_list = []
        outputs_list = []
        p_L_p_z_list = []
        outputs_batch = np.zeros((self.batch_size, self.episodes[0].output_size))
        prev_outputs_batch = np.zeros((self.batch_size, self.episodes[0].output_size))
       
        ##==0. reset all the episodes
        for episode in self.episodes:
            episode.reset(cur_epoch)

        for i in range(1,train_cfg['training']['close_loop_horizon']+1):
            obs_batch_list = []
            ##== 1. get observations for every episode
            for k in range(self.batch_size):
                obs_batch_list.append(self.episodes[k].gate_step_and_obs(i))

            
            ##== 2. model forward in a batch    
            obs_batch=np.array(obs_batch_list)
            outputs_batch = self.model(torch.tensor(obs_batch,dtype=torch.float32).to(self.device),deterministic=True).to('cpu')
            
            ##== 3. step for every episode
            for k in range(self.batch_size):
                self.episodes[k].step(outputs_batch[k])
            
            ## since the SQP_RTI first solution is not feasible
            if i > 1:
                outputs_list.append(outputs_batch)
                ##== 4. Multi-process calculate each episode's penalty and gradient p_L_i_p_X_traj_i

                if self.batch_size!=1:
                    ### multi-process
                    for k in range(self.batch_size):
                        p = Process(target=self.get_penalty_episodes, args=(i,
                                                            self.episodes[k],
                                                            R_Grad_queue))
                        processes.append(p)
                        p.start()

                    for p in processes:
                        p.join()
                else:
                ##== single-process
                    for k in range(self.batch_size):
                        self.get_penalty_episodes(i,self.episodes[k],R_Grad_queue)


                ##== collect the penalty and gradient from each episode
                for k in range(self.batch_size):

                    if self.mission_cfg['penalty']['control_reg_w']!=0:
                        self.episodes[k].get_reg_control()
                        self.reg+=self.episodes[k].reg_control
                    if self.mission_cfg['penalty']['det_reg_w']!=0:
                        self.episodes[k].get_reg_det()
                        self.reg+=self.episodes[k].reg_det
                    single_episode_r_grad = R_Grad_queue.get()
                    self.episodes[k].L_i.append(single_episode_r_grad[0]+self.reg)
                    self.episodes[k].p_L_i_p_X_traj_i.append(single_episode_r_grad[1])

                ##== 5. backward the gradient to get the p_L_p_z
                for k in range(self.batch_size):
                    self.episodes[k].backward_per_step(train_cfg['training']['dyn_decay'])   

                prev_outputs_batch = outputs_batch

            ##== record NN obs and output per episode step
            log_drone_state(writer,obs_batch[0,-1,:],self.episodes[0].control,self.global_step)
            euler_nn,gate_pitch = log_train_IO(writer,obs_batch[0,-1,:],outputs_batch[0,:].data.numpy().reshape(self.episodes[0].output_size),self.global_step)
            writer.add_scalar('penalty_single_step', self.episodes[0].penalty, self.global_step)

            
            self.global_step  += 1
        
        ##== collect the penalty and gradient from each episode
        for k in range(self.batch_size):
            penalty_list.append(self.episodes[k].penalty)
            p_L_p_z_list.append(self.episodes[k].p_L_p_z) 
        
        if not GRAD_VIS: 
            ## assemble *(0.05*magni(euler_nn))
            ## if BPTT all, /10000 0
            self.p_L_p_z_batch = np.array(p_L_p_z_list)/(10000*(0.05*magni(euler_nn)))
     
            # (close_loop_horizon, batch_size, 13)->(batch_size, close_loop_horizon, 13)
            self.outputs_stack = torch.stack(outputs_list).permute(1,0,2) 
        
            # ->(batch_size, close_loop_horizon, 13, 1)
            self.outputs_stack = self.outputs_stack.unsqueeze(-1) 
            
            self.penalty_batch = sum(penalty_list)/self.batch_size
            

           
            ##== 4. model backward in a batch
            # new=False
            # if new:
            #     self.optimizer.zero_grad()
            #     outputs_stack = outputs_stack.to(self.device)
            #     outputs_stack.backward(gradient=torch.tensor(p_L_p_z_batch,dtype=torch.float32).transpose(2,3).to(self.device))
            #     self.optimizer.step()

            if mission_cfg['LBFGS']:
                self.update_network_lbfgs()
        
            else:
                self.loss=self.model.loss_close_loop(self.outputs_stack.to(self.device), self.p_L_p_z_batch, self.device)
                self.update_network()
            self.p_L_p_z_batch = self.p_L_p_z_batch.squeeze(2)
            ##== record the gradient and the penalty
            log_gradient(writer,self.p_L_p_z_batch[0,0,:],self.penalty_batch[0],self.global_step)

        else:
            return np.array(p_L_p_z_list).squeeze(2)

    def train(self):
        """
        the main function of the training, with multiple epochs,multiple episodes
        """
        self.model.train()
        self.global_step = 0
        num_epochs = self.train_cfg['training']['num_epochs']
        with tqdm(total=num_epochs) as pbar:
            for epoch in range(num_epochs):
                self.train_one_epoch(epoch)
                pbar.update(1)
                pbar.set_description(f"epoch:{epoch}, penalty:{self.penalty_batch[0]}")
                if epoch % 10 == 0:
                    model_file=os.path.join(trained_model_folder, f"NN_close_{epoch}.pth")
                    torch.save(self.model, model_file)

                if (epoch+1) % 100 == 0:
                    self.success_rate=mc_evaluation(writer=writer,options=options,model_file=model_file,global_step=self.global_step)
    
    def batch_gradient_visual(self):
        p_L_p_z_batch=self.train_one_epoch(0,GRAD_VIS=True)
        vis_gradient_norm(p_L_p_z_batch)
if __name__ == "__main__":

    apg = LearningAgileAPG(mission_cfg,train_cfg,options)
    apg.init_train(model_folder,checkpoint_trained_model_folder)
    apg.train()
    # apg.batch_gradient_visual()


    # def get_observations(self, i:int,  
    #                      episode:LearningAgileBase,
    #                      obs_queue:Queue):#, drones_state_queue:Queue 
    #     """
    #     get the observations from the single episode

    #     Args:
    #         episode (LearningAgileBase): _description_
        
    #     Returns:
    #         observations
    #     """
        
    #     obs=episode.gate_step_and_obs(i)
    #     obs_queue.put(obs)
    #     # drones_state_queue.put(episode.drone_state)
    #     # return obs_queue
    
    # def run_step_episodes(self, i:int, 
    #                       episode:LearningAgileBase, 
    #                       nn_output:torch.tensor, 
    #                       R_Grad_queue:Queue):
    #     """
    #     run the episode and put the penalty and gradient into the queue

    #     Args:
    #         index (int): _description_
    #         episode (LearningAgileBase): _description_
    #         R_Grad_queue (Queue): _description_
        
    #     Returns:
    #         each episode's penalty and gradient
    #     """
    #     episode.step(nn_output)


    #     ## at the end of the close loop,
    #     ## return the penalty and gradient
    #     if i==train_cfg['training']['close_loop_horizon']-1:
    #         R_Grad_queue.put([episode.penalty, episode.p_L_p_z])