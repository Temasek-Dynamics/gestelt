import numpy as np
import torch
import os
from multiprocessing import Process, Queue
from tqdm import tqdm


from torch.utils.tensorboard import SummaryWriter

from solid_geometry import magni
from learningAgileBase import LearningAgileBase
from config import mission_cfg,train_cfg,current_dir,setup_training_directories
from logger_misc import log_drone_state,log_train_IO,log_gradient
import logging

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
options['ORIGIN_REWARD']=False
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
        self.reward_batch=torch.zeros(1)
        self.gradient_batch=torch.zeros(1)

        self.episodes = []
        for _ in range(self.batch_size):
            self.episodes.append(LearningAgileBase(mission_cfg=self.mission_cfg,
                                                 train_cfg=self.train_cfg,
                                                 options=options))

        
    def init_train(self,model_folder,checkpoint_trained_model_folder):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        if options['TRAIN_FROM_CHECKPOINT'] or options['STATE_2_MOVING_GATE']:
            FILE = os.path.join(checkpoint_trained_model_folder, "new_format/2024-12-21/15-49-52/trained_model/NN_close_300.pth")

            self.learning_rate = self.train_cfg['training']['learning_rate']*0.9**(800/100)
        else:
            FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        self.model = torch.load(FILE).to(self.device)

        
        self.learning_rate = self.train_cfg['training']['learning_rate']
        self.dyn_decay = self.train_cfg['training']['dyn_decay']

        # Loss and optimizer
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)  #,weight_decay=0.01
        # learning rate scheduler
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=100, gamma=0.9)
        # self.scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(self.optimizer,T_max=50,eta_min=self.train_cfg['training']['eta_min'])

    def get_reward_episodes(self, i:int,
                            episode:LearningAgileBase,
                            R_Grad_queue:Queue):
        R_i = np.array(episode.planner.get_reward(episode.pred_st_traj,real_state_i=i)[0])
        p_R_i_p_X_traj_i = (episode.planner.get_reward(episode.pred_st_traj,real_state_i=i)[1])

        R_Grad_queue.put([R_i, p_R_i_p_X_traj_i])

    def update_network(self):
        self.optimizer.zero_grad()
        self.loss.backward()
        self.optimizer.step()
        self.scheduler.step()

    def train_one_epoch(self,cur_epoch:int):
        """
        run the episodes in parallel, collect the reward and gradient and update the network
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
   

        
        reward_list = []
        outputs_list = []
        p_R_p_z_list = []
       
        ##==0. reset all the episodes
        for episode in self.episodes:
            episode.reset(cur_epoch)

        for i in range(1,train_cfg['training']['close_loop_horizon']+1):
            obs_batch_list = []
            ##== 1. get observations for every episode
            for k in range(self.batch_size):
                obs_batch_list.append(self.episodes[k].get_obs(i))

            
            ##== 2. model forward in a batch    
            obs_batch=np.array(obs_batch_list)
            outputs_batch = self.model(torch.tensor(obs_batch,dtype=torch.float32).to(self.device),deterministic=True).to('cpu')
            
            
            ##== 3. step for every episode
            for k in range(self.batch_size):
                self.episodes[k].step(outputs_batch[k])
            
            ## since the SQP_RTI first solution is not feasible
            if i > 1:
                outputs_list.append(outputs_batch)
                ##== 4. Multi-process calculate each episode's reward and gradient p_R_i_p_X_traj_i

                if self.batch_size!=1:
                    ### multi-process
                    for k in range(self.batch_size):
                        p = Process(target=self.get_reward_episodes, args=(i,
                                                            self.episodes[k],
                                                            R_Grad_queue))
                        processes.append(p)
                        p.start()

                    for p in processes:
                        p.join()
                else:
                ##== single-process
                    for k in range(self.batch_size):
                        self.get_reward_episodes(i,self.episodes[k],R_Grad_queue)


                ##== collect the reward and gradient from each episode
                for k in range(self.batch_size):
                    single_episode_r_grad = R_Grad_queue.get()
                    self.episodes[k].R_i.append(single_episode_r_grad[0])
                    self.episodes[k].p_R_i_p_X_traj_i.append(single_episode_r_grad[1])

                ##== 5. backward the gradient to get the p_R_p_z
                for k in range(self.batch_size):
                    self.episodes[k].backward_per_step(train_cfg['training']['dyn_decay'])   

            ##== record NN obs and output per episode step
            log_drone_state(writer,obs_batch[0,-1,:],self.global_step)
            euler_nn = log_train_IO(writer,obs_batch[0,-1,:],outputs_batch[0,:].data.numpy().reshape(self.episodes[0].output_size),self.global_step)
            writer.add_scalar('reward_single_step', self.episodes[0].reward, self.global_step)
            
            self.global_step  += 1
        
        ##== collect the reward and gradient from each episode
        for k in range(self.batch_size):
            reward_list.append(self.episodes[k].reward)
            p_R_p_z_list.append(self.episodes[k].p_R_p_z) 
        
        ## assemble
        p_R_p_z_list = np.array(p_R_p_z_list)/(10000*(0.1*magni(euler_nn))) #*((10*euler_nn[1]))(batch_size, close_loop_horizon, 1, 13)
        p_R_p_z_list = np.clip(p_R_p_z_list, -0.02, 0.02)
        # (close_loop_horizon, batch_size, 13)->(batch_size, close_loop_horizon, 13)
        outputs_stack = torch.stack(outputs_list).permute(1,0,2) 
       
        # ->(batch_size, close_loop_horizon, 13, 1)
        outputs_stack = outputs_stack.unsqueeze(-1) 
        
        self.reward_batch = sum(reward_list)/self.batch_size
        self.p_R_p_z_batch = np.array(p_R_p_z_list).squeeze(2)

        
        ##== 4. model backward in a batch
        self.loss=self.model.loss_close_loop(outputs_stack.to(self.device), p_R_p_z_list, self.device)
        self.update_network()

        ##== record the gradient and the reward
        log_gradient(writer,self.p_R_p_z_batch[0,0,:],self.reward_batch[0],self.global_step)
       

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
                pbar.set_description(f"epoch:{epoch}, reward:{self.reward_batch[0]}")
                if epoch % 2 == 0:
                    torch.save(self.model, os.path.join(trained_model_folder, f"NN_close_{epoch}.pth"))

if __name__ == "__main__":

    apg = LearningAgileAPG(mission_cfg,train_cfg,options)
    apg.init_train(model_folder,checkpoint_trained_model_folder)
    apg.train()



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
        
    #     obs=episode.get_obs(i)
    #     obs_queue.put(obs)
    #     # drones_state_queue.put(episode.drone_state)
    #     # return obs_queue
    
    # def run_step_episodes(self, i:int, 
    #                       episode:LearningAgileBase, 
    #                       nn_output:torch.tensor, 
    #                       R_Grad_queue:Queue):
    #     """
    #     run the episode and put the reward and gradient into the queue

    #     Args:
    #         index (int): _description_
    #         episode (LearningAgileBase): _description_
    #         R_Grad_queue (Queue): _description_
        
    #     Returns:
    #         each episode's reward and gradient
    #     """
    #     episode.step(nn_output)


    #     ## at the end of the close loop,
    #     ## return the reward and gradient
    #     if i==train_cfg['training']['close_loop_horizon']-1:
    #         R_Grad_queue.put([episode.reward, episode.p_R_p_z])