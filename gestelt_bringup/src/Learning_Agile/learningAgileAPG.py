import numpy as np
import torch
import os
from multiprocessing import Process, Array, Queue
import multiprocessing

from scipy.spatial.transform import Rotation as R


from solid_geometry import magni
from learningAgileBase import LearningAgileBase

from config import mission_cfg, train_cfg,current_dir

## this options is for close loop training
options = {}
options['MPC_BACKWARD']=True
options['USE_PREV_SOLVER']=False
options['PDP_GRADIENT']= True
options['SQP_RTI_OPTION']=False
options['JAX_SVD']=False
options['STATIC_GATE_TEST']=False
options['ORIGIN_REWARD']=False
options['CLOSE_LOOP_TRAINING']=True
options['TRAINING']=True
options['DEBUG']=False
options['BACKWARD']=True
class LearningAgileAPG:
    """
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
        for i in range(self.batch_size):
            self.episodes.append(LearningAgileBase(mission_cfg=self.mission_cfg,
                                            train_cfg=self.train_cfg,
                                            options=options))
    def init_train(self,model_folder):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        self.model = torch.load(FILE).to(self.device)

        
        self.learning_rate = self.train_cfg['training']['learning_rate']
        self.dyn_decay = self.train_cfg['training']['dyn_decay']

        # Loss and optimizer
        self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)  
        # learning rate scheduler
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=5, gamma=0.9)\
    
    def get_observations(self,i, episode:LearningAgileBase,obs_queue:Queue): 
        """
        get the observations from the single episode

        Args:
            episode (LearningAgileBase): _description_
        
        Returns:
            observations
        """
        obs=episode.get_obs(i)
        obs_queue.put(obs)
        return obs_queue
    
    def run_step_episodes(self, i:int, episode:LearningAgileBase , nn_output:torch.tensor, R_Grad_queue:Queue):
        """
        run the episode and put the reward and gradient into the queue

        Args:
            index (int): _description_
            episode (LearningAgileBase): _description_
            R_Grad_queue (Queue): _description_
        
        Returns:
            each episode's reward and gradient
        """
        episode.run_single_step(nn_output)


        ## at the end of the close loop,
        ## return the reward and gradient
        if i==train_cfg['training']['close_loop_horizon']-1:
            R_Grad_queue.put([episode.reward, episode.p_R_p_z])
        

    def update_network(self):
        pass  

    def train(self, episodes):
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

       ## assign each episode to a process
        processes = []
        R_Grad_queue=Queue()
        obs_queue = Queue()

        
        reward_list = []
        p_R_p_z_list = []
       
        ##==0. reset all the episodes
        for episode in episodes:
            episode.reset()

        for i in range(train_cfg['training']['close_loop_horizon']):
            obs_batch_list = []
            ##== 1. get observations for every episode
            for k in range(self.batch_size):
                p = Process(target=self.get_observations, args=(i,
                                                                episodes[k],
                                                                obs_queue))
                p.start()
                processes.append(p)
            for p in processes:
                p.join()

            for k in range(self.batch_size):
                obs_batch_list.append(obs_queue.get())
            
            
            
            ##== 2. model forward in a batch    
            obs_batch=np.array(obs_batch_list)
            print('obs_batch shape',obs_batch.shape)
            outputs_batch = self.model(torch.tensor(obs_batch,dtype=torch.float32).to(self.device)).to('cpu')
            print('nn_output shape',outputs_batch.shape)
            # print('nn_output',outputs_batch)
            
            #== 3. step for every episode
            for k in range(self.batch_size):
            
                p = Process(target=self.run_step_episodes, args=(i,
                                                            episodes[k],
                                                            outputs_batch[k],
                                                            R_Grad_queue))
                p.start()
                processes.append(p)


            for p in processes:
                p.join()
            
            
        for k in range(self.batch_size):
            single_episodes_result=R_Grad_queue.get()
            reward_list.append(single_episodes_result[0])
            p_R_p_z_list.append(single_episodes_result[1])

        self.reward_batch = sum(reward_list)
        self.p_R_p_z_batch = np.array(p_R_p_z_list)

        
        ##== 4. model backward in a batch
        loss=self.model.myloss(outputs_batch.to(self.device), self.p_R_p_z_batch, self.device)

        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        self.scheduler.step()

if __name__ == "__main__":
    training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
    model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
    
    # multiprocessing.set_start_method('spawn')
    apg = LearningAgileAPG(mission_cfg,train_cfg,options)
    apg.init_train(model_folder)
    apg.train(apg.episodes)