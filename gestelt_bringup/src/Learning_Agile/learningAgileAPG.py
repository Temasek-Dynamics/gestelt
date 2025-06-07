import numpy as np
import torch
import os
from tqdm import tqdm

os.environ["RAY_DEDUP_LOGS"]="0"
import ray

import wandb

from learningAgileBase import LearningAgileBase,vis_gradient_norm
from config import mission_cfg,train_cfg,current_dir,setup_training_directories, get_time_name
from logger_misc import log_drone_state_wandb,log_train_IO_wandb,log_gradient_wandb
from mc_evaluation import mc_evaluation
from geometry.solid_geometry import magni
from misc.misc import load_demo_traj
from quad_nn import network
folder_dict=setup_training_directories()
trained_model_folder=folder_dict['trained_model_folder']
log_folder=folder_dict['log_folder']

## this options is for close loop training
training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_results'))
model_folder=os.path.abspath(os.path.join(training_data_folder, 'pretrain_model'))

checkpoint_trained_model_folder=os.path.abspath(os.path.join(current_dir,'training_results/'))
options = {}
options['MPC_BACKWARD']=True
options['USE_PREV_SOLVER']= True
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
options['MULTI_COLLISION_POINT_CHECK']=True

run=wandb.init(
    project='Learning Agile',
    name= get_time_name(),
    config={
        "mission_cfg":mission_cfg,
        "train_cfg":train_cfg,
        "options":options
    },
    dir=log_folder
)

if mission_cfg['LEARNING_FROM_DEMO']:
    demo_traj_file = os.path.join(current_dir, 'MinimumSnapDemo/demo_traj.npy')
    demo_state_traj=load_demo_traj(demo_traj_file)
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
        # RAY
        self.episodes=[LearningAgileBase.remote(mission_cfg=self.mission_cfg,
                                         train_cfg=self.train_cfg,
                                         options=options) for _ in range(self.batch_size)]

        
    def init_train(self,model_folder,checkpoint_trained_model_folder):
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        # self.device = torch.device('cpu')

        if options['TRAIN_FROM_CHECKPOINT'] or options['STATE_2_MOVING_GATE']:
            FILE = os.path.join(checkpoint_trained_model_folder, "new_format/2025-05-16/21-02-51/trained_model/NN_close_100.pth")

            self.learning_rate = self.train_cfg['training']['learning_rate']#*0.9**(300/self.train_cfg['training']['lr_decay_num_epochs'])
        else:
            if mission_cfg['POSITION_ENCODING']:
                FILE = os.path.join(model_folder, "NN_close_pretrain_position_encode.pth")
            else:
                FILE = os.path.join(model_folder, "NN_close_pretrain.pth")
        # self.model = torch.load(FILE).to(self.device)
        self.model = network(
            train_cfg['model']['input_size'], 
            train_cfg['model']['hidden_size'], 
            train_cfg['model']['hidden_size'],
            weights_vector_length=train_cfg['model']['weights_vector_length'],
            activation=train_cfg['model']['activation']
        ).to(self.device)

        self.model.load_state_dict(torch.load(FILE,map_location=self.device))
        
        self.learning_rate = self.train_cfg['training']['learning_rate']
        self.dyn_decay = self.train_cfg['training']['dyn_decay']
        lr_gamma = self.train_cfg['training']['lr_gamma']
        lr_decay_num_epochs = self.train_cfg['training']['lr_decay_num_epochs']
        
        if mission_cfg['LBFGS']:
            self.optimizer = torch.optim.LBFGS(self.model.parameters(), lr=self.learning_rate)
        else:
            self.optimizer = torch.optim.Adam(self.model.parameters(), lr=self.learning_rate)  
            # SGD
            # self.optimizer = torch.optim.SGD(self.model.parameters(), lr=self.learning_rate, momentum=0.9)

        # learning rate scheduler
        self.scheduler = torch.optim.lr_scheduler.StepLR(self.optimizer, step_size=lr_decay_num_epochs, gamma=lr_gamma)
        # self.scheduler=torch.optim.lr_scheduler.CosineAnnealingLR(self.optimizer,T_max=50,eta_min=self.train_cfg['training']['eta_min'])


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

        variables:
            obs_batch_list: list element is the obs of each episode at each step
            obs_batch: array of the obs of each episode at each step. shape (batch_size, input_size)

            outputs_batch: array of the Neural Network output (batch_size, output_size)
            outputs_list: list element is the Neural Network batch output at each step
                          shape (close_loop_horizon, batch_size, output_size)
            euler_nn_list: list element is the Euler NN output for each episode
                           shape (close_loop_horizon, batch_size, 3)

            penalty_list: list element is the penalty for each episode
            p_L_p_z_list: list element is the gradient for each episode
            
            epoch_solution_flags: list of solution flags for each episode. 
                                  One step of the episode failed, skip this episode 
            
            
        Args:
            episodes (list): list of episodes
        """
        
        penalty_list = []
        euler_nn_list = []
        outputs_list = []
        p_L_p_z_list = []
        outputs_batch = np.zeros((self.batch_size, train_cfg['model']['output_size']))
        # prev_outputs_batch = np.zeros((self.batch_size, train_cfg['model']['output_size']))
       
        ##==0. reset all the episodes
        [episode.reset.remote(cur_epoch) for episode in self.episodes]
        epoch_solution_flags =[False for _ in range(self.batch_size)]
        for i in range(1,train_cfg['training']['close_loop_horizon']+1):
            obs_batch_list = []
            ##== 1. get observations for every episode
            obs_batch_list = ray.get([episode.gate_step_and_obs.remote(i) for episode in self.episodes])

            ##== 2. model forward in a batch    
            obs_batch=np.array(obs_batch_list)
            outputs_batch = self.model(torch.tensor(obs_batch,dtype=torch.float32).to(self.device),deterministic=True).to('cpu')
            
            ##== 3. step for every episode
            for k, episode in enumerate(self.episodes):
                if not epoch_solution_flags[k]: 
                    # [episode.step.remote(outputs_batch[k]) for k, episode in enumerate(self.episodes)]
                    episode.save_last_u.remote()
                    episode.step.remote(outputs_batch[k])
            
            
            euler_nn_list.append(ray.get([episode.get_euler_nn.remote() for episode in self.episodes]))
            for k, episode in enumerate(self.episodes):
                if ray.get(episode.get_solution_flag.remote()):
                    epoch_solution_flags[k] = True
                    episode.reset.remote(cur_epoch)
                    model_file=os.path.join(trained_model_folder, f"NN_close_leads_solver_failed_{cur_epoch}_batch_num_{k}.pth")
                    torch.save(self.model.state_dict(), model_file)
            
            ## since the SQP_RTI first solution is not feasible
            if i > 1:
                outputs_list.append(outputs_batch)
            
                ##== 4. Multi-process calculate each episode's penalty and gradient p_L_i_p_X_traj_i
                for k, episode in enumerate(self.episodes):
                    if not epoch_solution_flags[k]:  # NO_SOLUTION_FLAG == False
                        episode.get_immed_penalty.remote(i, self.success_rate)
                         ##== 5. backward the gradient to get the p_L_p_z
                        episode.backward_per_step.remote()

            ##== record NN obs and output per episode step
            if not epoch_solution_flags[0]:  # NO_SOLUTION_FLAG == False
                log_drone_state_wandb(obs_batch[0,:],ray.get(self.episodes[0].get_control.remote()),self.global_step)
                euler_nn,_= log_train_IO_wandb(obs_batch[0,:],outputs_batch[0,:].data.numpy().reshape(train_cfg['model']['output_size']),self.global_step)
                wandb.log({"penalty_single_step":ray.get(self.episodes[0].get_penalty.remote())},step=self.global_step)

            
            self.global_step  += 1
        
        ##== collect the penalty and gradient from each episode
        for k in range(self.batch_size):
            if not epoch_solution_flags[k]:  # NO_SOLUTION_FLAG == False
                penalty_list.append(ray.get(self.episodes[k].get_penalty.remote()))
                p_L_p_z_list.append(ray.get(self.episodes[k].get_p_L_p_z.remote())) 
            else:
                penalty_list.append(np.array([0.0]))
                p_L_p_z_list.append(np.zeros((train_cfg['training']['close_loop_horizon']-1,1,train_cfg['model']['output_size'])))
        
        if not GRAD_VIS: 
            ## if BPTT all, /10000 0
            
            # the larger the neural network angle output, the smaller the gradient
            euler_scaler = 0.05*np.array([(max(np.linalg.norm(np.array(euler_nn_list)[:,k,:],axis=1,keepdims=True))) for k in range(self.batch_size)])
            self.p_L_p_z_batch = np.array(p_L_p_z_list)
            self.p_L_p_z_batch[:,:,:,3:12] /= euler_scaler.reshape(self.batch_size,1,1,1)
            # self.p_L_p_z_batch[:,:,:,:3] /= 5
                
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
            log_gradient_wandb(self.p_L_p_z_batch[0,0,:],self.penalty_batch[0],self.global_step)

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
                    torch.save(self.model.state_dict(), model_file)

                if epoch % 100 == 0:
                    self.success_rate=mc_evaluation(test_num=48,model_file=model_file,global_step=self.global_step)
        wandb.finish()
    def batch_gradient_visual(self):
        p_L_p_z_batch=self.train_one_epoch(0,GRAD_VIS=True)
        vis_gradient_norm(p_L_p_z_batch)
if __name__ == "__main__":
    ray.init()
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