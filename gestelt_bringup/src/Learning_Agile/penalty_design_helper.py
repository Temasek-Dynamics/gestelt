'''
this file will plot the penalty value change w.r.t the euler angle change of the quadrotor
'''
import numpy as np
import time

import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R



from quad_policy import PlanFwdBwdWrapper
from learning_agile_sim import MovingGate
from quad_nn import nn_sample
from config import mission_cfg
class PenaltyDesignHelper():
    '''
    this class is used to help design the penalty function w.r.t the quadrotor's euler angle change and translation change
    '''
    def __init__(self,
                 trans_choice,
                 euler_choice,
                 ROT_VIS=False):
        self.num_of_pts=50
        self.ROT_VIS=ROT_VIS
        self.trans_range=np.linspace(-1,1,self.num_of_pts)
        self.axis_angle_range=np.linspace(-np.pi/2,np.pi/2,self.num_of_pts)
        
        ## for 3d penalty w.r.t angle plot
        self.pitch_seq=self.axis_angle_range
        self.roll_seq=self.axis_angle_range
        self.R,self.P=np.meshgrid(self.roll_seq,self.pitch_seq)
        
        euler_angle=np.zeros((self.num_of_pts,3))

        ## choose the roll, pitch, or yaw
        self.euler_table={0:'yaw',1:'pitch',2:'roll'}
        self.trans_table={0:'x',1:'y',2:'z'}
        self.euler_choice=euler_choice
        euler_angle[:,self.euler_choice]=self.axis_angle_range
        self.quad_quat=R.from_euler('zyx', euler_angle).as_quat()
        self.quad_quat=np.roll(self.quad_quat,1,axis=1)
        
        ## choose the translation direction
        self.trans_table={0:'x',1:'y',2:'z'}
        self.trans_choice=trans_choice

    def load_config(self, mission_config):
        self.config_dict=mission_config
       


    def init_env(self):
        options={
            
        'SQP_RTI_OPTION' : False,
        'USE_PREV_SOLVER'  : False,
        'JAX_SVD' : False, # JAX_SVD or CasADi_SVD

        ## BACKWARD required
        'MPC_BACKWARD' : True,
        'ORIGIN_REWARD'  : False,
        'PDP_GRADIENT' : True,
        'CLOSE_LOOP_TRAINING' : True,

        ## training option
        'MULTI_CORE'  : False,
        'TRAIN_FROM_CHECKPOINT' : False
        }
        
        self.planner = PlanFwdBwdWrapper(self.config_dict, options)

        ##== let the gate to be horizontal
        inputs=np.zeros(17)
        inputs[7]=nn_sample()[7]
        # inputs[7]=self.config_dict['gate']['width']
        inputs[8:17]=np.eye(3).flatten()

        moving_gate = MovingGate(inputs,
                                gate_center=np.zeros(3),
                                gate_length=self.config_dict['gate']['length'])
    
        gate_t_0 = moving_gate.gate
        # initialize the narrow window
        self.planner.init_obstacle(gate_t_0)
    
    
        

    def penalty_cal(self,state_traj):
        penalty,_,_=self.planner.obstacle.penalty_cal_diff_collision(self.config_dict,
                                                                state_traj=state_traj,
                                                                gate_corners=self.planner.gate_corners,
                                                                gate_quat=self.planner.gate_quat,
                                                                vert_traj=state_traj[:,0:3],
                                                                goal_pos=np.zeros(3),
                                                                PENALTY_HELPER=True)
        return penalty
    def plot_penalty_seq(self):
        plt.figure()
        # ax=fig.add_subplot(projection='3d')
        state_traj=np.zeros((1,10))
        penalty=np.zeros(len(self.quad_quat))
        start_time = time.time()
        for i in range(len(self.quad_quat)):
            if self.ROT_VIS:
                state_traj[:,6:10]=self.quad_quat[i]
                plt.xlabel(f'{self.euler_table[self.euler_choice]} angle')
            else:
                state_traj[:,6:10]=np.array([1,0,0,0])
                state_traj[:,self.trans_choice]=self.trans_range[i]
                plt.xlabel(f'{self.trans_table[self.trans_choice]} axis')
            penalty[i]=self.penalty_cal(state_traj)
        print("--- %s seconds ---" % (time.time() - start_time))
        plt.plot(self.axis_angle_range,penalty)
        
        plt.grid()
        plt.show()
    
    def plot_penalty_3d(self):
        fig=plt.figure()
        ax=fig.add_subplot(projection='3d')
        state_traj=np.zeros((1,10))
        penalty=np.zeros((100,100))
       
        for i in range(len(self.roll_seq)):
            for j in range(len(self.pitch_seq)):
                euler_angle=np.zeros(3)
                euler_angle[2]=self.roll_seq[i]
                euler_angle[1]=self.pitch_seq[j]
                quad_quat=R.from_euler('zyx', euler_angle).as_quat()
                quad_quat=np.roll(quad_quat,1)
                state_traj[:,6:10]=quad_quat
                penalty[j,i]=self.penalty_cal(state_traj)
        
        ax.plot_surface(self.R,self.P,penalty*2,cmap=plt.cm.CMRmap)
        # Tweak the limits and add latex math labels.
        
        ax.set_xlabel('roll')
        ax.set_ylabel('pitch')
        ax.set_zlabel('penalty')
        plt.show()
        
if __name__ == '__main__':
    # if ROT_VIS is True, the penalty will be calculated w.r.t the quadrotor's euler angle change
    helper=PenaltyDesignHelper(trans_choice=0,euler_choice=2,ROT_VIS=True) 
    helper.load_config(mission_config=mission_cfg)
    helper.init_env()
    helper.plot_penalty_seq()
    # helper.plot_penalty_3d()
    