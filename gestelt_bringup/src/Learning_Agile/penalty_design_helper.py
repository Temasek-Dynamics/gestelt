import numpy as np
# from differentiable_collision_wrapper import *

import yaml
import os
from scipy.spatial.transform import Rotation as R
from solid_geometry import *
from quad_policy import *
from learning_agile_agent import MovingGate
"""
this file will plot the penalty value change w.r.t the euler angle change of the quadrotor
"""
class PenaltyDesignHelper():
    def __init__(self):
        pass

    def rotating_quad(self):
        self.axis_angle_range=np.linspace(-np.pi/2,np.pi/2,100)
        self.pitch_seq=self.axis_angle_range
        self.roll_seq=self.axis_angle_range
        self.R,self.P=np.meshgrid(self.roll_seq,self.pitch_seq)
        
        euler_angle=np.zeros((100,3))

        ## choose the roll, pitch, or yaw
        self.euler_table={0:'yaw',1:'pitch',2:'roll'}
        self.euler_choose=2
        euler_angle[:,self.euler_choose]=self.axis_angle_range
        self.quad_quat=R.from_euler('zyx', euler_angle).as_quat()
        self.quad_quat=np.roll(self.quad_quat,1,axis=1)
        
    
        

    def load_config(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
        yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
        with open(yaml_file, 'r', encoding='utf-8') as file:
            self.config_dict = yaml.safe_load(file)
       


    def init_env(self):
        options={
            
        'SQP_RTI_OPTION' : False,
        'USE_PREV_SOLVER'  : False,
        'JAX_SVD' : False, # JAX_SVD or CasADi_SVD

        ## BACKWARD required
        'MPC_BACKWARD' : True,
        'ORIGIN_REWARD'  : False,
        'PDP_GRADIENT' : True,


        ## training option
        'MULTI_CORE'  : False,
        'TRAIN_FROM_CHECKPOINT' : False
        }
        
        self.planner = PlanFwdBwdWrapper(self.config_dict, options)

        ##== let the gate to be horizontal
        inputs=np.zeros(17)
        inputs[7]=self.config_dict['gate']['width']
        inputs[8:17]=np.eye(3).flatten()

        moving_gate = MovingGate(inputs,
                                gate_cen_h=0,
                                gate_length=self.config_dict['gate']['length'])
    
        gate_point = moving_gate.gate.gate_point
        # initialize the narrow window
        self.planner.init_obstacle(gate_point.reshape(12),gate_pitch=moving_gate.gate_init_pitch)
        
    def penalty_cal(self,state_traj):
        penalty,_=self.planner.obstacle.reward_calc_differentiable_collision(self.config_dict,
                                                                state_traj=state_traj,
                                                                gate_corners=self.planner.gate_corners,
                                                                gate_quat=self.planner.gate_quat,
                                                                vert_traj=state_traj[:,0:3],
                                                                goal_pos=np.zeros(3),
                                                                PENALTY_HELPER=True)
        return penalty
    def plot_penalty_seq(self):
        fig=plt.figure()
        # ax=fig.add_subplot(projection='3d')
        state_traj=np.zeros((1,10))
        penalty=np.zeros(len(self.quad_quat))
        for i in range(len(self.quad_quat)):
            state_traj[:,6:10]=self.quad_quat[i]
            penalty[i]=self.penalty_cal(state_traj)

        plt.plot(self.axis_angle_range,penalty)
        plt.xlabel(f'{self.euler_table[self.euler_choose]} angle')
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
if __name__ == "__main__":
    helper=PenaltyDesignHelper()
    helper.load_config()
    helper.rotating_quad()
    helper.init_env()
    helper.plot_penalty_seq()
    # helper.plot_penalty_3d()
    