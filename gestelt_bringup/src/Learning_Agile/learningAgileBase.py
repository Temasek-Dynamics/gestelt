import numpy as np
from collections import deque
from math import atan,magni
from scipy.spatial.transform import Rotation as R

from learning_agile_sim import LearningAgileSim, Gate
class LearningAgileBase:
    """
    this class is responsible for wrap the single epsiode
    """
    def __init__(self,config_dict,options):
        self.config_dict = config_dict
        self.gate_v = np.array(self.config_dict['gate']['linear_vel'])
        self.gate_w = config_dict['gate']['angular_vel'] 
        self.NN_freq = config_dict['NN2_freq']
        self.learning_agile_sim = LearningAgileSim(python_sim_time=5,
                                                       yaml_file=self.mission_yaml,
                                                       dyn_step=1/self.NN_freq,
                                                       options=options)
        ## keep history five states, RING BUFFER
        self.input_size = 23
        self.history_obs = deque(maxlen=5)

    def get_action(self, state):
        pass

    def get_reward(self, state):
        pass

    def get_state(self):
        pass

    def reset(self):
        #== random generate the env and set to the mpc solver
        self.learning_agile_sim.generate_mission()
        self.planner = self.learning_agile_sim.planner
        self.state = self.planner.ini_state

        #== reset the gate
        self.learning_agile_sim.prepare_gate()
        self.i = 0
        self.gate_points_list = self.learning_agile_sim.gate_points_list
        
        #== reset the obs
        
        
        self.init_gradient()
        self.state_traj=[]
        
        obs=self.get_obs(0)
        return obs



    def get_obs(self,i):
        
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
        
        obs=np.array(self.history_obs)
        

        self.planner.init_obstacle(gate_t_i.gate_point[:,:].reshape(12),gate_pitch)
        return obs

    def init_gradient(self):
        self.R_i = 0
        self.p_R_i_p_X_traj_i = []
        self.p_X_traj_i_p_x_i = []
        self.p_X_traj_i_p_z_i = []
        self.p_z_i_p_w = []
