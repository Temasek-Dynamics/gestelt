##this file is the package about neural network

from cmath import tan
from math import cos, pi, sin, sqrt, tan
import math
from numpy import random
import torch
import torch.nn as nn
import numpy as np
from quad_model import toQuaternion
from solid_geometry import norm,magni
from solid_geometry import plane
from scipy.spatial.transform import Rotation as R
import scipy.stats as stats
import os
import yaml
current_dir = os.path.dirname(os.path.abspath(__file__))
conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
with open(yaml_file, 'r', encoding='utf-8') as file:
    config_dict = yaml.safe_load(file) 
pre_ini_pos=np.array(config_dict['mission']['initial_position'])
pre_end_pos=np.array(config_dict['mission']['goal_position'])
desired_average_vel=config_dict['training_param']['desired_average_vel']
gate_width = config_dict['gate']['width']
# load the configuration file

## sample an input for the neural network 1
def nn_sample(init_pos=None,final_pos=None,init_angle=None,cur_epoch=100,pretrain=False):
    inputs = np.zeros(17)
    if init_pos is None:
        inputs[0:3] = np.random.uniform(-1,1,size=3) + pre_ini_pos #-5~5, -9

        # TODO: transfer to trauncated normal distribution
        if pretrain:
            inputs[1] = np.random.uniform(-5,5) + pre_ini_pos[1]
        else:
            inputs[1] = np.clip(inputs[1],pre_ini_pos[1]-1,pre_ini_pos[1]+1) 
    else:
        inputs[0:3] = init_pos
    ## random final position 
    if final_pos is None:
        inputs[3:6] = np.random.uniform(-2,2,size=3) + pre_end_pos #-2~2, 6

        inputs[4]=np.clip(inputs[4],pre_end_pos[1]-0.5,pre_end_pos[1]+0.5)
    else:
        inputs[3:6] = final_pos

        
    ##random initial yaw angle of the quadrotor ##
    inputs[6] = np.random.uniform(-0.1,0.1)
    
    ## === random width of the gate  =========##
    inputs[7] = np.clip(np.random.normal(0.6,0.2),gate_width,gate_width) #(0.9,0.3),0.5,1.25 
  
    ## === random pitch angle of the gate ====##
    # angle = np.clip(1.3*(1.2-inputs[7]),0,pi/3)
    # angle1 = (pi/2-angle)/3
    # judge = np.random.normal(0,1)
    # if init_angle is None:
    #     if judge > 0:
    #         inputs[8] = np.clip(np.random.normal(angle + angle1, 2*angle1/3),angle,pi/2)
    #         # inputs[8] = np.random.uniform(angle - angle1, angle + angle1)
    #     else:
    #         inputs[8] = np.clip(np.random.normal(-angle - angle1, 2*angle1/3),-pi/2,-angle)
    # else:
    #     inputs[8] = init_angle

    ###==== curriculum learning ===###
    # 0 -> gate is horizontal
    # pi/2 -> gate is vertical
    if pretrain:
        gate_pitch = np.random.uniform(-pi/2,pi/2)
    else:
        des_pitch_mean_min = 1*pi/6
        des_pitch_mean_max = 1*pi/6
        des_pitch_mean = des_pitch_mean_min - (des_pitch_mean_min - des_pitch_mean_max) * (cur_epoch / 100) 
        # gate_pitch = np.clip(np.random.normal(0,pi/18),-pi/6,pi/6) 
        # truncated normal distribution
        mu,sigma = 0,pi/18
        lower,upper = -pi/6,pi/6
        X = stats.truncnorm((lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma)
        gate_pitch = X.rvs(1)[0]
        if gate_pitch>0:
            gate_pitch=gate_pitch+des_pitch_mean
        else:
            gate_pitch=gate_pitch-des_pitch_mean
    

    

    ##==calculate the gate RM
    rot=R.from_euler('zyx',[0,gate_pitch,0])
    inputs[8:17]=rot.as_matrix().flatten()
    return inputs

## define the expected output of an input (for pretraining)
def t_output(inputs):
    inputs = np.array(inputs)
    outputs = np.zeros(13)
    R_gate=inputs[8:17].reshape(3,3)
    outputs[3:12]=R_gate.T.flatten()
    # outputs[3:12] = np.array([[0.0007963,  0.0000000, -0.9999997],
    #                         [0.0000000,  1.0000000,  0.0000000],
    #                         [0.9999997,  0.0000000,  0.0007963]]).flatten()  
    #outputs[5] = math.tan(inputs[6]/2)
    ## traversal time is propotional to the distance of the centroids
    if inputs[1]>0:
        raw_time = -round(magni(inputs[0:3])/2,1)
    else:
        raw_time=round(magni(inputs[0:3])/2,1)
    outputs[-1] = raw_time #np.clip(raw_time,3,3)
    print('desired_traversing_time',outputs[-1])
    return outputs

## sample a random gate (not necessary in our method) (not important)
def gene_gate():
    point1 = np.array([0,0,0])
    #generate diagonal line and point3
    dia_line = np.random.uniform(1.5,3)
    point3 = np.array([dia_line,0,0])
    # generate point2
    point2x = np.random.normal(dia_line/2,dia_line/2)
    point2z = np.random.uniform(0,dia_line)
    point2 = np.array([point2x,0,point2z])
    # generate point4
    point4x = np.random.normal(dia_line/2,dia_line/2)
    point4z = np.random.uniform(-dia_line, 0)
    point4 = np.array([point4x,0,point4z])
    return np.array([point1,point2,point3,point4])


## sample any initial state, final point and 12 elements window (not necessary in our method) (not important)
def con_sample():
    inputs = np.zeros(25)
    # generate first three inouts
    scaling = np.random.uniform(3,16)
    phi = np.random.uniform(0,2*pi)
    theta = np.clip(np.random.normal(pi/2,pi/8,size=1), pi/4, 3*pi/4)
    #transformation
    inputs[0] = scaling*sin(theta)*cos(phi)
    inputs[1] = scaling*sin(theta)*sin(phi)
    inputs[2] = scaling*cos(theta)
    beta = np.random.uniform(0,2*pi)
    rotation1 = np.array([[cos(beta),0,sin(beta)],[0,1,0],[-sin(beta),0,cos(beta)]])
    rotation2 = np.array([[cos(phi-pi/2),-sin(phi-pi/2),0],[sin(phi-pi/2),cos(phi-pi/2),0],[0,0,1]])
    rotation  = np.matmul(rotation2,rotation1)
    # generate rotation pair
    l = norm(np.random.normal(0,1,size=3))
    a = np.random.normal(0,pi/16)
    r = R.from_rotvec(a * l)
    rotation = np.matmul(r.as_matrix(),rotation)
    # generate translation
    length = np.random.uniform(2,scaling-1) 
    tranlation1 = np.array([length*sin(theta)*cos(phi),length*sin(theta)*sin(phi),length*cos(theta)])
    tranlation = tranlation1 + np.random.normal(0,1,size=3)
    # generate real obstacle
    gate = gene_gate()
    for i in range(4):
        gate[i] = np.matmul(rotation,gate[i]) + tranlation
    inputs[3:15] = gate.reshape(12)
        #generate velocity
    inputs[15:18] = np.random.normal(0,3,size=3)
    #generate quaternions
    Rd = np.random.normal(0,0.5,size=3)
    rp = Rd2Rp(Rd)
    inputs[18:22] = toQuaternion(rp[0],rp[1])
    distance = np.random.uniform(0,scaling)
    inputs[22] = distance*sin(theta)*cos(phi)+np.random.normal(0,1)
    inputs[23] = distance*sin(theta)*sin(phi)+np.random.normal(0,1)
    inputs[24] = distance*cos(theta)+np.random.normal(0,1)
    return inputs


## define the class of neural network (2 hidden layers, unit = ReLU)
class network(nn.Module):
    def __init__(self, D_in, D_h1, D_h2, D_out):
        super(network, self).__init__()        
        # D_in : dimension of input layer
        # D_h  : dimension of hidden layer
        # D_out: dimension of output layer
        self.l1 = nn.Linear(D_in, D_h1)
        self.F1 = nn.ReLU()
        self.l2 = nn.Linear(D_h1, D_h2)
        self.F2 = nn.ReLU()
        self.l3 = nn.Linear(D_h2, D_out)

    def forward(self, input):
        # convert state s to tensor
        S = input # column 2D tensor
        out = self.l1(S) # linear function requires the input to be a row tensor
        out = self.F1(out)
        out = self.l2(out)
        out = self.F2(out)
        out = self.l3(out)
        return out

    def myloss_original(self, para, dp):
        # convert np.array to tensor
        Dp = torch.tensor(dp, dtype=torch.float) # row 2D tensor
        loss_nn = torch.matmul(Dp, para)
        return loss_nn

    def myloss(self, para, dp, device='cpu'):
        # convert np.array to tensor
        Dp = torch.tensor(dp, dtype=torch.float).to(device) # row 2D tensor
        # loss_nn = torch.matmul(Dp, para)
        loss_nn =torch.trace(torch.matmul(Dp, para.t()))/(Dp.shape[0])
        return loss_nn # size is 1

class network_with_GRU(nn.Module):
    def __init__(self, D_in, D_h1, D_h2, D_out):
        super(network_with_GRU, self).__init__()        
        # D_in : dimension of input layer
        # D_h  : dimension of hidden layer
        # D_out: dimension of output layer
        self.l1 = nn.Linear(D_in, D_h1)
        self.F1 = nn.ReLU()
        self.l2 = nn.Linear(D_h1, D_h2)
        self.F2 = nn.ReLU()
        self.GRU = nn.GRU(input_size=D_h2, hidden_size=D_h2,num_layers=1,batch_first=True)
        self.l3 = nn.Linear(D_h2, D_out)
        
    def forward(self, input):
        # convert state s to tensor
        S = input.unsqueeze(0) # column 2D tensor
        out = self.l1(S) # linear function requires the input to be a row tensor
        out = self.F1(out)
        out = self.l2(out)
        out = self.F2(out)
        out = self.GRU(out)
        out = self.l3(out)
        return out


## run the above code
if __name__ == "__main__":
    # sample 1000 nn_sample() and plot the gate_pitch
    import matplotlib.pyplot as plt
    gate_euler_list = []
    for i in range(1000):
        inputs = nn_sample()
        gate_euler = R.from_matrix(inputs[8:17].reshape(3,3)).as_euler('zyx')
        gate_euler_list.append(gate_euler[1])
    plt.hist(gate_euler_list,bins=100)
    plt.show()