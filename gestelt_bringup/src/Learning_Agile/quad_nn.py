##this file is the package about neural network
from torch.distributions.normal import Normal
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
from config import mission_cfg, train_cfg,current_dir

pre_ini_pos=np.array(mission_cfg['mission']['initial_position'])
pre_end_pos=np.array(mission_cfg['mission']['goal_position'])
desired_average_vel=mission_cfg['pretrain_param']['desired_average_vel']
gate_width = mission_cfg['gate']['width']
init_gate_width = mission_cfg['gate']['init_width']
input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size']  
# load the configuration file

## sample an input for the neural network 1
def nn_sample(init_pos=None,final_pos=None,init_angle=None,cur_epoch=train_cfg['training']['num_epochs'],pretrain=False):
    env_init_set = np.zeros(17)
    if init_pos is None:
        env_init_set[0] = pre_ini_pos[0] #np.random.uniform(-1,1) + pre_ini_pos[0] #-5~5, -9

        # TODO: transfer to trauncated normal distribution
        if pretrain:
            env_init_set[1] = np.random.uniform(-5,5) #+ pre_ini_pos[1]
        else:
            env_init_set[1] = np.random.uniform(-0.5,0.5) + pre_ini_pos[1]

        env_init_set[2] = np.random.uniform(-0.5, 0.5) + pre_ini_pos[2] #-5~5, 0
    else:
        env_init_set[0:3] = init_pos
    ## random final position 
    if final_pos is None:
        env_init_set[3] = np.random.uniform(-1,1) + pre_end_pos[0] #-2~2, 6

        env_init_set[4] = np.random.uniform(-0.5,0.5)+pre_end_pos[1]
        env_init_set[5] = np.random.uniform(-0.5,0.5)+pre_end_pos[2]
    else:
        env_init_set[3:6] = final_pos

        
    ##random initial yaw angle of the quadrotor ##
    env_init_set[6] = np.random.uniform(-0.1,0.1)
    
    ## === random width of the gate  =========##
    # env_init_set[7] = np.clip(np.random.normal(0.6,0.2),gate_width,gate_width) #(0.9,0.3),0.5,1.25 
    env_init_set[7] = init_gate_width - (init_gate_width - gate_width) * (cur_epoch / train_cfg['training']['num_epochs'])
  
    ## === random pitch angle of the gate ====##
    # angle = np.clip(1.3*(1.2-env_init_set[7]),0,pi/3)
    # angle1 = (pi/2-angle)/3
    # judge = np.random.normal(0,1)
    # if init_angle is None:
    #     if judge > 0:
    #         env_init_set[8] = np.clip(np.random.normal(angle + angle1, 2*angle1/3),angle,pi/2)
    #         # env_init_set[8] = np.random.uniform(angle - angle1, angle + angle1)
    #     else:
    #         env_init_set[8] = np.clip(np.random.normal(-angle - angle1, 2*angle1/3),-pi/2,-angle)
    # else:
    #     env_init_set[8] = init_angle

    ###==== curriculum learning ===###
    # 0 -> gate is horizontal
    # pi/2 -> gate is vertical
    if pretrain:
        # gate_pitch = np.random.uniform(-pi/2,pi/2)
        gate_pitch = 0
    else:
        des_pitch_mean_min = 1*pi/4
        des_pitch_mean_max = 1*pi/4
        des_pitch_mean = des_pitch_mean_min - (des_pitch_mean_min - des_pitch_mean_max) * (cur_epoch / 100) 

        # truncated normal distribution
        mu,sigma = 0,pi/16
        lower,upper = -pi/3,pi/3
        X = stats.truncnorm((lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma)
        gate_pitch = X.rvs(1)[0]
        
        judge = np.random.normal(0,1)
        # if gate_pitch>0:
        if judge > 0:
            gate_pitch=gate_pitch+des_pitch_mean
        else:
            gate_pitch=gate_pitch-des_pitch_mean
        
        gate_pitch = mission_cfg['mission']['gate_ori_euler'][1] #1.2rad = 68.754 degrees, 0.8rad = 45.729 degrees 
        # gate_pitch = np.random.uniform(-pi/6,pi/6)

    ##==calculate the gate RM
    rot=R.from_euler('zyx',[0,gate_pitch,0])
    env_init_set[8:17]=rot.as_matrix().flatten()
    return env_init_set

## define the expected output of an input (for pretraining)
def t_output(inputs):
    inputs = np.array(inputs[0])
    
    outputs = np.zeros(output_size)
    outputs[0:3]=mission_cfg['mission']['gate_position']
    R_gate=inputs[-9:].reshape(3,3)
    outputs[3:12]=R_gate.T.flatten()

    ## wrp
    # outputs[-2]=mission_cfg['learning_agile']['wrp']
    
    ## wrp
    # outputs[-5]=10

    # ## max_tra_w
    # outputs[-4]=100

    # ## wrt
    # outputs[-3]=5

    # ## wqt
    # outputs[-2]=10

    ## traversal time is proportional to the distance of the centroids
    if inputs[1]>0:
        raw_time = round(magni(inputs[0:3])/desired_average_vel,2) #3
    else:
        raw_time = -round(magni(inputs[0:3])/desired_average_vel,2) #4
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
# def con_sample():
    # inputs = np.zeros(25)
    # # generate first three inouts
    # scaling = np.random.uniform(3,16)
    # phi = np.random.uniform(0,2*pi)
    # theta = np.clip(np.random.normal(pi/2,pi/8,size=1), pi/4, 3*pi/4)
    # #transformation
    # inputs[0] = scaling*sin(theta)*cos(phi)
    # inputs[1] = scaling*sin(theta)*sin(phi)
    # inputs[2] = scaling*cos(theta)
    # beta = np.random.uniform(0,2*pi)
    # rotation1 = np.array([[cos(beta),0,sin(beta)],[0,1,0],[-sin(beta),0,cos(beta)]])
    # rotation2 = np.array([[cos(phi-pi/2),-sin(phi-pi/2),0],[sin(phi-pi/2),cos(phi-pi/2),0],[0,0,1]])
    # rotation  = np.matmul(rotation2,rotation1)
    # # generate rotation pair
    # l = norm(np.random.normal(0,1,size=3))
    # a = np.random.normal(0,pi/16)
    # r = R.from_rotvec(a * l)
    # rotation = np.matmul(r.as_matrix(),rotation)
    # # generate translation
    # length = np.random.uniform(2,scaling-1) 
    # tranlation1 = np.array([length*sin(theta)*cos(phi),length*sin(theta)*sin(phi),length*cos(theta)])
    # tranlation = tranlation1 + np.random.normal(0,1,size=3)
    # # generate real obstacle
    # gate = gene_gate()
    # for i in range(4):
    #     gate[i] = np.matmul(rotation,gate[i]) + tranlation
    # inputs[3:15] = gate.reshape(12)
    #     #generate velocity
    # inputs[15:18] = np.random.normal(0,3,size=3)
    # #generate quaternions
    # Rd = np.random.normal(0,0.5,size=3)
    # rp = Rd2Rp(Rd)
    # inputs[18:22] = toQuaternion(rp[0],rp[1])
    # distance = np.random.uniform(0,scaling)
    # inputs[22] = distance*sin(theta)*cos(phi)+np.random.normal(0,1)
    # inputs[23] = distance*sin(theta)*sin(phi)+np.random.normal(0,1)
    # inputs[24] = distance*cos(theta)+np.random.normal(0,1)
    # return inputs


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
        self.GRU = nn.GRU(input_size=D_in, hidden_size=D_h2,num_layers=1,batch_first=True)
        self.l1 = nn.Linear(D_h1, D_h1)
        self.F1 = nn.ReLU()
        self.l2 = nn.Linear(D_h1, D_h2)
        self.F2 = nn.ReLU()
        self.l3 = nn.Linear(D_h2, D_out)

        # logstd = -3
        # self.logstd = nn.Parameter(torch.ones(D_out, dtype=torch.float32) * logstd)

        
    def forward(self, input,deterministic=True):
        # convert state s to tensor
        S = input # column 2D tensor
        out,hidden = self.GRU(S)
        out = out [:,-1,:]
        out = self.l1(out) # linear function requires the input to be a row tensor
        out = self.F1(out)
        out = self.l2(out)
        out = self.F2(out)
        out = out.squeeze(1)
        out = self.l3(out)

        # if deterministic:
        return out
        # else:
        #    std=self.logstd.exp()
        #    dist=Normal(out,std)
        #    sample=dist.rsample()
           
        #    return sample
    
    def myloss(self, para, dp, device='cpu'):
        # convert np.array to tensor
        Dp = torch.tensor(dp, dtype=torch.float).to(device) # row 2D tensor
        # loss_nn = torch.matmul(Dp, para)
        para=para.to(device)
        loss_nn =torch.trace(torch.matmul(Dp, para.t()))/(Dp.shape[0])
        return loss_nn # size is 1

    def loss_close_loop(self, para, dp, device='cpu'):
        # convert np.array to tensor
        Dp = torch.tensor(dp, dtype=torch.float).to(device) 
        para=para.to(device)
        

        # bxHx1x13 x bxHx13x1 -> 1
        loss_nn = torch.sum(torch.einsum('bijk,bikj -> b',para,Dp))/(Dp.shape[0]*Dp.shape[1])

        return loss_nn # size is 1

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