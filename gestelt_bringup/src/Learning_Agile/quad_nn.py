##this file is the package about neural network
from config import mission_cfg, train_cfg
from math import pi
import torch
import torch.nn as nn
from torch.nn.utils import spectral_norm
import numpy as np
from scipy.spatial.transform import Rotation as R
import scipy.stats as stats


from geometry.solid_geometry import magni


pre_ini_pos=np.array(mission_cfg['mission']['initial_position'])
pre_end_pos=np.array(mission_cfg['mission']['goal_position'])
gate_width = mission_cfg['gate']['width']
init_std_dev=mission_cfg['gate']['init_std_dev']
final_std_dev=mission_cfg['gate']['final_std_dev']
input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size']  
# load the configuration file

## sample an input for the neural network 1
def nn_sample(init_pos=None,
              final_pos=None,
              init_angle=None,
              cur_epoch=train_cfg['training']['num_epochs'], # default is the last epoch
              PRTRAIN=False,
              TEST=False):
    env_init_set = np.zeros(17)
    if init_pos is None:
        env_init_set[0:3] =  np.random.uniform(-0.2,0.2,3) + pre_ini_pos #-5~5, -9 
        env_init_set[1] = np.random.uniform(-0.2,0.2) + pre_ini_pos[1]
        if PRTRAIN:
            env_init_set[1] = np.random.uniform(-5,5) #+ pre_ini_pos[1]
            env_init_set[2] = np.random.uniform(0,2.5) #+ pre_ini_pos[0]

    else:
        env_init_set[0:3] = init_pos
    ## random final position 
    if final_pos is None:
       
        env_init_set[3:6]=np.random.uniform(-0.1,0.1,3) + pre_end_pos
        env_init_set[4]=np.random.uniform(-0.1,0.1) + pre_end_pos[1]
    else:
        env_init_set[3:6] = final_pos

        
    ##random initial yaw angle of the quadrotor ##
    env_init_set[6] = np.random.uniform(-0.1,0.1)
    
    ## === random width of the gate  =========##
    # env_init_set[7] = np.clip(np.random.normal(0.6,0.2),gate_width,gate_width) #(0.9,0.3),0.5,1.25 
    env_init_set[7] = gate_width
  
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
    if PRTRAIN:
        # gate_pitch = np.random.uniform(-pi/2,pi/2)
        # gate_pitch=mission_cfg['mission']['gate_ori_euler'][1] 
        gate_pitch = 0
    elif TEST:
        if not mission_cfg['FIX_GATE_PITCH_TEST']:
           gate_pitch =  np.random.uniform(-pi/2,pi/2)
        else:
            gate_pitch = mission_cfg['mission']['gate_ori_euler'][1] 
    else:
        des_pitch_mean_min = 1*pi/4
        des_pitch_mean_max = 1*pi/4
        des_pitch_mean = des_pitch_mean_min - (des_pitch_mean_min - des_pitch_mean_max) * (cur_epoch / 100) 

        # truncated normal distribution
        mu,sigma = 0,init_std_dev+(final_std_dev-init_std_dev)*(cur_epoch/train_cfg['training']['num_epochs'])
        lower,upper = -pi/3,pi/3
        X = stats.truncnorm((lower - mu) / sigma, (upper - mu) / sigma, loc=mu, scale=sigma)
        gate_pitch = X.rvs(1)[0]
        
        judge = np.random.normal(0,1)
        # if gate_pitch>0:
        if judge > 0:
            gate_pitch=gate_pitch+des_pitch_mean
        else:
            gate_pitch=gate_pitch-des_pitch_mean
        
        ## or 
        # gate_pitch = mission_cfg['mission']['gate_ori_euler'][1] #1.2rad = 68.754 degrees, 0.8rad = 45.729 degrees 

        ## or 
        gate_pitch = np.random.uniform(-pi/2,pi/2)
        
        

    ##==calculate the gate RM
    rot=R.from_euler('zyx',[0,gate_pitch,0])
    env_init_set[8:17]=rot.as_matrix().flatten()
    return env_init_set

## define the expected output of an input (for pretraining)
def t_output(inputs,gate_rot_matrix):
    """the traverse time is calculated based on the signed distance between the drone position and the gate position.

    Args:
        inputs (array): the NN input

    Returns:
        array: preset output of the NN
    """
    
    outputs = np.zeros(output_size)
    outputs[0:3]=mission_cfg['mission']['gate_position']-inputs[0:3]*mission_cfg['pos_norm_factor']# gate position
    
    # outputs[3:12]=gate_rot_matrix
    # or
    outputs[3:12]=np.eye(3).flatten()

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




## define the class of neural network (2 hidden layers, unit = ReLU)
# class network(nn.Module):
#     def __init__(self, D_in, D_h1, D_h2, D_out):
#         super(network, self).__init__()        
#         # D_in : dimension of input layer
#         # D_h  : dimension of hidden layer
#         # D_out: dimension of output layer
#         self.l1 = nn.Linear(D_in, D_h1)
#         self.F1 = nn.ReLU()
#         self.l2 = nn.Linear(D_h1, D_h2)
#         self.F2 = nn.ReLU()
#         self.l3 = nn.Linear(D_h2, D_out)

#     def forward(self, input):
#         # convert state s to tensor
#         S = input # column 2D tensor
#         out = self.l1(S) # linear function requires the input to be a row tensor
#         out = self.F1(out)
#         out = self.l2(out)
#         out = self.F2(out)
#         out = self.l3(out)
#         return out

#     def myloss_original(self, para, dp):
#         # convert np.array to tensor
#         Dp = torch.tensor(dp, dtype=torch.float) # row 2D tensor
#         loss_nn = torch.matmul(Dp, para)
#         return loss_nn

#     def myloss(self, para, dp, device='cpu'):
#         # convert np.array to tensor
#         Dp = torch.tensor(dp, dtype=torch.float).to(device) # row 2D tensor
#         # loss_nn = torch.matmul(Dp, para)
#         loss_nn =torch.trace(torch.matmul(Dp, para.t()))/(Dp.shape[0])
#         return loss_nn # size is 1

class network_with_GRU(nn.Module):
    def __init__(self, D_in, D_h1, D_h2, D_out):
        super(network_with_GRU, self).__init__()        
        # D_in : dimension of input layer
        # D_h  : dimension of hidden layer
        # D_out: dimension of output layer
        # self.GRU = nn.GRU(input_size=D_in, hidden_size=D_h2,num_layers=1,batch_first=True)
        # self.input_norm=nn.LayerNorm(D_in)
        # self.out_norm = nn.LayerNorm(D_h2)
        self.l1 = nn.Linear(D_in, D_h1)
        if train_cfg['model']['activation'] == 'tanh':
            self.F1 = nn.Tanh()
            self.F2 = nn.Tanh()
        elif train_cfg['model']['activation'] == 'silu':
            self.F1 = nn.SiLU()
            self.F2 = nn.SiLU()
        self.l2 = spectral_norm(nn.Linear(D_h1, D_h2))
        # self.l3 = nn.Linear(D_h2, D_out)


        # positional head
        if mission_cfg['POSITION_ENCODING']:
            self.sing_axis_K=4
            self.positional_head = nn.Linear(D_h2, 3*self.sing_axis_K)
            self.xy_bins=torch.linspace(-1, 1, self.sing_axis_K+1)
            self.z_bins=torch.linspace(1.2, 2, self.sing_axis_K+1)

            self.register_buffer('xy_centers',0.5*(self.xy_bins[1:]+self.xy_bins[:-1]).reshape(1,-1))
            self.register_buffer('z_centers',0.5*(self.z_bins[1:]+self.z_bins[:-1]).reshape(1,-1))
        else:
            self.positional_head = nn.Linear(D_h2, 3)

        # rotation head
        self.rotation_head = nn.Linear(D_h2, 9)

        # weights vector head
        self.weights_head = nn.Linear(D_h2, train_cfg["model"]["weights_vector_length"])

        
    def forward(self, input,deterministic=True):
        #add layer norm
        # input = self.input_norm(input)
        # out,hidden = self.GRU(input)
        # out = out [:,-1,:]
        # out = hidden[-1,:,:]
        out = self.l1(input) # linear function requires the input to be a row tensor
        out = self.F1(out)
        out = self.l2(out)
        out = self.F2(out)
        out = out.squeeze(1)

        # position head   
        if mission_cfg['POSITION_ENCODING']:     
            position_logit=self.positional_head(out)

            # # traverse position x, y, z
            x_logit = position_logit[:,:self.sing_axis_K]
            y_logit = position_logit[:,self.sing_axis_K:2*self.sing_axis_K]
            z_logit = position_logit[:,2*self.sing_axis_K:3*self.sing_axis_K]

            x_probs = torch.softmax(x_logit, dim=-1)
            y_probs = torch.softmax(y_logit, dim=-1)
            z_probs = torch.softmax(z_logit, dim=-1)

            x_hat = x_probs @ self.xy_centers.T
            y_hat = y_probs @ self.xy_centers.T
            z_hat = z_probs @ self.z_centers.T
        else:
            # Keep the batch dimension
            x_hat = torch.zeros((out.shape[0], 1),device=out.device)
            y_hat = torch.zeros((out.shape[0], 1),device=out.device)
            z_hat = torch.zeros((out.shape[0], 1),device=out.device)
            x_hat[:,0] = torch.tanh(self.positional_head(out)[:,0])*2
            y_hat[:,0] = torch.tanh(self.positional_head(out)[:,1])*2
            z_hat[:,0] = torch.tanh(self.positional_head(out)[:,2])*2
            
        # orientation 
        orientation = self.rotation_head(out)

        # vector head
        weights = self.weights_head(out)
        weights[:,0:3]=torch.sigmoid(weights[:,0:3])*300+10 # wrp
        weights[:,3:6]=torch.sigmoid(weights[:,3:6])*300+10 # wrt
        weights[:,6]=torch.sigmoid(weights[:,6])*50+10
        # gamma
        weights[:,7]=torch.sigmoid(weights[:,7])*100+5



        return torch.hstack([x_hat, y_hat, z_hat,orientation, weights])

    
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


class network(nn.Module):
    def __init__(self, D_in, D_h1, D_h2, weights_vector_length, activation='silu'):
        super(network, self).__init__()        
        # D_in : dimension of input layer
        # D_h  : dimension of hidden layer
        # D_out: dimension of output layer
        self.D_out = 3 + 9 + weights_vector_length # 3 for position, 9 for orientation, and weights_vector_length
        self.l1 = nn.Linear(D_in, D_h1)
        self.act1 = nn.SiLU() if activation == "silu" else nn.Tanh()
        self.l2 = spectral_norm(nn.Linear(D_h1, D_h2))           
        self.act2 = nn.SiLU() if activation == "silu" else nn.Tanh()
        self.l3 = nn.Linear(D_h2, self.D_out)

    def forward(self, input,deterministic=True):
        out = self.l1(input) # linear function requires the input to be a row tensor
        out = self.act1(out)
        out = self.l2(out)
        out = self.act2(out)
        out = out.squeeze(1)
        y   = self.l3(out)

        pos      = y[..., 0:3]      # (B,3)
        rot      = y[..., 3:12]     # (B,9)
        w_raw    = y[..., 12:]      # (B, D_out-12)

        pos_final = torch.tanh(pos) * 2                             # (B,3)
        rot_final = rot                                             # (B,9)

        wrp   = torch.sigmoid(w_raw[:, 0:3]) * 300 + 10
        wrt   = torch.sigmoid(w_raw[:, 3:6]) * 300 + 10
        wqt   = torch.sigmoid(w_raw[:, 6:7]) *  50 + 10
        gamma = torch.sigmoid(w_raw[:, 7:8]) * 300 + 30
        weights_final = torch.cat([wrp, wrt, wqt, gamma], dim=-1)   # (B,W)

        return torch.cat([pos_final, rot_final, weights_final], dim=-1)

    
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
    

# class network_with_GRU_heads(nn.Module):
#     def __init__(self, D_in, D_h1, D_h2, D_out):
#         super(network_with_GRU_heads, self).__init__()        
#         # D_in : dimension of input layer
#         # D_h  : dimension of hidden layer
#         # D_out: dimension of output layer
#         self.GRU = nn.GRU(input_size=D_in, hidden_size=D_h2,num_layers=1,batch_first=True)
#         self.input_norm=nn.LayerNorm(D_in)
#         self.out_norm = nn.LayerNorm(D_h2)
#         self.l1 = nn.Linear(D_h1, D_h1)
#         self.F1 = nn.ReLU()
#         self.l2 = nn.Linear(D_h1, D_h2)
#         self.F2 = nn.ReLU()
#         # self.l3 = nn.Linear(D_h2, D_out)
        
#         # replace l3 with heads
#         self.position_head = nn.Linear(D_h2, 3)
#         self.orientation_head = nn.Linear(D_h2, 9)
#         self.traverse_time_head = nn.Linear(D_h2, 1)
#         self.weights_head = nn.Linear(D_h2, 3)  

#     def forward(self, input,deterministic=True):
#         out,hidden = self.GRU(input)
#         out = out [:,-1,:]
#         # out = hidden[-1,:,:]
#         out = self.l1(out) # linear function requires the input to be a row tensor
#         out = self.F1(out)
#         out = self.l2(out)
#         out = self.F2(out)
#         out = out.squeeze(1)
        
#         # position head
#         position = self.position_head(out)
#         # orientation head
#         orientation = self.orientation_head(out)
#         # traverse time head
#         traverse_time = self.traverse_time_head(out)
#         # weights head
#         weights = self.weights_head(out)

#         final_out = torch.zeros(input.shape[0],output_size)
    
#         # traverse position x,y
#         final_out[:,0:2]=torch.tanh(position[:,0:2])*3
        
#         # traverse position z
#         final_out[:,2] = torch.sigmoid(position[:,2])*2+0.5
        
#         # orientation
#         final_out[:,3:12]=orientation

#         # wrp
#         final_out[:,-4]=torch.sigmoid(weights[:,-3])*50+10

#         # wrt
#         final_out[:,-3]=torch.sigmoid(weights[:,-2])*20

#         # wqt
#         final_out[:,-2]=torch.sigmoid(weights[:,-1])*20

#         ## t_tra
#         final_out[:,-1]=traverse_time

#         return final_out

    
#     def myloss(self, para, dp, device='cpu'):
#         # convert np.array to tensor
#         Dp = torch.tensor(dp, dtype=torch.float).to(device) # row 2D tensor
#         # loss_nn = torch.matmul(Dp, para)
#         para=para.to(device)
#         loss_nn =torch.trace(torch.matmul(Dp, para.t()))/(Dp.shape[0])
#         return loss_nn # size is 1

#     def loss_close_loop(self, para, dp, device='cpu'):
#         # convert np.array to tensor
#         Dp = torch.tensor(dp, dtype=torch.float).to(device) 
#         para=para.to(device)
        

#         # bxHx1x13 x bxHx13x1 -> 1
#         loss_nn = torch.sum(torch.einsum('bijk,bikj -> b',para,Dp))/(Dp.shape[0]*Dp.shape[1])

#         return loss_nn # size is 1
    
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