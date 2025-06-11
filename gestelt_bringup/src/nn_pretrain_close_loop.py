## this file is for neural network training
import os
import torch
import torch.nn as nn
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R
from Learning_Agile.config import mission_cfg, train_cfg

from Learning_Agile.quad_model import get_gate_points
from gestelt_bringup.src.quad_nn import network,nn_sample, t_output
# Device configuration
device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# device = torch.device('cpu')#
# Hyper-parameters 
input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size']
num_epochs = 1000 
batch_size = 32
learning_rate = 2e-5
current_dir = os.path.dirname(os.path.abspath(__file__))
training_data_folder=os.path.abspath(os.path.join(current_dir, 'Learning_Agile/training_results'))
model_folder=os.path.abspath(os.path.join(training_data_folder, 'pretrain_model'))

if mission_cfg['POSITION_ENCODING']:
    FILE = model_folder+"/NN_close_pretrain_position_encode.pth"
else:
    FILE = model_folder+"/NN_close_pretrain.pth"
# select the seed
torch.manual_seed(train_cfg['model']['seed'])
model = network(input_size, hidden_size, hidden_size,
                weights_vector_length=train_cfg['model']['weights_vector_length'],
                activation=train_cfg['model']['activation']).to(device)

# Loss and optimizer
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)   #, weight_decay=1e-2

def input_cal():
    """This function is used in pretrain, to calculate the input for the neural network 
       in the pretrain, the gate is in the preset position, with no pitch.
                        the drone position is rondomly generated, 
    env_init_set:[0:3] drone position, 
                    [3:6] goal position, 
                    [6] drone yaw angle,
                    [7] gate width, 
                    [8] gate pitch angle, 
                    [8:17] gate rotation matrix
    Returns:
        _type_: _description_
    """
    inputs=np.zeros(input_size)
    env_init_set = nn_sample(PRTRAIN=True)
    
    ## drone initial position
    inputs[0:3] = env_init_set[0:3]/mission_cfg['pos_norm_factor'] # normalize the position to [-1,1]
    
    ## drone initial velocity
    inputs[3:6] = np.array([0,0,0])/mission_cfg['vel_norm_factor']  # static env[3:6] # normalize the velocity to [-1,1]

    ## drone initial orientation: yaw to quaternion
    r = R.from_euler('zyx', np.array([env_init_set[6],0,0]), degrees=True)
    inputs[6:10]= r.as_quat()
    inputs[6:10]=np.roll(inputs[6:10],1)

    inputs[10:13] = (env_init_set[3:6]-env_init_set[0:3])/mission_cfg['pos_norm_factor'] # goal position
    


    ## gate points
    gate_width  = mission_cfg['gate']['width']
    gate_length = mission_cfg['gate']['length']
    gate_center = mission_cfg['mission']['gate_position']
    relative_gate_points=get_gate_points(gate_center,gate_length,gate_width)-env_init_set[0:3]
    inputs[13:25] = relative_gate_points.flatten()/mission_cfg['pos_norm_factor'] # gate points
    # inputs[25:37] = relative_gate_points.flatten()/mission_cfg['pos_norm_factor'] # gate position
    
    return inputs,env_init_set[8:17]

def input_cal_batch(batch_size):
    """
    batch input_cal function
    Returns:
        inputs: [batch_size, input_size]
        gate_rot_matrix: [batch_size, ...]
    """
    inputs = np.zeros((batch_size, input_size))

    gate_rot_matrices = np.zeros((batch_size, 9)) # 假设 gate_rot_matrix 每个是9维, 你可按需调整

    for i in range(batch_size):
        _input, _gate_rot = input_cal()  # 单个样本
        inputs[i] = _input
        gate_rot_matrices[i] = _gate_rot  # 这里假设gate_rot_matrix shape为[9]
    return inputs, gate_rot_matrices

# for epoch in range(num_epochs):
#     for i in range(batch_size):  
        
#         inputs,gate_rot_matrix=input_cal()
#         outputs  = torch.tensor(t_output(inputs, gate_rot_matrix), dtype=torch.float).to(device)
        
#         # Forward pass
#         pre_outputs = model(torch.tensor(inputs, dtype=torch.float).unsqueeze(0).to(device),deterministic=False)[0]
#         # loss = criterion(pre_outputs[:12], outputs[:12])#+criterion(pre_outputs[-1],outputs[-1])
#         weight = torch.tensor([100,100,100,1,1,1,1,1,1,1,1,1], dtype=torch.float).to(device)
#         loss = ((pre_outputs[:12] - outputs[:12])**2 * weight).mean() 
#         # Backward and optimize
#         optimizer.zero_grad()
#         loss.backward()
#         optimizer.step()
        
#         if (i+1) % 100 == 0:
#             print (f'Epoch [{epoch+1}/{num_epochs}], Step [{i+1}/{batch_size}], Loss: {loss.item():.4f}')

for epoch in range(num_epochs):
    # ===== 批量生成输入和目标 =====
    inputs, gate_rot_matrix = input_cal_batch(batch_size)  # [batch, input_size], [batch, 9]

    # 批量计算outputs
    batch_outputs = []
    for i in range(batch_size):
        batch_outputs.append(t_output(inputs[i], gate_rot_matrix[i]))
    batch_outputs = np.stack(batch_outputs, axis=0)  # [batch, output_dim]

    # 转成torch tensor
    inputs_tensor = torch.tensor(inputs, dtype=torch.float, device=device)            # [B, input_size]
    outputs_tensor = torch.tensor(batch_outputs, dtype=torch.float, device=device)    # [B, output_dim]

    # ==== 前向传播 ====
    pre_outputs = model(inputs_tensor, deterministic=False)  # [B, output_dim]

    # ==== loss ====
    # 只用前12维和targets前12维，可以直接这样：
    loss = criterion(pre_outputs[:, :12], outputs_tensor[:, :12])
    # 如果 criterion 是 MSELoss/reduction='mean'，会自动按 batch 做平均。

    # ==== 反向传播 ====
    optimizer.zero_grad()
    loss.backward()
    optimizer.step()

    # ==== 打印 ====
    print(f'Epoch [{epoch+1}/{num_epochs}], Loss: {loss.item():.4f}')


#save model
torch.save(model.state_dict(), FILE)

# Test the model
# In test phase, we don't need to compute gradients (for memory efficiency)
model.eval()
with torch.no_grad():
    n_loss = 0
    for i in range(100):
        
        inputs,gate_rot_matrix=input_cal()
        ## obtain the expected output
        outputs  = torch.tensor(t_output(inputs, gate_rot_matrix), dtype=torch.float).to(device)
        
        # Forward pass
        pre_outputs = model(torch.tensor(inputs, dtype=torch.float).unsqueeze(0).to(device),deterministic=False)[0]
        loss = criterion(pre_outputs, outputs).cpu().data.numpy()
        # max returns (value ,index)
        #_, predicted = torch.max(outputs.data, 1)
        n_loss += loss

    
    print(n_loss/100)

input,_=input_cal()
target = torch.tensor(t_output(input, _), dtype=torch.float).to(device)
print('model input',input)
print('model target',target)
print('model output',model(torch.tensor(input, dtype=torch.float).unsqueeze(0).to(device),deterministic=False)[0])

