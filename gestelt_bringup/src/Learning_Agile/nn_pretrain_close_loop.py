## this file is for neural network training
import os
import torch
import torch.nn as nn
import numpy as np
from collections import deque
from scipy.spatial.transform import Rotation as R
from config import mission_cfg, train_cfg

from quad_model import get_gate_points
from quad_nn import network,nn_sample, t_output
# Device configuration
device=torch.device('cuda' if torch.cuda.is_available() else 'cpu')
# device = torch.device('cpu')#
# Hyper-parameters 
input_size = train_cfg['model']['input_size'] 
hidden_size = train_cfg['model']['hidden_size']
output_size = train_cfg['model']['output_size']
num_epochs = 3  
batch_size = 1000
learning_rate = 2e-5
current_dir = os.path.dirname(os.path.abspath(__file__))
training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_results'))
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
    Returns:
        _type_: _description_
    """
    inputs=np.zeros(input_size)
    static_env = nn_sample(PRTRAIN=True)
    
    ## drone initial position
    inputs[0:3] = static_env[0:3]/2 # normalize the position to [-1,1]
    
    ## drone initial velocity
    inputs[3:6] = np.array([0,0,0])/5 # static env[3:6] # normalize the velocity to [-1,1]

    ## drone initial orientation: yaw to quaternion
    r = R.from_euler('zyx', np.array([static_env[6],0,0]), degrees=True)
    inputs[6:10]= r.as_quat()
    inputs[6:10]=np.roll(inputs[6:10],1)

    inputs[10:13] = static_env[3:6]/2 # goal position
    


    ## gate points
    gate_width  = mission_cfg['gate']['width']
    gate_length = mission_cfg['gate']['length']
    gate_center = mission_cfg['mission']['gate_position']
    relative_gate_points=get_gate_points(gate_center,gate_length,gate_width)-static_env[0:3]
    inputs[13:25] = relative_gate_points.flatten()/2 # gate points
    inputs[25:37] = relative_gate_points.flatten()/2 # gate position
    
    return inputs,static_env[8:17]

for epoch in range(num_epochs):
    for i in range(batch_size):  
        
        inputs,gate_rot_matrix=input_cal()
        outputs  = torch.tensor(t_output(inputs, gate_rot_matrix), dtype=torch.float).to(device)
        
        # Forward pass
        pre_outputs = model(torch.tensor(inputs, dtype=torch.float).unsqueeze(0).to(device),deterministic=False)[0]
        loss = criterion(pre_outputs[:12], outputs[:12])#+criterion(pre_outputs[-1],outputs[-1])
        
        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        if (i+1) % 100 == 0:
            print (f'Epoch [{epoch+1}/{num_epochs}], Step [{i+1}/{batch_size}], Loss: {loss.item():.4f}')

#save model
torch.save(model.state_dict(), FILE)

# Test the model
# In test phase, we don't need to compute gradients (for memory efficiency)
with torch.no_grad():
    n_loss = 0
    for i in range(100):
        
        inputs=input_cal()
        ## obtain the expected output
        outputs  = torch.tensor(t_output(inputs), dtype=torch.float).to(device)
        
        # Forward pass
        pre_outputs = model(torch.tensor(inputs, dtype=torch.float).unsqueeze(0).to(device))[0]
        loss = criterion(pre_outputs, outputs).cpu().data.numpy()
        # max returns (value ,index)
        #_, predicted = torch.max(outputs.data, 1)
        n_loss += loss

    
    print(n_loss/100)

a=input_cal()
print(a,' ',model(torch.tensor(a, dtype=torch.float).to(device)))

