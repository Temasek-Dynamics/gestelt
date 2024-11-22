## this file is for neural network training
from quad_nn import *
import os
from collections import deque
from scipy.spatial.transform import Rotation as R
from config import mission_cfg
# Device configuration
device = torch.device('cpu')#torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Hyper-parameters 
input_size = 38 # current drone state (10), goal position (3),gate points (4x3), gate position(3),  gate width(1) and orientation(9)
hidden_size = 128 
output_size = 13  # #tra_pos(3), tra_9D_orientation(9), tra_gamma, traversing_time(1)
num_epochs = 3
batch_size = 10000
learning_rate = 2e-5
current_dir = os.path.dirname(os.path.abspath(__file__))
training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
FILE = model_folder+"/NN_close_pretrain.pth"
model = network_with_GRU(input_size, hidden_size, hidden_size,output_size).to(device)
# model = torch.load(FILE)
# Loss and optimizer
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)   #, weight_decay=1e-2

def input_cal():
    inputs=np.zeros(input_size)
    static_env = nn_sample(pretrain=True)
    inputs[0:3] = static_env[0:3]
    inputs[3:6] = np.array([0,0,0])

    ## drone initial orientation: yaw to quaternion
    r = R.from_euler('zyx', np.array([static_env[6],0,0]), degrees=True)
    inputs[6:10]= r.as_quat()
    inputs[6:10]=np.roll(inputs[6:10],1)

    inputs[10:13] = static_env[3:6] # goal position
    


    ## gate points
    gate_cen_h=0
    gate_width = mission_cfg['gate']['width']
    gate_length = mission_cfg['gate']['length']
    inputs[13:25] = np.array([[-gate_length/2, 0, gate_cen_h+gate_width/2],
                              [ gate_length/2, 0, gate_cen_h+gate_width/2],
                              [ gate_length/2, 0, gate_cen_h-gate_width/2],
                              [-gate_length/2, 0, gate_cen_h-gate_width/2]]).flatten() # gate points     
    
    inputs[25:28] = np.array([0,0,0]) # gate position
    inputs[28:38] = static_env[7:17] # gate width and gate orientation
    return inputs

history_state=deque(maxlen=5)
for epoch in range(num_epochs):
    for i in range(batch_size):  
        
        inputs=input_cal()
        for i in range(5):
            history_state.append(inputs)
        full_input=np.array(history_state)
        outputs  = torch.tensor(t_output(full_input), dtype=torch.float).to(device)
        
        # Forward pass
        pre_outputs = model(torch.tensor(full_input, dtype=torch.float).unsqueeze(0).to(device),deterministic=False)[0]
        print("desired_traversing_time",pre_outputs[-1])
        #print(inputs,' ',pre_outputs)
        loss = criterion(pre_outputs, outputs)
        
        # Backward and optimize
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()
        
        if (i+1) % 100 == 0:
            print (f'Epoch [{epoch+1}/{num_epochs}], Step [{i+1}/{batch_size}], Loss: {loss.item():.4f}')

#save model
torch.save(model, FILE)

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

