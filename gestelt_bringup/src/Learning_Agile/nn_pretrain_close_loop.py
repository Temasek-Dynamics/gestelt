## this file is for neural network training
from quad_nn import *
import os
# Device configuration
device = torch.device('cpu')#torch.device('cuda' if torch.cuda.is_available() else 'cpu')

# Hyper-parameters 
input_size = 18 
hidden_size = 128 
output_size = 7
num_epochs = 3
batch_size = 10000
learning_rate = 2e-5
current_dir = os.path.dirname(os.path.abspath(__file__))
training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
FILE = model_folder+"/NN_close_pretrain.pth"
model = network(input_size, hidden_size, hidden_size,output_size).to(device)
# model = torch.load(FILE)
# Loss and optimizer
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)  

for epoch in range(num_epochs):
    for i in range(batch_size):  
        
        inputs=np.zeros(18)
        static_env = nn_sample(pretrain=True)
        inputs[0:3] = static_env[0:3]
        inputs[3:6] = np.array([0,0,0])
        inputs[6:10]= np.array([1,0,0,0])
        inputs[10:13] = static_env[3:6] # goal position
        inputs[13:16] = np.array([0,0,0]) # gate position
        inputs[16:18] = static_env[6:8] # gate width and gate orientation

        outputs  = torch.tensor(t_output(inputs), dtype=torch.float).to(device)
        
        # Forward pass
        pre_outputs = model(torch.tensor(inputs, dtype=torch.float).to(device))
        print("desired_traversing_time",pre_outputs[6])
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
        inputs = nn_sample(pretrain=True)

        ## obtain the expected output
        outputs  = torch.tensor(t_output(inputs), dtype=torch.float).to(device)
        
        # Forward pass
        pre_outputs = model(inputs)
        loss = criterion(pre_outputs, outputs).cpu().data.numpy()
        # max returns (value ,index)
        #_, predicted = torch.max(outputs.data, 1)
        n_loss += loss

    
    print(n_loss/100)

a = nn_sample()
print(a,' ',model(a))

