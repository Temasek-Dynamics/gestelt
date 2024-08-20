## this file is for neural network training
from quad_nn import *
from quad_policy import *
from multiprocessing import Process, Array
import numpy as np
import os
import yaml
# Device configuration
device = torch.device('cpu')
#torch.device('cuda' if torch.cuda.is_available() else 'cpu')

conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
with open(yaml_file, 'r', encoding='utf-8') as file:
    config_dict = yaml.safe_load(file)
current_dir = os.path.dirname(os.path.abspath(__file__))
training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
FILE_INPUT = model_folder+"/NN1_deep2_16.pth"
model_nn1 = torch.load(FILE_INPUT)
# Hyper-parameters 
input_size = 15 
hidden_size = 128 
output_size = 7
num_epochs = 16*200 #16*100
batch_size = config_dict['learning_agile']['horizon']
num_cores = 20
learning_rate = 1e-6

model_nn2 = network(input_size, hidden_size, hidden_size,output_size).to(device)

# Loss and optimizer
criterion = nn.MSELoss()
optimizer = torch.optim.Adam(model_nn2.parameters(), lr=learning_rate)  

def traj(inputs, outputs, state_traj):
    gate_point = np.array([[-inputs[7]/2,0,1],[inputs[7]/2,0,1],[inputs[7]/2,0,-1],[-inputs[7]/2,0,-1]])
    gate1 = gate(gate_point)
    gate_point = gate1.rotate_y_out(inputs[8])

    # quad1 = run_quad(goal_pos=inputs[3:6],ini_r=inputs[0:3].tolist(),ini_q=toQuaternion(inputs[6],[0,0,1]),horizon=20)
    # quad1.init_obstacle(gate_point.reshape(12))

    # quad1.mpc_update(ini_state=quad1.ini_state,tra_pos=outputs[0:3],tra_ang=outputs[3:6],t=outputs[6],Ulast=[0,0,0,0])

    final_q=R.from_euler('xyz',[0,0,inputs[6]]).as_quat()
    final_q=np.roll(final_q,1)
    
    
    # print('inputs:',inputs)
    quad1.init_state_and_mission(
                goal_pos=inputs[3:6],
                goal_ori=final_q.tolist(),
                
                ini_r=inputs[0:3].tolist(),
                ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                ini_q=toQuaternion(inputs[6],[0,0,1]))
    
    Ulast=np.array([2,0.0,0.0,0.0])
    quad1.mpc_update(quad1.ini_state,
                     Ulast,
                     outputs[0:3],
                     outputs[3:6],
                     outputs[6],
                     )
    state_t = np.reshape(quad1.sol1['state_traj_opt'],(batch_size+1)*10)
    state_traj[:] = state_t

if __name__ == '__main__':
     # load the configuration file
    # yaml file dir#
    
    # generate the solver
    quad1 = run_quad(config_dict,  
                    SQP_RTI_OPTION=False, 
                    USE_PREV_SOLVER=False)
    
    for epoch in range(int(num_epochs/num_cores)):
        n_inputs = []
        n_outputs = []
        n_out = []
        n_traj = []
        n_process = []
        for _ in range(num_cores):
            # sample
            inputs = nn_sample()
            # inputs[0:3]=np.array([0,1.8,1.4])
            # inputs[3:6]=np.array([0,-1.8,1.4])
            # forward pass
            outputs = model_nn1(inputs)
            out = outputs.data.numpy()
            # create shared variables
            state_traj = Array('d',np.zeros((batch_size+1)*10))
            # collection
            n_inputs.append(inputs)
            n_outputs.append(outputs)
            n_out.append(out)
            n_traj.append(state_traj)

        for j in range(num_cores):
            p = Process(target=traj,args=(n_inputs[j],n_out[j],n_traj[j]))
            p.start()
            n_process.append(p)

        for process in n_process:
            process.join()

        loss_n = 0
        for k in range(num_cores):
            state_traj = np.reshape(n_traj[k],[(batch_size+1),10])
            for i in range(batch_size):  
        
                inputs = np.zeros(15)
                inputs[0:10] = state_traj[i,:] # in world frame
                inputs[10:13] = n_inputs[k][3:6] # final position
                inputs[13:15] = n_inputs[k][7:9] # gap information, width and pitch angle

                out = np.zeros(7)
                out[0:6] = n_out[k][0:6]
                out[6] = n_out[k][6]-i*0.10
        
                t_out = torch.tensor(out, dtype=torch.float).to(device)
                # Forward pass
                pre_outputs = model_nn2(inputs)
                #print(inputs,' ',pre_outputs)
                loss = criterion(pre_outputs, t_out)
                loss_t = loss.data.numpy()
                # Backward and optimize
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                loss_n += loss_t
        
        print (f'Epoch [{(epoch+1)*num_cores}/{num_epochs}], Loss: {loss_n/(batch_size*num_cores):.4f}')

#save model
torch.save(model_nn2, model_folder+"/NN2_imitate_1.pth")


