## this file is for deep learning
"""
here, DNN1 is trained for static gate, random drone static initial position and orientation
DNN1 should generate single one open loop MPC decision variables. supervised by finite policy gradient
"""
from quad_nn import *
from quad_model import *
from quad_policy import *
from multiprocessing import Process, Array
import yaml
# initialization
# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

# build the path to the subdirectory
subdirectory_path = os.path.join(current_dir, 'Learning_Agile')

## deep learning
# Hyper-parameters 

num_epochs = 10 #100
batch_size = 100 # 100
learning_rate = 1e-4
num_cores =1 #5

# FILE = "nn_pre.pth"
# model = torch.load(FILE)
Every_reward = np.zeros((num_epochs,batch_size))


# for multiprocessing, obtain the gradient

"""
finite policy gradient,
output is the decision variables z. [x,y,z,a,b,c,t_traverse]
[a,b,c] is the theta*k, k is the rotation axis
"""
def calc_grad(config_dict,inputs, outputs, gra):
    """ 

    Args:
        inputs (_type_): DNN1 input, [p_init,p_goal,yaw_init,gate_state]
        outputs (_type_): DNN1 output, [x,y,z,Rx,Ry,Rz,t_traverse]
        gra (_type_): DNN1 Reward/z gradient. [-drdx,-drdy,-drdz,-drda,-drdb,-drdc,-drdt,j])
                        j: reward after MPC plan and execute
    """
    ## gate initialization
    gate_point = np.array([[-inputs[7]/2,0,1],[inputs[7]/2,0,1],[inputs[7]/2,0,-1],[-inputs[7]/2,0,-1]])
    gate1 = gate(gate_point)
    gate_point = gate1.rotate_y_out(inputs[8])

    # quadrotor & MPC initialization
    # quad1 = run_quad(config_dict,goal_pos=inputs[3:6],ini_r=inputs[0:3].tolist(),ini_q=toQuaternion(inputs[6],[0,0,1]),horizon=20)
    
    # quad1 = run_quad(config_dict,  
    #                 USE_PREV_SOLVER=True)
    final_q=R.from_euler('xyz',[0,0,inputs[6]]).as_quat()
    final_q=np.roll(final_q,1)
    
    
    # print('inputs:',inputs)
    quad1.init_state_and_mission(
                goal_pos=inputs[3:6],
                goal_ori=final_q.tolist(),
                
                ini_r=inputs[0:3].tolist(),
                ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                ini_q=toQuaternion(inputs[6],[0,0,1]))
   

    # initialize the narrow window
    quad1.init_obstacle(gate_point.reshape(12))
    quad1.uav1.setDyn(0.002)

    # only allow pitch
    outputs[3]=0
    outputs[5]=0
    # receive the decision variables from DNN1, do the MPC, then calculate d_reward/d_z
    gra[:] = quad1.sol_gradient(outputs[0:3],outputs[3:6],np.clip(outputs[6],1.5,3))

    

if __name__ == '__main__':

    # load the configuration file
    # yaml file dir#
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    print(current_dir)
    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    with open(yaml_file, 'r', encoding='utf-8') as file:
        config_dict = yaml.safe_load(file)

    FILE = os.path.join(current_dir, "nn_pre.pth")
    model = torch.load(FILE)

    # generate the solver
    quad1 = run_quad(config_dict,  
                    USE_PREV_SOLVER=False)
    for k in range(1):

        optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
        Iteration = []
        Mean_r = []
        for epoch in range(num_epochs):
        #move = gate1.plane_move()
            evalue = 0
            Iteration += [epoch+1]
            for i in range(int(batch_size/num_cores)):
                n_inputs = []
                n_outputs = []
                n_out = []
                n_gra = []
                n_process = []
                    
          
                for _ in range(num_cores):
                    # sample
                    inputs = nn_sample()
                    inputs[0:3]=np.array([0,1.8,1.4])
                    inputs[3:6]=np.array([0,-1.8,1.4])
                    
                    # forward pass
                    outputs = model(inputs)
                    out = outputs.data.numpy()
                    # print(out)
                    
                    # create shared variables (shared between processes)
                    gra = Array('d',np.zeros(8))
                
                    # collection
                    n_inputs.append(inputs)
                    n_outputs.append(outputs)
                    n_out.append(out)

                    # create a gradient array for assemble all process gradient result
                    n_gra.append(gra)

                #calculate gradient and loss
                for j in range(num_cores):
                    
                    
                    p = Process(target=calc_grad,args=(config_dict,n_inputs[j],n_out[j],n_gra[j]))
                    p.start()
                    n_process.append(p)
        
                for process in n_process:
                    process.join()

                # Backward and optimize
                for j in range(num_cores):                
                    outputs = model(n_inputs[j])

                    # d_reward/d_z * z
                    loss = model.myloss(outputs,n_gra[j][0:7])        

                    optimizer.zero_grad()

                    # d_reward/d_z * d_z/d_dnn1
                    loss.backward()
                    optimizer.step()
                    evalue += n_gra[j][7]
                    Every_reward[epoch,j+num_cores*i]=n_gra[j][7]
    


                if (i+1)%1 == 0:
                    print (f'Iterate: {k}, Epoch [{epoch+1}/{num_epochs}], Step [{(i+1)*num_cores}/{batch_size}], Reward: {n_gra[0][7]:.4f}')
            # change state
            mean_reward = evalue/batch_size # evalue/int(batch_size/num_cores)
            Mean_r += [mean_reward]
            print('evaluation: ',mean_reward)
            np.save('Iteration',Iteration)
            np.save('Mean_Reward'+str(k),Mean_r)
            np.save('Every_reward'+str(k),Every_reward)
        torch.save(model, "nn_deep2_"+str(k))