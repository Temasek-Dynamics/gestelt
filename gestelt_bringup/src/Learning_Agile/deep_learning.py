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



# for multiprocessing, obtain the gradient

"""
finite policy gradient,
output is the decision variables z. [x,y,z,a,b,c,t_traverse]
[a,b,c] is the theta*k, k is the rotation axis
"""
def calc_grad(config_dict,
              quad_instance,
              inputs, 
              outputs, 
              gra):
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
    

    final_q=R.from_euler('xyz',[0,0,inputs[6]]).as_quat()
    final_q=np.roll(final_q,1)
    
    
    # print('inputs:',inputs)
    quad_instance.init_state_and_mission(
                goal_pos=inputs[3:6],
                goal_ori=final_q.tolist(),
                
                ini_r=inputs[0:3].tolist(),
                ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                ini_q=toQuaternion(inputs[6],[0,0,1]))
   

    # initialize the narrow window
    quad_instance.init_obstacle(gate_point.reshape(12))

    # print("gate pose: ",inputs[8])
    # print("NN1 output traversal pose pitch: ",outputs[4])
    # print("NN1 output traversal time: ",outputs[6])
    # receive the decision variables from DNN1, do the MPC, then calculate d_reward/d_z
    Ulast_value=np.array([2,0.0,0.0,0.0])
    gra[:] = quad_instance.sol_gradient(outputs[0:3].astype(np.float64),
                                        outputs[3:6],
                                        outputs[6],
                                        Ulast_value,
                                        PDP_GRAIDENT)

    

if __name__ == '__main__':

    ###############################################################
    ###----------------- deep learning option-------------------###
    ###############################################################
    # initialization
    ## deep learning
    # Hyper-parameters 
    TRAIN_FROM_CHECKPOINT = False
    MULTI_CORE = False
    PDP_GRAIDENT = True
    num_epochs = 50 #100
    batch_size = 100 # 100

    if PDP_GRAIDENT:
        learning_rate = 5e-5
    else:
        learning_rate = 1e-4
    num_cores =1 #5

    ###############################################################
    ###------------------ load the files -----------------------###
    ###############################################################
    
    # acquire the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
    model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
    print(current_dir)
    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    with open(yaml_file, 'r', encoding='utf-8') as file:
        config_dict = yaml.safe_load(file)

    ###############################################################
    ###------------------ generate the solver ------------------###
    ###############################################################
    quad_instance_list = []
    # for each core, generate a quadrotor MPC solver
    for i in range(num_cores):
        quad = run_quad(config_dict,  
                    SQP_RTI_OPTION=False,
                    USE_PREV_SOLVER=False)
        
        quad_instance_list.append(quad)
    

   
    ###############################################################
    ###------------------ load the model -----------------------###
    ###############################################################
    
    
    if TRAIN_FROM_CHECKPOINT:
        FILE = os.path.join(model_folder, "NN1_deep2_2.pth")
        # checkpoint = torch.load(FILE)
        # start_epoch = checkpoint['epoch']   
        # model = checkpoint['model']
        # optimizer = checkpoint['optimizer']
        model = torch.load(FILE)
        start_epoch = 0

    else:
        FILE = os.path.join(model_folder, "NN1_pretrain.pth")
        model = torch.load(FILE)
        start_epoch = 0
    
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    
    ## prepare logging
    Every_reward = np.zeros((num_epochs,batch_size))    
    Iteration = []
    Mean_r = []
    

    ###############################################################
    ###------------------ training -----------------------------###
    ###############################################################
    for epoch in range(start_epoch,num_epochs):
    #move = gate1.plane_move()
        evalue = 0
        Iteration += [epoch+1]
        for i in range(int(batch_size/num_cores)):

            if MULTI_CORE:
                n_inputs = []
                n_outputs = []
                n_out = []
                n_gra = []
                n_process = []
                

                for _ in range(num_cores):
                    # sample
                    inputs = nn_sample()
                    
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
                    
                    
                    p = Process(target=calc_grad,args=(config_dict,
                                                    quad_instance_list[j],
                                                    n_inputs[j],
                                                    n_out[j],
                                                    n_gra[j]))
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
                    print (f'Epoch [{epoch+1}/{num_epochs}], Step [{(i+1)*num_cores}/{batch_size}], Reward: {n_gra[0][7]:.4f}')
        
            else:
                # sample
                inputs = nn_sample()
                
                # forward pass
                outputs = model(inputs)
                out = outputs.data.numpy()
                # print(out)
                
                # create shared variables (shared between processes)
                gra = Array('d',np.zeros(8)
                )
                
                # calculate gradient and loss
                calc_grad(config_dict,
                        quad_instance_list[0],
                        inputs,
                        out,
                        gra)
                
                # Backward and optimize
                outputs = model(inputs)

                # d_reward/d_z * z
                loss = model.myloss(outputs,gra[0:7])        

                optimizer.zero_grad()

                # d_reward/d_z * d_z/d_dnn1
                loss.backward()
                optimizer.step()
                evalue += gra[7]
                Every_reward[epoch,i]=gra[7]


            if (i+1)%1 == 0:
                print (f'Epoch [{epoch+1}/{num_epochs}], Step [{(i+1)*num_cores}/{batch_size}], Reward: {gra[7]:.4f}')
        
        # change state
        mean_reward = evalue/batch_size # evalue/int(batch_size/num_cores)
        Mean_r += [mean_reward]
        print('evaluation: ',mean_reward)
        

        if (epoch)%2 == 0:
            torch.save(model, model_folder+"/NN1_deep2_"+str(epoch)+".pth")
    
        np.save(training_data_folder+'/iteration',Iteration)
        np.save(training_data_folder+'/mean_reward',Mean_r)
        np.save(training_data_folder+'/every_reward',Every_reward)