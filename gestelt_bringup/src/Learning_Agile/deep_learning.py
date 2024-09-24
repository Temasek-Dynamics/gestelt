## this file is for deep learning
"""
here, DNN1 is trained for static gate, random drone static initial position and orientation
DNN1 should generate single one open loop MPC decision variables. supervised by finite policy gradient
"""
from quad_nn import *
from quad_model import *
from quad_policy import *
from learning_agile_agent import MovingGate
from multiprocessing import Process, Array
import yaml
from logger_config import LoggerConfig
import logging
from tqdm import tqdm
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
    # gate_point = np.array([[-inputs[7]/2,0,0.6],[inputs[7]/2,0,0.6],[inputs[7]/2,0,-0.6],[-inputs[7]/2,0,-0.6]])
    # gate = Gate(gate_point)
    # gate_point = gate.rotate_y_out(inputs[8])
    
    moving_gate = MovingGate(inputs,
                            gate_cen_h=0,
                            gate_length=config_dict['gate']['length'])
    
    gate_point = moving_gate.gate.gate_point
    
    final_q=R.from_euler('xyz',[0,0,inputs[6]]).as_quat()
    final_q=np.roll(final_q,1)
    
    
    # logging.info('inputs:',inputs)
    quad_instance.init_state_and_mission(
                goal_pos=inputs[3:6],
                goal_ori=final_q.tolist(),
                
                ini_r=inputs[0:3].tolist(),
                ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                ini_q=toQuaternion(inputs[6],[0,0,1]))
   

    # initialize the narrow window
    quad_instance.init_obstacle(gate_point.reshape(12),gate_pitch=inputs[8])

    # receive the decision variables from DNN1, do the MPC, then calculate d_reward/d_z
    Ulast_value=np.array([2,0.0,0.0,0.0])
    gra[:] = quad_instance.sol_gradient(outputs[0:3].astype(np.float64),
                                        outputs[3:6],
                                        outputs[6],
                                        Ulast_value)

    

if __name__ == '__main__':
    ###############################################################
    ###----------------- deep learning option-------------------###
    ###############################################################
    # initialization
    # Hyper-parameters 
    TRAIN_FROM_CHECKPOINT = False
    PDP_GRADIENT = True
    USE_PREV_SOLVER = False
    MULTI_CORE = False
    ORIGIN_REWARD = True

    num_cores = 1 #5
    num_epochs = 100 #100
    batch_size = 40 # 100
    step_pre_epoch = 20
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    if ORIGIN_REWARD:
        PDP_GRADIENT = False
        MULTI_CORE = True
        num_cores = 20
        device = torch.device('cpu')
        batch_size = 100

    if PDP_GRADIENT:
        learning_rate = 1e-4
    else:
        learning_rate = 1e-4

    training_notes = "Trial_1"

    logger_config=LoggerConfig("NN1_training_logs")
    
    ###############################################################
    ###------------------ load the files -----------------------###
    ###############################################################
    
    # acquire the current directory
    current_dir = os.path.dirname(os.path.abspath(__file__))

    ## logging initialization
    log_dir = os.path.join(current_dir, "NN1_training_logs")
    current_time = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    file_dir = os.path.join(log_dir, f"train-{current_time}-{method_name}-{training_notes}")
    writer = SummaryWriter(log_dir=file_dir)
    logging.info(current_dir)
    
    ## configuration file and model file
    conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))
    training_data_folder=os.path.abspath(os.path.join(current_dir, 'training_data'))
    model_folder=os.path.abspath(os.path.join(training_data_folder, 'NN_model'))
    logging.info(current_dir)
    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    with open(yaml_file, 'r', encoding='utf-8') as file:
        config_dict = yaml.safe_load(file)
    logger_config.log_yaml_data(config_dict)
    ###############################################################
    ###------------------ generate the solver ------------------###
    ###############################################################
    quad_instance_list = []
    # for each core, generate a quadrotor MPC solver
    for i in range(num_cores):
        quad = run_quad(config_dict,  
                    SQP_RTI_OPTION=False,
                    USE_PREV_SOLVER=USE_PREV_SOLVER,
                    PDP_GRADIENT=PDP_GRADIENT,
                    ORIGIN_REWARD=ORIGIN_REWARD)
        
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
        model = torch.load(FILE).to(device)
        start_epoch = 0
    
    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)

    
    ## prepare logging
    Every_reward = np.zeros((num_epochs,batch_size))    
    Iteration = []
    Mean_r = []
    

    ###############################################################
    ###------------------ training -----------------------------###
    ###############################################################

    model.train()
    for epoch in range(start_epoch,num_epochs):
        
            
        if MULTI_CORE:
            with tqdm(total=int(batch_size/num_cores), desc=f'Epoch {epoch+1}/{num_epochs}', unit='epoch') as pbar:
                
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
                        inputs = nn_sample(cur_epoch=epoch)   

                        # forward pass
                        outputs = model(torch.tensor(inputs, dtype=torch.float).to(device))
                        out = outputs.data.numpy()
                        # logging.info(out)
                        
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
                        outputs = model(torch.tensor(n_inputs[j], dtype=torch.float).to(device))

                        # d_reward/d_z * z
                        # normalize the gradient

                        loss = model.myloss_original(outputs,n_gra[j][0:7])        

                        optimizer.zero_grad()

                        # d_reward/d_z * d_z/d_dnn1
                        loss.backward()
                        optimizer.step()
                        evalue += n_gra[j][7]
                        # Every_reward[epoch,j+num_cores*i]=n_gra[j][7]

                    ## record the gradient and step reward
                    log_train_in_ouputs(writer,n_inputs[0],n_out[0],global_step)
                    log_gradient(writer,n_gra[0][:],global_step)

                    if (i+1)%1 == 0:
                        logging.info (f'Epoch [{epoch+1}/{num_epochs}], Step [{(i+1)*num_cores}/{batch_size}], Reward: {n_gra[0][7]:.4f}')

                    global_step += 1
                    pbar.set_postfix({'step': i+1, 'reward': n_gra[0][7]/batch_size})
                    pbar.update(1)
                
                # mean reward for a epoch
                mean_reward = evalue/step_pre_epoch # evalue/int(batch_size/num_cores)
                Mean_r += [mean_reward]
                logging.info('evaluation: %s ',mean_reward)
                writer.add_scalar('mean_reward', mean_reward, epoch)
                
                saved_folder=os.path.join(model_folder,f"{current_time}-{method_name}-{training_notes}")
                if saved_folder not in os.listdir(model_folder):
                    os.mkdir(saved_folder)

                if (epoch)%2 == 0:
                    torch.save(model, saved_folder+"/NN1_deep2_"+str(epoch)+".pth")
                
                
                np.save(training_data_folder+'/iteration',Iteration)
                np.save(training_data_folder+'/mean_reward',Mean_r)
                # np.save(training_data_folder+'/every_reward',Every_reward)
                writer.close()

        else:
            with tqdm(total=step_pre_epoch, desc=f'Epoch {epoch+1}/{num_epochs}', unit='epoch') as pbar:
                for i in range(step_pre_epoch):
                    n_inputs = []
                    n_outputs = []
                    n_out = []
                    n_gra = []
                    n_process = []
                    

                    # sampling in a batch
                    for k in range(batch_size): 
                        inputs = nn_sample(cur_epoch=epoch)
                        n_inputs.append(inputs)  

                    # forward pass
                    n_inputs = np.array(n_inputs)  # batch_size x 9

                    n_outputs = model(torch.tensor(n_inputs, dtype=torch.float).to(device)) # batch_size x 7
                    np_n_outputs = n_outputs.to('cpu')
                    np_n_outputs = np_n_outputs.data.numpy()

                        
                    
                    ## MPC forward for each batch element
                    for k in range(batch_size):     
                        # create shared variables (shared between processes)
                        gra = Array('d',np.zeros(8)
                        )
                        
                        # calculate gradient and loss
                        calc_grad(config_dict,
                                quad_instance_list[0],
                                n_inputs[k,:].reshape(9),
                                np_n_outputs[k,:].reshape(7),
                                gra)
                        
                        
                        # create a gradient array for assemble all process gradient result
                        n_gra.append(gra)

                    ##=== Backward and optimize ===##
                    n_gra = np.array(n_gra)
                    # d_reward/d_z * z
                    loss = model.myloss(n_outputs,n_gra[:,0:7],device)        

                    optimizer.zero_grad()

                    # d_reward/d_z * d_z/d_dnn1
                    loss.backward()
                    optimizer.step()

                    ##  record the average reward for a batch
                    evalue += n_gra[:,7].sum()/batch_size
                    Every_reward[epoch,i]=n_gra[:,7].sum()/batch_size


                
                    ## record the gradient and step reward
                    log_train_in_ouputs(writer,n_inputs[0,:],np_n_outputs[0,:].reshape(7),global_step)
                    log_gradient(writer,n_gra[0,:].reshape(8),global_step)

                    global_step += 1
                    pbar.set_postfix({'step': i+1, 'reward': n_gra[:,7].sum()/batch_size})
                    pbar.update(1)
            
                # mean reward for a epoch
                mean_reward = evalue/step_pre_epoch # evalue/int(batch_size/num_cores)
                Mean_r += [mean_reward]
                logging.info('evaluation: %s ',mean_reward)
                writer.add_scalar('mean_reward', mean_reward, epoch)
                

                saved_folder=os.path.join(model_folder,f"{current_time}-{method_name}-{training_notes}")
                if saved_folder not in os.listdir(model_folder):
                    os.mkdir(saved_folder)

                if (epoch)%2 == 0:
                    torch.save(model, saved_folder+"/NN1_deep2_"+str(epoch)+".pth")
                
                
                np.save(training_data_folder+'/iteration',Iteration)
                np.save(training_data_folder+'/mean_reward',Mean_r)
                np.save(training_data_folder+'/every_reward',Every_reward)
                writer.close()




# #if MULTI_CORE:
#                     n_inputs = []
#                     n_outputs = []
#                     n_out = []
#                     n_gra = []
#                     n_process = []
                    

#                     for _ in range(num_cores):
#                         # sample
#                         inputs = nn_sample(cur_epoch=epoch)   

#                         # forward pass
#                         outputs = model(inputs)
#                         out = outputs.data.numpy()
#                         # logging.info(out)
                        
#                         # create shared variables (shared between processes)
#                         gra = Array('d',np.zeros(8))
                    
#                         # collection
#                         n_inputs.append(inputs)
#                         n_outputs.append(outputs)
#                         n_out.append(out)

#                         # create a gradient array for assemble all process gradient result
#                         n_gra.append(gra)

#                     #calculate gradient and loss
#                     for j in range(num_cores):
                        
                        
#                         p = Process(target=calc_grad,args=(config_dict,
#                                                         quad_instance_list[j],
#                                                         n_inputs[j],
#                                                         n_out[j],
#                                                         n_gra[j]))
#                         p.start()
#                         n_process.append(p)
            
#                     for process in n_process:
#                         process.join()

#                     # Backward and optimize
#                     for j in range(num_cores):                
#                         outputs = model(n_inputs[j])

#                         # d_reward/d_z * z
#                         # normalize the gradient

#                         loss = model.myloss(outputs,n_gra[j][0:7])        

#                         optimizer.zero_grad()

#                         # d_reward/d_z * d_z/d_dnn1
#                         loss.backward()
#                         optimizer.step()
#                         evalue += n_gra[j][7]
#                         Every_reward[epoch,j+num_cores*i]=n_gra[j][7]
                
#                     if (i+1)%1 == 0:
#                         logging.info (f'Epoch [{epoch+1}/{num_epochs}], Step [{(i+1)*num_cores}/{batch_size}], Reward: {n_gra[0][7]:.4f}')
            
#                 else:
