## this file is for deep learning
"""
here, DNN1 is trained for static gate, random drone static initial position and orientation
DNN1 should generate single one open loop MPC decision variables. supervised by finite policy gradient
"""

from quad_model import *
from quad_policy import *
from learning_agile_agent import MovingGate,verify_SVD_casadi
from quad_nn import *
from multiprocessing import Process, Array
import yaml
from logger_misc import LoggerConfig,log_gradient,log_train_IO
import logging
from tqdm import tqdm
from torch.utils.tensorboard import SummaryWriter
import datetime
import os

# from nn_train import input_size,output_size
input_size=17
output_size=13

# for multiprocessing, obtain the gradient

"""
finite policy gradient,
output is the decision variables z. [x,y,z,a,b,c,t_traverse]
[a,b,c] is the theta*k, k is the rotation axis
"""
def calc_grad(config_dict,
              planner,
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
    
   
    final_q=R.from_euler('xyz',[0,0,inputs[6]]).as_quat()
    final_q=np.roll(final_q,1)
    
    
    # logging.info('inputs:',inputs)
    planner.init_state_and_mission(
                goal_pos=inputs[3:6],
                goal_ori=final_q.tolist(),
                
                ini_r=inputs[0:3].tolist(),
                ini_v_I = [0.0, 0.0, 0.0], # initial velocity
                ini_q=toQuaternion(inputs[6],[0,0,1]))
   

    # initialize the narrow window
    planner.init_obstacle(moving_gate.gate.gate_point.reshape(12),gate_pitch=moving_gate.gate_init_pitch)


    # receive the decision variables from DNN1, do the MPC, then calculate d_reward/d_z
    gra[:] = planner.sol_gradient(outputs[0:3],
                                 outputs[3:12],
                                 outputs[-1])



if __name__ == '__main__':    
    ###############################################################
    ###----------------- deep learning option-------------------###
    ###############################################################
    # initialization
    # Hyper-parameters 
    options={
    
    'SQP_RTI_OPTION' : False,
    'USE_PREV_SOLVER'  : False,
    'JAX_SVD' : False, # JAX_SVD or CasADi_SVD

    ## BACKWARD required
    'MPC_BACKWARD' : True,
    'ORIGIN_REWARD'  : False,
    'PDP_GRADIENT' : True,
    
    
    ## training option
    'MULTI_CORE'  : False,
    'TRAIN_FROM_CHECKPOINT' : False
    }
    num_cores = 1 #5
    num_epochs = 100 #100
    batch_size = 10 # 100
    step_pre_epoch = 20
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

    if options['ORIGIN_REWARD']:
        options['PDP_GRADIENT']= False
        options['MULTI_CORE']=True
        num_cores = 20
        device = torch.device('cpu')
        batch_size = 100

    if options['PDP_GRADIENT']:
        learning_rate = 1e-4
        method_name = 'PDP'
    else:
        learning_rate = 1e-4
        method_name = 'FD'

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
    saved_folder=os.path.join(model_folder,f"{current_time}-{method_name}-{training_notes}")
    if saved_folder not in os.listdir(model_folder):
        os.mkdir(saved_folder)

    yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
    with open(yaml_file, 'r', encoding='utf-8') as file:
        config_dict = yaml.safe_load(file)
    logger_config.log_yaml_data(config_dict)
    ###############################################################
    ###------------------ generate the solver ------------------###
    ###############################################################
    planner_list = []
    # for each core, generate a quadrotor MPC solver
    for i in range(num_cores):
        planner = PlanFwdBwdWrapper(config_dict, options)
        
        planner_list.append(planner)
    

   
    ###############################################################
    ###------------------ load the model -----------------------###
    ###############################################################
    
    
    if options['TRAIN_FROM_CHECKPOINT']:
        existing_model_folder = os.path.join(model_folder, 'empty')
        FILE = os.path.join(existing_model_folder, "NN1_deep2_48.pth")
        # checkpoint = torch.load(FILE)
        # start_epoch = checkpoint['epoch']   
        # model = checkpoint['model']
        # optimizer = checkpoint['optimizer']
        model = torch.load(FILE)
        start_epoch = 0
        saved_folder = existing_model_folder
        logging.info('load model from %s',FILE)
    else:
        FILE = os.path.join(model_folder, "NN1_pretrain.pth")
        model = torch.load(FILE).to(device)
        start_epoch = 0

    optimizer = torch.optim.Adam(model.parameters(), lr=learning_rate)
    
    # learning rate scheduler
    scheduler = torch.optim.lr_scheduler.StepLR(optimizer, step_size=5, gamma=0.9)
    
    ## prepare logging
    Every_reward = np.zeros((num_epochs,step_pre_epoch))    
    Iteration = []
    Mean_r = []
    

    ###############################################################
    ###------------------ training -----------------------------###
    ###############################################################

    model.train()
    global_step = 0
    for epoch in range(start_epoch,num_epochs):
        
            
        if options['MULTI_CORE']:
            with tqdm(total=int(batch_size/num_cores), desc=f'Epoch {epoch+1}/{num_epochs}', unit='epoch') as pbar:
                
                evalue = 0
                Iteration += [epoch+1]
                for i in range(int(batch_size/num_cores)):
          
                    inputs_list = []
                    outputs_list = []
                    n_out = []
                    grads_list = []
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
                        inputs_list.append(inputs)
                        outputs_list.append(outputs)
                        n_out.append(out)

                        # create a gradient array for assemble all process gradient result
                        grads_list.append(gra)

                    #calculate gradient and loss
                    for j in range(num_cores):
                        
                        
                        p = Process(target=calc_grad,args=(config_dict,
                                                        planner_list[j],
                                                        inputs_list[j],
                                                        n_out[j],
                                                        grads_list[j]))
                        p.start()
                        n_process.append(p)
            
                    for process in n_process:
                        process.join()

                    # Backward and optimize
                    for j in range(num_cores):                
                        outputs = model(torch.tensor(inputs_list[j], dtype=torch.float).to(device))

                        # d_reward/d_z * z
                        # normalize the gradient

                        loss = model.myloss_original(outputs,grads_list[j][0:7])        

                        optimizer.zero_grad()

                        # d_reward/d_z * d_z/d_dnn1
                        loss.backward()
                        optimizer.step()
                        evalue += grads_list[j][7]
                        # Every_reward[epoch,j+num_cores*i]=grads_list[j][7]

                    ## record the gradient and step reward
                    # log_train_IO(writer,inputs_list[0],n_out[0],global_step)
                    log_gradient(writer,grads_list[0][:],global_step)

                    if (i+1)%1 == 0:
                        logging.info (f'Epoch [{epoch+1}/{num_epochs}], Step [{(i+1)*num_cores}/{batch_size}], Reward: {grads_list[0][7]:.4f}')

                    global_step += 1
                    pbar.set_postfix({'step': i+1, 'reward': grads_list[0][7]/batch_size})
                    pbar.update(1)
                
                # mean reward for a epoch
                mean_reward = evalue/step_pre_epoch # evalue/int(batch_size/num_cores)
                Mean_r += [mean_reward]
                logging.info('evaluation: %s ',mean_reward)
                writer.add_scalar('mean_reward', mean_reward, epoch)
                
        
                if (epoch)%2 == 0:
                    torch.save(model, saved_folder+"/NN1_deep2_"+str(epoch)+".pth")
                
                
                np.save(training_data_folder+'/iteration',Iteration)
                np.save(training_data_folder+'/mean_reward',Mean_r)
                # np.save(training_data_folder+'/every_reward',Every_reward)
                writer.close()

        else:
            with tqdm(total=step_pre_epoch, desc=f'Epoch {epoch+1}/{num_epochs}', unit='epoch') as pbar:
                evalue = 0
                Iteration += [epoch+1]
                for i in range(step_pre_epoch):
                    inputs_list = []
                    outputs_list = []
                    grads_list = []
                    

                    # sampling in a batch
                    for k in range(batch_size): 
                        inputs = nn_sample(cur_epoch=epoch)
                        inputs_list.append(inputs)  

                    # forward pass
                    inputs_list = np.array(inputs_list)  # batch_size x 9

                    outputs_list = model(torch.tensor(inputs_list, dtype=torch.float).to(device)) # batch_size x 7
                    np_outputs_list = outputs_list.to('cpu')
                    np_outputs_list = np_outputs_list.data.numpy()

                        
                    
                    ## MPC forward for each batch element
                    for k in range(batch_size):     
                        # create shared variables (shared between processes)
                        gra = Array('d',np.zeros(output_size+1)
                        )
                        
                        # calculate gradient and loss
                        calc_grad(config_dict,
                                planner_list[0],
                                inputs_list[k,:].reshape(input_size),
                                np_outputs_list[k,:].reshape(output_size),
                                gra)
                        
                        
                        # create a gradient array for assemble all process gradient result
                        grads_list.append(gra)

                    ##=== Backward and optimize ===##
                    grads_list = np.array(grads_list)
                    # d_reward/d_z * z
                    loss = model.myloss(outputs_list,grads_list[:,0:output_size],device)        

                    optimizer.zero_grad()

                    # d_reward/d_z * d_z/d_dnn1
                    loss.backward()
                    optimizer.step()
                       
                    ##  record the average reward for a batch
                    evalue += grads_list[:,-1].sum()/batch_size
                    Every_reward[epoch,i]=grads_list[:,-1].sum()/batch_size


                
                    ## record the gradient and step reward
                    log_train_IO(writer,inputs_list[0,:],np_outputs_list[0,:].reshape(output_size),global_step)
                    log_gradient(writer,grads_list[0,:].reshape(output_size+1),global_step)

                    global_step += 1
                    pbar.set_postfix({'step': i+1, 'reward': grads_list[:,-1].sum()/batch_size})
                    pbar.update(1)
                scheduler.step() # update the learning rate
                # mean reward for a epoch
                mean_reward = evalue/step_pre_epoch # evalue/int(batch_size/num_cores)
                Mean_r += [mean_reward]
                logging.info('evaluation: %s ',mean_reward)
                writer.add_scalar('mean_reward', mean_reward, epoch)
                

                if (epoch)%2 == 0:
                    torch.save(model, saved_folder+"/NN1_deep2_"+str(epoch)+".pth")
                
                
                np.save(training_data_folder+'/iteration',Iteration)
                np.save(training_data_folder+'/mean_reward',Mean_r)
                np.save(training_data_folder+'/every_reward',Every_reward)
                writer.close()




# #if MULTI_CORE:
#                     inputs_list = []
#                     outputs_list = []
#                     n_out = []
#                     grads_list = []
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
#                         inputs_list.append(inputs)
#                         outputs_list.append(outputs)
#                         n_out.append(out)

#                         # create a gradient array for assemble all process gradient result
#                         grads_list.append(gra)

#                     #calculate gradient and loss
#                     for j in range(num_cores):
                        
                        
#                         p = Process(target=calc_grad,args=(config_dict,
#                                                         planner_list[j],
#                                                         inputs_list[j],
#                                                         n_out[j],
#                                                         grads_list[j]))
#                         p.start()
#                         n_process.append(p)
            
#                     for process in n_process:
#                         process.join()

#                     # Backward and optimize
#                     for j in range(num_cores):                
#                         outputs = model(inputs_list[j])

#                         # d_reward/d_z * z
#                         # normalize the gradient

#                         loss = model.myloss(outputs,grads_list[j][0:7])        

#                         optimizer.zero_grad()

#                         # d_reward/d_z * d_z/d_dnn1
#                         loss.backward()
#                         optimizer.step()
#                         evalue += grads_list[j][7]
#                         Every_reward[epoch,j+num_cores*i]=grads_list[j][7]
                
#                     if (i+1)%1 == 0:
#                         logging.info (f'Epoch [{epoch+1}/{num_epochs}], Step [{(i+1)*num_cores}/{batch_size}], Reward: {grads_list[0][7]:.4f}')
            
#                 else: