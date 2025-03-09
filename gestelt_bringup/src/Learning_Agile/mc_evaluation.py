import os
from learning_agile_sim import eval_sim_interface,parse_options
from visualization.python_sim_vis import plot_mc_traj
from config import mission_cfg,train_cfg,current_dir

def mc_evaluation(test_num=24,\
                 writer=None,\
                 options=None,\
               model_file=os.path.join(current_dir,mission_cfg['NN_model_name']),\
               global_step=None,
               STAB_TEST=False,
               VIS_BATCH=False):
    """evaluate the success rate every 20 epsiodes, by running the trained model 24 times
    Args:
        model_file (str): the path to the model file
    """
    
    ## run the success evaluation 32 times and return the success rate
    count=0
    options=parse_options()
    
    gate_traj_batch=[]
    state_traj_batch=[]
    failed_batch=[]
    for _ in range(test_num):
        out = eval_sim_interface(mission_cfg,
                                train_cfg,
                                options,
                                model_file,
                                INTRAIN=True,
                                STAB_TEST=STAB_TEST)
        FAILED=out['FAILED']
        state_traj_batch.append(out['state_traj'])
        gate_traj_batch.append(out['gate_traj'])
        failed_batch.append(out['FAILED'])
        if FAILED:
            count+=1
        
        if STAB_TEST:
            print(f"STAB_TEST_FAILED: {FAILED}")
        else:
            print(f"SUCC_TEST_FAILED: {FAILED}")
    success_rate=1-count/test_num

    if writer is not None:
        writer.add_scalar('success_rate', success_rate, global_step)
    print(f"Success rate: {success_rate}")

    if VIS_BATCH:
        print("Visualizing the batch")
        plot_mc_traj(state_traj_batch,gate_traj_batch,failed_batch)
    return success_rate

if __name__ == "__main__":
    mc_evaluation(test_num=24,STAB_TEST=True,VIS_BATCH=True)