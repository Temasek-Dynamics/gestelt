import argparse
import os

def str2bool(value):
    if isinstance(value, bool):
        return value
    if value.lower() in ('yes', 'true', 't', '1'):
        return True
    elif value.lower() in ('no', 'false', 'f', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")


def save_state_csv(time,drone_state,python_sim_data_folder):
    data={
        "Time":time.transpose(),
        "Position_x":drone_state[:,0].transpose(),
        "Position_y":drone_state[:,1].transpose(),
        "Position_z":drone_state[:,2].transpose(),
        "Velocity_x":drone_state[:,3].transpose(),
        "Velocity_y":drone_state[:,4].transpose(),
        "Velocity_z":drone_state[:,5].transpose(),
        "quat_w":drone_state[:,6].transpose(),
        "quat_x":drone_state[:,7].transpose(),
        "quat_y":drone_state[:,8].transpose(),
        "quat_z":drone_state[:,9].transpose(),
    }
    import pandas as pd
    df=pd.DataFrame(data)
    output_file=os.path.join(python_sim_data_folder,"python_sim_drone_state.csv")
    df.to_csv(output_file,index=False)

def save_mpc_ctl_csv(time,ctl,python_sim_data_folder):
    data={
        "Time":time.transpose(),
        "thrust":ctl[:,0].transpose(),
        "body_rate_x":ctl[:,1].transpose(),
        "body_rate_y":ctl[:,2].transpose(),
        "body_rate_z":ctl[:,3].transpose(),
    }
    import pandas as pd
    df=pd.DataFrame(data)
    output_file=os.path.join(python_sim_data_folder,"python_sim_mpc_ctl.csv")
    df.to_csv(output_file,index=False)


def save_nn_decision_csv(time,nn_decision,python_sim_data_folder):
    data={
        "Time":time[::5].transpose(),
        "trav_position_x":nn_decision[:,0].transpose(),
        "trav_position_y":nn_decision[:,1].transpose(),
        "trav_position_z":nn_decision[:,2].transpose(),
        "trav_euler_x":nn_decision[:,3].transpose(),
        "trav_euler_y":nn_decision[:,4].transpose(),
        "trav_euler_z":nn_decision[:,5].transpose(),
        "wrp":nn_decision[:,-2].transpose(),
        "trav_time":nn_decision[:,-1].transpose()
    }
    import pandas as pd
    df=pd.DataFrame(data)
    output_file=os.path.join(python_sim_data_folder,"python_sim_nn_decision.csv")
    df.to_csv(output_file,index=False)

def load_demo_traj(file_path):
    import numpy as np
    data = np.load(file_path, allow_pickle=True).item()
    t_list = data['t_list']
    p = data['p']
    v = data['v']
    a = data['a']
    q = data['q']
    R = data['R']
    
   
    demo_traj=np.concatenate((p,v,q),axis=1)
    
    # gap 10 points
    demo_traj = demo_traj[::10, :]
    if demo_traj.shape[0]<=21:
        # copy the last row to fill the rest of the array
        last_row = demo_traj[-1, :]
        while demo_traj.shape[0] < 21:
            demo_traj = np.vstack((demo_traj, last_row))
    return demo_traj
        