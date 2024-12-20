
import logging
from datetime import datetime
import os
import yaml
from scipy.spatial.transform import Rotation as R
from solid_geometry import *
import numpy as np
import argparse
# import pandas as pd
class LoggerConfig:
    def __init__(self, log_dir="logs"):
        # 创建日志目录，如果不存在的话
        current_dir = os.path.dirname(os.path.abspath(__file__))
        log_dir = os.path.join(current_dir, log_dir)
        today_log_dir = os.path.join(log_dir, datetime.now().strftime("%Y-%m-%d"))
        os.makedirs(today_log_dir, exist_ok=True)

        # 获取当前时间，并格式化为字符串，用作日志文件名
        current_time = datetime.now().strftime("%H-%M-%S")
        log_filename = os.path.join(today_log_dir, f"logfile_{current_time}.log")
        
        # 配置日志记录器
        logging.basicConfig(
            filename=log_filename,               # 使用当前时间命名的日志文件
            level=logging.INFO,                  # 设置日志级别为INFO
            format='%(asctime)s - %(levelname)s - %(message)s'  # 设置日志格式
        )

        # 输出到控制台的处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.WARNING)
        console_formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(console_formatter)

        # 获取根记录器并添加控制台处理器
        logger = logging.getLogger()
        logger.addHandler(console_handler)

        # 记录日志初始化信息
        logging.info(f"Logging initialized. Log file: {log_filename}")

    def log_yaml_data(self, yaml_data):
        yaml_str=yaml.dump(yaml_data,default_flow_style=False)
        logging.info(f"YAML data:\n{yaml_str}")
        logging.info(f"YAML data logged successfully.")

def log_train_IO(writer,inputs,outputs,global_step):

    R_nn=verify_SVD_casadi(outputs[3:12])
    quat_nn=R.from_matrix(R_nn.reshape(3,3))
    euler_nn=quat_nn.as_euler('zyx', degrees=True)
    
    gate_euler=R.from_matrix(inputs[-9:].reshape(3,3)).as_euler('zyx')
    gate_pitch=gate_euler[1]*180/np.pi
    writer.add_scalar('env/gate_pitch', gate_pitch, global_step)
  
    writer.add_scalar('NN_output/x_tra', outputs[0], global_step)
    writer.add_scalar('NN_output/y_tra', outputs[1], global_step)
    writer.add_scalar('NN_output/z_tra', outputs[2], global_step)
    writer.add_scalar('NN_output/roll_tra', euler_nn[0], global_step)
    writer.add_scalar('NN_output/pitch_tra', euler_nn[1], global_step)
    writer.add_scalar('NN_output/yaw_tra', euler_nn[2], global_step)
    writer.add_scalar('NN_output/wrp', outputs[-2], global_step)
    # writer.add_scalar('max_tra_w', outputs[-4], global_step)
    # writer.add_scalar('wrt', outputs[-3], global_step)
    # writer.add_scalar('wqt', outputs[-2], global_step)
    writer.add_scalar('NN_output/t_tra', outputs[-1], global_step)


    return euler_nn


def log_gradient(writer,gra,reward,global_step):
    ## log as a group named gradient:
    
    writer.add_scalar('gradient/drdx', gra[0], global_step)
    writer.add_scalar('gradient/drdy', gra[1], global_step)
    writer.add_scalar('gradient/drdz', gra[2], global_step)
    drd9D_norm = np.linalg.norm(gra[3:12])
    writer.add_scalar('gradient/drd9D_norm', drd9D_norm, global_step)
    writer.add_scalar('gradient/drdwrp',gra[-3], global_step)
    # writer.add_scalar('drdmax_tra_w',gra[-5], global_step)
    # writer.add_scalar('drdwrt',gra[-4], global_step)
    # writer.add_scalar('drdwqt',gra[-3], global_step)
    writer.add_scalar('gradient/drdt', gra[-2], global_step)
    writer.add_scalar('mean_reward_pre_batch',reward, global_step)


def log_drone_state(writer,drone_state, global_step):
    writer.add_scalar('drone_state/actual_x', drone_state[0], global_step)
    writer.add_scalar('drone_state/actual_y', drone_state[1], global_step)
    writer.add_scalar('drone_state/actual_z', drone_state[2], global_step)

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
    df=pd.DataFrame(data)
    output_file=os.path.join(python_sim_data_folder,"python_sim_mpc_ctl.csv")
    df.to_csv(output_file,index=False)
    
    
def str2bool(value):
    if isinstance(value, bool):
        return value
    if value.lower() in ('yes', 'true', 't', '1'):
        return True
    elif value.lower() in ('no', 'false', 'f', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError(f"Invalid boolean value: {value}")
