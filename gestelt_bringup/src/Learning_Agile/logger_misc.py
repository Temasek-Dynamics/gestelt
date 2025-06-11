
import logging
from datetime import datetime
import os
import yaml
import numpy as np
import wandb

from scipy.spatial.transform import Rotation as R
from Learning_Agile.geometry.solid_geometry import pitch_from_gate,recover_euler_from_9d

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


    euler_nn=recover_euler_from_9d(outputs,deg_unit=True)
    det_m = np.linalg.det(outputs[3:12].reshape(3,3))
    abs_gate_point=inputs[13:25].reshape(-1,3)+inputs[0:3]
    gate_pitch = pitch_from_gate(abs_gate_point)
    gate_pitch = gate_pitch*180/np.pi
    writer.add_scalar('env/gate_pitch', gate_pitch, global_step)
  
    writer.add_scalar('NN_output/x_tra', outputs[0], global_step)
    writer.add_scalar('NN_output/y_tra', outputs[1], global_step)
    writer.add_scalar('NN_output/z_tra', outputs[2], global_step)
    writer.add_scalar('NN_output/yaw_tra', euler_nn[0], global_step)
    writer.add_scalar('NN_output/pitch_tra', euler_nn[1], global_step)
    writer.add_scalar('NN_output/roll_tra', euler_nn[2], global_step)
    writer.add_scalar('NN_output/determinant_m', det_m, global_step)
    writer.add_scalar('NN_output/tra_throttle', outputs[-5], global_step)
    writer.add_scalar('NN_output/wrp', outputs[-4], global_step)
    writer.add_scalar('NN_output/wrt', outputs[-3], global_step)
    writer.add_scalar('NN_output/wqt', outputs[-2], global_step)
    # writer.add_scalar('max_tra_w', outputs[-4], global_step)
    writer.add_scalar('NN_output/t_tra', outputs[-1], global_step)


    return euler_nn,gate_pitch


def log_gradient(writer,gra,reward,global_step):
    ## log as a group named gradient:
    
    writer.add_scalar('gradient/drdx', gra[0], global_step)
    writer.add_scalar('gradient/drdy', gra[1], global_step)
    writer.add_scalar('gradient/drdz', gra[2], global_step)
    drd9D_norm = np.linalg.norm(gra[3:12])
    writer.add_scalar('gradient/drd9D_norm', drd9D_norm, global_step)
    writer.add_scalar('gradient/drdwthrottle',gra[-6], global_step)
    writer.add_scalar('gradient/drdwrp',gra[-5], global_step)
    # writer.add_scalar('drdmax_tra_w',gra[-5], global_step)
    writer.add_scalar('gradient/drdwrt',gra[-4], global_step)
    writer.add_scalar('gradient/drdwqt',gra[-3], global_step)
    writer.add_scalar('gradient/drdt', gra[-2], global_step)
    writer.add_scalar('mean_penalty_pre_batch',reward, global_step)


def log_drone_state(writer,drone_state,control, global_step):
    writer.add_scalar('drone_state/actual_x', drone_state[0], global_step)
    writer.add_scalar('drone_state/actual_y', drone_state[1], global_step)
    writer.add_scalar('drone_state/actual_z', drone_state[2], global_step)    
    writer.add_scalar('drone_state/thrust', control[0], global_step)
    writer.add_scalar('drone_state/body_rate_x', control[1], global_step)
    writer.add_scalar('drone_state/body_rate_y', control[2], global_step)
    writer.add_scalar('drone_state/body_rate_z', control[3], global_step)



def log_train_IO_wandb(inputs,outputs,global_step):
    euler_nn=recover_euler_from_9d(outputs,deg_unit=True)
    det_m = np.linalg.det(outputs[3:12].reshape(3,3))
    abs_gate_point=inputs[13:25].reshape(-1,3)+inputs[0:3]
    gate_pitch = pitch_from_gate(abs_gate_point)
    gate_pitch = gate_pitch*180/np.pi
    wandb.log({"env/gate_pitch":gate_pitch},step=global_step)
    wandb.log({"NN_output/x_tra":outputs[0],"NN_output/y_tra":outputs[1],"NN_output/z_tra":outputs[2],
                "NN_output/yaw_tra":euler_nn[0],"NN_output/pitch_tra":euler_nn[1],"NN_output/roll_tra":euler_nn[2],
                "NN_output/determinant_m":det_m,
                "NN_output/wrpx":outputs[-8],
                "NN_output/wrpy":outputs[-7],
                "NN_output/wrpz":outputs[-6],
                "NN_output/wrtx":outputs[-5],
                "NN_output/wrty":outputs[-4],
                "NN_output/wrtz":outputs[-3],
                "NN_output/wqt":outputs[-2],
                "NN_output/gamma":outputs[-1]},step=global_step)
    return euler_nn,gate_pitch

def log_gradient_wandb(gra,reward,global_step):
    wandb.log({"gradient/drdx":gra[0],"gradient/drdy":gra[1],"gradient/drdz":gra[2],
                "gradient/drd9D_norm":np.linalg.norm(gra[3:12]),
                # "gradient/drdwthrottle":gra[-6],
                "gradient/drdwrpx":gra[-9],
                "gradient/drdwrpy":gra[-8],
                "gradient/drdwrpz":gra[-7],
                "gradient/drdwrtx":gra[-6],
                "gradient/drdwrty":gra[-5],
                "gradient/drdwrtz":gra[-4],
                "gradient/drdwqt":gra[-3],
                "gradient/drdgamma":gra[-2],
                "mean_penalty_pre_batch":reward},step=global_step)
    
def log_drone_state_wandb(drone_state,control, global_step):
    wandb.log({"drone_state/actual_x":drone_state[0],"drone_state/actual_y":drone_state[1],"drone_state/actual_z":drone_state[2],
                "drone_state/thrust":control[0],
                "drone_state/body_rate_x":control[1],
                "drone_state/body_rate_y":control[2],
                "drone_state/body_rate_z":control[3]},step=global_step)