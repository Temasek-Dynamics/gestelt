
import logging
from datetime import datetime
import os
import yaml
from scipy.spatial.transform import Rotation as R
from solid_geometry import *
import numpy as np
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
    
    gate_euler=R.from_matrix(inputs[8:17].reshape(3,3)).as_euler('zyx')
    gate_pitch=gate_euler[1]*180/np.pi
    writer.add_scalar('gate_pitch', gate_pitch, global_step)
  
    writer.add_scalar('x_tra', outputs[0], global_step)
    writer.add_scalar('y_tra', outputs[1], global_step)
    writer.add_scalar('z_tra', outputs[2], global_step)
    writer.add_scalar('roll_tra', euler_nn[0], global_step)
    writer.add_scalar('pitch_tra', euler_nn[1], global_step)
    writer.add_scalar('yaw_tra', euler_nn[2], global_step)
    writer.add_scalar('t_tra', outputs[-1], global_step)



def log_gradient(writer,gra,global_step):
    writer.add_scalar('drdx', gra[0], global_step)
    writer.add_scalar('drdy', gra[1], global_step)
    writer.add_scalar('drdz', gra[2], global_step)
    # writer.add_scalar('drda', gra[3], global_step)
    # writer.add_scalar('drdb', gra[4], global_step)
    # writer.add_scalar('drdc', gra[5], global_step)
    # writer.add_scalar('drdt', gra[6], global_step)
    writer.add_scalar('step_reward', gra[-1], global_step)