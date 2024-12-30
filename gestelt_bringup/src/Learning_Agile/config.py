import os
import datetime
import logging
import yaml


# acquire the current directory
current_dir = os.path.dirname(os.path.abspath(__file__))

## configuration file and model file
conf_folder=os.path.abspath(os.path.join(current_dir, '..', '..','config'))


mission_yaml_file = os.path.join(conf_folder, 'learning_agile_mission.yaml')
training_yaml_file = os.path.join(conf_folder, 'training_params.yaml')

with open(mission_yaml_file, 'r', encoding='utf-8') as file:
    mission_cfg = yaml.safe_load(file)
with open(training_yaml_file, 'r', encoding='utf-8') as file:
    train_cfg = yaml.safe_load(file)


##== training results folder
## day 
## time
## log + model
# training_results_folder=os.path.abspath(os.path.join(current_dir,'training_results'))
# if not os.path.exists(training_results_folder):
#     os.makedirs(training_results_folder)

# ## today:
# current_day = datetime.datetime.now().strftime("%Y-%m-%d")
# current_time = datetime.datetime.now().strftime("%H-%M-%S")
# current_train_folder=os.path.join(training_results_folder,current_day,current_time)
# if not os.path.exists(current_train_folder):
#     os.makedirs(current_train_folder)

# trained_model_folder=os.path.join(current_train_folder,'trained_model')
# if not os.path.exists(trained_model_folder):
#     os.makedirs(trained_model_folder)

# log_folder=os.path.join(current_train_folder,'log')
# if not os.path.exists(log_folder):
#     os.makedirs(log_folder)
# 延迟执行的目录创建逻辑
def setup_training_directories(base_dir='training_results/new_format'):
    """仅在需要时创建训练目录和子目录"""
   
    training_results_folder = os.path.abspath(os.path.join(current_dir, base_dir))
    if not os.path.exists(training_results_folder):
        os.makedirs(training_results_folder)

    # 获取当天日期和当前时间
    current_day = datetime.datetime.now().strftime("%Y-%m-%d")
    current_time = datetime.datetime.now().strftime("%H-%M-%S")
    current_train_folder = os.path.join(training_results_folder, current_day, current_time)

    if not os.path.exists(current_train_folder):
        os.makedirs(current_train_folder)

    # 创建子文件夹
    trained_model_folder = os.path.join(current_train_folder, 'trained_model')
    log_folder = os.path.join(current_train_folder, 'log')

    os.makedirs(trained_model_folder, exist_ok=True)
    os.makedirs(log_folder, exist_ok=True)

    # 返回路径信息
    return {
        "training_results_folder": training_results_folder,
        "current_train_folder": current_train_folder,
        "trained_model_folder": trained_model_folder,
        "log_folder": log_folder,
    }