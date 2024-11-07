import os
import datetime
import logging
import yaml
from torch.utils.tensorboard import SummaryWriter

from logger_misc import LoggerConfig,log_gradient,log_train_IO

###############################################################
###------------------ load the files -----------------------###
###############################################################
logger_config=LoggerConfig("NN1_training_logs")

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
