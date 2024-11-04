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

## tensorboard logging initialization
log_dir = os.path.join(current_dir, "NN1_training_logs")
current_day = datetime.datetime.now().strftime("%Y-%m-%d")
current_time = datetime.datetime.now().strftime("%H%M%S")
file_dir = os.path.join(log_dir,current_day, f"train-{current_time}-{method_name}-{training_notes}")
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