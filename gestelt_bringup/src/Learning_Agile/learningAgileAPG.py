import numpy as np
from collections import deque
from math import atan,magni
from scipy.spatial.transform import Rotation as R

from learning_agile_sim import LearningAgileSim, Gate
class LearningAgileAPG:
    """
    this class is responsible for running episodes batches in multi-process manner,
    collect gradients in a batch and update the network
    """
    def __init__(self,config_dict,options):
        pass