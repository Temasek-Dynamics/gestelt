#!/usr/bin/env python3
import sys
print("Running with Python:", sys.executable)
import time
import numpy as np
import rospy
import pkg_resources
import yaml
import os
import rospkg
import yaml
from std_msgs.msg import Int8
from gestelt_msgs.msg import CommanderState, ExecTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from enum import Enum
import roslib.packages
from std_msgs.msg import Bool
import tf2_ros
import threading
from std_msgs.msg import Int8
from mavros_msgs.msg import AttitudeTarget
# import tf2_geometry_msgs
# from geometry_msgs.msg import Vector3Stamped
# from geometry_msgs import Posestamped
from orbax.checkpoint import PyTreeCheckpointer
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Imu
import json

import time

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import optax
from flax.training.train_state import TrainState
from collections import deque
import pickle
import threading

from flightning import FLIGHTNING_PATH
from flightning.algos import bptt
from flightning.envs import HoveringStateEnv, rollout
from flightning.envs.wrappers import MinMaxObservationWrapper
from flightning.modules import MLP

from orbax.checkpoint import PyTreeCheckpointer
from scipy.spatial.transform import Rotation as R

# from modules.policy_simple import *

from signal import signal, SIGINT
import random

def handler(signal_received, frame):
    # Handle any cleanup here
    print('SIGINT or CTRL-C detected. Exiting gracefully')

    exit(0)

class DRONESTATE(Enum):
    INIT = 0
    IDLE = 1
    TAKEOFF = 2
    LAND = 3
    HOVER = 4
    MISSION = 5
    E_STOP = 6

class ServerEvent(Enum):
    TAKEOFF_E = 0   
    LAND_E = 1           
    MISSION_E = 2        
    HOVER_E = 3          
    E_STOP_E = 4       
    EMPTY_E = 5      


class TEST_RENDER(object):

    def __init__(self):
        seed = 0
        key = jax.random.key(seed)
        key_init, key_bptt = jax.random.split(key, 2)
        path = FLIGHTNING_PATH + "/../examples/saved_params"
        ckptr = PyTreeCheckpointer()
        params_loaded = ckptr.restore(path)
        drone_path = FLIGHTNING_PATH + "/objects/quadrotor_files/example_quad.yaml"
        dt = 0.02

        self.env = HoveringStateEnv(
            max_steps_in_episode=3 * int(1 / dt),
            dt=dt,
            delay=0.03,
            velocity_std=0.1,
            yaw_scale=1.0,
            pitch_roll_scale=0.1,
            omega_std=0.1,
            drone_path=drone_path,
            reward_sharpness=5.0,
            action_penalty_weight=0.5,
        )

        self.env = MinMaxObservationWrapper(self.env)
        action_dim = self.env.action_space.shape[0]
        obs_dim = self.env.observation_space.shape[0]

        policy_net = MLP(
            [obs_dim, 512, 512, 512, action_dim],
            initial_scale=0.01,
            action_bias=self.env.hovering_action,
        )

        policy_params = policy_net.initialize(key_init)
        self.train_state_obj = TrainState.create(apply_fn=policy_net.apply, params=params_loaded,tx=optax.adam(1e-3))
        
        # self.obs_min = np.array([-5,-5,0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-10,-10,-10,0.018,-6,-6,-4,0.018,-6,-6,-4,0.018,-6,-6,-4])
        # self.obs_max = np.array([15, 15, 5, 1, 1, 1, 1, 1, 1, 1, 1, 1, 10,10,10,6,6,6,4,6,6,6,4,6,6,6,4])
        self.obs_min = self.env._obs_min
        self.obs_max = self.env._obs_max

        self.last_actions_queue = deque()
        self.last_actions_queue.append(self.env.hovering_action)
        self.last_actions_queue.append(self.env.hovering_action)
        self.last_actions_queue.append(self.env.hovering_action)

        self.list_of_pos = []
        self.list_of_vel = []
        self.list_of_rot = []
        self.list_of_act = []




    def policy_trained_new(self,obs):
        return self.train_state_obj.apply_fn(self.train_state_obj.params, obs)
    
    def normalize(self,a, a_min, a_max):
        """
        Maps input a from [a_min, a_max] to [-1, 1]
        """
        return 2 * (a - a_min) / (a_max - a_min) - 1
    
    def evaluate_(self, pos, rotation, vel, mission_command_mode):
        last_actions = np.array(self.last_actions_queue).reshape(-1)
        obs = np.concatenate([pos,rotation.reshape(-1),vel,last_actions])
        obs_norm = self.normalize(obs, self.obs_min, self.obs_max)

        a = self.policy_trained_new(obs_norm)
        if mission_command_mode == 2:
            self.last_actions_queue.popleft()
            # a_norm = self.normalize(a, self.action_min, self.action_max)
            self.last_actions_queue.append(a)
    
        # print(vel)
        # print(rotation)
        # print(a)
        # self.list_of_rot.append(rotation)
        # self.list_of_vel.append(vel)
        # self.list_of_pos.append(pos)
        # self.list_of_act.append(a)
        
        return a, obs, obs_norm
    
    

class NN_POLICY_PLANNER(object):

    def __init__(self, mission_command_mode, policy, inference_timestep, max_angular_rates, warp_jax):
        #Creating subscribers and Publishers
        self.bullet_sim_mutex = threading.Lock()
        self.tfBuffer =  tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.warp_pose_msg = PoseStamped()
        # Drone Goal Point past gate
        self.goal = np.array([3,0,0.5])
        rospy.sleep(1)

        self.max_angular_rates = max_angular_rates
        self.policy = policy
        self.last_pos_time = None
        self.last_odom_time = None
        self.init_pos = np.zeros((7))
        self.init_pos[2] = 1.0
        self.init_pos[1] = 0.0

        self.swarm_mode_pub_ = rospy.Publisher('/traj_server/swarm_command', Int8, queue_size=5)
        self.commander_state_sub_ = rospy.Subscriber("/drone0/traj_server/state",CommanderState, self.commStateCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/pose",PoseStamped, self.poseCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/odom",Odometry, self.odomCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/mode_change", Bool, self.modeChgCb, queue_size = 10)
        self.mission_mode_sub_ = rospy.Subscriber("/traj_server/warp_mission_command", Int8, self.missionModeCb, queue_size = 5)
        self.init_pos_sub_ = rospy.Subscriber("/drone0/jax/init_pose", PoseStamped, self.initPosCb, queue_size = 5)
        self.target_position_sub_ = rospy.Subscriber("/drone0/warp/local_position/target_position", PoseStamped, self.targetPosCb, queue_size = 5)

        self.data_collect_period_pub_ = rospy.Publisher("/traj_server/warp_mission_period_completed", Bool, queue_size=5,latch=False)
        self.data_collect_global_completed_sub_ = rospy.Subscriber("/traj_server/warp_mission_global2local_completed", Bool, self.glob2local_completed, queue_size=5)
        self.ready_to_start_pub_ = rospy.Publisher("/traj_server/ready_to_start", Bool, queue_size=5, latch=False)
        self.glob2local_completed_msg = False

        if warp_jax == 0.0:
            self.warp_drone_pose_pub_ = rospy.Subscriber('/drone0/warp/local_position/pose', PoseStamped, self.warpPoseCB, queue_size=5)
        elif warp_jax == 1.0:
            self.warp_drone_pose_pub_ = rospy.Subscriber('/drone0/mavros/local_position/pose', PoseStamped, self.warpPoseCB, queue_size=5)

        self.warp_drone_odom_sub_ = rospy.Subscriber('/drone0/warp/local_position/odom', Odometry, self.warpOdomCB, queue_size=5)
        self.geom_controller_sub_ = rospy.Subscriber('/drone0/setpoint_raw/attitude', AttitudeTarget, self.geomCB, queue_size=5)
        self.geom_controller_pub_ = rospy.Publisher("/drone0/geom_ctrl", AttitudeTarget, queue_size = 5)
        
        #PVA controller trajectory Publisherlast_actions
        self.pva_traj_pub_ = rospy.Publisher("/drone0/planner_adaptor/exec_trajectory", ExecTrajectory, queue_size = 5)
        self.mission_mode_pub_ = rospy.Publisher("/traj_server/mission_command", Int8, queue_size = 5, latch=False)

        #Listening out for acceleration
        self.lin_acc_sub_ = rospy.Subscriber('/drone0/jax/imu/data', Imu, self.accCB, queue_size=10)
        self.recorder_sub_ = rospy.Subscriber('/traj_server/warp_mission_recorder', Bool, self.recorderCB, queue_size=10)
        
        self.rate = rospy.Rate(0.02)
        self.event_manager = rospy.Timer(rospy.Duration(inference_timestep), self.eventCB)



        #DRONE STATE MACHINE
        self.drone_state = 0
        self.servent_event = 0

        # DRONE STATES
        self.drone_pos = np.zeros((3,1))
        self.drone_qd = np.zeros((3,1))
        self.drone_quat = np.zeros((4,1))

        self.warp_pos = np.zeros((3,1))
        self.warp_quat = np.zeros((4,1))
        self.warp_qd = np.zeros((3,1))
        self.lin_acc = np.zeros((3,1))
        self.data_store = {}
        self.data_store_global = []
        self.data_store_global_counter = 0
        self.record_counter = 0
        self.save_directory = os.path.join(os.path.dirname(FLIGHTNING_PATH), "data")
        files = [f for f in os.listdir(self.save_directory) if os.path.isfile(os.path.join(self.save_directory, f))]
        self.total_files = len(files)
        self.full_save_path = os.path.join(self.save_directory, "data_collected_" + str(self.total_files) + ".pkl")
        self.record_now = False
        self.lock = threading.Lock()

        
        self.mission_command_mode = 1
        self.warp_mission_command_mode = mission_command_mode
        self.attitude_mode_toggle = 0
        self.action = np.zeros(4)
        time.sleep(1)
        self.policy_evaluation_timer = rospy.Timer(rospy.Duration(0.01), self.nn_evaluation)

    def initPosCb(self,msg):
        self.init_pos = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    def glob2local_completed(self,msg):
        self.glob2local_completed_msg = msg.data

    def recorderCB(self,msg):
        if self.record_now == True and msg == False:
            print("Completed data collection. Saving to file NOW")
            self.record_now = msg
            self.saving_to_file()
        if self.record_now == False and msg == True:
            print("Starting data recording")
            self.record_now = msg
        
        # print(self.record_now)
        # if self.record_now == False:
        #     self.record_counter = 0

    def saving_to_file(self):
        with self.lock:
            self.data_store[self.record_counter] = {
                                                    "time": -1,
                                                    "position": -1,
                                                    "rotation_matrix": -1,
                                                    "velocity": -1,
                                                    "action": -1,
                                                    "lin_acc": -1,
                                                    "obs": -1,
                                                    "obs_norm": -1
                                                }
            # with open(self.full_save_path, "wb") as f:
            #     pickle.dump(self.data_store, f)
            
            self.data_store_global.append(self.data_store)
            self.data_store = {}
            self.record_counter = 0
            self.data_store_global_counter += 1
            print("saving completed")

            if self.glob2local_completed_msg == True:
                with open(self.full_save_path, "wb") as f:
                    pickle.dump(self.data_store_global, f)


    def accCB(self,msg):
        with self.lock:
            self.lin_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def commStateCb(self,msg):
        self.drone_state = DRONESTATE[msg.traj_server_state].value

    def missionModeCb(self,msg):
        self.warp_mission_command_mode = msg.data
        print(f"Mission Mode changed to {self.warp_mission_command_mode}")
        if self.warp_mission_command_mode == 2:   
            print("HELLPPPPPP")        
            self.recorderCB(True)

    def publish_mission(self, mission_num):
        mission_idx = Int8()
        mission_idx.data = int(mission_num)
        self.swarm_mode_pub_.publish(mission_idx)

    def poseCb(self, msg):
        self.drone_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.drone_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        rot = R.from_quat(self.drone_quat)
        self.rotation_matrix = rot.as_matrix()
        if self.last_pos_time is not None:
           time_diff = (msg.header.stamp- self.last_pos_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_pos_time = msg.header.stamp
        dist_from_window = np.linalg.norm(self.drone_pos[0] - self.goal[0])
        if np.abs(dist_from_window) < 0.5:
            self.recorderCB(False)
        if np.abs(dist_from_window)<0.3:
            self.warp_mission_command_mode = 1.0
            ##Need to publish completed to the global manager
            completed_msg = Bool()
            completed_msg.data = True
            self.data_collect_period_pub_.publish(completed_msg)

        if np.linalg.norm(self.drone_pos - self.init_pos[:3]) < 0.1:
            ##Ready to start data collection. Need to tell global manager that it is time to execute
            ready_to_start_msg = Bool()
            ready_to_start_msg.data = True
            self.ready_to_start_pub_.publish(ready_to_start_msg)



        self._pose_odom_pub_callback()

    def odomCb(self, msg):
        self.drone_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        if self.last_odom_time is not None:
           time_diff = (msg.header.stamp- self.last_odom_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE ODOM EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_odom_time = msg.header.stamp

    def warpOdomCB(self,msg):

        self.warp_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z ])
        #print(msg.header.stamp)
        # if self.last_odom_time is not None:
        #    time_diff = (msg.header.stamp- self.last_odom_time).to_sec() 
        #    if time_diff > 0.03:
        #        print(f"TIME DIFFERENCE ODOM EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        # self.last_odom_time = msg.header.stamp

    def geomCB(self, msg):
        self.geom_body_rate = np.array([msg.body_rate.x, msg.body_rate.y, msg.body_rate.z])
        self.geom_thrust = msg.thrust

    def warpPoseCB(self,msg):
        self.warp_q = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        
        # if self.last_pos_time is not None:
        #    time_diff = (msg.header.stamp- self.last_pos_time).to_sec() 
        #    if time_diff > 0.03:
        #        print(f"TIME DIFFERENCE EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        # self.last_pos_time = msg.header.stamp

    def targetPosCb(self,msg):
        self.warp_target_pos = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z])
        self.policy.update_target_pos(self.warp_target_pos)

    def eventCB(self, event):
        if self.drone_state == DRONESTATE["IDLE"].value:
            self.publish_mission(ServerEvent["TAKEOFF_E"].value)
        elif self.drone_state == DRONESTATE["HOVER"].value:
            self.publish_mission(ServerEvent["MISSION_E"].value)
        elif self.drone_state == DRONESTATE["MISSION"].value:
            self.executeMission()

    def publishPVA(self):
        pva_traj_msg = ExecTrajectory()
        pva_traj_msg.transform.translation.x = self.init_pos[0]
        pva_traj_msg.transform.translation.y = self.init_pos[1]
        pva_traj_msg.transform.translation.z = self.init_pos[2]
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0
        pva_traj_msg.transform.rotation.w = 1.0
        pva_traj_msg.type_mask = 2048


        #This part is non essential. Merely for debugging purposes
        pva_traj_msg.type_mask = self.attitude_mode_toggle
        pva_traj_msg.throttle = self.action[0]
        pva_traj_msg.angular_rates.angular.x = self.action[1]   #body rate x
        pva_traj_msg.angular_rates.angular.y = self.action[2]     #body rate y
        pva_traj_msg.angular_rates.angular.z = self.action[3]     #body rate z

        #Publish the PVA
        self.pva_traj_pub_.publish(pva_traj_msg)

    def publishVEL(self):
        pva_traj_msg = ExecTrajectory()
        pva_traj_msg.transform.translation.x = 0.0
        pva_traj_msg.transform.translation.y = 0.0
        pva_traj_msg.transform.translation.z = 1.0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0 #0.707
        pva_traj_msg.transform.rotation.w = 1.0 #0.707
        pva_traj_msg.velocity.linear.x = 3.0
        pva_traj_msg.velocity.linear.y = 0.0
        pva_traj_msg.velocity.linear.z = 0.0
        pva_traj_msg.type_mask = 2048

        #Publish the PVA
        self.pva_traj_pub_.publish(pva_traj_msg)

    def publishGeomCtrl(self):
        pva_traj_msg = AttitudeTarget()
        pva_traj_msg.body_rate.x = self.geom_body_rate[0]
        pva_traj_msg.body_rate.y = self.geom_body_rate[1]
        pva_traj_msg.body_rate.z = self.geom_body_rate[2]
        pva_traj_msg.thrust = self.geom_thrust

        #Publish the PVA
        self.geom_controller_pub_.publish(pva_traj_msg)


    def publishATT(self, type_mask, nn_action):
        pva_traj_msg = ExecTrajectory()

        pva_traj_msg.type_mask = type_mask
        # print(self.action)
        pva_traj_msg.throttle = nn_action[0] / 6.95   #0.321

        ### This part will only be taken in by trajectory server if type_mask == 0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.707 
        pva_traj_msg.transform.rotation.w = 0.707

        ### This part will only be taken in by trajectory server if type_mask == 1
        pva_traj_msg.angular_rates.angular.x = nn_action[1]      #body rate x
        pva_traj_msg.angular_rates.angular.y = nn_action[2]   #body rate y
        pva_traj_msg.angular_rates.angular.z = nn_action[3]

        self.pva_traj_pub_.publish(pva_traj_msg)


    def checkNNReadiness(self):
        return not (self.action == 0).all()
    
    def publishMissionCmdMode(self, mode):
        mission_pub_msg = Int8()
        mission_pub_msg.data = mode
        self.mission_mode_pub_.publish(mission_pub_msg)
        if mode == 2:
            print("switched to mission mode 2: ATTITUDE CONTROL")
        elif mode == 4:
            print("switched to mission mode 2: Geom CONTROL")
        else:
            print("switched to mission mode 1: PVA CONTROL")

    def executeMission(self):
        if self.drone_state == DRONESTATE["MISSION"].value:
            if self.mission_command_mode == 1:
                self.publishPVA()
                if self.warp_mission_command_mode == 2:
                    #Check if ready to switch
                    if self.checkNNReadiness():
                        print("me here")
                        self.publishMissionCmdMode(2)
                        self.mission_command_mode = 2
                if self.warp_mission_command_mode == 3:
                    self.publishMissionCmdMode(3)
                    self.mission_command_mode = 3
                if self.warp_mission_command_mode == 4:
                    self.publishMissionCmdMode(4)
                    self.mission_command_mode = 4

                        
            elif self.mission_command_mode == 2:  #This controls the orientation. Attitude and thrust
                if self.checkNNReadiness():
                    self.publishATT(self.attitude_mode_toggle, self.action)


                if self.warp_mission_command_mode == 1:
                    #Check if ready to switch
                    self.publishMissionCmdMode(1)
                    self.mission_command_mode = 1

            elif self.mission_command_mode == 3:
                self.publishVEL()

            elif self.mission_command_mode == 4:
                self.publishGeomCtrl()
                

    def modeChgCb(self, msg):
        if msg.data == True:
            self.attitude_mode_toggle = 1
        elif msg.data == False:
            self.attitude_mode_toggle = 0

    def _pose_odom_pub_callback(self):
        with self.bullet_sim_mutex:
            timestamp = rospy.Time.now()
            
            try:
                trans = self.tfBuffer.lookup_transform("warp", "body", rospy.Time(0))  #"global" + str(i + 1
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("error")
            
            # pose
            self.warp_pose_msg.header.stamp = timestamp
            self.warp_pose_msg.header.frame_id = "warp" #TODO: to change to world add static tf
            self.warp_pose_msg.pose.position.x = trans.transform.translation.x
            self.warp_pose_msg.pose.position.y = trans.transform.translation.y
            self.warp_pose_msg.pose.position.z = trans.transform.translation.z
            self.warp_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            self.warp_pose_msg.pose.orientation.x = trans.transform.rotation.x
            self.warp_pose_msg.pose.orientation.y = trans.transform.rotation.y
            self.warp_pose_msg.pose.orientation.z = trans.transform.rotation.z
            self.warp_pose_msg.pose.orientation.w = trans.transform.rotation.w
            self.warp_quat = np.array([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
            # self.warp_drone_pose_pub_.publish(self.warp_pose_msg)
            # print(trans.transform.translation)


    def nn_evaluation(self, event):
        warp_q = self.warp_q[3:]
        warp_pos = self.warp_q[:3]
        warp_qd = self.warp_qd[3:]
        r = R.from_quat(warp_q)
        rotation_matrix = r.as_matrix()
        self.action, obs, obs_norm = self.policy.evaluate_(warp_pos, rotation_matrix, warp_qd, self.warp_mission_command_mode)

        if self.record_now == True:
            # print("loading data into list")
            if self.record_counter == 0.02:
                print("recording now")
            now_time = rospy.Time.now().to_sec() 
            with self.lock:
                self.data_store[self.record_counter] = {
                                                            "time": now_time,
                                                            "position": warp_pos,
                                                            "rotation_matrix": rotation_matrix,
                                                            "velocity": warp_qd,
                                                            "action": self.action,
                                                            "lin_acc": self.lin_acc,
                                                            "obs": obs,
                                                            "obs_norm": obs_norm
                                                        }
                # print(self.record_counter)
                if self.record_counter == 0:
                    print(warp_pos)

            self.record_counter += 0.02








if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")
    policy_file = "20250627-155147" #0.02 good enough for real drone 20250527-122703   0.05-to test 20250624-181715
    print(f"POLICY PATH IS {policy_file}") 
    full_path = "/home/yanrui/storage/gestelt_ws/src/gestelt/gestelt_navigation/nn_policy/logs/vel_tracking"  
    actual_full_path = os.path.join(full_path, policy_file)
    config_path = os.path.join(actual_full_path,"training_config.yaml")
    full_policy_path = os.path.join(actual_full_path, "policy.pth")

    rospy.init_node("nn_policy_planner2")
    ros_lib = roslib.packages.get_pkg_dir("gestelt_bringup")
    full_config_path = os.path.join(ros_lib, "config/traj_server_default.yaml")
    with open(full_config_path, 'r') as file:
        loaded_params = yaml.safe_load(file)

    to_transform_odom = loaded_params["to_transform_odom"]
    to_transform_policy = loaded_params["to_transform_policy"]
    warp_jax = loaded_params["warp_jax"]

    # with open(config_path, 'r') as file:
    #     config_params = yaml.safe_load(file)

    mission_command_mode = loaded_params["mission_command_mode"]

    position_control = True #config_params["position_control"]
    delta_time = 0.02 #float(config_params["delta_time"])
    max_angular_rate = 3.0 #float(config_params["max_angular_rates"])


    if to_transform_odom != 1.0:
        raise ValueError("to_transform_odom should be 1.0")
    if to_transform_policy != 0.0:
        raise ValueError("to_transform_policy should be 0.0")
    if warp_jax != 1.0:
        raise ValueError("to_transform_policy should be 1.0")

    
      #vel 20250424-161234 #position 20250424-131220, 20250424-161345
    nn_policy = TEST_RENDER() 

    nn_policy_planner = NN_POLICY_PLANNER(mission_command_mode=int(mission_command_mode), policy=nn_policy,
                                          inference_timestep=delta_time, max_angular_rates = max_angular_rate, warp_jax=warp_jax)

    rospy.spin()

    print("done")

