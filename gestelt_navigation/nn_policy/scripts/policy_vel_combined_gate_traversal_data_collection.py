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
from scipy.spatial.transform import Rotation as R
import copy
# import tf2_geometry_msgs
# from geometry_msgs.msg import Vector3Stamped
# from geometry_msgs import Posestamped
from plotting_scripts import *
from modules.policy_simple_nwu_global_gate_traversal import *
import pickle
from sensor_msgs.msg import Imu
from dataclasses import asdict, is_dataclass

from signal import signal, SIGINT
import random

FLIGHT_Z = 1.0  # metres — change this to fly at a different height

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

    def __init__(self, policy_path, pc, warp_frame, use_gru=False, include_actions = False):
        self.pc = pc
        self.use_gru = use_gru
        self.include_actions = include_actions
        self.warp_frame = warp_frame
        self.height_offset = FLIGHT_Z - 1.0
        self.h = None
        self.start_msg = False

        gru_action_extra = 4 if (use_gru and include_actions) else 0
        ## Initializing task parameters
        self.init_a = np.zeros((1,4))
        self.init_a[0,0] = 0.32
        target_vel = np.zeros((1, 3))
        self.previous_action = torch.tensor(self.init_a, dtype=torch.float32)
        print(gru_action_extra)

        ## Loading Policy
        # input_dims = 16 + 4 + 3
        input_dims = 16
        if self.pc == True:
            if use_gru == False:
                self.policy = TrackVelGate(input_dim=input_dims)
            else:
                print(f"LOADING POLICY WITH GRU and {gru_action_extra} extra actions")
                self.policy = TrackVelGRU(input_dim=16 + gru_action_extra) if use_gru else TrackVelGate(input_dim=16)
        else:
            self.policy = TrackVelGate(input_dim = 10)

        print(self.policy)
        self.policy.load_state_dict(torch.load(policy_path))
        self.policy.eval()


        if self.warp_frame == 0.0:
            target_pos = np.array([0, 1,0.0]).reshape(1,3)
        elif self.warp_frame == 1.0:
            target_pos = np.array([0, 0, FLIGHT_Z]).reshape(1,3)

        ## Converting numpy to tensor
        self.t_vel = torch.tensor(target_vel, dtype=torch.float32)
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32)
        self.init_pos = torch.tensor(target_pos, dtype=torch.float32)
        

    def reset_h(self, reset_msg):
        if reset_msg == True:
            self.h = None
            self.init_a = np.zeros((1,4))
            self.init_a[0,0] = 0.32
            target_vel = np.zeros((1, 3))
            self.previous_action = torch.tensor(self.init_a, dtype=torch.float32)

    def update_window_info(self, window_pose, window_velocity, window_orientation):
        self.window_position = torch.tensor(window_pose, dtype=torch.float32)
        self.window_velocity = torch.tensor(window_velocity, dtype=torch.float32)
        self.window_quaternion = torch.tensor(window_orientation, dtype=torch.float32)
    
    def update_init_pos_drone(self, init_pos):
        init_pos = np.zeros((1,3))
        init_pos[:,0] = -2
        init_pos[:,1] = 0
        init_pos[:,2] = 1.0
        self.init_pos_numpy = init_pos
        self.init_pos_t = torch.tensor(self.init_pos_numpy, dtype=torch.float32)
    

    def evaluate_(self, pos, att, qd):
        start_time = time.time()
        ## Concatenating observations
        if self.pc == True: 
            # diff_pos = self.t_pos - pos
            # x = torch.cat((self.init_pos_t, pos, att, qd, self.window_velocity, self.window_quaternion), dim=1)
            pos[:,2] = pos[:,2] - self.height_offset  # shift to NN's trained reference frame
            diff_pos = torch.tensor([1.5,0,1.0]) - pos
            pos_offset = pos
            x = torch.cat((diff_pos, pos, att, qd), dim=1)
            if self.include_actions:
                x = torch.cat([x, self.previous_action], dim=1)

        else:
            vel = qd[:,3:]
            angvel = qd[:,:3]
            diff_vel = self.t_vel - vel
            x = torch.cat((att, angvel, diff_vel), dim=1)
        
        ## Evaluating policy
        if self.use_gru == False:
            a = self.policy(x)
        else:
            if self.start_msg == True:
                # print("true")
                a, self.h = self.policy(x, self.h)
                self.prev_action = a
            else:
                # print("false")
                a, _ = self.policy(x, self.h)
        
        self.previous_action = a
        end_time = time.time()
        # print(f"Time taken: {end_time - start_time:.4f} seconds")
        return a
    
    def vector_to_line(self,P, A, d):
        d_unit = d / torch.norm(d, dim=-1, keepdim=True)  # Normalize direction
        AP = P - A
        proj_len = torch.sum(AP * d_unit, dim=-1, keepdim=True)  # scalar projection
        proj = proj_len * d_unit  # vector projection
        Q = A + proj  # Closest point on the line
        return Q - P  # Vector from P to closest point
    
    def update_target_pos(self,P):
        target_pos = P
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32)

    def quaternion_loss(self,y_true, y_pred):
        """
        Computes the angular difference between two quaternions in radians.
        Args:
            y_true: Tensor of shape (batch_size, 4), ground truth quaternions.
            y_pred: Tensor of shape (batch_size, 4), predicted quaternions.
        Returns:
            Scalar tensor: Mean angular difference across the batch.
        """
        # Normalize quaternions to ensure they are unit quaternions
        y_true = F.normalize(y_true, dim=-1)
        y_pred = F.normalize(y_pred, dim=-1)

        # y_pred = y_pred[:,3:]

        # Compute the dot product
        dot_product = torch.sum(y_true * y_pred, dim=-1)  # Shape: (batch_size,)

        # Clamp the dot product to avoid invalid values due to numerical errors
        dot_product_clamped = torch.clamp(dot_product, -1.0 + 1e-6, 1.0 - 1e-6)

        # Compute angular difference (in radians)
        angular_diff = 2 * torch.acos(torch.abs(dot_product_clamped))

        # Return the mean angular difference
        return angular_diff.mean(), angular_diff
    

class NN_POLICY_PLANNER(object):

    def __init__(self, mission_command_mode, policy, inference_timestep, max_angular_rates, warp_jax,
                 to_transform_odom, to_transform_policy, config_param=None):
        #Creating subscribers and Publishers
        self.individual_recording = False
        self.bullet_sim_mutex = threading.Lock()
        self.tfBuffer =  tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.warp_pose_msg = PoseStamped()
        #Create a policy field
        self.policy = policy
        rospy.sleep(1)

        ##Define initial window location
        self.window_position = np.zeros((1,3))
        self.window_position[:,0] = 1.5
        self.window_position[:,1] = 0.0
        self.window_position[:,2] = FLIGHT_Z
        self.window_velocity = np.zeros((1,3))
        self.window_velocity[:,0] = 3.0 
        # window_quaternion = np.zereos((1,4))
        self.window_degrees = np.ones((1,)) * 30
        self.window_quaternion = self.convert_window_degrees_to_quaternion_vector(self.window_degrees)
        #Update policy with the window information
        self.policy.update_window_info(self.window_position, self.window_velocity, self.window_quaternion)

        ## Saving list for plotting
        self.position_list = []
        self.velocity_list = []
        self.attitude_list = []


        ## Define initial starting location of the drone
        init_pos = np.zeros((1,3))
        init_pos[:,0] = -2
        init_pos[:,1] = 0
        init_pos[:,2] = FLIGHT_Z
        self.init_pos_numpy = init_pos
        self.init_quat = self.update_init_orientation_drone(self.init_pos_numpy, self.window_position)
        self.curr_init_pose = PoseStamped()
        self.curr_init_pose.pose.position.x = copy.deepcopy(init_pos[:,0])
        self.curr_init_pose.pose.position.y = copy.deepcopy(init_pos[:,1])
        self.curr_init_pose.pose.position.z = copy.deepcopy(init_pos[:,2])
        self.curr_init_pose.pose.orientation.x = copy.deepcopy(self.init_quat[:,0])
        self.curr_init_pose.pose.orientation.y = copy.deepcopy(self.init_quat[:,1])
        self.curr_init_pose.pose.orientation.z = copy.deepcopy(self.init_quat[:,2])
        self.curr_init_pose.pose.orientation.w = copy.deepcopy(self.init_quat[:,3])
        #Update policy with initial starting location of the drone
        self.policy.update_init_pos_drone(self.init_pos_numpy)

        self.max_angular_rates = max_angular_rates
        self.last_pos_time = None
        self.last_odom_time = None

        self.warp_jax = warp_jax
        self.to_transform_odom = to_transform_odom
        self.to_transform_policy = to_transform_policy
        

        self.swarm_mode_pub_ = rospy.Publisher('/traj_server/swarm_command', Int8, queue_size=5)
        self.commander_state_sub_ = rospy.Subscriber("/drone0/traj_server/state",CommanderState, self.commStateCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/pose",PoseStamped, self.poseCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/odom",Odometry, self.odomCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/mode_change", Bool, self.modeChgCb, queue_size = 10)
        self.mission_mode_sub_ = rospy.Subscriber("/traj_server/warp_mission_command", Int8, self.missionModeCb, queue_size = 5)
        self.target_position_sub_ = rospy.Subscriber("/drone0/warp/local_position/target_position", PoseStamped, self.targetPosCb, queue_size = 5)
        
        self.warp_drone_pose_pub_ = rospy.Subscriber('/drone0/warp/local_position/pose', PoseStamped, self.warpPoseCB, queue_size=5)
        self.reset_drone_init_sub_ = rospy.Subscriber('/drone0/warp/local_position/init_pose', PoseStamped, self.initPoseCB, queue_size=5)
        self.fixed_reset_init_sub_ = rospy.Subscriber('/drone0/warp/local_position/reset_pose', Bool, self.resetPoseCB, queue_size=5)
        self.warp_drone_odom_sub_ = rospy.Subscriber('/drone0/warp/local_position/odom', Odometry, self.warpOdomCB, queue_size=5)
        self.geom_controller_sub_ = rospy.Subscriber('/drone0/setpoint_raw/attitude', AttitudeTarget, self.geomCB, queue_size=5)
        self.geom_controller_pub_ = rospy.Publisher("/drone0/geom_ctrl", AttitudeTarget, queue_size = 5)

        # Data collection trigger subscriber (missing previously)
        self.recorder_sub_ = rospy.Subscriber('/traj_server/warp_mission_recorder', Bool, self.recorderCB, queue_size=10)

        # Initialize nwu_odom to zeros (missing previously)
        self.nwu_odom = np.zeros(6)
        
        #PVA controller trajectory Publisher
        self.pva_traj_pub_ = rospy.Publisher("/drone0/planner_adaptor/exec_trajectory", ExecTrajectory, queue_size = 5)
        self.mission_mode_pub_ = rospy.Publisher("/traj_server/mission_command", Int8, queue_size = 5, latch=False)
        

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

        self.data_store = {
            i: {
                "time_stamp": [],
                "position": [],
                "velocity": [],
                "rotation": [],
                "omega": [],
                "action": [],
            }
            for i in range(1)
        }
        self.record_counter = 0
        self.awaiting_restart = False  # True while flying back to init between episodes

        self.save_directory = os.path.join(rospack.get_path('nn_policy'), "warp_data")
        os.makedirs(self.save_directory, exist_ok=True)     
        folders = [
            f for f in os.listdir(self.save_directory)
            if os.path.isdir(os.path.join(self.save_directory, f))
        ]

        self.total_files = len(folders)
        full_save_path = os.path.join(self.save_directory, str(self.total_files))
        os.makedirs(full_save_path, exist_ok=True)     
        self.full_save_path = os.path.join(full_save_path, "data.pkl")

        self.full_save_config_path = os.path.join(full_save_path, "training_config.yaml")
        self.record_now = False
        self.pending_save = False
        self.lock = threading.Lock()
        self.inference_timestep = inference_timestep
        self.config_param = config_param

        self.mission_command_mode = 1
        self.warp_mission_command_mode = mission_command_mode
        self.attitude_mode_toggle = 0
        self.action = np.zeros((1,4))
        time.sleep(1)
        self.policy_evaluation_timer = rospy.Timer(rospy.Duration(0.01), self.nn_evaluation)

    def recorderCB(self, msg):
        if self.record_now == True and msg.data == False:
            # Stop signal: cancel any pending restart
            self.awaiting_restart = False
            self.record_now = False
            self.record_counter = 0
            if self.individual_recording:
                # Mid-episode: let the gate-passing logic finish the episode then save
                print("Stop signal received. Waiting for current episode to finish before saving.")
                self.pending_save = True
            else:
                # Between episodes: safe to save immediately
                print("Recording stopped. Saving to file NOW.")
                self.saving_to_file()
            return

        if self.record_now == False and msg.data == True:
            # Start signal: clear buffer, randomize init, fly to it
            print("Starting data recording cycle")
            self.data_store = {
                i: {
                    "time_stamp": [],
                    "position": [],
                    "velocity": [],
                    "rotation": [],
                    "omega": [],
                    "action": [],
                }
                for i in range(1)
            }
            self.record_now = True
            self.record_counter = 0
            self.randomize_init_pos()
            self.mission_command_mode = 1
            self.warp_mission_command_mode = 1
            self.policy.start_msg = False
            self.policy.reset_h(True)
            self.individual_recording = False
            self.awaiting_restart = True
    
    def saving_to_file(self):
        with self.lock:
            with open(self.full_save_path, "wb") as f:
                pickle.dump(self.data_store, f)
            
            config = self.config_param
            if is_dataclass(config):
                config = asdict(config)  # ensure YAML-serializable

            with open(self.full_save_config_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(
                    config,
                    f,
                    sort_keys=False,
                    default_flow_style=False,
                    allow_unicode=True
                )


    def save_args_to_yaml(args, output_path='saved_config.yaml'):
        # Convert Namespace to dict
        args_dict = vars(args)

        # Save to YAML
        with open(output_path, 'w') as f:
            yaml.dump(args_dict, f)

    def randomize_init_pos(self):
        """Pick a random start position behind the gate and update the drone's init pose."""
        x = random.uniform(-3.5, -1.5)
        y = random.uniform(-1.0,  1.0)
        z = random.uniform(FLIGHT_Z - 0.5, FLIGHT_Z + 0.5)
        new_pos = np.array([[x, y, z]])
        self.init_pos_numpy = new_pos
        self.init_quat = self.update_init_orientation_drone(new_pos, self.window_position)
        self.policy.update_init_pos_drone(new_pos)
        # Keep curr_init_pose in sync for resetPoseCB
        self.curr_init_pose.pose.position.x = float(new_pos[0, 0])
        self.curr_init_pose.pose.position.y = float(new_pos[0, 1])
        self.curr_init_pose.pose.position.z = float(new_pos[0, 2])
        self.curr_init_pose.pose.orientation.x = float(self.init_quat[0, 0])
        self.curr_init_pose.pose.orientation.y = float(self.init_quat[0, 1])
        self.curr_init_pose.pose.orientation.z = float(self.init_quat[0, 2])
        self.curr_init_pose.pose.orientation.w = float(self.init_quat[0, 3])
        print(f"New init position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def initPoseCB(self,msg):
        self.init_pos_numpy[:,0] = msg.pose.position.x 
        self.init_pos_numpy[:,1] = msg.pose.position.y
        self.init_pos_numpy[:,2] = msg.pose.position.z
        self.init_quat = self.update_init_orientation_drone(self.init_pos_numpy, self.window_position)

    def resetPoseCB(self,msg):
        print(msg)
        if msg.data == True:
            print(self.curr_init_pose)
            self.initPoseCB(self.curr_init_pose)

    def commStateCb(self,msg):
        self.drone_state = DRONESTATE[msg.traj_server_state].value

    def update_init_orientation_drone(self, init_pos, window_center_array):
        quat_init = self.quaternion_facing_goal(init_pos, window_center_array)
        return quat_init

    def convert_window_degrees_to_quaternion_vector(self,degrees):
        degrees = np.asarray(degrees).reshape(-1)
        angles = degrees * np.pi / 180.0

        N = angles.shape[0]
        window_R = np.zeros((N, 3, 3))

        window_R[:, 0, 0] = 1
        window_R[:, 1, 1] = np.cos(angles)
        window_R[:, 1, 2] = -np.sin(angles)
        window_R[:, 2, 1] = np.sin(angles)
        window_R[:, 2, 2] = np.cos(angles)

        rot = R.from_matrix(window_R)
        window_q = rot.as_quat()   # (N, 4)

        return window_q

    def quaternion_facing_goal(self, pos, goal):
        dx = goal[:,0] - pos[:,0]
        dy = goal[:,1] - pos[:,1]

        yaw = np.arctan2(dy, dx)

        qw = np.cos(yaw / 2.0)
        qx = np.zeros((qw.shape[0]))
        qy = np.zeros((qw.shape[0]))
        qz = np.sin(yaw / 2.0)

        quat_init = np.zeros((dx.shape[0],4))
        quat_init[:,0] = qx
        quat_init[:,1] = qy
        quat_init[:,2] = qz
        quat_init[:,3] = qw

        return quat_init

    def missionModeCb(self,msg):
        self.warp_mission_command_mode = msg.data
        print(f"Mission Mode changed to {self.warp_mission_command_mode}")
        if msg.data == 2:
            self.policy.start_msg = True
            # Only set individual_recording if a recording cycle is active;
            # otherwise _settle_and_start_nn_cb handles it automatically
            if self.record_now:
                self.individual_recording = True

    def publish_mission(self, mission_num):
        mission_idx = Int8()
        mission_idx.data = int(mission_num)
        self.swarm_mode_pub_.publish(mission_idx)

    def poseCb(self, msg):
        self.drone_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.drone_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        if self.to_transform_odom == 0.0:
            self.warp_q = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.position.z, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        # Checking timing 
        if self.last_pos_time is not None:
           time_diff = (msg.header.stamp- self.last_pos_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_pos_time = msg.header.stamp

        # self._pose_odom_pub_callback()

    def odomCb(self, msg):
        self.drone_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
        if self.to_transform_odom == 0.0:
            self.warp_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z ])
        if self.last_odom_time is not None:
           time_diff = (msg.header.stamp- self.last_odom_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE ODOM EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_odom_time = msg.header.stamp
        self.nwu_odom = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z ])

    def warpOdomCB(self,msg):
        if self.to_transform_odom == 1.0:
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
        if self.to_transform_odom == 1.0:
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
        pva_traj_msg.transform.translation.x = self.init_pos_numpy[:,0]
        pva_traj_msg.transform.translation.y = self.init_pos_numpy[:,1]
        pva_traj_msg.transform.translation.z = self.init_pos_numpy[:,2]
        pva_traj_msg.transform.rotation.x = self.init_quat[:,0]
        pva_traj_msg.transform.rotation.y = self.init_quat[:,1]
        pva_traj_msg.transform.rotation.z = self.init_quat[:,2]
        pva_traj_msg.transform.rotation.w = self.init_quat[:,3]
        pva_traj_msg.type_mask = 2048


        #This part is non essential. Merely for debugging purposes
        pva_traj_msg.type_mask = self.attitude_mode_toggle
        pva_traj_msg.throttle = self.action[0,0]
        pva_traj_msg.angular_rates.angular.x = self.action[0,1]   #body rate x
        pva_traj_msg.angular_rates.angular.y = self.action[0,2]     #body rate y
        pva_traj_msg.angular_rates.angular.z = self.action[0,3]     #body rate z

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
        pva_traj_msg.velocity.linear.x = 1.0
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
        pva_traj_msg.throttle = nn_action[0,0]  #0.321

        ### This part will only be taken in by trajectory server if type_mask == 0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.707 
        pva_traj_msg.transform.rotation.w = 0.707

        ### This part will only be taken in by trajectory server if type_mask == 1
        pva_traj_msg.angular_rates.angular.x = nn_action[0,1] * self.max_angular_rates     #body rate x
        pva_traj_msg.angular_rates.angular.y = nn_action[0,2] * self.max_angular_rates    #body rate y
        pva_traj_msg.angular_rates.angular.z = nn_action[0,3] * self.max_angular_rates

        self.pva_traj_pub_.publish(pva_traj_msg)

        warp_q = self.warp_q[3:]
        warp_pos = torch.Tensor(self.warp_q[:3]).unsqueeze(0)
        warp_q = torch.Tensor(warp_q).unsqueeze(0)
        warp_qd = torch.Tensor(self.warp_qd).unsqueeze(0)
        rotation_quat = warp_q[:,3:]

        if self.individual_recording == True:
            now_time = rospy.Time.now().to_sec()
            with self.lock:
                if warp_pos[0, 0] < self.window_position[0, 0] + 0.05:
                    self.data_store[0]["time_stamp"].append(now_time)
                    self.data_store[0]["position"].append(warp_pos.squeeze(0).detach().cpu().numpy())
                    # self.data_store[0]["velocity"].append(warp_qd[:,3:].squeeze(0).detach().cpu().numpy())
                    self.data_store[0]["velocity"].append(self.nwu_odom[3:])
                    self.data_store[0]["rotation"].append(warp_q.squeeze(0).detach().cpu().numpy())
                    # self.data_store[0]["omega"].append(warp_qd[:,:3].squeeze(0).detach().cpu().numpy())
                    self.data_store[0]["omega"].append(self.nwu_odom[:3])
                    self.data_store[0]["action"].append(nn_action.squeeze(0).detach().cpu().numpy())

                self.record_counter += self.inference_timestep


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

    def _settle_and_start_nn_cb(self, event):
        """Called by a one-shot timer after the drone has settled at the init position."""
        if not self.record_now:
            return
        print("Settle complete. Starting NN recording.")
        self.individual_recording = True
        self.policy.start_msg = True
        self.warp_mission_command_mode = 2

    def _append_separator_and_restart_cb(self, event):
        """0.1 s after gate pass: flush stray data and write the None separator,
        then wait for the drone to stabilize before flying to the next init."""
        with self.lock:
            for key in self.data_store[0]:
                self.data_store[0][key].append(None)
        print("Episode separator written. Stabilizing before next episode...")
        rospy.Timer(rospy.Duration(2.0), self._go_to_next_init_cb, oneshot=True)

    def _go_to_next_init_cb(self, event):
        """Called after the stabilization delay — randomize init and start flying."""
        if not self.record_now:
            return
        self.randomize_init_pos()
        self.awaiting_restart = True
        print("Flying to next random init position.")

    def _flush_and_save_cb(self, event):
        """Called by a one-shot timer after the final episode finishes.
        Waits for any in-flight data to settle before saving."""
        print("Final episode finished. Saving to file NOW.")
        self.pending_save = False
        self.saving_to_file()

    def executeMission(self):
        if self.drone_state == DRONESTATE["MISSION"].value:
            if self.mission_command_mode == 1:
                self.publishPVA()
                # During a recording cycle, wait for drone to arrive at init,
                # then start a settle timer before switching to NN
                if self.awaiting_restart and self.record_now:
                    dist = np.linalg.norm(self.drone_pos - self.init_pos_numpy[0])
                    if dist < 0.1:
                        self.awaiting_restart = False
                        print("Arrived at init. Settling for 2 s before starting NN.")
                        rospy.Timer(rospy.Duration(2.0), self._settle_and_start_nn_cb, oneshot=True)
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
                    ## Check if it has passed through the gate
                
                if (self.drone_pos[0] - self.window_position[:,0]) > 0.1:
                    #Means drone has passed gate. Switch back to position control.
                    print("Drone passed gate. Switching back to position control.")
                    # Disable recording immediately so no stray samples follow
                    self.individual_recording = False
                    # Hold at current position first to avoid a sudden setpoint jump
                    self.init_pos_numpy = self.drone_pos.reshape(1, 3).copy()
                    self.init_pos_numpy[0,0] += 1.0  # small forward step to avoid hovering on the gate line
                    self.init_quat = self.drone_quat.reshape(1, 4).copy()
                    self.mission_command_mode = 1
                    self.publishMissionCmdMode(1)
                    self.warp_mission_command_mode = 1
                    self.policy.start_msg = False
                    self.policy.reset_h(True)

                    if self.record_now:
                        # After a short delay (to flush any in-flight data), write
                        # the None separator and kick off the next episode
                        rospy.Timer(rospy.Duration(0.5),
                                    self._append_separator_and_restart_cb, oneshot=True)
                    elif self.pending_save:
                        # Stop was requested mid-episode; save once data has settled
                        rospy.Timer(rospy.Duration(0.1),
                                    self._flush_and_save_cb, oneshot=True)

                    self.position_array = np.array(self.position_list)
                    self.velocity_array = np.array(self.velocity_list)
                    self.attitude_array = np.array(self.attitude_list)
                    if len(self.position_array) > 0:
                        print(self.position_array[:,None,:].shape)
                        plot_spatial_plots(self.position_array[:,None,:])
                        gate_world = gate_geometry(self.window_degrees[0])
                        plot_gate_travesal(self.position_array[:,None,:], self.attitude_array[:,None,:], gate_world)

                    self.position_list = []
                    self.velocity_list = []
                    self.attitude_list = []
                    
                    

                self.position_list.append(self.drone_pos)
                self.velocity_list.append(self.warp_qd)
                self.attitude_list.append(self.drone_quat)

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
        warp_pos = torch.Tensor(self.warp_q[:3]).unsqueeze(0)
        warp_q = torch.Tensor(warp_q).unsqueeze(0)
        warp_qd = torch.Tensor(self.warp_qd).unsqueeze(0)
        self.action = self.policy.evaluate_(warp_pos, warp_q, warp_qd)
        # print(self.action)




if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")
    policy_file = "20260325-152321" #"20260306-154450 - with gru. more reasonable" #"20260304-161010" #"20260304-160736 - this reasonable"#"20260225-165700" #0.02 good enough for real drone 20250527-122703   0.05-to test 20250624-181715
    print(f"POLICY PATH IS {policy_file}") 

    rospack = rospkg.RosPack()
    path = rospack.get_path('nn_policy')
    full_path = os.path.join(path, "logs/gate_traversal")
    print(f"The full path is {full_path}")
    actual_full_path = os.path.join(full_path, policy_file)
    config_path = os.path.join(actual_full_path,"training_config.yaml")
    full_policy_path = os.path.join(actual_full_path, "policy.pth")

    rospy.init_node("nn_policy_planner2")
    ros_lib = roslib.packages.get_pkg_dir("gestelt_bringup")
    full_config_path = os.path.join(ros_lib, "config/traj_server_default.yaml")
    with open(full_config_path, 'r') as file:
        loaded_params = yaml.safe_load(file)

    with open(config_path, 'r') as file:
        config_params = yaml.safe_load(file)

    mission_command_mode = loaded_params["mission_command_mode"]
    to_transform_odom = loaded_params["to_transform_odom"]
    to_transform_policy = loaded_params["to_transform_policy"]
    warp_jax = loaded_params["warp_jax"]

    position_control = True #config_params["position_control"]
    delta_time = 0.02 #float(config_params["delta_time"])
    max_angular_rate = 3.0 #float(config_params["max_angular_rates"])

    if "use_gru" in config_params:
        use_gru = config_params["use_gru"]
    else:
        use_gru = False

    if "gru_include_prev_action" in config_params:
        gru_include_prev_action = config_params["gru_include_prev_action"]
    else:
        gru_include_prev_action = False

    print("=" * 50)
    print(f"  Policy file    : {policy_file}")
    print(f"  Use GRU        : {use_gru}")
    print(f"  Include actions: {gru_include_prev_action if use_gru else 'N/A (no GRU)'}")
    print("=" * 50)

    #This code is primarily for warp policies. So warp_jax has to be 0.0
    if warp_jax != 0.0:
        raise ValueError("warp_jax should be 0.0")

    if "warp_frame" in config_params:
        warp_frame = config_params["warp_frame"]
        if warp_frame == 0.0: #if warp_frame = 0.0 this means that this is the y-up frame. Then this means that everything needs to be transformed
            if to_transform_odom != 1.0:
                raise ValueError("to_transform_odom should be 1.0")
            if to_transform_policy != 1.0:
                raise ValueError("to_transform_policy should be 1.0")
        if warp_frame == 1.0: #if warp_frame = 1.0, this means that this is the z-up frame. Then no need to transform anything
            if "policy_global" in config_params:
                policy_global = config_params["policy_global"]
                if policy_global == 1.0:
                    if to_transform_odom != 1.0:
                        raise ValueError("to_transform_odom should be 0.0.")
                    if to_transform_policy != 1.0:
                        raise ValueError("to_transform_policy should be 0.0")
                else:
                    if to_transform_odom != 0.0:
                        raise ValueError("to_transform_odom should be 0.0.")
                    if to_transform_policy != 0.0:
                        raise ValueError("to_transform_policy should be 0.0")
            else:
                if to_transform_odom != 0.0:
                    raise ValueError("to_transform_odom should be 0.0.")
                if to_transform_policy != 0.0:
                    raise ValueError("to_transform_policy should be 0.0")
    else:
        #This is assumed to be pre warp_frame period. so should transform
        if to_transform_odom != 1.0:
            raise ValueError("to_transform_odom should be 1.0")
        if to_transform_policy != 1.0:
            raise ValueError("to_transform_policy should be 1.0")

    
    #vel 20250424-161234 #position 20250424-131220, 20250424-161345

    nn_policy = TEST_RENDER(full_policy_path, position_control, warp_frame, use_gru=use_gru, include_actions = gru_include_prev_action) 

    nn_policy_planner = NN_POLICY_PLANNER(mission_command_mode=int(mission_command_mode), policy=nn_policy,
                                          inference_timestep=delta_time, max_angular_rates = max_angular_rate,
                                          warp_jax=warp_jax, to_transform_odom=to_transform_odom, to_transform_policy=to_transform_policy,
                                          config_param=config_params)

    rospy.spin()

    print("done")

