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
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
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
from modules.policy_simple_nwu_global import *
from modules.scene_manager import *
import pickle
from dataclasses import asdict, is_dataclass

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


def quat_to_rotmat_flat(q_xyzw, eps=1e-8):
    q = q_xyzw / q_xyzw.norm(dim=-1, keepdim=True).clamp_min(eps)
    x, y, z, w = q.unbind(-1)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    R = torch.stack([
        1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy),
        2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx),
        2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy),
    ], dim=-1)
    return R


# def create_obs_vec(pos, obs_dict):
#     """
#     Create obstacle vector by stacking relative positions (x-y only) and radii for all obstacles.
    
#     Args:
#         pos: Current drone positions [batch_size, 3]
#         obs_dict: List of obstacle dictionaries with 'position' and 'radius'
    
#     Returns:
#         Tensor of shape [batch_size, num_obstacles * 3] containing stacked
#         relative positions (2D x-y) and radius (1D) for each obstacle
#     """
#     if not obs_dict:
#         # Return zero vector if no obstacles
#         return torch.zeros((pos.shape[0], 0), device=pos.device, dtype=pos.dtype)
    
#     batch_size = pos.shape[0]
#     obs_vec_list = []
    
#     for obs in obs_dict:
#         # Convert obstacle position to tensor
#         obs_pos = torch.as_tensor(obs["position"], device=pos.device, dtype=pos.dtype)
#         obs_radius = torch.tensor(obs["radius"], device=pos.device, dtype=pos.dtype)
        
#         # Handle different shapes of obs_pos (could be [3] or [1, 3])
#         if obs_pos.dim() == 2:
#             obs_pos = obs_pos.squeeze(0)  # [1, 3] -> [3]
        
#         # Compute relative position (x-y only): obstacle_pos - drone_pos [batch_size, 2]
#         rel_pos = obs_pos[:2] - pos[:, :2]
        
#         # Expand radius for batch dimension [batch_size, 1]
#         obs_radius_batch = obs_radius.unsqueeze(0).expand(batch_size, 1)
        
#         # Stack relative position (2D) and radius [batch_size, 3]
#         obs_contribution = torch.cat([rel_pos, obs_radius_batch], dim=1)
#         obs_vec_list.append(obs_contribution)
    
#     # Concatenate all obstacles along feature dimension [batch_size, num_obstacles * 3]
#     obs_vec = torch.cat(obs_vec_list, dim=1)
    
#     return obs_vec


def create_obs_vec(pos, obs_dict):
    """
    Create obstacle vector by stacking nearest-to-farthest relative obstacle
    positions (x-y only) and radii for all obstacles.
    
    Args:
        pos: Current drone positions [batch_size, 3]
        obs_dict: List of obstacle dictionaries with 'position' and 'radius'
    
    Returns:
        Tensor of shape [batch_size, num_obstacles * 3] containing stacked
        relative positions (2D x-y) and radius (1D) for each obstacle
    """
    if not obs_dict:
        # Return zero vector if no obstacles
        return torch.zeros((pos.shape[0], 0), device=pos.device, dtype=pos.dtype)
    
    batch_size = pos.shape[0]
    obs_positions = []
    obs_radii = []
    for obs in obs_dict:
        obs_pos = torch.as_tensor(obs["position"], device=pos.device, dtype=pos.dtype)
        if obs_pos.dim() == 2:
            obs_pos = obs_pos.reshape(-1, 3)[0]
        obs_positions.append(obs_pos[:3])
        obs_radii.append(torch.as_tensor(obs["radius"], device=pos.device, dtype=pos.dtype))

    obs_positions = torch.stack(obs_positions, dim=0)  # [num_obstacles, 3]
    obs_radii = torch.stack(obs_radii, dim=0)  # [num_obstacles]

    # Compute relative XY for every env/obstacle pair, then sort per env by
    # center distance so each MLP slot has stable nearest-to-farthest meaning.
    rel_xy = obs_positions[:, :2].unsqueeze(0) - pos[:, :2].unsqueeze(1)
    sort_indices = torch.argsort(torch.norm(rel_xy, dim=-1), dim=1)
    gather_indices = sort_indices.unsqueeze(-1).expand(batch_size, -1, 2)
    sorted_rel_xy = torch.gather(rel_xy, dim=1, index=gather_indices)
    sorted_radii = obs_radii[sort_indices].unsqueeze(-1)

    obs_vec = torch.cat([sorted_rel_xy, sorted_radii], dim=-1).reshape(batch_size, -1)
    
    return obs_vec


class TEST_RENDER(object):

    def __init__(self, policy_path, pc, warp_frame, config_params):
        self.pc = pc
        self.config_params = config_params
        self.max_speed = config_params["max_speed"]
        
        self.warp_frame = warp_frame
        self.h = None
        self.start_msg = False


        ## Initializing task parameters
        self.init_a = np.zeros((1,4))
        self.init_a[0,0] = 0.30

        self.previous_action = torch.tensor(self.init_a, dtype=torch.float32)

        ## Loading Policy
        # att_dim = 9 if use_rotmat_obs else 4
        # input_dims = 12 + att_dim + (4 if include_gate_ori else 0)
        input_dims = 22
        print(f"Policy input_dims: {input_dims}")# (use_rotmat_obs={use_rotmat_obs}, include_gate_ori={include_gate_ori})")
        if self.pc == True:
            raise ValueError("self.pc cannot be True")
        else:
            print("LOADING VELOCITY TRACKING POLICY WITHOUT PC")
            self.policy = TrackVel(input_dim = input_dims)

        print(self.policy)
        self.policy.load_state_dict(torch.load(policy_path, map_location='cpu'))
        self.policy.eval()

        ### Loading scene (obstacles are built once here and stay fixed afterwards)
        device = "cpu"
        self.env_copy = 1  # test time: single environment
        room_size = config_params["room_size"]
        self.env_manager = SceneManager(batch_size=self.env_copy, device=device)
        env_manager = self.env_manager
        self.centers = None  # obstacle centers (poisson room only), reused when resampling
        self.radii = None    # obstacle radii (poisson room only), reused when resampling
        obstacles_cfg = config_params.get("obstacles", None)
        if obstacles_cfg:
            # Config records explicit obstacle locations/sizes -> reuse the exact
            # scene the policy was trained/evaluated on instead of resampling one.
            print(f"Building scene from {len(obstacles_cfg)} explicit obstacles in config")
            centers, radii = env_manager.setup_room_from_obstacles(room_size=room_size,
                                    obstacles=obstacles_cfg, obs_dim=config_params["obs_dim"])
            self.centers = centers
            self.radii = radii  # already the sphere-approx radius (no extra factor)
        elif config_params["poisson_room"] == False:
            env_manager.setup_room(room_size=room_size, num_objects=config_params["num_obstacles"], rand_obs=config_params["to_randomize_obs_location"],
                                    obs_size=config_params["obs_size"], obs_type=config_params["obs_type"], obs_dim=config_params["obs_dim"])
        else:
            centers, radii = env_manager.setup_poisson_room(room_size=room_size, num_objects=config_params["num_obstacles"], rand_obs=config_params["to_randomize_obs_location"],
                                    obs_size=config_params["obs_size"], obs_dim=config_params["obs_dim"], min_obs_gap=config_params["min_obs_gap"])
            factor = np.sqrt(2)
            self.centers = centers
            self.radii = radii * factor

        ## Sample the initial and target positions from the scene
        self.resample_init_target()
        

    def resample_init_target(self):
        """Sample a fresh (init, target) position pair from the existing scene.

        The room and obstacles built in __init__ are left unchanged; only the
        start and goal positions are redrawn. Updates self.init_pos and
        self.t_pos, and returns both as numpy arrays of shape [1, 3].
        """
        env_manager = self.env_manager
        room_size = self.config_params["room_size"]

        # --- Fixed initial and target locations ---
        # When True, always use the hardcoded init/target below instead of
        # sampling a fresh pair from the scene each episode. Positions are in the
        # sim/room frame [x, y, z]; edit the two arrays to taste.
        use_fixed_init_target = True
        if use_fixed_init_target:
            init_pos = np.array([[0.0, 8.0, 2.0]], dtype=np.float32)
            target_pos = np.array([[6.0, 5.0, 1.0]], dtype=np.float32)
            # init_pos = np.array([[0.0, 0.0, 2.0]], dtype=np.float32)
            # target_pos = np.array([[3.0, 0.0, 1.0]], dtype=np.float32)
            self.init_pos = torch.tensor(init_pos, dtype=torch.float32)
            self.t_pos = torch.tensor(target_pos, dtype=torch.float32)
            return init_pos, target_pos

        # self.centers is populated by both the poisson-room and explicit-obstacle
        # paths; only the plain setup_room path leaves it None.
        if self.centers is None:
            init_pos = env_manager.sample_safe_points(n_points=self.env_copy)
            target_pos = env_manager.sample_safe_points(n_points=self.env_copy)
        else:
            init_bounds_2D = [0, room_size/4, 0, room_size]  #(x_min, x_max, y_min, y_max) region to sample initial point
            init_pos = env_manager.sample_safe_points_in_region(room_region=init_bounds_2D, obstacles_center=self.centers,
                                                                obstacles_radius=self.radii, d_safe=0.0, n_points=self.env_copy)
            target_bounds_2D = [(3*room_size)/4, room_size, 0, room_size]
            target_pos = env_manager.sample_safe_points_in_region(room_region=target_bounds_2D, obstacles_center=self.centers,
                                                                obstacles_radius=self.radii, d_safe=0.0, n_points=self.env_copy)

        # Never let the drone height fall below 0.5 m
        init_pos[:, 2] = np.maximum(init_pos[:, 2], 0.5)

        # Push the target further out along the init->target direction so the
        # task spans more distance than the raw sampled pair, then clamp back
        # into the room bounds (obstacle clearance is not re-checked).
        push_factor = self.config_params.get("target_push_factor", 1.0)
        if push_factor and push_factor != 1.0:
            direction = target_pos - init_pos
            direction[:, 0] *= push_factor
            direction[:, 1] *= push_factor
            target_pos = init_pos + direction

            if self.config_params["poisson_room"] == False:
                x_min, x_max = -room_size, room_size
                y_min, y_max = -room_size, room_size
                z_min, z_max = 0.0, 3.0
            else:
                x_min, x_max = 0.0, room_size
                y_min, y_max = 0.0, room_size
                z_min, z_max = 1.0, 2.0

            # target_pos[:, 0] = np.clip(target_pos[:, 0], x_min, x_max)
            # target_pos[:, 1] = np.clip(target_pos[:, 1], y_min, y_max)
            # target_pos[:, 2] = np.clip(target_pos[:, 2], z_min, z_max)

        self.init_pos = torch.tensor(init_pos, dtype=torch.float32)
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32)
        return init_pos, target_pos

    def reset_h(self, reset_msg):
        if reset_msg == True:
            self.h = None
            self.init_a = np.zeros((1,4))
            self.init_a[0,0] = 0.30
            target_vel = np.zeros((1, 3))
            self.previous_action = torch.tensor(self.init_a, dtype=torch.float32)

    def update_window_info(self, window_pose, window_velocity, window_orientation):
        self.window_position = torch.tensor(window_pose, dtype=torch.float32)
        self.window_velocity = torch.tensor(window_velocity, dtype=torch.float32)
        self.window_quaternion = torch.tensor(window_orientation, dtype=torch.float32)
    
    

    def evaluate_(self, pos, att, qd):
        start_time = time.time()
        ## Concatenating observations
        if self.pc == True:
            raise ValueError("self.pc cannot be True")
            # diff_pos = self.t_pos - pos
            # x = torch.cat((self.init_pos_t, pos, att, qd, self.window_velocity, self.window_quaternion), dim=1)
            diff_pos = self.window_position - pos
            pos_offset = pos
            # pos[:,2] = pos[:,2] - self.height_offset
            att_in = quat_to_rotmat_flat(att) if self.use_rotmat_obs else att
            base_obs = (diff_pos, pos, att_in, qd)
            x = torch.cat(base_obs
                          + ((self.window_quaternion,) if self.include_gate_ori else ()),
                          dim=1)
            if self.include_actions:
                x = torch.cat([x, self.previous_action], dim=1)

        else:
            vel = qd[:,3:]
            angvel = qd[:,:3]
            diff_velocity_norm = torch.norm(self.t_pos - pos, dim=1, keepdim=True)
            # print(f"The target position is {self.t_pos}")
            diff_velocity_vector = (self.t_pos - pos) / (diff_velocity_norm + 1e-8)
            diff_velocity_vector = diff_velocity_vector * self.max_speed
            # print(f"The diff_velocity_vector is {diff_velocity_vector}")
            diff_vel = diff_velocity_vector - vel
            obs_vec = create_obs_vec(pos, self.env_manager.objects)
            x = torch.cat((att, angvel, diff_vel, obs_vec), dim=1)
        
        ## Evaluating policy

        a = self.policy(x)
        
        self.previous_action = a
        # print(a)
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
                 to_transform_odom, to_transform_policy, recovery_mode,
                 config_param=None, scene_offset=(0.0, 0.0, 0.0)):

        self.recovery_mode = recovery_mode
        self.individual_recording = False
        #Creating subscribers and Publishers
        self.bullet_sim_mutex = threading.Lock()
        self.tfBuffer =  tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.warp_pose_msg = PoseStamped()
        #Create a policy field
        self.policy = policy
        rospy.sleep(1)

        # Sim<->world (Vicon) frame offset. The drone flies physically in the
        # world/Vicon frame while the policy "imagines" the sim (room) frame:
        #   pos_sim   = pos_world + scene_offset
        #   pos_world = pos_sim   - scene_offset
        # Zero vector => no transform (drone and policy share one frame).
        self.scene_offset = np.asarray(scene_offset, dtype=float).reshape(1, 3)
        self.scene_offset_t = torch.tensor(self.scene_offset, dtype=torch.float32)
        print(f"Planner scene_offset (world -> sim): {self.scene_offset[0]}")

        ##Define initial window location

        ## Saving list for plotting
        self.position_list = []
        self.velocity_list = []
        self.attitude_list = []


        ## Define initial starting location of the drone (taken from the policy's sampled init position)
        ## Policy positions are in sim frame -> convert to world frame for commanding/checking.
        init_pos = self.policy.init_pos.detach().cpu().numpy() - self.scene_offset
        self.init_pos_numpy = init_pos
        ## Target location (sampled goal), used for the episode-end proximity check
        self.target_pos_numpy = self.policy.t_pos.detach().cpu().numpy() - self.scene_offset
        # self.init_quat = self.update_init_orientation_drone(self.init_pos_numpy, self.window_position)
        self.init_quat = np.array([[0.0, 0.0, 0.0, 1.0]])
        self.curr_init_pose = PoseStamped()
        self.curr_init_pose.pose.position.x = copy.deepcopy(init_pos[:,0])
        self.curr_init_pose.pose.position.y = copy.deepcopy(init_pos[:,1])
        self.curr_init_pose.pose.position.z = copy.deepcopy(init_pos[:,2])
        self.curr_init_pose.pose.orientation.x = copy.deepcopy(self.init_quat[:,0])
        self.curr_init_pose.pose.orientation.y = copy.deepcopy(self.init_quat[:,1])
        self.curr_init_pose.pose.orientation.z = copy.deepcopy(self.init_quat[:,2])
        self.curr_init_pose.pose.orientation.w = copy.deepcopy(self.init_quat[:,3])
        #Update policy with initial starting location of the drone


        self.max_angular_rates = max_angular_rates
        self.last_pos_time = None
        self.last_odom_time = None

        self.warp_jax = warp_jax
        self.to_transform_odom = to_transform_odom
        self.to_transform_policy = to_transform_policy
        

        self.obstacle_pub_ = rospy.Publisher('/policy_viz/obstacles', PointCloud2, queue_size=1, latch=True)
        self.start_target_pub_ = rospy.Publisher('/policy_viz/start_target', MarkerArray, queue_size=1, latch=True)
        self.trajectory_pub_ = rospy.Publisher('/policy_viz/trajectory', Path, queue_size=1)
        self.viz_frame_id = "map"
        self.trajectory_path_msg = Path()
        self.trajectory_path_msg.header.frame_id = self.viz_frame_id

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
        self.recorder_sub_ = rospy.Subscriber('/traj_server/warp_mission_recorder', Bool, self.recorderCB, queue_size=10)

        self.nwu_odom = np.zeros(6)

        #PVA controller trajectory Publisher
        self.pva_traj_pub_ = rospy.Publisher("/drone0/planner_adaptor/exec_trajectory", ExecTrajectory, queue_size = 5)
        self.pos_update_pub_ = rospy.Publisher("/drone0/planner_adaptor/pos_update", ExecTrajectory, queue_size = 5)
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

        # Data recording state
        self.data_store = {
            0: {
                "time_stamp": [],
                "position": [],
                "velocity": [],
                "rotation": [],
                "omega": [],
                "action": [],
            }
        }
        self.record_counter = 0
        self.record_now = False
        self.pending_save = False
        self.awaiting_restart = False
        self.lock = threading.Lock()
        self.inference_timestep = inference_timestep
        self.config_param = config_param

        self.save_directory = os.path.join(rospack.get_path('nn_policy'), "warp_data")
        os.makedirs(self.save_directory, exist_ok=True)
        folders = [f for f in os.listdir(self.save_directory)
                   if os.path.isdir(os.path.join(self.save_directory, f))]
        self.total_files = len(folders)
        full_save_path = os.path.join(self.save_directory, str(self.total_files))
        os.makedirs(full_save_path, exist_ok=True)
        self.full_save_path = os.path.join(full_save_path, "data.pkl")
        self.full_save_config_path = os.path.join(full_save_path, "training_config.yaml")

        self.mission_command_mode = 1
        self.warp_mission_command_mode = mission_command_mode
        self.attitude_mode_toggle = 0
        self.action = np.zeros((1,4))
        self.recovery_vel_start_time = None
        time.sleep(1)
        self.publish_obstacle_cloud()
        self.publish_start_target_markers()
        self.policy_evaluation_timer = rospy.Timer(rospy.Duration(0.01), self.nn_evaluation)
        
    def initPoseCB(self,msg):
        self.init_pos_numpy[:,0] = msg.pose.position.x 
        self.init_pos_numpy[:,1] = msg.pose.position.y
        self.init_pos_numpy[:,2] = msg.pose.position.z
        # self.init_quat = self.update_init_orientation_drone(self.init_pos_numpy, self.window_position)
        self.init_quat = np.array([[0.0, 0.0, 0.0, 1.0]])

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
        print(self.to_transform_odom)
        if self.to_transform_odom == 0.0:
            self.warp_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z ])
        if self.last_odom_time is not None:
           time_diff = (msg.header.stamp- self.last_odom_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE ODOM EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_odom_time = msg.header.stamp
        self.nwu_odom = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z,
                                   msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z])

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
        # print(self.init_pos_numpy)


        #This part is non essential. Merely for debugging purposes
        pva_traj_msg.type_mask = self.attitude_mode_toggle
        pva_traj_msg.throttle = self.action[0,0]
        pva_traj_msg.angular_rates.angular.x = self.action[0,1]   #body rate x
        pva_traj_msg.angular_rates.angular.y = self.action[0,2]     #body rate y
        pva_traj_msg.angular_rates.angular.z = self.action[0,3]     #body rate z

        #Publish the PVA
        self.pva_traj_pub_.publish(pva_traj_msg)

    def publishVEL(self):
        # print("velocity")
        pva_traj_msg = ExecTrajectory()
        pva_traj_msg.transform.translation.x = 0.0
        pva_traj_msg.transform.translation.y = 0.0
        pva_traj_msg.transform.translation.z = 1.0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0 #0.707
        pva_traj_msg.transform.rotation.w = 1.0 #0.707
        pva_traj_msg.velocity.linear.x = 0.0
        pva_traj_msg.velocity.linear.y = 0.0
        pva_traj_msg.velocity.linear.z = 0.0
        pva_traj_msg.type_mask = 2048
        # print("here")

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
        if type_mask == 0:
            pva_traj_msg.throttle = 0.31 #nn_action[0,0]  #0.321
            print("in here")
        else:
            pva_traj_msg.throttle = nn_action[0,0]

        ### This part will only be taken in by trajectory server if type_mask == 0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0 
        pva_traj_msg.transform.rotation.w = 1.0

        ### This part will only be taken in by trajectory server if type_mask == 1
        pva_traj_msg.angular_rates.angular.x = nn_action[0,1] * self.max_angular_rates     #body rate x
        pva_traj_msg.angular_rates.angular.y = nn_action[0,2] * self.max_angular_rates    #body rate y
        pva_traj_msg.angular_rates.angular.z = nn_action[0,3] * self.max_angular_rates

        self.pva_traj_pub_.publish(pva_traj_msg)

        if self.individual_recording:
            warp_q = self.warp_q[3:]
            warp_pos = torch.Tensor(self.warp_q[:3]).unsqueeze(0)
            warp_q_t = torch.Tensor(warp_q).unsqueeze(0)
            now_time = rospy.Time.now().to_sec()
            with self.lock:
                # Record the whole trajectory while individual_recording is active
                # (bounded by settle-start and the target-proximity episode end).
                self.data_store[0]["time_stamp"].append(now_time)
                self.data_store[0]["position"].append(warp_pos.squeeze(0).detach().cpu().numpy())
                self.data_store[0]["velocity"].append(self.nwu_odom[3:])
                self.data_store[0]["rotation"].append(warp_q_t.squeeze(0).detach().cpu().numpy())
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

    def executeMission(self):
        if self.drone_state == DRONESTATE["MISSION"].value:
            if self.mission_command_mode == 1:
                self.publishPVA()
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
                
                if np.linalg.norm(self.drone_pos - self.target_pos_numpy[0]) < 1.0:
                    #Means drone has reached the target (within 0.3 m). Switch back to position control.
                    print("Reached target (within 0.3 m). Switching back to position control.")
                    self.individual_recording = False
                    self.init_pos_numpy = self.target_pos_numpy.copy()
                    self.init_quat = np.array([[0.0, 0.0, 0.0, 1.0]])

                    if self.recovery_mode == 1:
                        print("SWITCHING BACK TO POSITION!!")
                        self.publishPVA()  # push end_pos_numpy to traj server before mode flip
                        self.mission_command_mode = 1
                        self.publishMissionCmdMode(1)
                        self.warp_mission_command_mode = 1
                    elif self.recovery_mode == 2:
                        print("SWITCHING BACK TO VELOCITY!!")
                        self.mission_command_mode = 3
                        self.publishMissionCmdMode(3)
                        self.warp_mission_command_mode = 3
                        self.recovery_vel_start_time = rospy.Time.now()
                    elif self.recovery_mode == 3:
                        print("SWITCHING BACK TO ATTITUDE!!")
                        self.attitude_mode_toggle = 0
                        # self.mission_command_mode = 2
                        self.publishMissionCmdMode(2)
                        # self.warp_mission_command_mode = 2

                    self.policy.start_msg = False
                    self.policy.reset_h(True)                    
                    
                if self.attitude_mode_toggle == 1 and self.mission_command_mode == 2:
                    self.position_list.append(self.drone_pos)
                    self.velocity_list.append(self.warp_qd)
                    self.attitude_list.append(self.drone_quat)
                    self.publish_trajectory_point()

            elif self.mission_command_mode == 3:
                self.publishVEL()
                vel_threshold = 0.5  # m/s
                min_dwell_secs = 1.0
                if (self.recovery_vel_start_time is not None and
                        (rospy.Time.now() - self.recovery_vel_start_time).to_sec() > min_dwell_secs):
                    speed = np.linalg.norm(self.warp_qd[3:6])
                    if speed < vel_threshold:
                        for i in range(10):
                            print(f"RECOVERED (speed={speed:.2f} m/s): Switching back to POSITION CONTROL")
                            pva_traj_msg_update = ExecTrajectory()
                            pva_traj_msg_update.transform.translation.x = self.init_pos_numpy[:,0]
                            pva_traj_msg_update.transform.translation.y = self.init_pos_numpy[:,1]
                            pva_traj_msg_update.transform.translation.z = self.init_pos_numpy[:,2]
                            pva_traj_msg_update.transform.rotation.x = self.init_quat[:,0]
                            pva_traj_msg_update.transform.rotation.y = self.init_quat[:,1]
                            pva_traj_msg_update.transform.rotation.z = self.init_quat[:,2]
                            pva_traj_msg_update.transform.rotation.w = self.init_quat[:,3]
                            self.pos_update_pub_.publish(pva_traj_msg_update)
                         # push end_pos_numpy to traj server before mode flip
                        self.mission_command_mode = 1
                        self.publishMissionCmdMode(1)
                        self.warp_mission_command_mode = 1
                        self.recovery_vel_start_time = None
                        # --- Gate-specific metrics/plots disabled for the velocity task ---
                        # self.position_array = np.array(self.position_list)
                        # self.velocity_array = np.array(self.velocity_list)
                        # self.attitude_array = np.array(self.attitude_list)
                        # gate_center = self.window_position[0]       # (3,) [x, y, z]
                        # window_quat = self.window_quaternion[0]     # (4,) [x, y, z, w]
                        # target_vel  = self.window_velocity[0]       # (3,) desired velocity at crossing
                        # t_star, pos_at_gate, pos_err, vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot = \
                        #     compute_gate_metrics(self.position_array, self.velocity_array,
                        #                          self.attitude_array, window_quat,
                        #                          gate_center=gate_center, target_vel=target_vel)
                        # plot_spatial_plots(self.position_array[:,None,:])
                        # gate_world = gate_geometry(self.window_degrees[0], gate_center=gate_center)
                        # plot_gate_travesal(self.position_array[:,None,:], self.attitude_array[:,None,:],
                        #                    gate_world, t_star, pos_at_gate, pos_err,
                        #                    vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot)
                        # plot_metrics_timeseries(self.position_array, self.velocity_array, self.attitude_array,
                        #                         window_quat, t_star, pos_at_gate, pos_err,
                        #                         vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot)
                        if self.record_now:
                            rospy.Timer(rospy.Duration(0.5), self._append_separator_and_restart_cb, oneshot=True)
                        elif self.pending_save:
                            rospy.Timer(rospy.Duration(0.1), self._flush_and_save_cb, oneshot=True)
                        self.position_list = []
                        self.velocity_list = []
                        self.attitude_list = []

            elif self.mission_command_mode == 4:
                self.publishGeomCtrl()
                

    def recorderCB(self, msg):
        if self.record_now and not msg.data:
            self.awaiting_restart = False
            self.record_now = False
            self.record_counter = 0
            if self.individual_recording:
                print("Stop signal received. Waiting for current episode to finish before saving.")
                self.pending_save = True
            else:
                print("Recording stopped. Saving to file NOW.")
                self.saving_to_file()
            return

        if not self.record_now and msg.data:
            print("Starting data recording cycle")
            self.data_store = {
                0: {
                    "time_stamp": [],
                    "position": [],
                    "velocity": [],
                    "rotation": [],
                    "omega": [],
                    "action": [],
                }
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
                config = asdict(config)
            with open(self.full_save_config_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(config, f, sort_keys=False, default_flow_style=False, allow_unicode=True)

    def randomize_init_pos(self):
        # Draw a fresh start + goal from the policy's scene manager (obstacles stay fixed).
        # This also updates the policy's self.init_pos and self.t_pos.
        new_pos, target_pos = self.policy.resample_init_target()
        # Policy positions are in sim frame -> convert to world frame.
        new_pos = new_pos - self.scene_offset
        target_pos = target_pos - self.scene_offset
        self.init_pos_numpy = new_pos
        self.target_pos_numpy = target_pos
        # self.init_quat = self.update_init_orientation_drone(new_pos, self.window_position)
        self.init_quat = np.array([[0.0, 0.0, 0.0, 1.0]])
        self.curr_init_pose.pose.position.x = float(new_pos[0, 0])
        self.curr_init_pose.pose.position.y = float(new_pos[0, 1])
        self.curr_init_pose.pose.position.z = float(new_pos[0, 2])
        self.curr_init_pose.pose.orientation.x = float(self.init_quat[0, 0])
        self.curr_init_pose.pose.orientation.y = float(self.init_quat[0, 1])
        self.curr_init_pose.pose.orientation.z = float(self.init_quat[0, 2])
        self.curr_init_pose.pose.orientation.w = float(self.init_quat[0, 3])
        self.trajectory_path_msg.poses = []
        self.publish_start_target_markers()

    def publish_obstacle_cloud(self):
        """Publish obstacle footprints as a PointCloud2 (filled vertical columns) for RViz.
        Obstacles are fixed for the whole run, so this only needs to be published once
        (latched so late-joining RViz subscribers still see it)."""
        points = []
        z_layers = np.linspace(0.0, 3.0, 6)
        n_theta = 16
        r_fracs = (0.33, 0.66, 1.0)
        for obj in self.policy.env_manager.objects:
            pos = np.asarray(obj["position"]).reshape(-1)
            radius = float(obj["radius"])
            for z in z_layers:
                points.append([pos[0], pos[1], z])
                for r_frac in r_fracs:
                    for theta in np.linspace(0, 2 * np.pi, n_theta, endpoint=False):
                        x = pos[0] + r_frac * radius * np.cos(theta)
                        y = pos[1] + r_frac * radius * np.sin(theta)
                        points.append([x, y, z])

        header = std_msgs.msg.Header()
        header.frame_id = self.viz_frame_id
        header.stamp = rospy.Time.now()
        cloud_msg = pc2.create_cloud_xyz32(header, points)
        self.obstacle_pub_.publish(cloud_msg)

    def publish_start_target_markers(self):
        """Publish the current init (green) and target (red) positions as RViz markers."""
        marker_array = MarkerArray()
        now = rospy.Time.now()
        # Markers are drawn in the room (sim) frame, same as the obstacle cloud,
        # so convert the world-frame setpoints back into sim coords for display.
        init_pos = np.asarray(self.init_pos_numpy).reshape(-1) + self.scene_offset.reshape(-1)
        target_pos = np.asarray(self.target_pos_numpy).reshape(-1) + self.scene_offset.reshape(-1)

        for marker_id, (pos, color) in enumerate([
            (init_pos, (0.0, 1.0, 0.0)),    # green = init
            (target_pos, (1.0, 0.0, 0.0)),  # red = target
        ]):
            m = Marker()
            m.header.frame_id = self.viz_frame_id
            m.header.stamp = now
            m.ns = "start_target"
            m.id = marker_id
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(pos[0])
            m.pose.position.y = float(pos[1])
            m.pose.position.z = float(pos[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.r, m.color.g, m.color.b = color
            m.color.a = 1.0
            marker_array.markers.append(m)

        self.start_target_pub_.publish(marker_array)

    def publish_trajectory_point(self):
        """Append the drone's current position to the live trajectory Path and publish it."""
        pose = PoseStamped()
        pose.header.frame_id = self.viz_frame_id
        pose.header.stamp = rospy.Time.now()
        # Draw the drone in the room (sim) frame to match the obstacle cloud.
        drone_pos_sim = np.asarray(self.drone_pos).reshape(-1) + self.scene_offset.reshape(-1)
        pose.pose.position.x = float(drone_pos_sim[0])
        pose.pose.position.y = float(drone_pos_sim[1])
        pose.pose.position.z = float(drone_pos_sim[2])
        pose.pose.orientation.x = float(self.drone_quat[0])
        pose.pose.orientation.y = float(self.drone_quat[1])
        pose.pose.orientation.z = float(self.drone_quat[2])
        pose.pose.orientation.w = float(self.drone_quat[3])

        self.trajectory_path_msg.header.stamp = pose.header.stamp
        self.trajectory_path_msg.poses.append(pose)
        self.trajectory_pub_.publish(self.trajectory_path_msg)


    def _settle_and_start_nn_cb(self, event):
        if not self.record_now:
            return
        print("Settle complete. Starting NN recording.")
        self.individual_recording = True
        self.policy.start_msg = True
        self.warp_mission_command_mode = 2

    def _append_separator_and_restart_cb(self, event):
        with self.lock:
            for key in self.data_store[0]:
                self.data_store[0][key].append(None)
        print("Episode separator written. Stabilizing before next episode...")
        rospy.Timer(rospy.Duration(2.0), self._go_to_next_init_cb, oneshot=True)

    def _go_to_next_init_cb(self, event):
        if not self.record_now:
            return
        self.randomize_init_pos()
        self.awaiting_restart = True
        print("Flying to next random init position.")

    def _flush_and_save_cb(self, event):
        print("Final episode finished. Saving to file NOW.")
        self.pending_save = False
        self.saving_to_file()

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
        # Drone position is in world/Vicon frame -> shift into sim frame so the
        # policy "sees" itself in the sim scene (obstacles/target stay in sim).
        warp_pos = torch.Tensor(self.warp_q[:3]).unsqueeze(0) + self.scene_offset_t
        warp_q = torch.Tensor(warp_q).unsqueeze(0)
        warp_qd = torch.Tensor(self.warp_qd).unsqueeze(0)
        self.action = self.policy.evaluate_(warp_pos, warp_q, warp_qd)
        print(self.action)




if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")
    policy_file = "20260707-114255"# This likely to work #"20260616-174028" to test in real flight #"20260616-150838" #"20260608-230203" bad 60 degrees. To compare with 20260608-170520 #"20260608-233141" good 30 degrees for gazebo trained with thrust DR also #"20260608-170520 good demo for 60 degrees gazebo. max body rates of 4.0 "#"20260604-222931" #"20260604-201453 good 30 degrees demo" #"20260604-095607" #"20260603-121142" #"20260603-121213" another 60 degrees gazebo demo. To test in real #"20260529-113935 60 degrees gazebo demo" #"20260521-090040" #"20260518-213037 30 degrees demo" #"20260519-121802" #"20260513-185143" #"20260513-185035" #"20260402-204842 This is high fidelity forward model." #"20260306-154450 - with gru. more reasonable" #"20260304-161010" #"20260304-160736 - this reasonable"#"20260225-165700" #0.02 good enough for real drone 20250527-122703   0.05-to test 20250624-181715
    print(f"POLICY PATH IS {policy_file}") 
    recovery_mode = 2 #1 for position, 2 for velocity, 3 for attitude

    rospack = rospkg.RosPack()
    path = rospack.get_path('nn_policy')
    full_path = os.path.join(path, "logs/cmdp")
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

    position_control = config_params["position_control"]
    delta_time = float(config_params["delta_time"])
    max_angular_rate = float(config_params["max_angular_rates"])

    if "use_gru" in config_params:
        use_gru = config_params["use_gru"]
    else:
        use_gru = False

    if "gru_include_prev_action" in config_params:
        gru_include_prev_action = config_params["gru_include_prev_action"]
    else:
        gru_include_prev_action = False
    print(gru_include_prev_action)

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


    include_gate_ori = config_params.get("include_gate_orientation_in_obs", False)
    use_rotmat_obs = config_params.get("use_rotation_matrix_obs", False)

    # ---- Sim <-> world (Vicon) frame offset ----
    # When True, the drone flies physically in the Vicon room while the policy
    # "imagines" the sim (room) frame. The scene (obstacles/target) stays in sim
    # coords; the drone's Vicon position is shifted into sim coords for the
    # policy, and commanded setpoints are shifted back. Default centers the sim
    # room [0, room_size] onto a center-origin room.
    use_scene_offset = True
    room_size = float(config_params["room_size"])
    scene_offset = [(room_size / 2.0) - 1, (room_size / 2.0) +3, 0.0] if use_scene_offset else [0.0, 0.0, 0.0]
    

    nn_policy = TEST_RENDER(full_policy_path, position_control, warp_frame, config_params)

    nn_policy_planner = NN_POLICY_PLANNER(mission_command_mode=int(mission_command_mode), policy=nn_policy,
                                          inference_timestep=delta_time, max_angular_rates = max_angular_rate,
                                          warp_jax=warp_jax, to_transform_odom=to_transform_odom, to_transform_policy=to_transform_policy, recovery_mode=recovery_mode,
                                          config_param=config_params, scene_offset=scene_offset)

    rospy.spin()

    print("done")

