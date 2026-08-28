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
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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
from modules.policy_simple_nwu_global_gate_traversal_vision import *
import torch
import torch.nn.functional as F
from types import SimpleNamespace
import datetime
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


def quat_rel_xyzw(q_a, q_b, eps=1e-8):
    """SO(3) difference: q_a^{-1} ⊗ q_b, both (N,4) in (x,y,z,w) format.

    Returns the rotation that takes q_a's frame to q_b's frame.
    Identity quaternion (0,0,0,1) when q_a == q_b (perfectly aligned).
    Used to give the policy a frame-invariant alignment-error signal rather
    than absolute gate orientation.
    """
    q_a = q_a / q_a.norm(dim=-1, keepdim=True).clamp_min(eps)
    q_b = q_b / q_b.norm(dim=-1, keepdim=True).clamp_min(eps)
    ax, ay, az, aw = q_a.unbind(-1)
    bx, by, bz, bw = q_b.unbind(-1)
    # conjugate of q_a is (-ax, -ay, -az, aw); multiply by q_b
    rx = aw * bx - bw * ax - ay * bz + az * by
    ry = aw * by - bw * ay - az * bx + ax * bz
    rz = aw * bz - bw * az - ax * by + ay * bx
    rw = aw * bw + ax * bx + ay * by + az * bz
    return torch.stack([rx, ry, rz, rw], dim=-1)


class TEST_RENDER(object):

    def __init__(self, args, policy_path, pc, warp_frame, use_gru=False, include_actions=False, include_gate_ori=False, use_rotmat_obs=False, use_so3_diff_obs=False):
        self.args = args
        self.pc = pc
        self.use_gru = use_gru
        self.include_actions = include_actions
        self.include_gate_ori = include_gate_ori
        self.use_rotmat_obs = use_rotmat_obs
        self.use_so3_diff_obs = use_so3_diff_obs
        self.warp_frame = warp_frame
        self.height_offset = 1.0
        self.h = None
        self.start_msg = False
        self.mar = float(getattr(self.args, "max_angular_rates", 4.5))
        self.device = "cuda:0" if torch.cuda.is_available() else "cpu"

        gru_action_extra = 4 if (use_gru and include_actions) else 0
        ## Initializing task parameters
        self.init_a = np.zeros((1,4))
        self.init_a[0,0] = 0.30
        target_vel = np.zeros((1, 3))
        self.previous_action = torch.tensor(self.init_a, dtype=torch.float32)
        # print(gru_action_extra)

        ## Loading Policy
        # Image size the CNN expects; the incoming depth is resized to this.
        self.img_h = int(self.args.height)
        self.img_w = int(self.args.width)
        # State = base 16 (diff_pos, pos, att, qd) + privileged gate-orientation slot
        # (4 dims, fed zeros at test -- see evaluate_). Width comes straight from the
        # checkpoint's own privileged_gate_orientation flag (matches
        # privileged_orientation_dim(cfg) in the _with_cbam training script) rather
        # than being hardcoded, so this stays correct whichever way that flag was set.
        # StateVisionGRUSpatialAttentionPolicy (CBAM) has no auxiliary orientation
        # head -- the training script explicitly rejects aux_orientation_weight > 0
        # for this policy class -- so no aux_orientation_dim kwarg here.
        self.priv_ori_dim = 4 if bool(getattr(self.args, "privileged_gate_orientation", False)) else 0
        self.state_dim = 16 + self.priv_ori_dim
        self.policy = StateVisionGRUSpatialAttentionPolicy(
            state_dim=self.state_dim,
            output_dim=4,
            img_size=(self.img_h, self.img_w),
            img_channels=1,
            img_latent_dims=int(getattr(self.args, "image_latent_dims", 64)),
            use_state_norm=False,
            recurrent_detach=bool(getattr(self.args, "gru_recurrent_detach", True)),
        ).to(self.device)

        print(self.policy)
        # checkpoint_latest.pth is a training checkpoint dict; weights are under
        # 'policy_state_dict'. Support a bare state_dict too, just in case.
        ckpt = torch.load(policy_path, map_location='cpu')
        state_dict = ckpt["policy_state_dict"] if isinstance(ckpt, dict) and "policy_state_dict" in ckpt else ckpt
        self.policy.load_state_dict(state_dict)
        self.policy.eval()

        if self.warp_frame == 0.0:
            target_pos = np.array([0, 1,0.0]).reshape(1,3)
        elif self.warp_frame == 1.0:
            target_pos = np.array([0, 0,1.0]).reshape(1,3)

        ## Converting numpy to tensor
        self.t_vel = torch.tensor(target_vel, dtype=torch.float32)
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32)
        self.init_pos = torch.tensor(target_pos, dtype=torch.float32)

        # ---- Debug: dump the exact HxW depth that is fed to the policy ----
        # Saved AFTER resize + normalize + invert, i.e. byte-for-byte what the CNN sees.
        self.save_depth = bool(getattr(self.args, "save_policy_depth", True))
        self.depth_save_every = int(getattr(self.args, "save_policy_depth_every", 10))
        self._depth_save_count = 0
        self.depth_save_dir = None
        if self.save_depth:
            stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
            self.depth_save_dir = os.path.join(rospkg.RosPack().get_path('nn_policy'),
                                               "depth_debug", stamp)
            os.makedirs(self.depth_save_dir, exist_ok=True)
            print(f"[depth-debug] saving every {self.depth_save_every}th "
                  f"{self.img_h}x{self.img_w} policy depth -> {self.depth_save_dir}")

        # ---- Live topic: the exact HxW depth fed to the policy (post crop/resize/
        # normalize/invert), published every evaluation for viewing in rqt/RViz.
        self.publish_policy_depth = bool(getattr(self.args, "publish_policy_depth", True))
        if self.publish_policy_depth:
            self._depth_cv_bridge = CvBridge()
            self.policy_depth_pub = rospy.Publisher("~policy_depth_view", Image, queue_size=1)

        # ---- Debug: override the state with a fixed vector ----
        # Isolates the depth/CNN path from odometry/pose: state is replaced entirely
        # (real depth still flows through), so any behavior change must come from vision.
        # Vector = [diff_pos(3), pos(3), att_xyzw(4), qd(6), privileged(0 or 4)]:
        #   diff_pos=[0.583,0,0] pos=[-0.333,-0,0.4] att=[0,0,0,1](level) qd=0 privileged=0
        # Built to self.state_dim rather than a hardcoded 20, so it stays valid whether
        # or not this checkpoint has the privileged gate-orientation slot.
        # A runtime debug toggle, not a training-config value -- read directly from the
        # ROS private param (rosrun ... _debug_override_state:=true), not self.args
        # (which is populated only from the checkpoint's training_config.yaml).
        self.debug_override_state = bool(rospy.get_param("~debug_override_state", False))
        _debug_vec = [0.5833333, 0., 0., -0.33333334, -0., 0.4,
                      0., 0., 0., 1., 0., 0.,
                      0., 0., 0., 0.] + [0.] * self.priv_ori_dim
        assert len(_debug_vec) == self.state_dim, \
            f"debug vector len {len(_debug_vec)} != state_dim {self.state_dim}"
        self.debug_state_vector = torch.tensor(
            [_debug_vec], dtype=torch.float32, device=self.device)
        if self.debug_override_state:
            print(f"[DEBUG] state override ACTIVE -- feeding fixed state every step:\n"
                  f"        {self.debug_state_vector.cpu().numpy()}")


    def reset_h(self, reset_msg):
        if reset_msg == True:
            self.h = None
            self.init_a = np.zeros((1,4))
            self.init_a[0,0] = 0.30
            target_vel = np.zeros((1, 3))
            self.previous_action = torch.tensor(self.init_a, dtype=torch.float32)
            self.policy.reset_hidden(batch_size=1, device=self.previous_action.device, dtype=self.previous_action.dtype)

    def update_window_info(self, window_pose, window_velocity, window_orientation):
        self.window_position = torch.tensor(window_pose, dtype=torch.float32)
        self.window_velocity = torch.tensor(window_velocity, dtype=torch.float32)
        self.window_quaternion = torch.tensor(window_orientation, dtype=torch.float32)
    
    def update_init_pos_drone(self, init_pos):
        init_pos = np.zeros((1,3))
        init_pos[:,0] = -2
        init_pos[:,1] = 0
        init_pos[:,2] = 2.0
        self.init_pos_numpy = init_pos
        self.init_pos_t = torch.tensor(self.init_pos_numpy, dtype=torch.float32)
    
    def depth_to_policy_input(self,depth, max_range, invert=True):
        if isinstance(depth, np.ndarray):
            depth = torch.from_numpy(depth).to(self.device)
        if depth.dim() == 4:
            if depth.shape[1] == 1:
                depth = depth[:, 0]
            elif depth.shape[0] == 1:
                depth = depth[0]
            else:
                raise RuntimeError(f"Unsupported depth shape: {tuple(depth.shape)}")
        if depth.dim() != 3:
            raise RuntimeError(f"Expected [B,H,W] depth, got {tuple(depth.shape)}")
        depth = torch.nan_to_num(depth.float(), nan=max_range, posinf=max_range, neginf=0.0)
        depth = torch.clamp(depth, 0.0, float(max_range)) / float(max_range)
        return 1.0 - depth if invert else depth
    
    def normalized_state(self,q, qd, gate_centers_t, cfg, mar):
        pos = q[:, :3]
        att = self.normalize_quat_xyzw(q[:, 3:])
        diff_pos = gate_centers_t - pos
        room = float(getattr(cfg, "room_size", 5.0))
        pos_scale = torch.tensor(
            getattr(cfg, "state_pos_scale", [room, room, room]),
            device=q.device,
            dtype=q.dtype,
        ).reshape(1, 3)
        vel_scale = float(getattr(cfg, "state_vel_scale", 4.5))
        omega_scale = float(getattr(cfg, "state_angvel_scale", mar))
        qd_n = torch.cat([qd[:, :3] / omega_scale, qd[:, 3:] / vel_scale], dim=1)
        # print(f"pos_scale: {pos_scale}, vel_scale: {vel_scale}, omega_scale: {omega_scale}")
        state = torch.cat([diff_pos / pos_scale, pos / pos_scale, att, qd_n], dim=1)
        clip = float(getattr(cfg, "state_input_clip", 3.0))
        return torch.clamp(state, -clip, clip) if clip > 0.0 else state
    
    def normalize_quat_xyzw(self,q, eps=1e-8):
        return q / q.norm(dim=1, keepdim=True).clamp_min(eps)

    def _publish_policy_depth(self, depth):
        """Publish the exact (1, H, W) depth tensor handed to the policy as a
        mono8 Image (0-255, near=bright since this is post-invert) for live
        viewing in rqt_image_view / RViz. Cheap: no disk I/O, one image per eval."""
        img = (torch.clamp(depth.detach().squeeze(0), 0.0, 1.0) * 255).to(torch.uint8).cpu().numpy()
        msg = self._depth_cv_bridge.cv2_to_imgmsg(img, encoding="mono8")
        msg.header.stamp = rospy.Time.now()
        self.policy_depth_pub.publish(msg)

    def _save_depth_frame(self, depth, raw=None):
        """Save the exact (1, H, W) depth tensor handed to the policy (post
        normalize/invert/resize), and optionally the raw full-resolution depth
        (metres, before any processing). .npy keeps true values; .png is viewable.
        Note: processed png has near=bright (inverted); raw png has near=dark."""
        idx = self._depth_save_count
        try:
            from PIL import Image as PILImage
        except Exception as e:
            PILImage = None
            rospy.logwarn_throttle(10.0, f"[depth-debug] png save skipped: {e}")

        # processed policy input (H, W) float ~[0, 1]
        img = depth.detach().squeeze(0).cpu().numpy()
        np.save(os.path.join(self.depth_save_dir, f"depth_{idx:06d}.npy"), img)
        if PILImage is not None:
            PILImage.fromarray((np.clip(img, 0.0, 1.0) * 255).astype(np.uint8)).save(
                os.path.join(self.depth_save_dir, f"depth_{idx:06d}.png"))

        # raw full-resolution depth in metres, before invert/resize
        if raw is not None:
            raw_np = raw if isinstance(raw, np.ndarray) else raw.detach().cpu().numpy()
            raw_np = np.asarray(raw_np).squeeze()          # (H, W) metres
            np.save(os.path.join(self.depth_save_dir, f"depth_{idx:06d}_raw.npy"), raw_np)
            if PILImage is not None:
                mr = float(getattr(self.args, "max_range", 20.0))
                render = np.clip(raw_np / mr, 0.0, 1.0) * 255   # far=bright, near=dark (like the topic)
                PILImage.fromarray(render.astype(np.uint8)).save(
                    os.path.join(self.depth_save_dir, f"depth_{idx:06d}_raw.png"))

    def evaluate_(self, pos, att, qd, depth_img):
        start_time = time.time()
        ## Concatenating observations
        # print("Depth image shape:", depth_img)
        depth = self.depth_to_policy_input(
            depth_img,
            max_range=float(self.args.max_range),
            invert=bool(getattr(self.args.cfg, "invert_depth", True)),
        )
        # Resize to the CNN's expected size (camera resolution != policy img_size).
        # Training rendered SQUARE images (isotropic pixel scale); the real camera is
        # 640x360 (16:9). A direct resize squashes width ~10x vs height ~5.6x, which
        # distorts silhouette shape cues (e.g. a tilted gate's apparent roll) -- so
        # center-crop to square (matching the vertical FOV) BEFORE resizing, instead
        # of an anisotropic squash straight to img_h x img_w.
        if depth.shape[-2:] != (self.img_h, self.img_w):
            h, w = depth.shape[-2:]
            side = min(h, w)
            top = (h - side) // 2
            left = (w - side) // 2
            depth = depth[:, top:top + side, left:left + side]
            depth = F.interpolate(depth.unsqueeze(1), size=(self.img_h, self.img_w),
                                  mode="bilinear", align_corners=False).squeeze(1)

        # Dump exactly what the policy sees (post resize/normalize/invert), plus the
        # raw full-resolution depth (depth_img) that came in before any processing.
        if self.save_depth:
            self._depth_save_count += 1
            if self._depth_save_count % self.depth_save_every == 0:
                self._save_depth_frame(depth, depth_img)

        # Publish the exact CNN input live (every evaluation) for rqt/RViz viewing.
        if self.publish_policy_depth:
            self._publish_policy_depth(depth)

        # Build the 7-DOF pose (pos + quat) and use the gate center for diff_pos.
        q = torch.cat([pos, att], dim=1)
        gate_centers_t = self.window_position
        # print(gate_centers_t)
        state16 = self.normalized_state(q, qd, gate_centers_t, self.args, self.mar)
        # Privileged gate-orientation slot: fed zeros at test time. Correct, not just
        # a safe fallback -- privileged_beta_schedule() in the training script fades
        # this input's scale to 0 well before training ends (fade_end_epoch well
        # short of the final epoch), so a fully-trained checkpoint has already been
        # weaned onto exactly this all-zero input for the privileged block.
        if self.priv_ori_dim > 0:
            priv_ori = torch.zeros(state16.shape[0], self.priv_ori_dim, dtype=state16.dtype, device=state16.device)
            state = torch.cat([state16, priv_ori], dim=1).to(self.device)
        else:
            state = state16.to(self.device)
        # print(state)
        if self.debug_override_state:
            state = self.debug_state_vector
            print("[DEBUG] state FED TO POLICY (overridden):", state)
        # print(state)
        action = self.policy(state, depth, update_norm=False)
        # print(a)
        end_time = time.time()
        # print(f"Time taken: {end_time - start_time:.4f} seconds")
        return action
    
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
                 window_position, window_velocity, window_degrees,
                 config_param=None):

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

        ##Define initial window location
        self.window_position = np.asarray(window_position, dtype=float).reshape(1, 3)
        self.window_velocity = np.asarray(window_velocity, dtype=float).reshape(1, 3)
        self.window_degrees_nominal = np.asarray(window_degrees, dtype=float).reshape(1)
        self.window_orientation_range = float(config_param.get("window_orientation_range", 0.0)) if config_param else 0.0
        self.window_degrees = self.window_degrees_nominal.copy()
        self.window_quaternion = self.convert_window_degrees_to_quaternion_vector(self.window_degrees)
        if self.window_orientation_range > 0:
            print(f"Window orientation will be randomized ±{self.window_orientation_range} deg around {self.window_degrees_nominal[0]} deg per episode")
        #Update policy with the window information
        self.policy.update_window_info(self.window_position, self.window_velocity, self.window_quaternion)

        ## Saving list for plotting
        self.position_list = []
        self.velocity_list = []
        self.attitude_list = []


        ## End pose: where the drone flies to after passing the gate
        self.end_pos_numpy = np.array([[3.0, 0.0, 2.0]])

        ## Define initial starting location of the drone
        init_pos = np.zeros((1,3))
        init_pos[:,0] = -2
        init_pos[:,1] = 0
        init_pos[:,2] = 2.0
        self.init_pos_numpy = init_pos
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
        self.policy.update_init_pos_drone(self.init_pos_numpy)

        self.max_angular_rates = max_angular_rates
        self.last_pos_time = None
        self.last_odom_time = None

        self.warp_jax = warp_jax
        self.to_transform_odom = to_transform_odom
        self.to_transform_policy = to_transform_policy
        

        # Optionally use /drone0/mavros/vision_pose/pose (e.g. mocap) for gate metrics, if it's actually publishing
        try:
            rospy.wait_for_message("/drone0/mavros/vision_pose/pose", PoseStamped, timeout=2.0)
            self.has_vision_pose = True
            print("Detected active /drone0/mavros/vision_pose/pose - using it for gate metrics")
        except rospy.ROSException:
            self.has_vision_pose = False
            print("/drone0/mavros/vision_pose/pose has no incoming messages - using local_position/pose for gate metrics")

        self.vision_pos = np.zeros(3)
        self.vision_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.vision_position_list = []
        self.vision_attitude_list = []
        if self.has_vision_pose:
            self.vision_pose_sub_ = rospy.Subscriber("/drone0/mavros/vision_pose/pose", PoseStamped, self.visionPoseCb, queue_size=10)

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

        # Depth camera for the vision policy. Latest frame (H, W) in metres, or None
        # until the first image arrives.
        self.latest_depth = None
        self.depth_lock = threading.Lock()
        self._cv_bridge = CvBridge()
        self.depth_topic = rospy.get_param("~depth_topic", "/agent001/stereo_left_depth")
        # mono8 -> metres, matching training's depth_norm: depth = (pixel/255) * far.
        self.depth_cam_far = float(rospy.get_param("~depth_cam_far", 20.0))
        self.depth_sub_ = rospy.Subscriber(self.depth_topic, Image, self.depthCb, queue_size=1)
        rospy.loginfo("[vision] depth topic: %s | mono8 linear decode far=%.2f m",
                      self.depth_topic, self.depth_cam_far)

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

        # The policy is a GRU trained at the sim step (config delta_time, 20 Hz for the
        # vision runs) but this timer fires at 100 Hz. Stepping the GRU 5x too fast
        # distorts its recurrent dynamics, so only evaluate every _eval_stride ticks and
        # hold the last action in between.
        POLICY_EVAL_DT = 0.01
        train_dt = float(config_param.get("delta_time", POLICY_EVAL_DT)) if config_param else POLICY_EVAL_DT
        self._eval_stride = max(1, int(round(train_dt / POLICY_EVAL_DT)))
        self._eval_count = 0
        print(f"[vision] policy eval stride = {self._eval_stride} "
              f"(train dt={train_dt}s -> {1.0/train_dt:.0f}Hz, timer {1/POLICY_EVAL_DT:.0f}Hz)")

        time.sleep(1)
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

    def visionPoseCb(self, msg):
        self.vision_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.vision_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])

    def odomCb(self, msg):
        self.drone_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z])
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
                if warp_pos[0, 0] < self.window_position[0, 0] + 0.05:
                # if warp_pos[0, 2] < 5.0:
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
                
                if (self.drone_pos[0] - self.window_position[:,0]) > 0.1:
                # if self.drone_pos[2] > 5.0:
                    #Means drone has passed gate. Switch back to position control.
                    print("in here for switching back")
                    self.individual_recording = False
                    self.init_pos_numpy = self.end_pos_numpy.copy()
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
                    if self.has_vision_pose:
                        self.vision_position_list.append(self.vision_pos)
                        self.vision_attitude_list.append(self.vision_quat)

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
                        if self.has_vision_pose:
                            self.position_array = np.array(self.vision_position_list)
                            self.attitude_array = np.array(self.vision_attitude_list)
                        else:
                            self.position_array = np.array(self.position_list)
                            self.attitude_array = np.array(self.attitude_list)
                        self.velocity_array = np.array(self.velocity_list)
                        gate_center = self.window_position[0]       # (3,) [x, y, z]
                        window_quat = self.window_quaternion[0]     # (4,) [x, y, z, w]
                        target_vel  = self.window_velocity[0]       # (3,) desired velocity at crossing
                        t_star, pos_at_gate, pos_err, vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot = \
                            compute_gate_metrics(self.position_array, self.velocity_array,
                                                 self.attitude_array, window_quat,
                                                 gate_center=gate_center, target_vel=target_vel)
                        plot_spatial_plots(self.position_array[:,None,:])
                        gate_world = gate_geometry(self.window_degrees[0], gate_center=gate_center)
                        plot_gate_travesal(self.position_array[:,None,:], self.attitude_array[:,None,:],
                                           gate_world, t_star, pos_at_gate, pos_err,
                                           vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot)
                        plot_metrics_timeseries(self.position_array, self.velocity_array, self.attitude_array,
                                                window_quat, t_star, pos_at_gate, pos_err,
                                                vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot)
                        if self.record_now:
                            rospy.Timer(rospy.Duration(0.5), self._append_separator_and_restart_cb, oneshot=True)
                        elif self.pending_save:
                            rospy.Timer(rospy.Duration(0.1), self._flush_and_save_cb, oneshot=True)
                        self.position_list = []
                        self.velocity_list = []
                        self.attitude_list = []
                        if self.has_vision_pose:
                            self.vision_position_list = []
                            self.vision_attitude_list = []

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
        wx, wy, wz = float(self.window_position[0, 0]), float(self.window_position[0, 1]), float(self.window_position[0, 2])
        x = random.uniform(wx - 4.0, wx - 3.0)
        y = random.uniform(wy - 0.5, wy + 0.5)
        z = random.uniform(wz - 0.5, wz + 0.5)
    
        # x = -2.0
        # y = 0.0
        # z = 1.5
        new_pos = np.array([[x, y, z]])
        self.init_pos_numpy = new_pos
        # self.init_quat = self.update_init_orientation_drone(new_pos, self.window_position)
        self.init_quat = np.array([[0.0, 0.0, 0.0, 1.0]])
        self.policy.update_init_pos_drone(new_pos)
        self.curr_init_pose.pose.position.x = float(new_pos[0, 0])
        self.curr_init_pose.pose.position.y = float(new_pos[0, 1])
        self.curr_init_pose.pose.position.z = float(new_pos[0, 2])
        self.curr_init_pose.pose.orientation.x = float(self.init_quat[0, 0])
        self.curr_init_pose.pose.orientation.y = float(self.init_quat[0, 1])
        self.curr_init_pose.pose.orientation.z = float(self.init_quat[0, 2])
        self.curr_init_pose.pose.orientation.w = float(self.init_quat[0, 3])

        # Randomize window orientation per episode if trained with a range
        if self.window_orientation_range > 0:
            angle_deg = np.round((self.window_degrees_nominal[0] + random.uniform(-self.window_orientation_range, self.window_orientation_range)) / 10) * 10
            print(f"windows_nominal: {self.window_degrees_nominal}")
            print(angle_deg)
            self.window_degrees = np.array([angle_deg])
            # self.window_degrees = np.array([60])
            self.window_quaternion = self.convert_window_degrees_to_quaternion_vector(self.window_degrees)
            self.policy.update_window_info(self.window_position, self.window_velocity, self.window_quaternion)
            print(f"New init position: x={x:.2f}, y={y:.2f}, z={z:.2f}  |  window_orientation={angle_deg:.1f} deg")
        else:
            print(f"New init position: x={x:.2f}, y={y:.2f}, z={z:.2f}")

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


    def depth_norm(self, depth):
        """Verbatim training decode: depth = ((pixel - 0) / (255 - 0)) * far."""
        return ((depth - 0) / (255 - 0)) * self.depth_cam_far

    def depthCb(self, msg):
        # passthrough: no cv_bridge re-encoding, native dtype (uint8 for mono8/8UC1).
        depth_unnorm = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding in ('mono8', '8UC1'):
            depth = self.depth_norm(depth_unnorm.astype(np.float32))
        else:
            # already metric (e.g. 32FC1 metres, 16UC1 mm -> metres)
            depth = depth_unnorm.astype(np.float32)
            if msg.encoding in ('16UC1', 'mono16'):
                depth = depth * 0.001
        with self.depth_lock:
            self.latest_depth = depth

    def nn_evaluation(self, event):
        # Only step the GRU policy at the training rate; hold self.action in between.
        self._eval_count += 1
        if self._eval_count % self._eval_stride != 0:
            return
        with self.depth_lock:
            depth = self.latest_depth
        if depth is None:
            return  # no depth frame received yet
        warp_q = self.warp_q[3:]
        warp_pos = torch.Tensor(self.warp_q[:3]).unsqueeze(0)
        warp_q = torch.Tensor(warp_q).unsqueeze(0)
        warp_qd = torch.Tensor(self.warp_qd).unsqueeze(0)
        depth_batched = depth[None]  # (1, H, W) expected by depth_to_policy_input
        self.action = self.policy.evaluate_(warp_pos, warp_q, warp_qd, depth_batched)
        # print(np.min(depth_batched))
        # print(self.action)




if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")
    policy_file = "20260828-095458"#"20260827-103303" not smooth #"20260825-183140"#"20260825-085946" smooth #"20260824-093459" not smooth #"20260702-145013" To test fly real drone #"20260629-091116" This is another good 60 degrees demo #"20260626-204830" #"20260618-005845" very bad#"20260618-005738"also pretty good #"20260618-005702" a bit vibratory #"20260618-005626" bad #"20260617-201947" bad #"20260617-201914 best tracking reasonable in flight" #"20260617-201703 worse tracking" #"20260617-172533"# This likely to work #"20260616-174028" to test in real flight #"20260616-150838" #"20260608-230203" bad 60 degrees. To compare with 20260608-170520 #"20260608-233141" good 30 degrees for gazebo trained with thrust DR also #"20260608-170520 good demo for 60 degrees gazebo. max body rates of 4.0 "#"20260604-222931" #"20260604-201453 good 30 degrees demo" #"20260604-095607" #"20260603-121142" #"20260603-121213" another 60 degrees gazebo demo. To test in real #"20260529-113935 60 degrees gazebo demo" #"20260521-090040" #"20260518-213037 30 degrees demo" #"20260519-121802" #"20260513-185143" #"20260513-185035" #"20260402-204842 This is high fidelity forward model." #"20260306-154450 - with gru. more reasonable" #"20260304-161010" #"20260304-160736 - this reasonable"#"20260225-165700" #0.02 good enough for real drone 20250527-122703   0.05-to test 20250624-181715
    print(f"POLICY PATH IS {policy_file}") 
    recovery_mode = 2 #1 for position, 2 for velocity, 3 for attitude

    rospack = rospkg.RosPack()
    path = rospack.get_path('nn_policy')
    full_path = os.path.join(path, "logs/vel_tracking_depth_cbam_gru_multi_env_full")
    print(f"The full path is {full_path}")
    actual_full_path = os.path.join(full_path, policy_file)
    config_path = os.path.join(actual_full_path,"training_config.yaml")
    full_policy_path = os.path.join(actual_full_path, "checkpoint_latest.pth")

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

    config_params.setdefault("warp_frame", 1.0)  # vision configs may omit it; z-up warp default
    config_params["policy_global"] = 1.0          # treat as a global policy
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

    
    # Prefer window_location_center_range (the actual per-axis [min,max] sampling range
    # used to place the gate during training's domain randomization) over window_location,
    # which can be stale/unused for some checkpoints. Take the midpoint per axis; when
    # min==max (a fixed-gate run) this is just that fixed value.
    if "window_location_center_range" in config_params:
        _range = config_params["window_location_center_range"]                      # [[xmin,xmax],[ymin,ymax],[zmin,zmax]]
        window_position = [(_lo + _hi) / 2.0 for _lo, _hi in _range]
        print(f"window_position from window_location_center_range midpoint: {window_position}")
    else:
        window_position = config_params["window_location"]                          # [x, y, z]
    window_speed = config_params.get("target_traversal_speed", 2.0)
    window_velocity = np.array([window_speed, 0.0, 0.0])   # desired velocity at crossing, in the frame of the window
    window_degrees  = config_params["window_orientation"]                           # rotation about X in degrees

    #vel 20250424-161234 #position 20250424-131220, 20250424-161345

    include_gate_ori = config_params.get("include_gate_orientation_in_obs", False)
    use_rotmat_obs = config_params.get("use_rotation_matrix_obs", False)
    use_so3_diff_obs = config_params.get("use_so3_diff_obs", False)
    window_degrees = 40.0 #for now hardcoding this. can be changed later to config param

    # Build the args object the vision policy needs (height/width/max_range/state scales/
    # invert_depth). All training-config keys become attributes; args.cfg self-references
    # so args.cfg.invert_depth resolves too.
    args = SimpleNamespace(**config_params)
    args.cfg = args

    nn_policy = TEST_RENDER(args, full_policy_path, position_control, warp_frame, use_gru=use_gru, include_actions=gru_include_prev_action, include_gate_ori=include_gate_ori, use_rotmat_obs=use_rotmat_obs, use_so3_diff_obs=use_so3_diff_obs)

    nn_policy_planner = NN_POLICY_PLANNER(mission_command_mode=int(mission_command_mode), policy=nn_policy,
                                          inference_timestep=delta_time, max_angular_rates = max_angular_rate,
                                          warp_jax=warp_jax, to_transform_odom=to_transform_odom, to_transform_policy=to_transform_policy, recovery_mode=recovery_mode,
                                          window_position=window_position, window_velocity=window_velocity, window_degrees=window_degrees,
                                          config_param=config_params)

    rospy.spin()

    print("done")

