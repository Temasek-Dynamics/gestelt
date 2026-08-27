#!/usr/bin/env python3
import sys

print("Running with Python:", sys.executable)
import time
import numpy as np
import rospy
import yaml
import os
import rospkg
from std_msgs.msg import Int8
from gestelt_msgs.msg import CommanderState, ExecTrajectory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from enum import Enum
import roslib.packages
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
import tf2_ros
import threading
from std_msgs.msg import Int8
from mavros_msgs.msg import AttitudeTarget
import cv2
from cv_bridge import CvBridge
import torch

# The ROS node is launched from the scripts/ directory, while the migrated
# network definition lives one level up in nn_policy/modules. Add the package
# root explicitly so this script always imports the same PolicyNetwork that was
# used by train_velpos_simple_replicate_nwu_global.py.
NN_POLICY_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if NN_POLICY_DIR not in sys.path:
    sys.path.insert(0, NN_POLICY_DIR)

from modules.policy_simple import PolicyNetwork

from signal import signal, SIGINT

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


class VisionObstacleAvoidancePolicy(object):
    """Runtime wrapper around the trained NWU-global vision policy.

    The migrated architecture uses one PolicyNetwork that consumes both the
    low-dimensional state and the depth image. This wrapper keeps the ROS-facing
    API small: update the target position, evaluate the policy, and return a
    numpy action for publishers.
    """

    def __init__(self, policy_path, config_params, pc, warp_frame, device="cpu"):
        # Training-time mode selector. The current 20260511-121842 checkpoint
        # has position_control=false, so it uses the 10-D velocity-control
        # observation below. Keeping the branch here lets position-control
        # checkpoints run without changing the ROS node.
        self.pc = pc
        self.warp_frame = warp_frame
        self.device = torch.device(device)

        # These values must match the checkpoint's training_config.yaml.
        # width/height define the CNN input resolution, not the Unity source
        # image resolution. Unity publishes 640x360; depthCB downsamples that
        # stream to this 64x64 network input.
        self.max_speed = float(config_params.get("max_speed", 1.0))
        self.image_width = int(config_params["width"])
        self.image_height = int(config_params["height"])
        self.max_depth = float(config_params.get("max_range", 20.0))

        # Observation dimensions match the training script:
        #   position-control: diff_pos(3) + att(4) + qd(6) + target_vel(3) = 16
        #   velocity-control: att(4) + angular_vel(3) + diff_vel(3) = 10
        input_dim = 16 if self.pc else 10
        self.policy = PolicyNetwork(
            input_dim=input_dim,
            output_dim=4,
            img_size=(self.image_height, self.image_width),
        ).to(self.device)

        # map_location allows a GPU-trained checkpoint to run on a CPU-only
        # ROS machine. eval() disables training-time behavior and keeps
        # inference deterministic for this feed-forward network.
        self.policy.load_state_dict(torch.load(policy_path, map_location=self.device))
        self.policy.eval()

        # Default target used before /drone0/warp/local_position/target_position
        # publishes. target_vel is only used by the position-control branch.
        target_vel = np.zeros((1, 3), dtype=np.float32)
        target_pos = np.array([[5.0, 0.0, 1.0]], dtype=np.float32)
        self.t_vel = torch.tensor(target_vel, dtype=torch.float32, device=self.device)
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32, device=self.device)
        self.last_diff_vel = None
        self.last_pos = None
        self.last_att = None
        self.last_angvel = None
        self.last_vel = None
        self.last_target_vel = None
        self.last_obs = None
        self.last_depth_stats = np.array([np.nan, np.nan, np.nan], dtype=np.float32)

    def evaluate_(self, pos, att, qd, depth_img):
        """Build the policy observation and run one forward pass.

        Args:
            pos: [1, 3] position in the same NWU/global frame used for training.
            att: [1, 4] quaternion in xyzw order.
            qd:  [1, 6] body/state velocity vector. The convention inherited
                 from training is angular velocity first, then linear velocity.
            depth_img: [1, H, W] metric depth tensor after ROS preprocessing.

        Returns:
            [1, 4] numpy action. action[:, 0] is normalized thrust in [0, 1],
            action[:, 1:4] are normalized body rates in [-1, 1].
        """
        pos = pos.to(self.device)
        att = att.to(self.device)
        att = torch.where(att[:, 3:4] < 0.0, -att, att)
        qd = qd.to(self.device)
        if depth_img is not None:
            depth_img = depth_img.to(self.device)
            depth_cpu = depth_img.detach().cpu().numpy()
            self.last_depth_stats = np.array(
                [np.nanmin(depth_cpu), np.nanmean(depth_cpu), np.nanmax(depth_cpu)],
                dtype=np.float32,
            )
        else:
            self.last_depth_stats = np.array([np.nan, np.nan, np.nan], dtype=np.float32)

        self.last_pos = pos.detach().cpu().numpy()
        self.last_att = att.detach().cpu().numpy()
        self.last_angvel = qd[:, :3].detach().cpu().numpy()
        self.last_vel = qd[:, 3:].detach().cpu().numpy()

        if self.pc:
            # Position-control observation matches the training script exactly:
            # desired displacement, attitude, angular+linear velocity, and a
            # fixed target velocity.
            diff_pos = self.t_pos - pos
            obs = torch.cat((diff_pos, att, qd, self.t_vel), dim=1)
            self.last_diff_vel = None
            self.last_target_vel = self.t_vel.detach().cpu().numpy()
        else:
            # Velocity-control training did not use a fixed target velocity.
            # It pointed the desired velocity from current position toward the
            # target position, scaled it to max_speed, then used the difference
            # from current linear velocity as the network input.
            vel = qd[:, 3:]
            angvel = qd[:, :3]
            diff_pos = self.t_pos - pos
            target_vel = diff_pos / (torch.norm(diff_pos, dim=1, keepdim=True) + 1e-8)
            target_vel = target_vel * self.max_speed
            diff_vel = target_vel - vel
            obs = torch.cat((att, angvel, diff_vel), dim=1)
            self.last_diff_vel = diff_vel.detach().cpu().numpy()
            self.last_target_vel = target_vel.detach().cpu().numpy()

        self.last_obs = obs.detach().cpu().numpy()

        with torch.no_grad():
            action = self.policy(obs, depth_img)

        return action.detach().cpu().numpy()

    def update_target_pos(self, target_pos):
        # ROS callback input is a length-3 numpy array. Store it as [1, 3] so
        # the policy observation keeps a batch dimension.
        target_pos = np.asarray(target_pos, dtype=np.float32).reshape(1, 3)
        self.t_pos = torch.tensor(target_pos, dtype=torch.float32, device=self.device)
    

class NN_POLICY_PLANNER(object):
    """ROS integration layer for the neural policy.

    This class owns all ROS subscribers/publishers and the state machine that
    switches between normal PVA control and NN attitude/body-rate control. It
    deliberately keeps neural-network details inside VisionObstacleAvoidancePolicy
    so the ROS plumbing is separated from checkpoint loading and observation
    construction.
    """

    def __init__(self, mission_command_mode, policy, inference_timestep, max_angular_rates, warp_jax,
                 to_transform_odom, to_transform_policy):
        # Shared ROS utilities. CvBridge converts sensor_msgs/Image into numpy;
        # tfBuffer is kept for the legacy transform path in _pose_odom_pub_callback.
        self.bullet_sim_mutex = threading.Lock()
        self.tfBuffer =  tf2_ros.Buffer(rospy.Duration(10))
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.warp_pose_msg = PoseStamped()
        self.bridge = CvBridge()

        # Latest preprocessed depth frame. The network timer and the camera
        # callback run independently; nn_evaluation always consumes the newest
        # tensor stored here. This avoids coupling Unity's 30 Hz image stream to
        # the policy inference frequency.
        self.max_depth = float(getattr(policy, "max_depth", 20.0))
        self.depth_width = int(getattr(policy, "image_width", 64))
        self.depth_height = int(getattr(policy, "image_height", 64))
        # self.depth_image = None
        self.depth_image = torch.full((1, self.depth_height, self.depth_width), 20.0)

        # Unity publishes 640x360 depth. The trained CNN expects depth_width x
        # depth_height from training_config.yaml, currently 64x64. A mismatch in
        # the source stream is not fatal because the preprocessor resizes, but it
        # is logged so simulator/camera settings are easy to catch.
        self.unity_depth_width = 640
        self.unity_depth_height = 360
        rospy.sleep(1)

        self.max_angular_rates = max_angular_rates
        self.policy = policy
        self.last_pos_time = None
        self.last_odom_time = None
        self.fixed_runtime_target_pos = np.array([20.0, 0.0, 1.0], dtype=np.float32)

        self.warp_jax = warp_jax
        self.to_transform_odom = to_transform_odom
        self.to_transform_policy = to_transform_policy
        self._init_runtime_debug_log()
        

        # Mission/state topics from the existing traj_server stack.
        self.swarm_mode_pub_ = rospy.Publisher('/traj_server/swarm_command', Int8, queue_size=5)
        self.commander_state_sub_ = rospy.Subscriber("/drone0/traj_server/state",CommanderState, self.commStateCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/pose",PoseStamped, self.poseCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/drone0/mavros/local_position/odom",Odometry, self.odomCb, queue_size = 10)
        self.drone_pose_sub_ = rospy.Subscriber("/mode_change", Bool, self.modeChgCb, queue_size = 10)
        self.mission_mode_sub_ = rospy.Subscriber("/traj_server/warp_mission_command", Int8, self.missionModeCb, queue_size = 5)
        self.target_position_sub_ = rospy.Subscriber("/drone0/warp/local_position/target_position", PoseStamped, self.targetPosCb, queue_size = 5)
        
        self.warp_drone_pose_pub_ = rospy.Subscriber('/drone0/warp/local_position/pose', PoseStamped, self.warpPoseCB, queue_size=5)
        self.warp_drone_odom_sub_ = rospy.Subscriber('/drone0/warp/local_position/odom', Odometry, self.warpOdomCB, queue_size=5)
        self.geom_controller_sub_ = rospy.Subscriber('/drone0/setpoint_raw/attitude', AttitudeTarget, self.geomCB, queue_size=5)
        self.geom_controller_pub_ = rospy.Publisher("/drone0/geom_ctrl", AttitudeTarget, queue_size = 5)

        # Keep only the newest Unity frame. With a 30 Hz depth stream and a
        # 20 Hz policy timer, larger queues would increase latency by letting
        # inference process stale frames.
        self.depth_sub = rospy.Subscriber('/agent001/mono_camera_depth', Image, self.depthCB, queue_size=1)
  
        # ExecTrajectory is used both for the initial PVA command and for the
        # NN attitude/body-rate command when mission mode switches to mode 2.
        self.pva_traj_pub_ = rospy.Publisher("/drone0/planner_adaptor/exec_trajectory", ExecTrajectory, queue_size = 5)
        self.mission_mode_pub_ = rospy.Publisher("/traj_server/mission_command", Int8, queue_size = 5, latch=False)
        

        # eventCB drives mission-state transitions at the same configured
        # cadence as the trained policy timestep.
        self.event_manager = rospy.Timer(rospy.Duration(inference_timestep), self.eventCB)

        #DRONE STATE MACHINE
        self.drone_state = 0
        self.servent_event = 0

        # Raw mavros state and the transformed/warp-frame state consumed by the
        # policy. warp_q is [x, y, z, qx, qy, qz, qw]; warp_qd is
        # [wx, wy, wz, vx, vy, vz].
        self.drone_pos = np.zeros((3,1))
        self.drone_qd = np.zeros((3,1))
        self.drone_quat = np.zeros((4,1))

        self.warp_pos = np.zeros(3)
        self.warp_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.warp_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        self.warp_qd = np.zeros(6)
        
        self.mission_command_mode = 1
        # Default to NN attitude/body-rate control after takeoff. Without this,
        # the node stays in PVA mode and keeps publishing the fixed setpoint
        # (0, 0, 1), so the drone only takes off and hovers.
        self.warp_mission_command_mode = 2
        # The migrated network outputs normalized body rates, not an attitude
        # quaternion, so ExecTrajectory.type_mask must be 1 by default.
        self.attitude_mode_toggle = 1
        self.action = np.zeros((1,4))
        self.has_nn_action = False
        self.last_published_command = "none"
        self.last_published_type_mask = -1
        self.last_published_body_rate_x = np.nan
        self.last_published_body_rate_y = np.nan
        self.last_published_body_rate_z = np.nan
        self.last_published_velocity_x = np.nan
        self.last_published_velocity_y = np.nan
        self.last_published_velocity_z = np.nan

        # NN handoff gate: take off/hold altitude in PVA, accelerate forward,
        # then switch to NN body-rate control as soon as forward speed reaches
        # the training-time 1 m/s initial condition.
        # Command a little above the switch threshold so the velocity
        # controller crosses 1 m/s instead of asymptotically stopping below it.
        self.handoff_velocity_command = 1.2
        self.handoff_position_lookahead = 0.0
        self.handoff_use_target_direction = False
        self.nn_handoff_speed = 1.0
        self.nn_handoff_min_altitude = 0.8
        self.nn_handoff_require_altitude = False
        self.nn_handoff_hold_time = 0.0
        self.nn_handoff_active = False
        self.nn_handoff_ready_since = None
        self.nn_handoff_announced = False
        self.last_handoff_forward_speed = 0.0
        self.last_handoff_altitude = 0.0
        self.last_handoff_gate_ready = False
        self.last_handoff_held_time = 0.0
        time.sleep(1)
        # Use the training delta_time from the checkpoint config. For the
        # current policy this is 0.05 s (20 Hz), while Unity can publish depth
        # at 30 Hz; the latest-frame buffer above bridges that rate difference.
        self.policy_evaluation_timer = rospy.Timer(rospy.Duration(inference_timestep), self.nn_evaluation)
        
    def _init_runtime_debug_log(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        self.runtime_log_dir = os.path.join(script_dir, "logs")
        os.makedirs(self.runtime_log_dir, exist_ok=True)

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.runtime_log_path = os.path.join(
            self.runtime_log_dir,
            f"runtime_diff_vel_{timestamp}.csv",
        )
        self.runtime_log_file = open(self.runtime_log_path, "w", buffering=1)
        self.runtime_log_file.write(
            "stamp,diff_vel_x,diff_vel_y,diff_vel_z,"
            "pos_x,pos_y,pos_z,"
            "att_x,att_y,att_z,att_w,"
            "angvel_x,angvel_y,angvel_z,"
            "vel_x,vel_y,vel_z,"
            "target_vel_x,target_vel_y,target_vel_z,"
            "depth_min,depth_mean,depth_max,"
            "action_throttle_norm,"
            "action_body_rate_x_norm,action_body_rate_y_norm,action_body_rate_z_norm,"
            "action_body_rate_x_scaled,action_body_rate_y_scaled,action_body_rate_z_scaled,"
            "throttle,"
            "mission_command_mode,warp_mission_command_mode,"
            "last_published_command,last_published_type_mask,"
            "published_body_rate_x,published_body_rate_y,published_body_rate_z,"
            "published_velocity_x,published_velocity_y,"
            "published_velocity_z,handoff_forward_speed,handoff_altitude,"
            "handoff_gate_ready,handoff_held_time,handoff_active\n"
        )
        rospy.on_shutdown(self.close_runtime_debug_log)
        print(f"Runtime debug log: {self.runtime_log_path}")

    def close_runtime_debug_log(self):
        log_file = getattr(self, "runtime_log_file", None)
        if log_file is not None and not log_file.closed:
            log_file.close()

    def log_runtime_debug(self):
        diff_vel = getattr(self.policy, "last_diff_vel", None)
        if diff_vel is None:
            return

        diff_vel = np.asarray(diff_vel, dtype=np.float64).reshape(-1, 3)[0]
        pos = np.asarray(getattr(self.policy, "last_pos", np.full((1, 3), np.nan)), dtype=np.float64).reshape(-1, 3)[0]
        att = np.asarray(getattr(self.policy, "last_att", np.full((1, 4), np.nan)), dtype=np.float64).reshape(-1, 4)[0]
        angvel = np.asarray(getattr(self.policy, "last_angvel", np.full((1, 3), np.nan)), dtype=np.float64).reshape(-1, 3)[0]
        vel = np.asarray(getattr(self.policy, "last_vel", np.full((1, 3), np.nan)), dtype=np.float64).reshape(-1, 3)[0]
        target_vel = np.asarray(getattr(self.policy, "last_target_vel", np.full((1, 3), np.nan)), dtype=np.float64).reshape(-1, 3)[0]
        depth_stats = np.asarray(getattr(self.policy, "last_depth_stats", np.full(3, np.nan)), dtype=np.float64).reshape(3)
        action_throttle_norm = float(self.action[0, 0])
        action_body_rate_x_norm = float(self.action[0, 1])
        action_body_rate_y_norm = float(self.action[0, 2])
        action_body_rate_z_norm = float(self.action[0, 3])
        action_body_rate_x_scaled = action_body_rate_x_norm * self.max_angular_rates
        action_body_rate_y_scaled = action_body_rate_y_norm * self.max_angular_rates
        action_body_rate_z_scaled = action_body_rate_z_norm * self.max_angular_rates
        throttle = action_throttle_norm
        published_body_rate_x = float(self.last_published_body_rate_x)
        published_body_rate_y = float(self.last_published_body_rate_y)
        published_body_rate_z = float(self.last_published_body_rate_z)
        published_velocity_x = float(self.last_published_velocity_x)
        published_velocity_y = float(self.last_published_velocity_y)
        published_velocity_z = float(self.last_published_velocity_z)
        stamp = rospy.Time.now().to_sec()

        print(
            "action "
            f"[{action_throttle_norm:.4f}, "
            f"{action_body_rate_x_norm:.4f}, "
            f"{action_body_rate_y_norm:.4f}, "
            f"{action_body_rate_z_norm:.4f}]"
        )
        self.runtime_log_file.write(
            f"{stamp:.6f},{diff_vel[0]:.9f},{diff_vel[1]:.9f},{diff_vel[2]:.9f},"
            f"{pos[0]:.9f},{pos[1]:.9f},{pos[2]:.9f},"
            f"{att[0]:.9f},{att[1]:.9f},{att[2]:.9f},{att[3]:.9f},"
            f"{angvel[0]:.9f},{angvel[1]:.9f},{angvel[2]:.9f},"
            f"{vel[0]:.9f},{vel[1]:.9f},{vel[2]:.9f},"
            f"{target_vel[0]:.9f},{target_vel[1]:.9f},{target_vel[2]:.9f},"
            f"{depth_stats[0]:.9f},{depth_stats[1]:.9f},{depth_stats[2]:.9f},"
            f"{action_throttle_norm:.9f},"
            f"{action_body_rate_x_norm:.9f},{action_body_rate_y_norm:.9f},{action_body_rate_z_norm:.9f},"
            f"{action_body_rate_x_scaled:.9f},{action_body_rate_y_scaled:.9f},{action_body_rate_z_scaled:.9f},"
            f"{throttle:.9f},"
            f"{self.mission_command_mode},{self.warp_mission_command_mode},"
            f"{self.last_published_command},{self.last_published_type_mask},"
            f"{published_body_rate_x:.9f},{published_body_rate_y:.9f},"
            f"{published_body_rate_z:.9f},{published_velocity_x:.9f},"
            f"{published_velocity_y:.9f},{published_velocity_z:.9f},"
            f"{self.last_handoff_forward_speed:.9f},"
            f"{self.last_handoff_altitude:.9f},"
            f"{int(self.last_handoff_gate_ready)},"
            f"{self.last_handoff_held_time:.9f},"
            f"{int(self.nn_handoff_active)}\n"
        )

    def commStateCb(self,msg):
        self.drone_state = DRONESTATE[msg.traj_server_state].value

    def missionModeCb(self,msg):
        self.warp_mission_command_mode = msg.data
        print(f"Mission Mode changed to {self.warp_mission_command_mode}")

    def publish_mission(self, mission_num):
        mission_idx = Int8()
        mission_idx.data = int(mission_num)
        self.swarm_mode_pub_.publish(mission_idx)

    def poseCb(self, msg):
        self.drone_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.drone_quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        if self.to_transform_odom == 0.0:
            # No external warp transform is requested, so the mavros local pose
            # is already treated as the policy frame.
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
            # qd convention must stay aligned with training:
            # angular velocity first, linear velocity second.
            self.warp_qd = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z, msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z ])
        if self.last_odom_time is not None:
           time_diff = (msg.header.stamp- self.last_odom_time).to_sec() 
           if time_diff > 0.03:
               print(f"TIME DIFFERENCE ODOM EXCEEDED!!! {time_diff} at {msg.header.stamp}")
        self.last_odom_time = msg.header.stamp

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
        # The policy internally converts this target position into a desired
        # velocity direction for velocity-control checkpoints.
        self.policy.update_target_pos(self.warp_target_pos)

    def eventCB(self, event):
        # High-level mission automation: take off from IDLE, switch to mission
        # from HOVER, then continuously publish whichever command mode is active.
        if self.drone_state == DRONESTATE["IDLE"].value:
            self.publish_mission(ServerEvent["TAKEOFF_E"].value)
        elif self.drone_state == DRONESTATE["HOVER"].value:
            self.publish_mission(ServerEvent["MISSION_E"].value)
        elif self.drone_state == DRONESTATE["MISSION"].value:
            self.executeMission()

    def publishPVA(self):
        # Baseline PVA setpoint used before the NN action is ready, or when the
        # velocity handoff needs to recover altitude.
        pva_traj_msg = ExecTrajectory()
        pva_traj_msg.transform.translation.x = 0.0
        pva_traj_msg.transform.translation.y = 0.0
        pva_traj_msg.transform.translation.z = 1.0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0
        pva_traj_msg.transform.rotation.w = 1.0
        pva_traj_msg.type_mask = 2048

        #Publish the PVA
        self.pva_traj_pub_.publish(pva_traj_msg)
        self.last_published_command = "pva"
        self.last_published_type_mask = pva_traj_msg.type_mask
        self.last_published_body_rate_x = np.nan
        self.last_published_body_rate_y = np.nan
        self.last_published_body_rate_z = np.nan
        self.last_published_velocity_x = np.nan
        self.last_published_velocity_y = np.nan
        self.last_published_velocity_z = np.nan

    def publishVEL(self):
        pva_traj_msg = ExecTrajectory()
        velocity_dir = self.get_handoff_velocity_direction()
        pva_traj_msg.transform.translation.x = 0.0
        pva_traj_msg.transform.translation.y = 0.0
        pva_traj_msg.transform.translation.z = 1.0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0 #0.707
        pva_traj_msg.transform.rotation.w = 1.0 #0.707
        vx = velocity_dir[0] * self.handoff_velocity_command
        vy = velocity_dir[1] * self.handoff_velocity_command
        vz = 0.0
        pva_traj_msg.velocity.linear.x = vx
        pva_traj_msg.velocity.linear.y = vy
        pva_traj_msg.velocity.linear.z = vz
        pva_traj_msg.type_mask = 2048

        #Publish the PVA
        self.pva_traj_pub_.publish(pva_traj_msg)
        self.last_published_command = "vel"
        self.last_published_type_mask = pva_traj_msg.type_mask
        self.last_published_body_rate_x = np.nan
        self.last_published_body_rate_y = np.nan
        self.last_published_body_rate_z = np.nan
        self.last_published_velocity_x = vx
        self.last_published_velocity_y = vy
        self.last_published_velocity_z = vz

    def publishPVAHandoff(self):
        pva_traj_msg = ExecTrajectory()
        velocity_dir = self.get_handoff_velocity_direction()
        current_pos = np.asarray(self.warp_q[:3], dtype=np.float64)

        vx = velocity_dir[0] * self.handoff_velocity_command
        vy = velocity_dir[1] * self.handoff_velocity_command
        vz = 0.0

        pva_traj_msg.transform.translation.x = current_pos[0] + velocity_dir[0] * self.handoff_position_lookahead
        pva_traj_msg.transform.translation.y = current_pos[1] + velocity_dir[1] * self.handoff_position_lookahead
        pva_traj_msg.transform.translation.z = 1.0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.0
        pva_traj_msg.transform.rotation.w = 1.0
        pva_traj_msg.velocity.linear.x = vx
        pva_traj_msg.velocity.linear.y = vy
        pva_traj_msg.velocity.linear.z = vz
        pva_traj_msg.type_mask = 2048

        self.pva_traj_pub_.publish(pva_traj_msg)
        self.last_published_command = "pva_handoff"
        self.last_published_type_mask = pva_traj_msg.type_mask
        self.last_published_body_rate_x = np.nan
        self.last_published_body_rate_y = np.nan
        self.last_published_body_rate_z = np.nan
        self.last_published_velocity_x = vx
        self.last_published_velocity_y = vy
        self.last_published_velocity_z = vz

    def publishGeomCtrl(self):
        pva_traj_msg = AttitudeTarget()
        pva_traj_msg.body_rate.x = self.geom_body_rate[0]
        pva_traj_msg.body_rate.y = self.geom_body_rate[1]
        pva_traj_msg.body_rate.z = self.geom_body_rate[2]
        pva_traj_msg.thrust = self.geom_thrust

        #Publish the PVA
        self.geom_controller_pub_.publish(pva_traj_msg)
        self.last_published_command = "geom"
        self.last_published_type_mask = -1
        self.last_published_body_rate_x = pva_traj_msg.body_rate.x
        self.last_published_body_rate_y = pva_traj_msg.body_rate.y
        self.last_published_body_rate_z = pva_traj_msg.body_rate.z
        self.last_published_velocity_x = np.nan
        self.last_published_velocity_y = np.nan
        self.last_published_velocity_z = np.nan

    def publishATT(self, type_mask, nn_action):
        pva_traj_msg = ExecTrajectory()

        pva_traj_msg.type_mask = type_mask

        # Network action convention:
        #   action[0]     normalized collective thrust in [0, 1]
        #   action[1:4]   normalized body-rate commands in [-1, 1]
        # The trajectory server expects real body rates, so only scale by the
        # checkpoint's max_angular_rates. No frame conversion is needed here.
        pva_traj_msg.throttle = nn_action[0,0]  #0.321

        ### This part will only be taken in by trajectory server if type_mask == 0
        pva_traj_msg.transform.rotation.x = 0.0
        pva_traj_msg.transform.rotation.y = 0.0
        pva_traj_msg.transform.rotation.z = 0.707 
        pva_traj_msg.transform.rotation.w = 0.707

        ### This part will only be taken in by trajectory server if type_mask == 1
        body_rate_x = nn_action[0,1] * self.max_angular_rates
        pva_traj_msg.angular_rates.angular.x = body_rate_x     #body rate x
        body_rate_y = nn_action[0,2] * self.max_angular_rates
        pva_traj_msg.angular_rates.angular.y = body_rate_y   #body rate y
        body_rate_z = nn_action[0,3] * self.max_angular_rates
        pva_traj_msg.angular_rates.angular.z = body_rate_z

        self.pva_traj_pub_.publish(pva_traj_msg)
        self.last_published_command = "att"
        self.last_published_type_mask = pva_traj_msg.type_mask
        if pva_traj_msg.type_mask == 1:
            self.last_published_body_rate_x = body_rate_x
            self.last_published_body_rate_y = body_rate_y
            self.last_published_body_rate_z = body_rate_z
        else:
            self.last_published_body_rate_x = np.nan
            self.last_published_body_rate_y = np.nan
            self.last_published_body_rate_z = np.nan
        self.last_published_velocity_x = np.nan
        self.last_published_velocity_y = np.nan
        self.last_published_velocity_z = np.nan


    def checkNNReadiness(self):
        # Readiness means inference has produced a finite action at least once.
        # Do not key this off action magnitude: a valid policy output can be
        # close to zero for some axes.
        return self.has_nn_action and np.isfinite(self.action).all()

    def get_handoff_velocity_direction(self):
        if not self.handoff_use_target_direction:
            return np.array([1.0, 0.0, 0.0])

        target_pos = self.policy.t_pos.detach().cpu().numpy().reshape(3)
        delta = target_pos - self.warp_q[:3]
        delta[2] = 0.0
        norm = np.linalg.norm(delta)
        if norm < 1e-6:
            return np.array([1.0, 0.0, 0.0])
        return delta / norm

    def get_handoff_forward_speed(self):
        velocity_dir = self.get_handoff_velocity_direction()
        current_vel = np.asarray(self.warp_qd[3:], dtype=np.float64)
        return float(np.dot(current_vel, velocity_dir))

    def resetNNHandoffGate(self):
        self.nn_handoff_active = False
        self.nn_handoff_ready_since = None
        self.nn_handoff_announced = False
        self.last_handoff_gate_ready = False
        self.last_handoff_held_time = 0.0

    def checkNNHandoffReadiness(self):
        if not self.checkNNReadiness():
            self.nn_handoff_ready_since = None
            self.last_handoff_forward_speed = self.get_handoff_forward_speed()
            self.last_handoff_altitude = float(self.warp_q[2])
            self.last_handoff_gate_ready = False
            self.last_handoff_held_time = 0.0
            return False

        forward_speed = self.get_handoff_forward_speed()
        altitude = float(self.warp_q[2])
        gate_ready = (
            forward_speed >= self.nn_handoff_speed
            and (
                not self.nn_handoff_require_altitude
                or altitude >= self.nn_handoff_min_altitude
            )
        )
        self.last_handoff_forward_speed = forward_speed
        self.last_handoff_altitude = altitude
        self.last_handoff_gate_ready = gate_ready

        now = rospy.Time.now()
        if gate_ready:
            if self.nn_handoff_ready_since is None:
                self.nn_handoff_ready_since = now
            held_time = (now - self.nn_handoff_ready_since).to_sec()
            self.last_handoff_held_time = held_time
            if held_time >= self.nn_handoff_hold_time:
                print(
                    "NN handoff ready: "
                    f"forward_speed={forward_speed:.3f} m/s, "
                    f"altitude={altitude:.3f} m"
                )
                return True
        else:
            self.nn_handoff_ready_since = None
            self.last_handoff_held_time = 0.0

        if not self.nn_handoff_announced:
            print(
                "Waiting for NN handoff: "
                f"forward_speed={forward_speed:.3f}/{self.nn_handoff_speed:.3f} m/s, "
                f"altitude={altitude:.3f}/{self.nn_handoff_min_altitude:.3f} m"
            )
            self.nn_handoff_announced = True
        return False
    
    def publishMissionCmdMode(self, mode):
        mission_pub_msg = Int8()
        mission_pub_msg.data = mode
        self.mission_mode_pub_.publish(mission_pub_msg)
        if mode == 2:
            print("switched to mission mode 2: ATTITUDE CONTROL")
        elif mode == 3:
            print("switched to mission mode 3: VELOCITY CONTROL")
        elif mode == 4:
            print("switched to mission mode 4: Geom CONTROL")
        else:
            print("switched to mission mode 1: PVA CONTROL")

    def executeMission(self):
        # mission_command_mode is this node's current output mode. 
        # warp_mission_command_mode is the requested mode from the external
        # mission topic. Mode 2 is the neural attitude/body-rate controller.
        if self.drone_state == DRONESTATE["MISSION"].value:
            if self.mission_command_mode == 1:
                if self.warp_mission_command_mode == 2:
                    # Stay in PVA mode, but publish a mixed position+velocity
                    # handoff command: z position hold with +x velocity
                    # feed-forward. Once speed is close to 1 m/s, force NN.
                    if not self.nn_handoff_active:
                        self.publishMissionCmdMode(1)
                    self.nn_handoff_active = True
                    self.publishPVAHandoff()
                    if self.checkNNHandoffReadiness():
                        self.publishMissionCmdMode(2)
                        self.mission_command_mode = 2
                        self.resetNNHandoffGate()
                elif self.warp_mission_command_mode == 3:
                    self.publishMissionCmdMode(3)
                    self.mission_command_mode = 3
                    self.resetNNHandoffGate()
                elif self.warp_mission_command_mode == 4:
                    self.publishMissionCmdMode(4)
                    self.mission_command_mode = 4
                    self.resetNNHandoffGate()
                else:
                    self.publishPVA()
                    self.resetNNHandoffGate()

                        
            elif self.mission_command_mode == 2:  #This controls the orientation. Attitude and thrust
                if self.checkNNReadiness():
                    self.publishATT(self.attitude_mode_toggle, self.action)


                if self.warp_mission_command_mode == 1:
                    #Check if ready to switch
                    self.publishMissionCmdMode(1)
                    self.mission_command_mode = 1
                    self.resetNNHandoffGate()

            elif self.mission_command_mode == 3:
                if self.warp_q[2] < self.nn_handoff_min_altitude:
                    rospy.logwarn_throttle(
                        1.0,
                        "Handoff altitude %.3f below %.3f; publishing PVA hold before NN handoff.",
                        self.warp_q[2],
                        self.nn_handoff_min_altitude,
                    )
                    self.publishPVA()
                    self.resetNNHandoffGate()
                else:
                    self.publishVEL()
                if self.warp_mission_command_mode == 2:
                    if self.checkNNHandoffReadiness():
                        self.publishMissionCmdMode(2)
                        self.mission_command_mode = 2
                        self.resetNNHandoffGate()
                elif self.warp_mission_command_mode == 1:
                    self.publishMissionCmdMode(1)
                    self.mission_command_mode = 1
                    self.resetNNHandoffGate()

            elif self.mission_command_mode == 4:
                self.publishGeomCtrl()
                

    def modeChgCb(self, msg):
        if msg.data == True:
            self.attitude_mode_toggle = 1
        elif msg.data == False:
            self.attitude_mode_toggle = 0

    def preprocess_depth_image(self, depth_image_raw, encoding):
        """Convert Unity/ROS depth images into the tensor expected by the CNN.

        Unity currently publishes 640x360 at about 30 Hz. The trained network
        was built with training_config.yaml width/height, currently 64x64.
        This function handles the size and encoding gap:
          - 8-bit depth is assumed normalized to [0, max_depth].
          - 16UC1 is the common ROS depth convention in millimeters.
          - mono16 is treated as normalized [0, 65535].
          - float encodings are assumed metric depth in meters.

        Output shape is [1, depth_height, depth_width], with depth in meters.
        PolicyNetwork.forward accepts that 3-D shape and adds the channel
        dimension internally before the CNN encoder.
        """
        depth_image = depth_image_raw.astype(np.float32)

        # Some Unity/bridge configurations can deliver a single-channel image
        # with a trailing channel dimension. The network expects grayscale depth.
        if depth_image.ndim == 3:
            depth_image = depth_image[:, :, 0]

        src_height, src_width = depth_image.shape[:2]
        if src_width != self.unity_depth_width or src_height != self.unity_depth_height:
            rospy.logwarn_throttle(
                5.0,
                "Depth image is %dx%d, expected Unity depth %dx%d. Resizing anyway.",
                src_width,
                src_height,
                self.unity_depth_width,
                self.unity_depth_height,
            )

        if depth_image_raw.dtype == np.uint8:
            # mono8/8UC1 cannot store metric depth directly, so it is assumed
            # to be linearly normalized by the Unity side into [0, 255].
            depth_image = (depth_image / 255.0) * self.max_depth
        elif depth_image_raw.dtype == np.uint16:
            if encoding == "16UC1":
                # ROS depth cameras commonly publish 16UC1 in millimeters.
                depth_image = depth_image * 1e-3
            else:
                # mono16 is treated like a higher-precision normalized depth
                # image. This matches the mono8 convention above.
                depth_image = (depth_image / 65535.0) * self.max_depth

        # Replace invalid pixels before clipping so NaNs/Infs cannot propagate
        # through the CNN. Far/invalid positive values become max range; invalid
        # negative values become zero and are clipped below.
        depth_image = np.nan_to_num(
            depth_image,
            nan=self.max_depth,
            posinf=self.max_depth,
            neginf=0.0,
        )
        depth_image = np.clip(depth_image, 0.0, self.max_depth)

        # INTER_AREA is appropriate for downsampling 640x360 to 64x64 because it
        # averages source pixels and tends to preserve obstacle occupancy better
        # than nearest-neighbor sampling.
        depth_resized = cv2.resize(
            depth_image,
            (self.depth_width, self.depth_height),
            interpolation=cv2.INTER_AREA,
        )
        return torch.from_numpy(depth_resized).float().unsqueeze(0)

    def depthCB(self, msg):
        try:
            # desired_encoding='passthrough' preserves the real depth encoding
            # so preprocess_depth_image can choose the correct unit conversion.
            depth_image_raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_image = self.preprocess_depth_image(depth_image_raw, msg.encoding)
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")

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
        # Build batched tensors from the latest warp-frame pose/velocity. The
        # action is stored as numpy because the publisher path uses numpy-style
        # indexing and does not need gradients.
        self.policy.update_target_pos(self.fixed_runtime_target_pos)

        warp_q = self.warp_q[3:]
        warp_pos = torch.Tensor(self.warp_q[:3]).unsqueeze(0)
        warp_q = torch.Tensor(warp_q).unsqueeze(0)
        warp_qd = torch.Tensor(self.warp_qd).unsqueeze(0)
        self.action = self.policy.evaluate_(warp_pos, warp_q, warp_qd, self.depth_image)
        self.has_nn_action = np.isfinite(self.action).all()
        self.log_runtime_debug()




if __name__=="__main__":
    signal(SIGINT, handler)
    print("STARTING NODE")

    # Single-network obstacle-avoidance checkpoint to run. The code below loads
    # both policy.pth and training_config.yaml from this directory so runtime
    # dimensions and scaling stay tied to the checkpoint.
    policy_file = "20260517-151646"
    print(f"\nPOLICY PATH IS {policy_file}") 

    rospack = rospkg.RosPack()
    path = rospack.get_path('nn_policy')
    full_path = os.path.join(path, "logs/obstacle_avoidance")
    print(f"The full path is {full_path}")
    policy_full_path = os.path.join(full_path, policy_file)

    config_path = os.path.join(policy_full_path,"training_config.yaml")

    full_policy_path = os.path.join(policy_full_path, "policy.pth")

    rospy.init_node("nn_policy_planner2")

    # traj_server_default.yaml controls frame-transform behavior and initial
    # mission mode. training_config.yaml controls the neural policy dimensions,
    # timestep, max angular rate, and depth range.
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

    position_control = bool(config_params["position_control"])
    delta_time = float(config_params["delta_time"])
    max_angular_rate = float(config_params["max_angular_rates"])

    # This node consumes the ROS/warp-frame topics directly. It does not expect
    # a JAX-side warp transform to be active at the same time.
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

    
    # Construct the single migrated policy and pass it into the ROS planner.
    # The planner owns subscribers/timers; the policy object owns neural-network
    # inference and target-position state.
    nn_policy = VisionObstacleAvoidancePolicy(full_policy_path, config_params, position_control, warp_frame) 

    nn_policy_planner = NN_POLICY_PLANNER(mission_command_mode=int(mission_command_mode), policy=nn_policy,
                                          inference_timestep=delta_time, max_angular_rates = max_angular_rate,
                                          warp_jax=warp_jax, to_transform_odom=to_transform_odom, to_transform_policy=to_transform_policy)

    rospy.spin()

    print("done")
