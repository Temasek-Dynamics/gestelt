#!/usr/bin/env python3
import os
import sys
import time
from enum import Enum
from signal import SIGINT, signal

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import roslib.packages
import rospkg
import rospy
import torch
import torch.nn as nn
import torch.nn.functional as F
import yaml
from geometry_msgs.msg import PoseStamped
from gestelt_msgs.msg import CommanderState, ExecTrajectory
from mavros_msgs.msg import AttitudeTarget
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8
from visualization_msgs.msg import Marker, MarkerArray


print("Running with Python:", sys.executable)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NN_POLICY_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
DEFAULT_POLICY_DIR = os.path.join(
    NN_POLICY_DIR,
    "logs",
    "vel_tracking",
    "20260729-133916",
)
DEFAULT_OBSTACLE_DATA_PATH = os.path.join(DEFAULT_POLICY_DIR, "obstacle_data.npz")

# Manual test endpoints in the same Gazebo/map frame as obstacle_data.npz.
# Edit these two lines when you want to test a different route.
MANUAL_START_POS = np.array([0.0, 0.0, 1.0], dtype=np.float32)
MANUAL_TARGET_POS = np.array([2.0, 6.5, 1.0], dtype=np.float32)


def quaternion_facing_goal(start_pos, target_pos):
    direction = np.asarray(target_pos, dtype=np.float32)[:2] - np.asarray(start_pos, dtype=np.float32)[:2]
    yaw = float(np.arctan2(direction[1], direction[0]))
    return np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)], dtype=np.float32)


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


class TrackVelGRU(nn.Module):
    """Velocity-tracking MLP with a recurrent layer before the action head."""

    def __init__(
        self,
        input_dim=22,
        output_dim=4,
        gru_hidden_dim=64,
        recurrent_detach=True,
    ):
        super().__init__()
        self.gru_hidden_dim = int(gru_hidden_dim)
        self.recurrent_detach = bool(recurrent_detach)
        self._hidden_state = None

        self.fc1 = nn.Linear(input_dim, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, 32)
        self.gru = nn.GRU(
            input_size=32,
            hidden_size=self.gru_hidden_dim,
            batch_first=True,
        )
        self.output_layer = nn.Linear(self.gru_hidden_dim, output_dim)

    def reset_hidden(self, batch_size=None, device=None, dtype=None):
        if batch_size is None:
            self._hidden_state = None
            return

        parameter = next(self.parameters())
        self._hidden_state = torch.zeros(
            1,
            int(batch_size),
            self.gru_hidden_dim,
            device=device if device is not None else parameter.device,
            dtype=dtype if dtype is not None else parameter.dtype,
        )

    def _compatible_hidden(self, x):
        hidden = self._hidden_state
        if hidden is None:
            return None
        if hidden.shape[1] != x.shape[0]:
            return None
        if hidden.device != x.device or hidden.dtype != x.dtype:
            return None
        return hidden

    def forward(self, x):
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        x = F.relu(self.fc4(x))

        x = x.unsqueeze(1)
        x, next_hidden = self.gru(x, self._compatible_hidden(x))
        self._hidden_state = (
            next_hidden.detach() if self.recurrent_detach else next_hidden
        )

        output = self.output_layer(x[:, -1])
        thrust = torch.sigmoid(output[:, 0:1])
        body_rates = torch.tanh(output[:, 1:])
        return torch.cat((thrust, body_rates), dim=1)


def handler(signal_received, frame):
    print("SIGINT or CTRL-C detected. Exiting gracefully")
    raise SystemExit(0)


def generate_training_obstacles(config_params, expected_count):
    room_size = float(config_params["room_size"])
    obs_size = float(config_params.get("obs_size", 0.5))
    min_obs_gap = float(config_params.get("min_obs_gap", 0.0))
    seed = None if bool(config_params.get("to_randomize_obs_location", False)) else 51
    rng = np.random.RandomState(seed) if seed is not None else np.random

    centers = []
    raw_radii = []
    tries = 0
    while len(centers) < expected_count and tries < 50 * max(expected_count, 1):
        tries += 1
        radius = float(rng.uniform(0.5, obs_size))
        cx = float(rng.uniform(radius, room_size - radius))
        cy = float(rng.uniform(radius, room_size - radius))

        ok = True
        for (px, py), prev_radius in zip(centers, raw_radii):
            if np.hypot(cx - px, cy - py) < np.sqrt(2.0) * (radius + prev_radius + min_obs_gap):
                ok = False
                break
        if ok:
            centers.append((cx, cy))
            raw_radii.append(radius)

    if len(centers) != expected_count:
        raise RuntimeError(
            f"Could not regenerate {expected_count} obstacles with the training SceneManager rule."
        )

    positions = np.asarray([[cx, cy, 0.0] for cx, cy in centers], dtype=np.float32)
    radii = np.asarray([np.sqrt(2.0 * radius * radius) for radius in raw_radii], dtype=np.float32)
    return positions, radii, "regenerated_scene_manager_seed_51"


def load_obstacle_data_npz(obstacle_data_path, expected_count):
    data = np.load(obstacle_data_path)
    if "positions" not in data.files or "radii" not in data.files:
        raise ValueError(
            f"{obstacle_data_path} must contain 'positions' and 'radii'. "
            f"Found keys: {data.files}"
        )

    positions = np.asarray(data["positions"], dtype=np.float32).reshape(-1, 3)
    radii = np.asarray(data["radii"], dtype=np.float32).reshape(-1)
    if len(positions) != expected_count or len(radii) != expected_count:
        raise ValueError(
            "Obstacle count does not match policy input dimension: "
            f"npz has positions={len(positions)}, radii={len(radii)}, "
            f"policy expects {expected_count}."
        )
    return positions, radii, f"obstacle_data_npz:{obstacle_data_path}"


def parse_obstacles(config_params, expected_count, obstacle_data_path=DEFAULT_OBSTACLE_DATA_PATH):
    if obstacle_data_path and os.path.exists(obstacle_data_path):
        return load_obstacle_data_npz(obstacle_data_path, expected_count)

    raw_obstacles = config_params.get("obstacles", None)
    positions = []
    radii = []

    if raw_obstacles:
        if isinstance(raw_obstacles, dict):
            values = [
                raw_obstacles[key]
                for key in sorted(
                    raw_obstacles.keys(),
                    key=lambda name: int(str(name).replace("obs", "")) if str(name).replace("obs", "").isdigit() else str(name),
                )
            ]
        else:
            values = raw_obstacles
        for obs in values:
            pos = np.asarray(obs["position"], dtype=np.float32).reshape(-1)[:3]
            if pos.shape[0] < 3:
                pos = np.pad(pos, (0, 3 - pos.shape[0]), constant_values=0.0)
            positions.append(pos)
            radii.append(float(obs["radius"]))
        source = "training_config_obstacles"
    else:
        positions, radii, source = generate_training_obstacles(config_params, expected_count)

    if len(positions) != expected_count:
        raise ValueError(
            "Obstacle count does not match policy input dimension: "
            f"config has {len(positions)}, policy expects {expected_count}. "
            "Use the same training_config.yaml that produced this checkpoint."
        )

    return (
        np.asarray(positions, dtype=np.float32).reshape(-1, 3),
        np.asarray(radii, dtype=np.float32).reshape(-1),
        source,
    )


def create_obs_vec(pos, obstacle_positions, obstacle_radii):
    if obstacle_positions.numel() == 0:
        return torch.zeros((pos.shape[0], 0), device=pos.device, dtype=pos.dtype)

    rel_xy = obstacle_positions[:, :2].unsqueeze(0) - pos[:, :2].unsqueeze(1)
    sort_indices = torch.argsort(torch.norm(rel_xy, dim=-1), dim=1)
    gather_indices = sort_indices.unsqueeze(-1).expand(pos.shape[0], -1, 2)
    sorted_rel_xy = torch.gather(rel_xy, dim=1, index=gather_indices)
    sorted_radii = obstacle_radii[sort_indices].unsqueeze(-1)
    return torch.cat([sorted_rel_xy, sorted_radii], dim=-1).reshape(pos.shape[0], -1)


class VelocityPolicy:
    def __init__(self, policy_path, config_params, device="cpu"):
        self.device = torch.device(device)
        state_dict = torch.load(policy_path, map_location=self.device)
        required_keys = ("fc1.weight", "gru.weight_ih_l0", "gru.weight_hh_l0", "output_layer.weight")
        missing_keys = [key for key in required_keys if key not in state_dict]
        if missing_keys:
            raise RuntimeError(
                "This GRU test script expects a TrackVelGRU checkpoint. "
                f"Missing keys: {missing_keys}. "
                "Please point DEFAULT_POLICY_DIR/policy_path to the GRU policy.pth."
            )

        input_dim = int(state_dict["fc1.weight"].shape[1])
        gru_hidden_dim = int(state_dict["gru.weight_hh_l0"].shape[1])
        self.expected_obstacles = max(0, (input_dim - 10) // 3)
        self.policy = TrackVelGRU(
            input_dim=input_dim,
            output_dim=4,
            gru_hidden_dim=gru_hidden_dim,
        ).to(self.device)
        self.policy.load_state_dict(state_dict)
        self.policy.eval()
        self.reset_hidden(batch_size=1)

        self.max_speed = float(config_params.get("max_speed", 1.0))
        self.last_obs_vec = None
        self.last_desired_vel = None
        self.last_diff_vel = None
        self.last_nearest_clearance = np.nan
        obstacle_positions, obstacle_radii, obstacle_source = parse_obstacles(config_params, self.expected_obstacles)
        self.obstacle_source = obstacle_source
        self.obstacle_positions_np = obstacle_positions
        self.obstacle_radii_np = obstacle_radii
        self.obstacle_positions = torch.as_tensor(obstacle_positions, dtype=torch.float32, device=self.device)
        self.obstacle_radii = torch.as_tensor(obstacle_radii, dtype=torch.float32, device=self.device)

        target_pos = np.asarray(config_params.get("test_target_position", [6.0, 5.0, 1.0]), dtype=np.float32)
        self.t_pos = torch.as_tensor(target_pos.reshape(1, 3), dtype=torch.float32, device=self.device)
        self.last_obs = None

    def reset_hidden(self, batch_size=1):
        if hasattr(self.policy, "reset_hidden"):
            self.policy.reset_hidden(batch_size=batch_size, device=self.device)

    def update_target_pos(self, target_pos):
        target_pos = np.asarray(target_pos, dtype=np.float32).reshape(1, 3)
        self.t_pos = torch.as_tensor(target_pos, dtype=torch.float32, device=self.device)

    def evaluate(self, pos, att, qd):
        pos = pos.to(self.device)
        att = att.to(self.device)
        qd = qd.to(self.device)
        att = torch.where(att[:, 3:4] < 0.0, -att, att)

        vel = qd[:, 3:]
        angvel = qd[:, :3]
        diff_norm = torch.norm(self.t_pos - pos, dim=1, keepdim=True)
        desired_vel = (self.t_pos - pos) / (diff_norm + 1e-8)
        desired_vel = desired_vel * self.max_speed
        desired_vel = torch.cat(
            (desired_vel[:, :2], torch.clamp(desired_vel[:, 2:3], min=-0.3, max=0.3)),
            dim=1,
        )
        obs_vec = create_obs_vec(pos, self.obstacle_positions, self.obstacle_radii)
        if self.obstacle_positions.numel() > 0:
            clearances = torch.norm(self.obstacle_positions[:, :2].unsqueeze(0) - pos[:, :2].unsqueeze(1), dim=-1) - self.obstacle_radii.unsqueeze(0)
            self.last_nearest_clearance = float(clearances.min().detach().cpu().item())
        self.last_obs_vec = obs_vec.detach().cpu().numpy().reshape(-1)
        self.last_desired_vel = desired_vel.detach().cpu().numpy().reshape(-1)
        self.last_diff_vel = (desired_vel - vel).detach().cpu().numpy().reshape(-1)
        obs = torch.cat((att, angvel, desired_vel - vel, obs_vec), dim=1)
        self.last_obs = obs.detach().cpu().numpy()

        with torch.no_grad():
            action = self.policy(obs)
        return action.detach().cpu().numpy()


class GazeboPolicyTester:
    def __init__(self, policy, config_params, loaded_params, max_angular_rates, inference_timestep):
        self.policy = policy
        self.config_params = config_params
        self.max_angular_rates = max_angular_rates
        self.inference_timestep = inference_timestep

        self.to_transform_odom = loaded_params["to_transform_odom"]
        self.to_transform_policy = loaded_params["to_transform_policy"]
        self.warp_jax = loaded_params["warp_jax"]

        room_size = float(config_params["room_size"])
        # test.py trains/evaluates obstacles directly in the sim room frame
        # [0, room_size] x [0, room_size]. Gazebo SITL in this setup also
        # starts at world/map (0, 0), so the default test should not apply the
        # policy_vel_combined_cmdp_working.py visualization offset.
        self.scene_offset = np.asarray(
            config_params.get("scene_offset", [3.0, 3.0, 0.0]),
            dtype=np.float32,
        ).reshape(3)
        self.start_pos_sim = np.asarray(MANUAL_START_POS, dtype=np.float32)
        self.target_pos_sim = np.asarray(MANUAL_TARGET_POS, dtype=np.float32)
        self.start_pos_world = self.start_pos_sim - self.scene_offset
        self.target_pos_world = self.target_pos_sim - self.scene_offset
        self.start_quat = quaternion_facing_goal(self.start_pos_sim, self.target_pos_sim)
        self.policy.update_target_pos(self.target_pos_sim)

        self.drone_state = DRONESTATE.INIT.value
        self.mission_command_mode = 1
        self.warp_mission_command_mode = 1
        self.attitude_mode_toggle = 1
        self.action = np.zeros((1, 4), dtype=np.float32)
        self.has_nn_action = False
        self.mission_start_time = None
        self.nn_start_time = None
        self.test_finished = False
        self.saved_outputs = False
        self.max_nn_duration = float(config_params.get("test_max_duration", inference_timestep * int(config_params.get("sim_steps", 300))))

        self.drone_pos = np.zeros(3, dtype=np.float32)
        self.drone_quat = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.warp_q = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float32)
        self.warp_qd = np.zeros(6, dtype=np.float32)
        self.received_warp_pose = False
        self.received_warp_odom = False
        self.last_pos_time = None
        self.last_odom_time = None

        timestamp = time.strftime("%Y%m%d-%H%M%S")
        self.output_dir = os.path.join(DEFAULT_POLICY_DIR, f"gazebo_gru_test_{timestamp}")
        os.makedirs(self.output_dir, exist_ok=True)
        self.trajectory_world = []
        self.trajectory_sim = []
        self.action_log = []
        self.body_rate_log = []
        self.body_rate_time_log = []
        self.runtime_log_path = os.path.join(self.output_dir, "policy_runtime_obs.csv")
        self.runtime_log_file = open(self.runtime_log_path, "w", buffering=1)
        obs_columns = []
        for obs_id in range(len(self.policy.obstacle_positions_np)):
            obs_columns.extend([f"obs{obs_id + 1}_rel_x", f"obs{obs_id + 1}_rel_y", f"obs{obs_id + 1}_radius"])
        self.runtime_log_file.write(
            "stamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,"
            "desired_vel_x,desired_vel_y,desired_vel_z,"
            "diff_vel_x,diff_vel_y,diff_vel_z,nearest_clearance,"
            + ",".join(obs_columns)
            + ",action_throttle,action_rate_x,action_rate_y,action_rate_z\n"
        )

        self.swarm_mode_pub = rospy.Publisher("/traj_server/swarm_command", Int8, queue_size=5)
        self.pva_traj_pub = rospy.Publisher("/drone0/planner_adaptor/exec_trajectory", ExecTrajectory, queue_size=5)
        self.mission_mode_pub = rospy.Publisher("/traj_server/mission_command", Int8, queue_size=5)
        self.start_target_pub = rospy.Publisher("/policy_viz/start_target", MarkerArray, queue_size=1, latch=True)
        self.trajectory_pub = rospy.Publisher("/policy_viz/trajectory", Path, queue_size=1)
        self.trajectory_path_msg = Path()
        self.trajectory_path_msg.header.frame_id = "map"

        rospy.Subscriber("/drone0/traj_server/state", CommanderState, self.comm_state_cb, queue_size=10)
        rospy.Subscriber("/drone0/mavros/local_position/pose", PoseStamped, self.pose_cb, queue_size=10)
        rospy.Subscriber("/drone0/mavros/local_position/odom", Odometry, self.odom_cb, queue_size=10)
        rospy.Subscriber("/drone0/warp/local_position/pose", PoseStamped, self.warp_pose_cb, queue_size=5)
        rospy.Subscriber("/drone0/warp/local_position/odom", Odometry, self.warp_odom_cb, queue_size=5)

        rospy.on_shutdown(self.save_outputs)
        rospy.on_shutdown(self.close_runtime_log)
        rospy.sleep(1.0)
        self.publish_start_target_markers()
        self.event_timer = rospy.Timer(rospy.Duration(inference_timestep), self.event_cb)
        self.policy_timer = rospy.Timer(rospy.Duration(inference_timestep), self.nn_evaluation)
        print(f"Gazebo policy test output dir: {self.output_dir}")
        print(f"Start sim/world: {self.start_pos_sim} / {self.start_pos_world}")
        print(f"Target sim/world: {self.target_pos_sim} / {self.target_pos_world}")
        print(f"Start yaw-facing-goal quaternion xyzw: {self.start_quat.tolist()}")
        print(
            "Obstacles: "
            f"source={self.policy.obstacle_source}, "
            f"count={len(self.policy.obstacle_positions_np)}, "
            f"positions={self.policy.obstacle_positions_np.tolist()}, "
            f"radii={self.policy.obstacle_radii_np.tolist()}"
        )

    def comm_state_cb(self, msg):
        self.drone_state = DRONESTATE[msg.traj_server_state].value

    def pose_cb(self, msg):
        self.drone_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=np.float32)
        self.drone_quat = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ], dtype=np.float32)
        if self.to_transform_odom == 0.0 or not self.received_warp_pose:
            self.warp_q = np.concatenate((self.drone_pos, self.drone_quat)).astype(np.float32)
        self.last_pos_time = msg.header.stamp

    def odom_cb(self, msg):
        if self.to_transform_odom == 0.0 or not self.received_warp_odom:
            self.warp_qd = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ], dtype=np.float32)
        self.last_odom_time = msg.header.stamp

    def warp_pose_cb(self, msg):
        if self.to_transform_odom == 1.0:
            self.received_warp_pose = True
            self.warp_q = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ], dtype=np.float32)

    def warp_odom_cb(self, msg):
        if self.to_transform_odom == 1.0:
            self.received_warp_odom = True
            self.warp_qd = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z,
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ], dtype=np.float32)

    def publish_mission(self, mission_num):
        msg = Int8()
        msg.data = int(mission_num)
        self.swarm_mode_pub.publish(msg)

    def publish_mission_cmd_mode(self, mode):
        msg = Int8()
        msg.data = int(mode)
        self.mission_mode_pub.publish(msg)
        print(f"switched mission command mode to {mode}")

    def publish_pva(self):
        msg = ExecTrajectory()
        msg.transform.translation.x = float(self.start_pos_world[0])
        msg.transform.translation.y = float(self.start_pos_world[1])
        msg.transform.translation.z = float(self.start_pos_world[2])
        msg.transform.rotation.x = float(self.start_quat[0])
        msg.transform.rotation.y = float(self.start_quat[1])
        msg.transform.rotation.z = float(self.start_quat[2])
        msg.transform.rotation.w = float(self.start_quat[3])
        msg.type_mask = 2048
        self.pva_traj_pub.publish(msg)

    def publish_att(self):
        msg = ExecTrajectory()
        msg.type_mask = 1
        msg.throttle = float(self.action[0, 0])
        msg.angular_rates.angular.x = float(self.action[0, 1] * self.max_angular_rates)
        msg.angular_rates.angular.y = float(self.action[0, 2] * self.max_angular_rates)
        msg.angular_rates.angular.z = float(self.action[0, 3] * self.max_angular_rates)
        msg.transform.rotation.w = 1.0
        self.pva_traj_pub.publish(msg)
        self.body_rate_time_log.append(rospy.Time.now().to_sec())
        self.body_rate_log.append([
            msg.angular_rates.angular.x,
            msg.angular_rates.angular.y,
            msg.angular_rates.angular.z,
        ])

    def close_runtime_log(self):
        log_file = getattr(self, "runtime_log_file", None)
        if log_file is not None and not log_file.closed:
            log_file.close()

    def event_cb(self, event):
        if self.test_finished:
            if self.drone_state == DRONESTATE.MISSION.value:
                self.publish_pva()
            return

        if self.drone_state == DRONESTATE.IDLE.value:
            self.publish_mission(ServerEvent.TAKEOFF_E.value)
        elif self.drone_state == DRONESTATE.HOVER.value:
            self.publish_mission(ServerEvent.MISSION_E.value)
        elif self.drone_state == DRONESTATE.MISSION.value:
            self.execute_mission()

    def execute_mission(self):
        if self.test_finished:
            self.publish_pva()
            return

        if self.mission_start_time is None:
            self.mission_start_time = rospy.Time.now()

        if self.mission_command_mode == 1:
            self.publish_pva()
            dist_to_start = np.linalg.norm(np.asarray(self.warp_q[:3]) - self.start_pos_world)
            if dist_to_start < 0.35 and self.has_nn_action:
                self.policy.reset_hidden(batch_size=1)
                self.has_nn_action = False
                self.action = np.zeros((1, 4), dtype=np.float32)
                self.publish_mission_cmd_mode(2)
                self.mission_command_mode = 2
                self.nn_start_time = rospy.Time.now()
                print("NN policy control started.")
        elif self.mission_command_mode == 2:
            if self.has_nn_action:
                self.publish_att()
                self.record_trajectory()
                if self.reached_target() or self.nn_timed_out():
                    print("Test finished; switching back to PVA hold and saving GIFs.")
                    self.test_finished = True
                    self.publish_mission_cmd_mode(1)
                    self.mission_command_mode = 1
                    self.save_outputs()

    def nn_timed_out(self):
        if self.nn_start_time is None:
            return False
        return (rospy.Time.now() - self.nn_start_time).to_sec() >= self.max_nn_duration

    def reached_target(self):
        drone_sim = np.asarray(self.warp_q[:3], dtype=np.float32) + self.scene_offset
        return np.linalg.norm(drone_sim - self.target_pos_sim) < 0.8

    def nn_evaluation(self, event):
        if self.test_finished:
            return

        pos_sim = torch.as_tensor(self.warp_q[:3] + self.scene_offset, dtype=torch.float32).unsqueeze(0)
        quat = torch.as_tensor(self.warp_q[3:], dtype=torch.float32).unsqueeze(0)
        qd = torch.as_tensor(self.warp_qd, dtype=torch.float32).unsqueeze(0)
        self.action = self.policy.evaluate(pos_sim, quat, qd)
        self.has_nn_action = np.isfinite(self.action).all()
        self.log_policy_input(pos_sim.detach().cpu().numpy().reshape(3))

    def log_policy_input(self, pos_sim):
        if self.policy.last_obs_vec is None:
            return
        now = rospy.Time.now().to_sec()
        vel = np.asarray(self.warp_qd[3:], dtype=np.float32).reshape(3)
        desired_vel = np.asarray(self.policy.last_desired_vel, dtype=np.float32).reshape(3)
        diff_vel = np.asarray(self.policy.last_diff_vel, dtype=np.float32).reshape(3)
        obs_vec = np.asarray(self.policy.last_obs_vec, dtype=np.float32).reshape(-1)
        action = np.asarray(self.action, dtype=np.float32).reshape(4)
        row = [
            f"{now:.6f}",
            f"{pos_sim[0]:.9f}", f"{pos_sim[1]:.9f}", f"{pos_sim[2]:.9f}",
            f"{vel[0]:.9f}", f"{vel[1]:.9f}", f"{vel[2]:.9f}",
            f"{desired_vel[0]:.9f}", f"{desired_vel[1]:.9f}", f"{desired_vel[2]:.9f}",
            f"{diff_vel[0]:.9f}", f"{diff_vel[1]:.9f}", f"{diff_vel[2]:.9f}",
            f"{self.policy.last_nearest_clearance:.9f}",
        ]
        row.extend(f"{value:.9f}" for value in obs_vec)
        row.extend(f"{value:.9f}" for value in action)
        self.runtime_log_file.write(",".join(row) + "\n")

    def record_trajectory(self):
        world = np.asarray(self.warp_q[:3], dtype=np.float32).copy()
        sim = world + self.scene_offset
        self.trajectory_world.append(world)
        self.trajectory_sim.append(sim)
        self.action_log.append(self.action.reshape(-1).copy())

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = float(sim[0])
        pose.pose.position.y = float(sim[1])
        pose.pose.position.z = float(sim[2])
        pose.pose.orientation.x = float(self.warp_q[3])
        pose.pose.orientation.y = float(self.warp_q[4])
        pose.pose.orientation.z = float(self.warp_q[5])
        pose.pose.orientation.w = float(self.warp_q[6])
        self.trajectory_path_msg.header.stamp = pose.header.stamp
        self.trajectory_path_msg.poses.append(pose)
        self.trajectory_pub.publish(self.trajectory_path_msg)

    def publish_start_target_markers(self):
        marker_array = MarkerArray()
        for marker_id, (pos, color) in enumerate(
            [(self.start_pos_sim, (0.0, 1.0, 0.0)), (self.target_pos_sim, (1.0, 0.0, 0.0))]
        ):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "policy_test_start_target"
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(pos[0])
            marker.pose.position.y = float(pos[1])
            marker.pose.position.z = float(pos[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = 0.35
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            marker_array.markers.append(marker)
        self.start_target_pub.publish(marker_array)

    def save_outputs(self):
        if self.saved_outputs or len(self.trajectory_sim) < 2:
            return
        self.saved_outputs = True
        trajectory = np.asarray(self.trajectory_sim, dtype=np.float32)
        actions = np.asarray(self.action_log, dtype=np.float32)
        np.save(os.path.join(self.output_dir, "trajectory_sim.npy"), trajectory)
        np.save(os.path.join(self.output_dir, "actions.npy"), actions)
        if self.body_rate_log:
            np.save(os.path.join(self.output_dir, "body_rates.npy"), np.asarray(self.body_rate_log, dtype=np.float32))
        np.savez(
            os.path.join(self.output_dir, "obstacles.npz"),
            positions=self.policy.obstacle_positions_np,
            radii=self.policy.obstacle_radii_np,
        )
        xy_path = os.path.join(self.output_dir, "trajectory_xy.gif")
        xz_path = os.path.join(self.output_dir, "trajectory_xz.gif")
        body_rates_path = os.path.join(self.output_dir, "body_rates.png")
        self.save_xy_gif(trajectory, xy_path)
        self.save_xz_gif(trajectory, xz_path)
        self.save_body_rates_plot(body_rates_path)
        with open(os.path.join(self.output_dir, "metadata.yaml"), "w") as f:
            yaml.safe_dump(
                {
                    "policy_dir": DEFAULT_POLICY_DIR,
                    "start_pos_sim": self.start_pos_sim.tolist(),
                    "target_pos_sim": self.target_pos_sim.tolist(),
                    "start_quat_xyzw": self.start_quat.tolist(),
                    "scene_offset": self.scene_offset.tolist(),
                    "coordinate_note": (
                        "Obstacle, start, target, and trajectory coordinates are all in the "
                        "same Gazebo/map frame by default; scene_offset is applied only if "
                        "explicitly present in training_config.yaml."
                    ),
                    "obstacle_source": self.policy.obstacle_source,
                    "obstacle_count": int(len(self.policy.obstacle_positions_np)),
                    "obstacle_positions": self.policy.obstacle_positions_np.tolist(),
                    "obstacle_radii": self.policy.obstacle_radii_np.tolist(),
                    "trajectory_xy_gif": xy_path,
                    "trajectory_xz_gif": xz_path,
                    "body_rates_png": body_rates_path,
                    "runtime_obs_csv": self.runtime_log_path,
                },
                f,
                sort_keys=False,
            )
        print(f"Saved GIFs: {xy_path}, {xz_path}")
        print(f"Saved body rates plot: {body_rates_path}")

    def save_xy_gif(self, trajectory, save_path):
        fig, ax = plt.subplots(figsize=(7, 7))
        all_xy = np.vstack((trajectory[:, :2], self.start_pos_sim[:2], self.target_pos_sim[:2], self.policy.obstacle_positions_np[:, :2]))
        x_min, y_min = np.min(all_xy, axis=0) - 1.0
        x_max, y_max = np.max(all_xy, axis=0) + 1.0
        span = max(x_max - x_min, y_max - y_min, 1.0)
        x_mid = 0.5 * (x_min + x_max)
        y_mid = 0.5 * (y_min + y_max)
        ax.set_xlim(x_mid - span / 2.0, x_mid + span / 2.0)
        ax.set_ylim(y_mid - span / 2.0, y_mid + span / 2.0)
        ax.set_aspect("equal", adjustable="box")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("Gazebo Policy Test - XY")
        ax.grid(True, alpha=0.3)
        for idx, (pos, radius) in enumerate(zip(self.policy.obstacle_positions_np, self.policy.obstacle_radii_np), start=1):
            if radius <= 0.0:
                continue
            ax.add_patch(plt.Circle((pos[0], pos[1]), radius, color="orange", alpha=0.28))
            ax.text(pos[0], pos[1], f"obs{idx}", fontsize=8)
        ax.plot(self.start_pos_sim[0], self.start_pos_sim[1], "g*", markersize=14, label="start")
        ax.plot(self.target_pos_sim[0], self.target_pos_sim[1], "r*", markersize=14, label="target")
        line, = ax.plot([], [], "b-", linewidth=2, label="trajectory")
        dot, = ax.plot([], [], "bo", markersize=6)
        ax.legend()

        def update(frame):
            line.set_data(trajectory[: frame + 1, 0], trajectory[: frame + 1, 1])
            dot.set_data([trajectory[frame, 0]], [trajectory[frame, 1]])
            return line, dot

        anim = FuncAnimation(fig, update, frames=len(trajectory), interval=80, blit=True)
        anim.save(save_path, writer=PillowWriter(fps=12))
        plt.close(fig)

    def save_xz_gif(self, trajectory, save_path):
        fig, ax = plt.subplots(figsize=(8, 5))
        x_values = np.concatenate((trajectory[:, 0], [self.start_pos_sim[0], self.target_pos_sim[0]], self.policy.obstacle_positions_np[:, 0]))
        z_values = np.concatenate((trajectory[:, 2], [self.start_pos_sim[2], self.target_pos_sim[2], 0.0, 3.0]))
        x_pad = max(0.5, 0.05 * max(np.ptp(x_values), 1.0))
        ax.set_xlim(np.min(x_values) - x_pad, np.max(x_values) + x_pad)
        ax.set_ylim(np.min(z_values) - 0.2, np.max(z_values) + 0.2)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Z (m)")
        ax.set_title("Gazebo Policy Test - XZ")
        ax.grid(True, alpha=0.3)
        z_min, z_max = ax.get_ylim()
        for idx, (pos, radius) in enumerate(zip(self.policy.obstacle_positions_np, self.policy.obstacle_radii_np), start=1):
            if radius <= 0.0:
                continue
            ax.add_patch(plt.Rectangle((pos[0] - radius, z_min), 2.0 * radius, z_max - z_min, color="orange", alpha=0.22))
            ax.text(pos[0], z_max, f"obs{idx}", fontsize=8, ha="center", va="top")
        ax.plot(self.start_pos_sim[0], self.start_pos_sim[2], "g*", markersize=14, label="start")
        ax.plot(self.target_pos_sim[0], self.target_pos_sim[2], "r*", markersize=14, label="target")
        line, = ax.plot([], [], "b-", linewidth=2, label="trajectory")
        dot, = ax.plot([], [], "bo", markersize=6)
        ax.legend()

        def update(frame):
            line.set_data(trajectory[: frame + 1, 0], trajectory[: frame + 1, 2])
            dot.set_data([trajectory[frame, 0]], [trajectory[frame, 2]])
            return line, dot

        anim = FuncAnimation(fig, update, frames=len(trajectory), interval=80, blit=True)
        anim.save(save_path, writer=PillowWriter(fps=12))
        plt.close(fig)

    def save_body_rates_plot(self, save_path):
        if not self.body_rate_log:
            return

        body_rates = np.asarray(self.body_rate_log, dtype=np.float32).reshape(-1, 3)
        time_axis = np.asarray(self.body_rate_time_log, dtype=np.float64)
        time_axis = time_axis - time_axis[0]

        fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True)
        labels = ("body rate x / roll p", "body rate y / pitch q", "body rate z / yaw r")
        for axis_idx, ax in enumerate(axes):
            ax.plot(time_axis, body_rates[:, axis_idx], linewidth=1.6)
            ax.set_ylabel("rad/s")
            ax.set_title(labels[axis_idx])
            ax.grid(True, alpha=0.3)
        axes[-1].set_xlabel("time (s)")
        fig.suptitle("Published NN Body Rates")
        fig.tight_layout()
        fig.savefig(save_path, dpi=200)
        plt.close(fig)


def validate_frame_config(config_params, loaded_params):
    warp_jax = loaded_params["warp_jax"]
    to_transform_odom = loaded_params["to_transform_odom"]
    to_transform_policy = loaded_params["to_transform_policy"]
    if warp_jax != 0.0:
        rospy.logwarn("warp_jax is %s; expected 0.0 for this test node.", warp_jax)
    warp_frame = config_params.get("warp_frame", 1.0)
    if warp_frame == 1.0:
        policy_global = config_params.get("policy_global", 0.0)
        if policy_global == 1.0:
            if to_transform_odom != 1.0 or to_transform_policy != 1.0:
                rospy.logwarn(
                    "For policy_global=1.0, expected to_transform_odom=1.0 and "
                    "to_transform_policy=1.0, got %.1f/%.1f.",
                    to_transform_odom,
                    to_transform_policy,
                )
        else:
            if to_transform_odom != 0.0 or to_transform_policy != 0.0:
                rospy.logwarn(
                    "For z-up non-global policy, expected to_transform_odom=0.0 and "
                    "to_transform_policy=0.0, got %.1f/%.1f. Falling back to mavros "
                    "pose/odom if warp topics are unavailable.",
                    to_transform_odom,
                    to_transform_policy,
                )
    else:
        if to_transform_odom != 1.0 or to_transform_policy != 1.0:
            rospy.logwarn(
                "For warp_frame=0.0, expected to_transform_odom=1.0 and "
                "to_transform_policy=1.0, got %.1f/%.1f.",
                to_transform_odom,
                to_transform_policy,
            )


if __name__ == "__main__":
    signal(SIGINT, handler)
    print("STARTING GAZEBO CMDP GRU POLICY TEST NODE")
    policy_dir = DEFAULT_POLICY_DIR
    policy_path = os.path.join(policy_dir, "policy.pth")
    config_path = os.path.join(policy_dir, "training_config.yaml")

    with open(config_path, "r") as f:
        config_params = yaml.safe_load(f)

    ros_lib = roslib.packages.get_pkg_dir("gestelt_bringup")
    traj_config_path = os.path.join(ros_lib, "config/traj_server_default.yaml")
    with open(traj_config_path, "r") as f:
        loaded_params = yaml.safe_load(f)

    rospy.init_node("cmdp_gru_policy_gazebo_test")
    validate_frame_config(config_params, loaded_params)

    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    policy = VelocityPolicy(policy_path, config_params, device=device)
    tester = GazeboPolicyTester(
        policy=policy,
        config_params=config_params,
        loaded_params=loaded_params,
        max_angular_rates=float(config_params["max_angular_rates"]),
        inference_timestep=float(config_params["delta_time"]),
    )
    rospy.spin()
