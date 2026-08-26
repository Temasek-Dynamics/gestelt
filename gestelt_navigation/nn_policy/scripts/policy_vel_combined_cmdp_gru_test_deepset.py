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
    "20260804-155247",
)
DEFAULT_OBSTACLE_DATA_PATH = os.path.join(DEFAULT_POLICY_DIR, "obstacle_data.npz")
FIXED_OBSTACLE_SEED = 51
FIXED_OBSTACLE_POSITIONS = np.array(
    [
        [0.6867675442346135, 6.202834114336479, 0.0],
        [4.9865631107233135, 2.523169809777629, 0.0],
        [6.6682383653101756, 9.08054011575443, 0.0],
        [6.931182050579215, 5.382899312101351, 0.0],
    ],
    dtype=np.float32,
)
FIXED_OBSTACLE_RADII = np.array(
    [
        0.7071067811865476,
        0.7071067811865476,
        0.7071067811865476,
        0.7071067811865476,
    ],
    dtype=np.float32,
)

# Manual test endpoints in the training/policy room frame. DEFAULT_SCENE_OFFSET
# shifts them into Gazebo/map world coordinates for the PVA command.
MANUAL_START_POS = np.array([2.0, 2.5, 1.0], dtype=np.float32)
MANUAL_TARGET_POS = np.array([8.0, 2.5, 1.0], dtype=np.float32)
DEFAULT_SCENE_OFFSET = np.array([4.0, 3.0, 0.0], dtype=np.float32)


def quaternion_facing_goal(start_pos, target_pos):
    direction = np.asarray(target_pos, dtype=np.float32)[:2] - np.asarray(start_pos, dtype=np.float32)[:2]
    yaw = float(np.arctan2(direction[1], direction[0]))
    return np.array([0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)], dtype=np.float32)


def wrap_to_pi(angle):
    return float((angle + np.pi) % (2.0 * np.pi) - np.pi)


def quaternion_yaw_xyzw(quat):
    qx, qy, qz, qw = np.asarray(quat, dtype=np.float32).reshape(4)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return float(np.arctan2(siny_cosp, cosy_cosp))


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


class TrackVelDeepSetsGRU(nn.Module):
    """Permutation-invariant obstacle-set encoder followed by a recurrent policy."""

    def __init__(
        self,
        state_dim=10,
        obstacle_dim=5,
        obstacle_embedding_dim=32,
        recurrent_hidden_dim=64,
        output_dim=4,
        recurrent_detach=True,
    ):
        super().__init__()
        self.recurrent_hidden_dim = int(recurrent_hidden_dim)
        self.recurrent_detach = bool(recurrent_detach)
        self._hidden_state = None

        self.obstacle_phi = nn.Sequential(
            nn.Linear(obstacle_dim, 32),
            nn.ReLU(),
            nn.Linear(32, obstacle_embedding_dim),
            nn.ReLU(),
        )
        self.obstacle_rho = nn.Sequential(
            nn.Linear(2 * obstacle_embedding_dim + 1, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
        )
        self.state_encoder = nn.Sequential(
            nn.Linear(state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
        )
        self.fusion = nn.Sequential(
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 32),
            nn.ReLU(),
        )
        self.gru = nn.GRU(
            input_size=32,
            hidden_size=self.recurrent_hidden_dim,
            batch_first=True,
        )
        self.output_layer = nn.Linear(self.recurrent_hidden_dim, output_dim)

    def reset_hidden(self, batch_size=None, device=None, dtype=None):
        if batch_size is None:
            self._hidden_state = None
            return

        parameter = next(self.parameters())
        self._hidden_state = torch.zeros(
            1,
            int(batch_size),
            self.recurrent_hidden_dim,
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

    def encode_obstacles(self, obstacle_features, obstacle_mask):
        encoded = self.obstacle_phi(obstacle_features)
        mask = obstacle_mask.unsqueeze(-1)
        float_mask = mask.to(encoded.dtype)

        count = float_mask.sum(dim=1).clamp(min=1.0)
        mean_pool = (encoded * float_mask).sum(dim=1) / count

        max_input = encoded.masked_fill(~mask, torch.finfo(encoded.dtype).min)
        max_pool = max_input.max(dim=1).values
        no_obstacle = ~obstacle_mask.any(dim=1, keepdim=True)
        max_pool = torch.where(no_obstacle, torch.zeros_like(max_pool), max_pool)

        count_feature = torch.log1p(count)
        pooled = torch.cat((mean_pool, max_pool, count_feature), dim=-1)
        return self.obstacle_rho(pooled)

    def forward(self, state, obstacle_features, obstacle_mask):
        state_embedding = self.state_encoder(state)
        obstacle_embedding = self.encode_obstacles(obstacle_features, obstacle_mask)
        fused = self.fusion(torch.cat((state_embedding, obstacle_embedding), dim=-1)).unsqueeze(1)
        output, next_hidden = self.gru(fused, self._compatible_hidden(fused))
        self._hidden_state = (
            next_hidden.detach() if self.recurrent_detach else next_hidden
        )
        raw_action = self.output_layer(output[:, -1])
        thrust = torch.sigmoid(raw_action[:, 0:1])
        body_rates = torch.tanh(raw_action[:, 1:4])
        return torch.cat((thrust, body_rates), dim=-1)


def handler(signal_received, frame):
    print("SIGINT or CTRL-C detected. Exiting gracefully")
    raise SystemExit(0)


def load_policy_state_dict(policy_path, map_location):
    try:
        return torch.load(policy_path, map_location=map_location, weights_only=True)
    except TypeError:
        return torch.load(policy_path, map_location=map_location)


def generate_training_obstacles(config_params, expected_count):
    room_size = float(config_params["room_size"])
    obs_size = float(config_params.get("obs_size", 0.5))
    min_obs_gap = float(config_params.get("min_obs_gap", 0.0))
    # Keep Gazebo test obstacles at the same fixed locations used by the
    # original non-DeepSets test script and SceneManager rand_obs=False path.
    rng = np.random.RandomState(FIXED_OBSTACLE_SEED)

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
    return positions, radii, f"regenerated_scene_manager_seed_{FIXED_OBSTACLE_SEED}"


def infer_config_obstacle_count(config_params):
    for key in ("num_obstacles", "max_num_obstacles", "min_num_obstacles"):
        if key in config_params:
            return max(0, int(config_params[key]))
    raw_obstacles = config_params.get("obstacles", None)
    if raw_obstacles is None:
        return 0
    return len(raw_obstacles)


def load_obstacle_data_npz(obstacle_data_path, expected_count=None):
    data = np.load(obstacle_data_path)
    if "positions" not in data.files or "radii" not in data.files:
        raise ValueError(
            f"{obstacle_data_path} must contain 'positions' and 'radii'. "
            f"Found keys: {data.files}"
        )

    positions = np.asarray(data["positions"], dtype=np.float32).reshape(-1, 3)
    radii = np.asarray(data["radii"], dtype=np.float32).reshape(-1)
    if expected_count is not None and (len(positions) != expected_count or len(radii) != expected_count):
        raise ValueError(
            "Obstacle count does not match policy input dimension: "
            f"npz has positions={len(positions)}, radii={len(radii)}, "
            f"policy expects {expected_count}."
        )
    return positions, radii, f"obstacle_data_npz:{obstacle_data_path}"


def parse_obstacles(config_params, expected_count=None, obstacle_data_path=DEFAULT_OBSTACLE_DATA_PATH):
    positions = FIXED_OBSTACLE_POSITIONS.copy()
    radii = FIXED_OBSTACLE_RADII.copy()
    if expected_count is not None and len(positions) != expected_count:
        raise ValueError(
            "Hard-coded obstacle count does not match policy/config expectation: "
            f"fixed has {len(positions)}, policy/config expects {expected_count}."
        )
    return positions, radii, "hardcoded_test_20260803_171941_obstacle_data"

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
        if expected_count is None:
            expected_count = infer_config_obstacle_count(config_params)
        positions, radii, source = generate_training_obstacles(config_params, expected_count)

    if expected_count is not None and len(positions) != expected_count:
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


def create_obstacle_set(pos, obstacle_positions, obstacle_radii, max_range):
    if max_range <= 0.0:
        raise ValueError("max_range must be positive")
    batch_size = pos.shape[0]
    if obstacle_positions.numel() == 0:
        return (
            torch.zeros((batch_size, 1, 5), device=pos.device, dtype=pos.dtype),
            torch.zeros((batch_size, 1), device=pos.device, dtype=torch.bool),
        )

    rel_xy = obstacle_positions[None, :, :2] - pos[:, None, :2]
    centre_distance = torch.norm(rel_xy, dim=-1, keepdim=True)
    radii = obstacle_radii[None, :, None].expand(batch_size, -1, -1)
    clearance = centre_distance - radii
    obstacle_features = torch.cat(
        (rel_xy, radii, centre_distance, clearance),
        dim=-1,
    ) / float(max_range)
    obstacle_mask = centre_distance.squeeze(-1) <= float(max_range)
    return obstacle_features, obstacle_mask


class VelocityPolicy:
    def __init__(self, policy_path, config_params, device="cpu"):
        self.device = torch.device(device)
        state_dict = load_policy_state_dict(policy_path, map_location=self.device)
        required_keys = (
            "obstacle_phi.0.weight",
            "obstacle_phi.2.weight",
            "obstacle_rho.0.weight",
            "state_encoder.0.weight",
            "fusion.0.weight",
            "gru.weight_ih_l0",
            "gru.weight_hh_l0",
            "output_layer.weight",
        )
        missing_keys = [key for key in required_keys if key not in state_dict]
        if missing_keys:
            raise RuntimeError(
                "This DeepSets GRU test script expects a TrackVelDeepSetsGRU checkpoint. "
                f"Missing keys: {missing_keys}. "
                "Please point DEFAULT_POLICY_DIR/policy_path to the DeepSets GRU policy.pth."
            )

        state_dim = int(state_dict["state_encoder.0.weight"].shape[1])
        obstacle_dim = int(state_dict["obstacle_phi.0.weight"].shape[1])
        obstacle_embedding_dim = int(state_dict["obstacle_phi.2.weight"].shape[0])
        recurrent_hidden_dim = int(state_dict["gru.weight_hh_l0"].shape[1])
        output_dim = int(state_dict["output_layer.weight"].shape[0])
        if state_dim != 10:
            raise RuntimeError(f"Expected state_dim=10 for att+angvel+diff_vel, got {state_dim}.")
        if obstacle_dim != 5:
            raise RuntimeError(
                "This runtime builds 5-D obstacle features "
                "[rel_x, rel_y, radius, distance_xy, clearance], "
                f"but the checkpoint expects obstacle_dim={obstacle_dim}."
            )
        if output_dim != 4:
            raise RuntimeError(f"Expected output_dim=4 for throttle/body-rates, got {output_dim}.")

        self.expected_obstacles = infer_config_obstacle_count(config_params)
        self.policy = TrackVelDeepSetsGRU(
            state_dim=state_dim,
            obstacle_dim=obstacle_dim,
            obstacle_embedding_dim=obstacle_embedding_dim,
            recurrent_hidden_dim=recurrent_hidden_dim,
            output_dim=4,
        ).to(self.device)
        self.policy.load_state_dict(state_dict)
        self.policy.eval()
        self.reset_hidden(batch_size=1)

        self.max_speed = float(config_params.get("max_speed", 1.0))
        self.max_range = float(config_params.get("max_range", 20.0))
        self.altitude_kp = float(config_params.get("altitude_kp", 1.0))
        self.max_vertical_speed = float(config_params.get("max_vertical_speed", 0.3))
        self.target_velocity_ramp_duration = float(
            config_params.get("target_velocity_ramp_duration", 0.3)
        )
        if self.max_range <= 0.0:
            raise ValueError("max_range must be positive")
        if self.altitude_kp < 0.0:
            raise ValueError("altitude_kp must be non-negative")
        if self.max_vertical_speed <= 0.0:
            raise ValueError("max_vertical_speed must be positive")
        self.last_obs_vec = None
        self.last_obstacle_mask = None
        self.last_desired_vel = None
        self.last_diff_vel = None
        self.last_nearest_clearance = np.nan
        expected_count = self.expected_obstacles if self.expected_obstacles > 0 else None
        obstacle_positions, obstacle_radii, obstacle_source = parse_obstacles(
            config_params,
            expected_count,
        )
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

    def evaluate(self, pos, att, qd, elapsed_time=None):
        pos = pos.to(self.device)
        att = att.to(self.device)
        qd = qd.to(self.device)
        att = torch.where(att[:, 3:4] < 0.0, -att, att)

        vel = qd[:, 3:]
        angvel = qd[:, :3]

        position_error = self.t_pos - pos
        xy_error = position_error[:, :2]
        xy_distance = torch.norm(xy_error, dim=1, keepdim=True)
        target_velocity_xy = (xy_error / (xy_distance + 1e-8)) * self.max_speed
        xy_speed_scale = torch.clamp(xy_distance / 1.0, min=0.0, max=1.0)
        target_velocity_xy = target_velocity_xy * xy_speed_scale

        altitude_error = position_error[:, 2:3]
        target_velocity_z = torch.clamp(
            self.altitude_kp * altitude_error,
            min=-self.max_vertical_speed,
            max=self.max_vertical_speed,
        )

        if elapsed_time is None:
            ramp_scale = 1.0
        elif self.target_velocity_ramp_duration <= 0.0:
            ramp_scale = 1.0
        else:
            ramp_scale = min(float(elapsed_time) / self.target_velocity_ramp_duration, 1.0)
        target_velocity_xy = target_velocity_xy * ramp_scale
        desired_vel = torch.cat(
            (target_velocity_xy, target_velocity_z),
            dim=1,
        )
        obstacle_features, obstacle_mask = create_obstacle_set(
            pos,
            self.obstacle_positions,
            self.obstacle_radii,
            self.max_range,
        )
        if self.obstacle_positions.numel() > 0:
            clearances = torch.norm(self.obstacle_positions[:, :2].unsqueeze(0) - pos[:, :2].unsqueeze(1), dim=-1) - self.obstacle_radii.unsqueeze(0)
            self.last_nearest_clearance = float(clearances.min().detach().cpu().item())
        state = torch.cat((att, angvel, desired_vel - vel), dim=1)
        self.last_obs_vec = obstacle_features.detach().cpu().numpy().reshape(-1)
        self.last_obstacle_mask = obstacle_mask.detach().cpu().numpy().reshape(-1)
        self.last_desired_vel = desired_vel.detach().cpu().numpy().reshape(-1)
        self.last_diff_vel = (desired_vel - vel).detach().cpu().numpy().reshape(-1)
        self.last_obs = state.detach().cpu().numpy()

        with torch.no_grad():
            action = self.policy(state, obstacle_features, obstacle_mask)
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
        # [0, room_size] x [0, room_size]. scene_offset maps Gazebo world
        # coordinates into that policy room frame: sim = world + scene_offset.
        self.scene_offset = np.asarray(
            config_params.get("scene_offset", DEFAULT_SCENE_OFFSET),
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
        self.start_hold_start_time = None
        self.test_finished = False
        self.saved_outputs = False
        self.max_nn_duration = float(config_params.get("test_max_duration", inference_timestep * int(config_params.get("sim_steps", 300))))
        self.start_reached_radius = float(config_params.get("start_reached_radius", 0.20))
        self.start_hold_max_speed = float(config_params.get("start_hold_max_speed", 0.15))
        self.start_hold_duration = float(config_params.get("start_hold_duration", 1.0))
        self.start_yaw_tolerance_deg = float(config_params.get("start_yaw_tolerance_deg", 5.0))
        self.start_yaw_tolerance = np.deg2rad(self.start_yaw_tolerance_deg)
        self.last_start_yaw = np.nan
        self.last_start_target_yaw = quaternion_yaw_xyzw(self.start_quat)
        self.last_start_yaw_error = np.nan

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
        log_columns = [
            "stamp",
            "pos_x", "pos_y", "pos_z",
            "vel_x", "vel_y", "vel_z",
            "desired_vel_x", "desired_vel_y", "desired_vel_z",
            "diff_vel_x", "diff_vel_y", "diff_vel_z",
            "nearest_clearance",
            "yaw", "yaw_target", "yaw_error",
        ]
        for obs_id in range(len(self.policy.obstacle_positions_np)):
            log_columns.extend([
                f"obs{obs_id + 1}_rel_x",
                f"obs{obs_id + 1}_rel_y",
                f"obs{obs_id + 1}_radius",
                f"obs{obs_id + 1}_distance_xy",
                f"obs{obs_id + 1}_clearance",
            ])
        log_columns.extend(["action_throttle", "action_rate_x", "action_rate_y", "action_rate_z"])
        self.runtime_log_file.write(",".join(log_columns) + "\n")

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
            "Start hold before NN: "
            f"radius={self.start_reached_radius:.2f} m, "
            f"max_speed={self.start_hold_max_speed:.2f} m/s, "
            f"yaw_tolerance={self.start_yaw_tolerance_deg:.1f} deg, "
            f"duration={self.start_hold_duration:.2f} s"
        )
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
            linear_speed = np.linalg.norm(np.asarray(self.warp_qd[3:], dtype=np.float32))
            yaw, target_yaw, yaw_error = self.start_yaw_status()
            yaw_error_abs = abs(yaw_error)
            ready_to_hold = (
                dist_to_start < self.start_reached_radius
                and linear_speed < self.start_hold_max_speed
                and yaw_error_abs < self.start_yaw_tolerance
                and self.has_nn_action
            )
            now = rospy.Time.now()

            if ready_to_hold:
                if self.start_hold_start_time is None:
                    self.start_hold_start_time = now
                    print(
                        "Reached start; holding before NN "
                        f"(dist={dist_to_start:.3f} m, speed={linear_speed:.3f} m/s, "
                        f"yaw_error={np.rad2deg(yaw_error):.1f} deg)."
                    )
                hold_elapsed = (now - self.start_hold_start_time).to_sec()
            else:
                if self.start_hold_start_time is not None:
                    print(
                        "Start hold reset "
                        f"(dist={dist_to_start:.3f} m, speed={linear_speed:.3f} m/s, "
                        f"yaw_error={np.rad2deg(yaw_error):.1f} deg)."
                    )
                elif dist_to_start < self.start_reached_radius and linear_speed < self.start_hold_max_speed:
                    rospy.loginfo_throttle(
                        1.0,
                        "Waiting for start yaw alignment before NN "
                        "(yaw=%.1f deg, target=%.1f deg, error=%.1f deg, tol=%.1f deg).",
                        np.rad2deg(yaw),
                        np.rad2deg(target_yaw),
                        np.rad2deg(yaw_error),
                        self.start_yaw_tolerance_deg,
                    )
                self.start_hold_start_time = None
                hold_elapsed = 0.0

            if ready_to_hold and hold_elapsed >= self.start_hold_duration:
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

    def start_yaw_status(self):
        current_sim = np.asarray(self.warp_q[:3], dtype=np.float32) + self.scene_offset
        direction = self.target_pos_sim[:2] - current_sim[:2]
        if np.linalg.norm(direction) > 1e-4:
            target_yaw = float(np.arctan2(direction[1], direction[0]))
        else:
            target_yaw = self.last_start_target_yaw

        yaw = quaternion_yaw_xyzw(self.warp_q[3:])
        yaw_error = wrap_to_pi(target_yaw - yaw)
        self.last_start_yaw = yaw
        self.last_start_target_yaw = target_yaw
        self.last_start_yaw_error = yaw_error
        return yaw, target_yaw, yaw_error

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
        if self.nn_start_time is None:
            elapsed_time = 0.0
        else:
            elapsed_time = (rospy.Time.now() - self.nn_start_time).to_sec()
        self.action = self.policy.evaluate(pos_sim, quat, qd, elapsed_time=elapsed_time)
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
        yaw, target_yaw, yaw_error = self.start_yaw_status()
        row = [
            f"{now:.6f}",
            f"{pos_sim[0]:.9f}", f"{pos_sim[1]:.9f}", f"{pos_sim[2]:.9f}",
            f"{vel[0]:.9f}", f"{vel[1]:.9f}", f"{vel[2]:.9f}",
            f"{desired_vel[0]:.9f}", f"{desired_vel[1]:.9f}", f"{desired_vel[2]:.9f}",
            f"{diff_vel[0]:.9f}", f"{diff_vel[1]:.9f}", f"{diff_vel[2]:.9f}",
            f"{self.policy.last_nearest_clearance:.9f}",
            f"{yaw:.9f}", f"{target_yaw:.9f}", f"{yaw_error:.9f}",
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
        throttle_height_path = os.path.join(self.output_dir, "throttle_height.png")
        self.save_xy_gif(trajectory, xy_path)
        self.save_xz_gif(trajectory, xz_path)
        self.save_body_rates_plot(body_rates_path)
        self.save_throttle_height_plot(actions, trajectory, throttle_height_path)
        with open(os.path.join(self.output_dir, "metadata.yaml"), "w") as f:
            yaml.safe_dump(
                {
                    "policy_dir": DEFAULT_POLICY_DIR,
                    "start_pos_sim": self.start_pos_sim.tolist(),
                    "target_pos_sim": self.target_pos_sim.tolist(),
                    "start_quat_xyzw": self.start_quat.tolist(),
                    "scene_offset": self.scene_offset.tolist(),
                    "start_reached_radius": self.start_reached_radius,
                    "start_hold_max_speed": self.start_hold_max_speed,
                    "start_hold_duration": self.start_hold_duration,
                    "start_yaw_tolerance_deg": self.start_yaw_tolerance_deg,
                    "last_start_yaw_deg": float(np.rad2deg(self.last_start_yaw)),
                    "last_start_target_yaw_deg": float(np.rad2deg(self.last_start_target_yaw)),
                    "last_start_yaw_error_deg": float(np.rad2deg(self.last_start_yaw_error)),
                    "coordinate_note": (
                        "Obstacle, start, target, and saved trajectory coordinates are in the "
                        "training/policy room frame. Gazebo world coordinates are mapped with "
                        "sim = world + scene_offset and world = sim - scene_offset."
                    ),
                    "obstacle_source": self.policy.obstacle_source,
                    "obstacle_count": int(len(self.policy.obstacle_positions_np)),
                    "obstacle_positions": self.policy.obstacle_positions_np.tolist(),
                    "obstacle_radii": self.policy.obstacle_radii_np.tolist(),
                    "trajectory_xy_gif": xy_path,
                    "trajectory_xz_gif": xz_path,
                    "body_rates_png": body_rates_path,
                    "throttle_height_png": throttle_height_path,
                    "runtime_obs_csv": self.runtime_log_path,
                },
                f,
                sort_keys=False,
            )
        print(f"Saved GIFs: {xy_path}, {xz_path}")
        print(f"Saved body rates plot: {body_rates_path}")
        print(f"Saved throttle/height plot: {throttle_height_path}")

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

    def save_throttle_height_plot(self, actions, trajectory, save_path):
        if actions.size == 0 or trajectory.size == 0:
            return

        throttle = np.asarray(actions[:, 0], dtype=np.float32)
        height = np.asarray(trajectory[:, 2], dtype=np.float32)
        sample_count = min(len(throttle), len(height))
        throttle = throttle[:sample_count]
        height = height[:sample_count]

        if len(self.body_rate_time_log) >= sample_count:
            time_axis = np.asarray(self.body_rate_time_log[:sample_count], dtype=np.float64)
            time_axis = time_axis - time_axis[0]
        else:
            time_axis = np.arange(sample_count, dtype=np.float64) * float(self.inference_timestep)

        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        axes[0].plot(time_axis, throttle, linewidth=1.8)
        axes[0].set_ylabel("Throttle")
        axes[0].set_ylim(
            max(0.0, float(np.min(throttle)) - 0.05),
            min(1.0, float(np.max(throttle)) + 0.05),
        )
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title("Published NN Throttle and Height")

        axes[1].plot(time_axis, height, linewidth=1.8, color="tab:red")
        axes[1].set_ylabel("Height z (m)")
        axes[1].set_xlabel("Time since NN control start (s)")
        axes[1].grid(True, alpha=0.3)

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
