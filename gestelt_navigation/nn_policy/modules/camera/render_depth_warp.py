"""Depth rendering of the gate, extracted into a reusable function.

Mirrors the pipeline in tempstorage3's
  examples/train_velpos_simple_replicate_nwu_global_gate_trav_sjtu_normalize_depth_cbam_gru_multi_env.py
specifically `build_scene()` (line ~200) and `depth_to_policy_input()` (line ~78).
Same SceneManager, same gate builder, same mount quaternion, same normalisation —
so an image from here is the same tensor the depth policy would be fed.

    cam = GateDepthCamera(gate_centers, gate_quats)
    depth = cam.render(positions, orientations)      # (B,H,W) metres
    obs   = cam.render_policy_input(positions, orientations)   # (B,H,W) in [0,1]

Pipeline, per build_scene():

  SceneManager(batch_size, device)                     flyinglib.scene.scene_manager_randgate
    .add_gate_batch_from_training(centers, quats,      one gate MESH PER ENV
                                  hole_size, bar_size, frame_outer_size)
    .add_additional_rotation([-0.5, 0.5, -0.5, 0.5])   body -> camera, forward-looking
    .add_camera('front', cfg, pos, quat)               -> auto-selects "per_env" mode
    .capture_depth('front')                            -> (B,1,H,W) metres
  depth_to_policy_input(depth, max_range, invert)      -> (B,H,W) in [0,1]

WHY per_env MATTERS. The depth kernel does `mesh = mesh_ids[env_id]`
(warp_camera_kernels.py:190) — it raycasts against exactly ONE mesh. SceneManager
detects `len(self.meshes) == batch_size` and switches to num_envs=B / num_sensors=1
so each env indexes its own gate mesh. In the older "shared" mode (num_envs=1) only
mesh_ids[0] is ever visible, which is why gate geometry cannot be appended to a
shared scene with add_mesh().

NOTE: this required updating flyinglib/sensors/warp/warp_cam.py from tempstorage3.
The old version hardcoded `additional_camera_rotation.reshape(1, num_sensors, 4)`,
which cannot express per_env layout (num_envs=B, num_sensors=1). The new version
dispatches on the actual shape and is backward compatible with the num_envs=1 path.
"""

import math
import os
import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation as R

from scene_manager_randgate_ros import SceneManager

# nn_policy package root (modules/camera/render_depth_warp.py -> nn_policy), so config
# lookups don't depend on the caller's cwd (rosrun/roslaunch may start elsewhere).
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# Body -> camera mount, (x, y, z, w). Verbatim from build_scene() line ~222, and
# identical to `additional_camera_rot` in the older training scripts. Maps camera
# convention (+z forward, +x right, +y down) onto NWU body (+x fwd, +y left, +z up),
# i.e. forward-looking with no pitch. Verified: a box at (3,0,0) renders dead centre.
MOUNT_QUAT_FORWARD = np.array([-0.5, 0.5, -0.5, 0.5], dtype=np.float32)

# Defaults from diff_sim_velocity_tracking_nwu_gate_traversal_depth_cbam_gru_multi_env.yaml
DEFAULT_CAMERA_CONFIG = {
    'width': 64,
    'height': 64,
    'horizontal_fov_deg': 120.0,
    'max_range': 20.0,
    'calculate_depth': True,
    'segmentation_camera': False,
    'return_pointcloud': False,
}
DEFAULT_GATE = {
    'hole_size': [0.5, 0.3],          # gate_hole_size   — inner aperture (width, height)
    'frame_outer_size': [0.7, 0.5],   # gate_frame_outer_size — wall-frame outer extent
    'bar_size': 0.20,                 # gate_frame_depth — thickness along the gate normal
}


def gate_quat_from_roll(roll_deg) -> np.ndarray:
    """Gate quaternion (x,y,z,w) for a roll of `roll_deg` about world x.

    Matches how the training scripts build window_orientation_array:
        q = [sin(a/2), 0, 0, cos(a/2)]
    Accepts a scalar or an array; returns (N,4).
    """
    a = np.atleast_1d(np.asarray(roll_deg, np.float32)) * np.pi / 360.0   # a/2 in rad
    z = np.zeros_like(a)
    return np.stack([np.sin(a), z, z, np.cos(a)], axis=1).astype(np.float32)


def mount_quat_pitched(pitch_deg: float) -> np.ndarray:
    """Forward mount rotated by `pitch_deg`, positive = pitched DOWN. (x,y,z,w).

    The negation is deliberate: in the camera frame (+z fwd, +x right, +y down) a
    right-handed rotation about +x sends forward toward -y, i.e. UP. Verified — a box
    below the drone moves toward the image centre as pitch_deg increases.
    pitch_deg=0 reproduces MOUNT_QUAT_FORWARD exactly.
    """
    h = math.radians(-pitch_deg) / 2.0
    dq = np.array([math.sin(h), 0.0, 0.0, math.cos(h)], dtype=np.float32)
    return _quat_mul(MOUNT_QUAT_FORWARD, dq)


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=np.float32)


def depth_to_policy_input(depth, max_range, invert=True, device=None):
    """Verbatim port of depth_to_policy_input() from the multi_env training script.

    Accepts (B,H,W) or (B,1,H,W)/(1,B,H,W); returns (B,H,W) in [0,1].
    invert=True gives 1 - d/max_range, so "near" is bright and empty sky is 0.
    """
    if isinstance(depth, np.ndarray):
        depth = torch.from_numpy(depth)
    if device is not None:
        depth = depth.to(device)
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


def load_config(path):
    """Load a training config. Accepts any of:

      * a path to a .yaml file
      * a run directory containing training_config.yaml
      * a run timestamp, resolved against `log_roots` below

    Returns a SimpleNamespace, matching load_config() in the training script.
    """
    from types import SimpleNamespace

    p = str(path)
    if os.path.isdir(p):
        p = os.path.join(p, "training_config.yaml")
    elif not p.endswith((".yaml", ".yml")):
        log_roots = [
            "logs/vel_tracking_depth_cbam_gru_multi_env_full",
            "logs/vel_tracking_depth_cbam_gru_multi_env",
            "logs/vel_tracking",
        ]
        for root in log_roots:
            cand = os.path.join(_PKG_ROOT, root, p, "training_config.yaml")
            if os.path.exists(cand):
                p = cand
                break
    if not os.path.exists(p):
        raise FileNotFoundError(f"config not found: {path} (resolved to {p})")
    with open(p, "r") as f:
        data = yaml.safe_load(f)
    if not data:
        raise RuntimeError(f"Empty config: {p}")
    ns = SimpleNamespace(**data)
    ns._config_path = p
    return ns


def camera_config_from(cfg) -> dict:
    """Camera dict exactly as build_scene() assembles it (num_sensors added later).

    Every key is required by CameraConfig, so this fails loudly on a config that
    predates the depth pipeline rather than silently substituting a default.
    """
    missing = [k for k in ('width', 'height', 'horizontal_fov_deg', 'max_range',
                           'calculate_depth', 'segmentation_camera', 'return_pointcloud')
               if not hasattr(cfg, k)]
    if missing:
        raise KeyError(f"config {getattr(cfg, '_config_path', '?')} has no depth-camera "
                       f"section; missing {missing}")
    return {
        'width': int(cfg.width),
        'height': int(cfg.height),
        'horizontal_fov_deg': float(cfg.horizontal_fov_deg),
        'max_range': float(cfg.max_range),
        'calculate_depth': bool(cfg.calculate_depth),
        'segmentation_camera': bool(cfg.segmentation_camera),
        'return_pointcloud': bool(cfg.return_pointcloud),
    }


def gate_config_from(cfg) -> dict:
    """Gate geometry as build_scene() passes it to add_gate_batch_from_training."""
    return {
        'hole_size': list(getattr(cfg, 'gate_hole_size', DEFAULT_GATE['hole_size'])),
        'frame_outer_size': getattr(cfg, 'gate_frame_outer_size',
                                    DEFAULT_GATE['frame_outer_size']),
        'bar_size': float(getattr(cfg, 'gate_frame_depth', DEFAULT_GATE['bar_size'])),
    }


def sample_gate_poses(cfg, batch_size):
    """Verbatim port of sample_per_env_gate_poses() from the training script.

    `window_location_center_range` [3,2] overrides `window_location`, and
    `window_orientation_roll_range_deg` [2] overrides `window_orientation` +/-
    `window_orientation_range`. Returns (centers (B,3), quats (B,4) xyzw, rolls (B,)).
    """
    center = np.asarray(cfg.window_location, dtype=np.float32)
    center_range = getattr(cfg, "window_location_center_range", None)
    if center_range is None:
        centers = np.repeat(center[None, :], batch_size, axis=0)
    else:
        bounds = np.asarray(center_range, dtype=np.float32)
        if bounds.shape != (3, 2):
            raise ValueError("window_location_center_range must have shape [3,2]")
        centers = np.column_stack(
            [np.random.uniform(bounds[i, 0], bounds[i, 1], batch_size) for i in range(3)]
        ).astype(np.float32)

    roll_range = getattr(cfg, "window_orientation_roll_range_deg", None)
    if roll_range is None:
        half_range = float(getattr(cfg, "window_orientation_range", 0.0))
        nominal = float(cfg.window_orientation)
        rolls = np.random.uniform(nominal - half_range, nominal + half_range, batch_size)
    else:
        rolls = np.random.uniform(float(roll_range[0]), float(roll_range[1]), batch_size)
    quats = R.from_euler("x", rolls, degrees=True).as_quat().astype(np.float32)
    return centers.astype(np.float32), quats, rolls.astype(np.float32)


class GateDepthCamera:
    """Batched depth camera over a per-env gate scene.

    Scene and camera are built once; render() only updates the pose and relaunches.
    One gate mesh per env, so each env may have its own gate pose — exactly the
    layout build_scene() produces for multi-env training.
    """

    def __init__(self,
                 gate_centers,
                 gate_quats,
                 camera_config: dict = None,
                 gate: dict = None,
                 mount_quat=None,
                 invert: bool = True,
                 device: str = 'cuda:0'):
        """
        gate_centers  (B,3) gate positions, world NWU
        gate_quats    (B,4) gate orientations (x,y,z,w); see gate_quat_from_roll()
        camera_config overrides for DEFAULT_CAMERA_CONFIG (num_sensors is set for you)
        gate          overrides for DEFAULT_GATE (hole_size / frame_outer_size / bar_size)
        """
        centers = np.asarray(gate_centers, np.float32).reshape(-1, 3)
        quats = np.asarray(gate_quats, np.float32).reshape(-1, 4)
        if centers.shape[0] != quats.shape[0]:
            raise ValueError("gate_centers and gate_quats must have the same batch size")
        self.batch_size = centers.shape[0]
        self.device = device
        self.invert = bool(invert)          # cfg.invert_depth; used by render_policy_input
        self.gate_centers = centers
        self.gate_quats = quats

        cfg = dict(DEFAULT_CAMERA_CONFIG)
        if camera_config:
            cfg.update(camera_config)
        cfg['num_sensors'] = self.batch_size          # overridden to 1 by per_env mode
        self.camera_config = cfg
        self.max_range = float(cfg['max_range'])

        g = dict(DEFAULT_GATE)
        if gate:
            g.update(gate)
        self.gate = g

        # ---- build_scene(), step for step ----
        self.scene = SceneManager(batch_size=self.batch_size, device=device)
        self.scene.add_gate_batch_from_training(
            window_locations=centers,
            window_orientations_xyzw=quats,
            hole_size=g['hole_size'],
            bar_size=float(g['bar_size']),
            frame_outer_size=g['frame_outer_size'],
        )

        q = MOUNT_QUAT_FORWARD if mount_quat is None else np.asarray(mount_quat, np.float32)
        mount = np.tile(q, (self.batch_size, 1)).astype(np.float32)
        self.scene.add_additional_rotation(torch.tensor(mount, device=device))

        zp = torch.zeros((self.batch_size, 3), dtype=torch.float32, device=device)
        zq = torch.zeros((self.batch_size, 4), dtype=torch.float32, device=device)
        zq[:, 3] = 1.0
        self.scene.add_camera('front', config=cfg, positions=zp, orientations=zq)

        # build_scene() hard-fails here rather than silently rendering a shared scene,
        # because in shared mode only env 0's gate would be visible to every camera.
        if self.scene.camera_modes.get('front') != 'per_env':
            raise RuntimeError("Expected one independent scene/depth renderer per environment")

    def render(self, positions, orientations) -> torch.Tensor:
        """Raw depth in metres, (B,H,W).

        Rays that hit nothing return NO_HIT_RAY_VAL = 1000.0 (warp_camera_kernels.py:3),
        NOT max_range — so the raw tensor is not bounded by max_range and a naive
        imshow will be dominated by the sentinel. render_policy_input() clamps it,
        which is what the training script feeds the network.
        """
        p = self._as_tensor(positions, (self.batch_size, 3))
        o = self._as_tensor(orientations, (self.batch_size, 4))
        self.scene.set_camera_pose_tensor('front', p, o)
        depth = self.scene.capture_depth('front')          # (B,1,H,W) in per_env mode
        return depth.reshape(self.batch_size, self.camera_config['height'],
                             self.camera_config['width']).clone()

    @classmethod
    def from_config(cls, config, batch_size=None, gate_centers=None, gate_quats=None,
                    seed=None, **overrides):
        """Build straight from a training config — every camera and gate parameter
        comes from the YAML, so the render matches what that run was trained on.

        config       .yaml path, run directory, or run timestamp (see load_config)
        batch_size   how many envs; defaults to 1. Gate poses are sampled per
                     sample_per_env_gate_poses() unless you pass them explicitly.
        seed         seeds numpy before gate sampling, for a reproducible scene
        overrides    forwarded to __init__ (e.g. mount_quat, device)
        """
        cfg = config if hasattr(config, 'width') else load_config(config)

        if gate_centers is None or gate_quats is None:
            b = int(batch_size) if batch_size else 1
            if seed is None:
                gate_centers, gate_quats, _ = sample_gate_poses(cfg, b)
            else:
                # Save/restore the global numpy state around the seeded draw. sample_gate_poses
                # is a verbatim port and uses np.random, so seeding it would otherwise reset the
                # caller's stream — which matters here, where set_global_seed() governs run
                # reproducibility and a render call must not perturb it.
                state = np.random.get_state()
                try:
                    np.random.seed(int(seed))
                    gate_centers, gate_quats, _ = sample_gate_poses(cfg, b)
                finally:
                    np.random.set_state(state)

        cam = cls(gate_centers=gate_centers,
                  gate_quats=gate_quats,
                  camera_config=camera_config_from(cfg),
                  gate=gate_config_from(cfg),
                  invert=bool(getattr(cfg, 'invert_depth', True)),
                  **overrides)
        cam.cfg = cfg
        return cam

    def render_policy_input(self, positions, orientations, invert=None) -> torch.Tensor:
        """What the depth policy actually consumes: (B,H,W) in [0,1].

        `invert` defaults to the config's invert_depth (True unless set otherwise).
        """
        inv = self.invert if invert is None else bool(invert)
        return depth_to_policy_input(self.render(positions, orientations),
                                     self.max_range, invert=inv, device=self.device)

    def _as_tensor(self, x, shape):
        t = x if torch.is_tensor(x) else torch.as_tensor(np.asarray(x, np.float32))
        t = t.to(device=self.device, dtype=torch.float32).contiguous()
        if t.shape != shape:
            raise ValueError(f"expected {shape}, got {tuple(t.shape)}")
        return t


def render_gate_depth(positions, orientations, gate_centers, gate_quats, **kw):
    """One-shot wrapper: build, render once, discard. Rebuilds the scene every call —
    hold a GateDepthCamera instead if you are rendering a trajectory."""
    cam = GateDepthCamera(gate_centers, gate_quats, **kw)
    return cam.render(positions, orientations)


if __name__ == "__main__":
    import argparse, os
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    ap = argparse.ArgumentParser(description="Render the gate from a drone pose.")
    ap.add_argument('--config', type=str, default=None,
                    help="training config (.yaml path, run dir, or run timestamp). "
                         "Takes every camera/gate parameter from it; the flags below "
                         "are then only used for whatever the config does not specify.")
    ap.add_argument('--seed', type=int, default=None, help="seed for gate-pose sampling")
    ap.add_argument('--pos', type=float, nargs=3, default=[-1.5, 0.0, 2.0],
                    help="drone position x y z (NWU)")
    ap.add_argument('--quat', type=float, nargs=4, default=[0.0, 0.0, 0.0, 1.0],
                    help="drone orientation x y z w")
    ap.add_argument('--gate_pos', type=float, nargs=3, default=[1.5, 0.0, 2.0],
                    help="gate centre (charpi default)")
    ap.add_argument('--gate_roll_deg', type=float, default=60.0,
                    help="gate roll about world x (charpi default)")
    ap.add_argument('--hole_size', type=float, nargs=2, default=DEFAULT_GATE['hole_size'])
    ap.add_argument('--outer_size', type=float, nargs=2, default=DEFAULT_GATE['frame_outer_size'])
    ap.add_argument('--bar_size', type=float, default=DEFAULT_GATE['bar_size'])
    ap.add_argument('--pitch_deg', type=float, default=0.0, help="camera pitch, + = down")
    ap.add_argument('--width', type=int, default=DEFAULT_CAMERA_CONFIG['width'])
    ap.add_argument('--height', type=int, default=DEFAULT_CAMERA_CONFIG['height'])
    ap.add_argument('--fov', type=float, default=DEFAULT_CAMERA_CONFIG['horizontal_fov_deg'])
    ap.add_argument('--out', type=str, default='gate_depth')
    args = ap.parse_args()

    B = 1
    if args.config:
        # cam = GateDepthCamera.from_config(
        #     args.config, batch_size=1, seed=args.seed,
        #     mount_quat=mount_quat_pitched(args.pitch_deg))
        cam = GateDepthCamera.from_config(
        args.config,
        gate_centers=np.tile([1.5, 0, 2.0], (B, 1)).astype(np.float32),
        gate_quats=np.repeat(gate_quat_from_roll(60.0), B, axis=0))
        c = cam.camera_config
        print(f"[config] {cam.cfg._config_path}")
        print(f"[camera] {c['width']}x{c['height']}  fov={c['horizontal_fov_deg']} deg  "
              f"max_range={c['max_range']} m  invert_depth={cam.invert}")
        print(f"[gate]   hole={cam.gate['hole_size']}  outer={cam.gate['frame_outer_size']}  "
              f"depth={cam.gate['bar_size']}")
        print(f"[pose]   centre={np.round(cam.gate_centers[0], 3).tolist()}  "
              f"roll={np.degrees(2 * np.arctan2(cam.gate_quats[0][0], cam.gate_quats[0][3])):.1f} deg")
        args.gate_pos = cam.gate_centers[0].tolist()
    else:
        cam = GateDepthCamera(
            gate_centers=[args.gate_pos],
            gate_quats=gate_quat_from_roll(args.gate_roll_deg),
            camera_config={'width': args.width, 'height': args.height,
                           'horizontal_fov_deg': args.fov},
            gate={'hole_size': args.hole_size, 'frame_outer_size': args.outer_size,
                  'bar_size': args.bar_size},
            mount_quat=mount_quat_pitched(args.pitch_deg),
        )
    depth = cam.render([args.pos], [args.quat])
    obs = cam.render_policy_input([args.pos], [args.quat])
    d = depth[0].detach().cpu().numpy()
    o = obs[0].detach().cpu().numpy()

    np.save(f"{args.out}.npy", d)
    fig, ax = plt.subplots(1, 2, figsize=(9, 4))
    im0 = ax[0].imshow(d, cmap='turbo', vmin=0.0, vmax=cam.max_range)
    ax[0].set_title('depth [m]'); fig.colorbar(im0, ax=ax[0])
    im1 = ax[1].imshow(o, cmap='gray', vmin=0.0, vmax=1.0)
    ax[1].set_title('policy input (1 - d/max)'); fig.colorbar(im1, ax=ax[1])
    fig.suptitle(f"drone {args.pos}  gate {args.gate_pos} @ {args.gate_roll_deg:.0f}deg roll")
    plt.tight_layout(); plt.savefig(f"{args.out}.png", dpi=150)

    hit = d < cam.max_range
    print(f"depth {d.shape}  range [{d.min():.2f}, {d.max():.2f}] m")
    print(f"  gate occupies {100.0 * hit.sum() / hit.size:.1f}% of pixels; "
          f"nearest {d[hit].min():.2f} m" if hit.any() else "  GATE NOT VISIBLE")
    print(f"saved {os.path.abspath(args.out)}.npy / .png")
