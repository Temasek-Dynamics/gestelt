import warp as wp
import numpy as np
import torch
from typing import List, Dict, Optional, Union
from warp_cam_ros import WarpCam


class CameraConfig:
    def __init__(self, config_dict: Dict):
        self.width = config_dict['width']
        self.height = config_dict['height']
        self.horizontal_fov_deg = config_dict['horizontal_fov_deg']
        self.max_range = config_dict['max_range']
        self.calculate_depth = config_dict['calculate_depth']
        self.num_sensors = config_dict['num_sensors']
        self.segmentation_camera = config_dict['segmentation_camera']
        self.return_pointcloud = config_dict['return_pointcloud']


class SceneManager:
    def __init__(self, batch_size: int = 1, device: str = "cuda:0"):
        """
        Initialize the scene manager with batch support.

        Args:
            batch_size: Number of parallel environments/simulations
            device: torch device for depth tensors / camera pose tensors
        """
        wp.init()
        self.batch_size = batch_size
        self.device = device

        self.meshes = []                      # List[wp.Mesh]
        self.cameras: Dict[str, WarpCam] = {} # {camera_id: WarpCam}
        self.camera_modes: Dict[str, str] = {} # {camera_id: "shared"|"per_env"}
        self.mesh_ids = None                  # wp.array uint64 of mesh IDs
        self.depth_image = None

        self.objects = []                     # [{"position":..., "radius":..., ...}]
        self.target_pos = None
        self.room_size = None

        # IMPORTANT: default additional rotation for camera (identity)
        # shape should be (batch_size, 4) in xyzw (consistent with your other code)
        self.additional_rotation = torch.tensor(
            np.tile(np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32), (batch_size, 1)),
            dtype=torch.float32,
            device=self.device,
        )

    # -------------------- primitive meshes --------------------
    def create_box_mesh(self, size=1.0, flip_normals=False, size_x=None, size_y=None, size_z=None):
        sx = size_x if size_x is not None else size
        sy = size_y if size_y is not None else size
        sz = size_z if size_z is not None else size

        vertices = np.array([
            [-sx, -sy, -sz],
            [ sx, -sy, -sz],
            [ sx,  sy, -sz],
            [-sx,  sy, -sz],
            [-sx, -sy,  sz],
            [ sx, -sy,  sz],
            [ sx,  sy,  sz],
            [-sx,  sy,  sz]
        ], dtype=np.float32)

        faces = np.array([
            [0,1,2], [0,2,3],  # bottom
            [4,5,6], [4,6,7],  # top
            [0,1,5], [0,5,4],  # front
            [2,3,7], [2,7,6],  # back
            [1,2,6], [1,6,5],  # right
            [0,3,7], [0,7,4]   # left
        ], dtype=np.int32)

        if flip_normals:
            faces = np.flip(faces, axis=1)

        return wp.Mesh(
            points=wp.array(vertices, dtype=wp.vec3),
            indices=wp.array(faces.flatten(), dtype=wp.int32),
            velocities=wp.zeros(len(vertices), dtype=wp.vec3)
        )

    def create_sphere_mesh(self, radius=1.0, flip_normals=False):
        subdivisions = 16
        vertices = []
        for i in range(subdivisions + 1):
            theta = i * np.pi / subdivisions
            for j in range(subdivisions):
                phi = j * 2 * np.pi / subdivisions
                x = radius * np.sin(theta) * np.cos(phi)
                y = radius * np.sin(theta) * np.sin(phi)
                z = radius * np.cos(theta)
                vertices.append([x, y, z])

        faces = []
        for i in range(subdivisions):
            for j in range(subdivisions):
                p1 = i * subdivisions + j
                p2 = i * subdivisions + (j + 1) % subdivisions
                p3 = (i + 1) * subdivisions + j
                p4 = (i + 1) * subdivisions + (j + 1) % subdivisions

                if i != 0:
                    faces.append([p1, p2, p3])
                if i != subdivisions - 1:
                    faces.append([p3, p2, p4])

        vertices = np.array(vertices, dtype=np.float32)
        faces = np.array(faces, dtype=np.int32)

        if flip_normals:
            faces = np.flip(faces, axis=1)

        return wp.Mesh(
            points=wp.array(vertices, dtype=wp.vec3),
            indices=wp.array(faces.flatten(), dtype=wp.int32),
            velocities=wp.zeros(len(vertices), dtype=wp.vec3)
        )

    # -------------------- gate helpers --------------------
    def _rot_z(self, yaw: float) -> np.ndarray:
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array([[c, -s, 0.0],
                         [s,  c, 0.0],
                         [0.0, 0.0, 1.0]], dtype=np.float32)

    def _rot_x(self, tilt: float) -> np.ndarray:
        c, s = np.cos(tilt), np.sin(tilt)
        return np.array([[1.0, 0.0, 0.0],
                         [0.0,  c, -s],
                         [0.0,  s,  c]], dtype=np.float32)

    def _rot_y(self, pitch: float) -> np.ndarray:
        c, s = np.cos(pitch), np.sin(pitch)
        return np.array([[ c, 0.0,  s],
                         [0.0, 1.0, 0.0],
                         [-s, 0.0,  c]], dtype=np.float32)

    def _make_mesh_from_vertices_faces(self, vertices: np.ndarray, faces_flat: np.ndarray) -> wp.Mesh:
        return wp.Mesh(
            points=wp.array(vertices.astype(np.float32), dtype=wp.vec3),
            indices=wp.array(faces_flat.astype(np.int32), dtype=wp.int32),
            velocities=wp.zeros(len(vertices), dtype=wp.vec3)
        )

    def _transform_box_mesh(self, box_mesh: wp.Mesh, R: np.ndarray, t: np.ndarray) -> wp.Mesh:
        v = box_mesh.points.numpy()
        v_w = (v @ R.T) + t[None, :]
        faces_flat = box_mesh.indices.numpy()
        return self._make_mesh_from_vertices_faces(v_w, faces_flat)

    def add_gate(
        self,
        center: Union[np.ndarray, List[float]],
        hole_size: Union[np.ndarray, List[float]],
        roll_deg: float = 0.0,
        pitch_deg: float = 0.0,
        yaw_deg: float = 0.0,
        bar_size: float = 0.20,   # allow override
    ) -> None:
        """
        Create a "real" gate made of 4 thin boxes and add as a combined mesh.
        Also stores each bar as an object (for later collision/distance if you need).
        """
        bar = float(bar_size)

        c = np.array(center, dtype=np.float32).reshape(3)
        w, h = float(hole_size[0]), float(hole_size[1])

        yaw   = np.deg2rad(yaw_deg)
        pitch = np.deg2rad(pitch_deg)
        roll  = np.deg2rad(roll_deg)

        Rx = self._rot_x(roll)
        Ry = self._rot_y(pitch)
        Rz = self._rot_z(yaw)

        # yaw -> pitch -> roll
        R = (Rz @ Ry @ Rx).astype(np.float32)

        ex = R[:, 0]  # right
        ez = R[:, 2]  # up

        # half extents for vertical and horizontal bars
        v_half_x = bar * 0.5
        v_half_y = bar * 0.5
        v_half_z = (h * 0.5) + bar

        h_half_x = (w * 0.5) + bar
        h_half_y = bar * 0.5
        h_half_z = bar * 0.5

        dx = (w * 0.5) + (bar * 0.5)
        dz = (h * 0.5) + (bar * 0.5)

        left_center  = c - dx * ex
        right_center = c + dx * ex
        top_center   = c + dz * ez
        bot_center   = c - dz * ez

        v_bar_local = self.create_box_mesh(size_x=v_half_x, size_y=v_half_y, size_z=v_half_z, flip_normals=False)
        h_bar_local = self.create_box_mesh(size_x=h_half_x, size_y=h_half_y, size_z=h_half_z, flip_normals=False)

        bars = [
            ("gate_left",  v_bar_local, left_center,  (v_half_x, v_half_y, v_half_z)),
            ("gate_right", v_bar_local, right_center, (v_half_x, v_half_y, v_half_z)),
            ("gate_top",   h_bar_local, top_center,   (h_half_x, h_half_y, h_half_z)),
            ("gate_bot",   h_bar_local, bot_center,   (h_half_x, h_half_y, h_half_z)),
        ]

        # combine into one mesh
        all_points = []
        all_indices = []
        all_velocities = []
        vertex_count = 0

        for name, mesh_local, cc, half_ext in bars:
            mesh_world = self._transform_box_mesh(mesh_local, R, cc)

            pts = mesh_world.points.numpy()
            idx = mesh_world.indices.numpy()
            vel = mesh_world.velocities.numpy()

            all_points.extend(pts)
            all_indices.extend(idx + vertex_count)
            all_velocities.extend(vel)
            vertex_count += pts.shape[0]

            hx, hy, hz = half_ext
            radius = float(np.sqrt(hx*hx + hy*hy + hz*hz))
            self.objects.append({
                "position": cc.astype(np.float32),
                "radius": radius,
                "type": "box",
                "name": name,
                "half_extents": np.array([hx, hy, hz], dtype=np.float32),
                "rotation_matrix": R.copy(),
            })

        combined_mesh = wp.Mesh(
            points=wp.array(np.asarray(all_points, dtype=np.float32), dtype=wp.vec3),
            indices=wp.array(np.asarray(all_indices, dtype=np.int32), dtype=wp.int32),
            velocities=wp.array(np.asarray(all_velocities, dtype=np.float32), dtype=wp.vec3),
        )
        self.add_mesh(combined_mesh)


    def _quat_xyzw_to_rotmat(self, q_xyzw: Union[np.ndarray, List[float]]) -> np.ndarray:
        """
        Convert quaternion (x,y,z,w) to rotation matrix (3x3).
        Matches scipy Rotation.as_quat() convention.
        """
        q = np.array(q_xyzw, dtype=np.float32).reshape(4)
        x, y, z, w = q

        # normalize for safety
        n = np.sqrt(x*x + y*y + z*z + w*w) + 1e-8
        x, y, z, w = x/n, y/n, z/n, w/n

        xx, yy, zz = x*x, y*y, z*z
        xy, xz, yz = x*y, x*z, y*z
        wx, wy, wz = w*x, w*y, w*z

        R = np.array([
            [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
            [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
            [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)]
        ], dtype=np.float32)
        return R

    def sample_gate_pose_from_ranges(
        self,
        default_window_location: Union[np.ndarray, List[float]],
        default_window_roll_deg: float,
        window_location_center_range: Optional[Union[np.ndarray, List[List[float]], List[float]]] = None,
        window_orientation_roll_range_deg: Optional[Union[np.ndarray, List[float]]] = None,
    ):
        """
        Sample gate center and roll (about X axis only) from config ranges.

        Args:
            default_window_location: fallback center [x, y, z]
            default_window_roll_deg: fallback roll angle in degrees
            window_location_center_range:
                - [[xmin, xmax], [ymin, ymax], [zmin, zmax]] (recommended), or
                - [xmin, xmax, ymin, ymax, zmin, zmax]
            window_orientation_roll_range_deg: [roll_min_deg, roll_max_deg]
        """
        center = np.array(default_window_location, dtype=np.float32).reshape(3)

        if window_location_center_range is not None:
            center_range = np.array(window_location_center_range, dtype=np.float32)
            if center_range.shape == (3, 2):
                mins = center_range[:, 0]
                maxs = center_range[:, 1]
            elif center_range.shape == (6,):
                mins = center_range[[0, 2, 4]]
                maxs = center_range[[1, 3, 5]]
            else:
                raise ValueError(
                    "window_location_center_range must be shape (3,2) or (6,), got "
                    f"{center_range.shape}"
                )
            if np.any(maxs < mins):
                raise ValueError(
                    f"Invalid window_location_center_range: min must <= max, got mins={mins}, maxs={maxs}"
                )
            center = np.random.uniform(mins, maxs).astype(np.float32)

        roll_deg = float(default_window_roll_deg)
        if window_orientation_roll_range_deg is not None:
            roll_range = np.array(window_orientation_roll_range_deg, dtype=np.float32).reshape(-1)
            if roll_range.shape[0] != 2:
                raise ValueError(
                    "window_orientation_roll_range_deg must be [min_deg, max_deg], "
                    f"got shape={roll_range.shape}"
                )
            roll_min_deg = float(roll_range[0])
            roll_max_deg = float(roll_range[1])
            if roll_max_deg < roll_min_deg:
                raise ValueError(
                    "Invalid window_orientation_roll_range_deg: min_deg must <= max_deg, got "
                    f"[{roll_min_deg}, {roll_max_deg}]"
                )
            roll_deg = float(np.random.uniform(roll_min_deg, roll_max_deg))

        return center, roll_deg


    def add_gate_from_training(
        self,
        window_location,
        window_orientation_xyzw,
        hole_size,
        bar_size: float = 0.20,
        frame_outer_size: Optional[Union[np.ndarray, List[float]]] = None,
    ) -> None:
        """
        Gate plane: constant x = window_location[0] (opening in Y-Z plane).
        Roll about X is allowed (rotates inside the Y-Z plane).
        If frame_outer_size=[outer_w, outer_h] is provided, the gate is built
        as a large rectangular wall frame with inner hole size=hole_size.
        """
        c = np.array(window_location, dtype=np.float32).reshape(3)
        w, h = float(hole_size[0]), float(hole_size[1])
        bar = float(bar_size)

        R = self._quat_xyzw_to_rotmat(window_orientation_xyzw)  # (3,3)

        # local axes in world:
        ex_n = R[:, 0]   # gate normal (thickness direction)
        ey_r = R[:, 1]   # gate "right" inside plane
        ez_u = R[:, 2]   # gate "up" inside plane

        # thickness along normal (local x)
        half_depth = bar * 0.5

        # half extents for bars in *local* frame (x=depth, y=right, z=up)
        # optional large outer frame with fixed inner hole (w, h)
        if frame_outer_size is not None:
            outer_w = float(frame_outer_size[0])
            outer_h = float(frame_outer_size[1])
            if outer_w <= w or outer_h <= h:
                raise ValueError(
                    f"frame_outer_size must be larger than hole_size, got "
                    f"hole=({w}, {h}), outer=({outer_w}, {outer_h})"
                )

            # side wall thickness within plane
            side_thick_y = 0.5 * (outer_w - w)
            side_thick_z = 0.5 * (outer_h - h)

            # vertical bars: thick in y, full outer height in z
            v_half_x = half_depth
            v_half_y = 0.5 * side_thick_y
            v_half_z = 0.5 * outer_h

            # horizontal bars: full outer width in y, thick in z
            h_half_x = half_depth
            h_half_y = 0.5 * outer_w
            h_half_z = 0.5 * side_thick_z

            # place bars so inner faces stay at hole boundary
            dy = (w * 0.5) + v_half_y
            dz = (h * 0.5) + h_half_z
        else:
            # default behavior: frame thickness controlled by bar_size
            v_half_x = half_depth
            v_half_y = bar * 0.5
            v_half_z = (h * 0.5) + bar

            h_half_x = half_depth
            h_half_y = (w * 0.5) + bar
            h_half_z = bar * 0.5

            dy = (w * 0.5) + (bar * 0.5)
            dz = (h * 0.5) + (bar * 0.5)

        left_center  = c - dy * ey_r
        right_center = c + dy * ey_r
        top_center   = c + dz * ez_u
        bot_center   = c - dz * ez_u

        v_bar_local = self.create_box_mesh(size_x=v_half_x, size_y=v_half_y, size_z=v_half_z, flip_normals=False)
        h_bar_local = self.create_box_mesh(size_x=h_half_x, size_y=h_half_y, size_z=h_half_z, flip_normals=False)

        bars = [
            ("gate_left",  v_bar_local, left_center,  (v_half_x, v_half_y, v_half_z)),
            ("gate_right", v_bar_local, right_center, (v_half_x, v_half_y, v_half_z)),
            ("gate_top",   h_bar_local, top_center,   (h_half_x, h_half_y, h_half_z)),
            ("gate_bot",   h_bar_local, bot_center,   (h_half_x, h_half_y, h_half_z)),
        ]

        all_points, all_indices, all_velocities = [], [], []
        vertex_count = 0

        for name, mesh_local, cc, half_ext in bars:
            mesh_world = self._transform_box_mesh(mesh_local, R, cc)

            pts = mesh_world.points.numpy()
            idx = mesh_world.indices.numpy()
            vel = mesh_world.velocities.numpy()

            all_points.extend(pts)
            all_indices.extend(idx + vertex_count)
            all_velocities.extend(vel)
            vertex_count += pts.shape[0]

            hx, hy, hz = half_ext
            radius = float(np.sqrt(hx*hx + hy*hy + hz*hz))
            self.objects.append({
                "position": cc.astype(np.float32),
                "radius": radius,
                "type": "box",
                "name": name,
                "half_extents": np.array([hx, hy, hz], dtype=np.float32),
                "rotation_matrix": R.copy(),
                "gate_center": c.copy(),
            })

        combined_mesh = wp.Mesh(
            points=wp.array(np.asarray(all_points, dtype=np.float32), dtype=wp.vec3),
            indices=wp.array(np.asarray(all_indices, dtype=np.int32), dtype=wp.int32),
            velocities=wp.array(np.asarray(all_velocities, dtype=np.float32), dtype=wp.vec3),
        )
        self.add_mesh(combined_mesh)

    def add_gate_batch_from_training(
        self,
        window_locations,
        window_orientations_xyzw,
        hole_size,
        bar_size: float = 0.20,
        frame_outer_size: Optional[Union[np.ndarray, List[float]]] = None,
    ) -> None:
        """
        Add one independent gate mesh per batch environment.
        window_locations: (B,3), window_orientations_xyzw: (B,4)
        """
        locs = np.array(window_locations, dtype=np.float32)
        quats = np.array(window_orientations_xyzw, dtype=np.float32)

        if locs.shape != (self.batch_size, 3):
            raise ValueError(
                f"window_locations must have shape ({self.batch_size}, 3), got {locs.shape}"
            )
        if quats.shape != (self.batch_size, 4):
            raise ValueError(
                f"window_orientations_xyzw must have shape ({self.batch_size}, 4), got {quats.shape}"
            )

        for env_id in range(self.batch_size):
            self.add_gate_from_training(
                window_location=locs[env_id],
                window_orientation_xyzw=quats[env_id],
                hole_size=hole_size,
                bar_size=bar_size,
                frame_outer_size=frame_outer_size,
            )



    # -------------------- room / obstacles (same behavior as before) --------------------
    def random_pos(self):
        x = np.random.uniform(-self.room_size * 0.5, self.room_size * 0.5)
        y = np.random.uniform(-self.room_size * 0.5, self.room_size * 0.5)
        z = np.random.uniform(1, self.room_size)
        return np.array([x, y, z], dtype=np.float32)

    def setup_room(self, room_size=10.0, num_objects=2, rand_obs=True,
                   obs_size=1.0, obs_type=1.0, obs_dim=2.0):
        """
        Build a room + obstacles into ONE combined mesh and add it.
        NOTE: This clears objects+meshes then rebuilds.
        """
        self.room_size = room_size

        # clear previous
        self.objects.clear()
        self.meshes.clear()
        self._update_mesh_ids()

        all_points = []
        all_indices = []
        all_velocities = []
        vertex_count = 0

        # room (big inverted box)
        room = self.create_box_mesh(size=100000, flip_normals=True)
        room_points = room.points.numpy()
        room_indices = room.indices.numpy()
        room_velocities = room.velocities.numpy()

        all_points.extend(room_points)
        all_indices.extend(room_indices)
        all_velocities.extend(room_velocities)
        vertex_count += len(room_points)

        for _ in range(num_objects):
            if obs_type == 3.0:
                if np.random.rand() > 0.5:
                    size = np.random.uniform(0.1, obs_size)
                    if obs_dim == 2.0:
                        obj = self.create_box_mesh(size=size, size_z=room_size, flip_normals=True)
                    else:
                        obj = self.create_box_mesh(size=size, flip_normals=True)
                    radius = size
                    obj_type_s = "box"
                else:
                    radius = np.random.uniform(0.1, obs_size)
                    obj = self.create_sphere_mesh(radius=radius, flip_normals=True)
                    obj_type_s = "sphere"

            elif obs_type == 2.0:
                radius = np.random.uniform(0.1, obs_size)
                obj = self.create_sphere_mesh(radius=radius, flip_normals=True)
                obj_type_s = "sphere"

            else:  # obs_type == 1.0
                size = np.random.uniform(0.5, obs_size) / 2.0
                if obs_dim == 2.0:
                    obj = self.create_box_mesh(size=size, size_z=100000, flip_normals=True)
                else:
                    obj = self.create_box_mesh(size=size, flip_normals=True)
                radius = float(np.sqrt(2 * (size * size)))
                obj_type_s = "box"

            pos = self.random_pos() if rand_obs else (self.random_pos() * 0.0)

            obj_points = obj.points.numpy() + pos
            obj_indices = obj.indices.numpy() + vertex_count
            obj_velocities = obj.velocities.numpy()

            self.objects.append({
                "position": pos.astype(np.float32),
                "radius": radius,
                "type": obj_type_s,
            })

            all_points.extend(obj_points)
            all_indices.extend(obj_indices)
            all_velocities.extend(obj_velocities)
            vertex_count += len(obj_points)

        combined_mesh = wp.Mesh(
            points=wp.array(np.asarray(all_points, dtype=np.float32), dtype=wp.vec3),
            indices=wp.array(np.asarray(all_indices, dtype=np.int32), dtype=wp.int32),
            velocities=wp.array(np.asarray(all_velocities, dtype=np.float32), dtype=wp.vec3),
        )
        self.add_mesh(combined_mesh)

    # -------------------- mesh management --------------------
    def add_mesh(self, mesh: wp.Mesh) -> None:
        self.meshes.append(mesh)
        self._update_mesh_ids()

    def remove_mesh(self, mesh: wp.Mesh) -> None:
        if mesh in self.meshes:
            self.meshes.remove(mesh)
            self._update_mesh_ids()

    def _update_mesh_ids(self) -> None:
        if self.meshes:
            self.mesh_ids = wp.array([m.id for m in self.meshes], dtype=wp.uint64)
        else:
            self.mesh_ids = None

    # -------------------- sampling (the one your training needs) --------------------
    def sample_gate_traversal_initial_tests_start_points(
        self,
        window_location,
        end_goal,
        n_points=100,
        min_dist=0.5
    ):
        """
        Match your training script call.
        Sample initial drone positions "before the gate".
        """
        window_location = np.array(window_location, dtype=np.float32).reshape(3)
        end_goal = np.array(end_goal, dtype=np.float32).reshape(3)

        bounds = [
            [window_location[0] - 7.0, window_location[1] - 1.5, window_location[2] - 1.5],
            [window_location[0] - 5.0, window_location[1] + 1.5, window_location[2] + 1.5],
        ]
        xmin, ymin, zmin = bounds[0]
        xmax, ymax, zmax = bounds[1]

        pts = []
        while len(pts) < n_points:
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
            z = np.random.uniform(zmin, zmax)
            pts.append([x, y, z])

        return np.array(pts, dtype=np.float32)

    # -------------------- camera / depth (make it actually usable) --------------------
    def add_additional_rotation(self, additional_rotation: torch.Tensor):
        """
        additional_rotation: (batch, 4) xyzw
        """
        self.additional_rotation = additional_rotation.to(device=self.device, dtype=torch.float32)

    def add_camera(self, camera_id: str, config: Dict,
                   positions: torch.Tensor, orientations: torch.Tensor) -> None:
        """
        Create WarpCam and bind depth tensor. Camera uses current mesh_ids (must exist).
        positions: (batch, 3)
        orientations: (batch, 4) xyzw
        """
        if self.mesh_ids is None:
            raise ValueError("No meshes in scene, cannot add camera. Call setup_room/add_gate first.")
        # If the scene holds exactly one mesh per environment, render in true per-env mode:
        # num_envs=B, num_sensors=1.
        per_env_mesh_mode = len(self.meshes) == self.batch_size

        if per_env_mesh_mode:
            camera_config = dict(config)
            camera_config["num_sensors"] = 1
            self.depth_image = torch.zeros(
                (self.batch_size, 1, config['height'], config['width']),
                device=self.device,
                requires_grad=False
            )

            camera = WarpCam(
                num_envs=self.batch_size,
                config=CameraConfig(camera_config),
                mesh_ids_array=self.mesh_ids,
                additional_rotation=self.additional_rotation,
                device=self.device,
            )
            camera.set_pose_tensor(
                positions.reshape(self.batch_size, 1, 3),
                orientations.reshape(self.batch_size, 1, 4)
            )
            self.camera_modes[camera_id] = "per_env"
        else:
            # Legacy shared scene mode: one scene, many sensors.
            self.depth_image = torch.zeros(
                (1, self.batch_size, config['height'], config['width']),
                device=self.device,
                requires_grad=False
            )

            camera = WarpCam(
                num_envs=1,
                config=CameraConfig(config),
                mesh_ids_array=self.mesh_ids,
                additional_rotation=self.additional_rotation,
                device=self.device,
            )
            camera.set_pose_tensor(
                positions.reshape(1, self.batch_size, 3),
                orientations.reshape(1, self.batch_size, 4)
            )
            self.camera_modes[camera_id] = "shared"

        camera.set_image_tensors(self.depth_image)
        self.cameras[camera_id] = camera

    def set_camera_pose_tensor(self, camera_id: str,
                               positions: torch.Tensor,
                               orientations: torch.Tensor) -> None:
        """
        positions: (batch, 3)
        orientations: (batch, 4)
        """
        if camera_id not in self.cameras:
            raise ValueError(f"Camera {camera_id} not found")
        mode = self.camera_modes.get(camera_id, "shared")
        if mode == "per_env":
            self.cameras[camera_id].set_pose_tensor(
                positions.reshape(self.batch_size, 1, 3),
                orientations.reshape(self.batch_size, 1, 4)
            )
        else:
            self.cameras[camera_id].set_pose_tensor(
                positions.reshape(1, self.batch_size, 3),
                orientations.reshape(1, self.batch_size, 4)
            )

    def capture_depth(self, camera_id: str) -> torch.Tensor:
        if camera_id not in self.cameras:
            raise ValueError(f"Camera {camera_id} not found")
        return self.cameras[camera_id].capture()
