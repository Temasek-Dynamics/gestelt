# import nvtx
import warp as wp
import math

from warp_camera_kernels_ros import (
    DepthCameraWarpKernels,
)


class WarpCam:
    def __init__(self, num_envs, config, mesh_ids_array, additional_rotation, device="cuda:0"):
        self.cfg = config
        self.num_envs = num_envs
        self.num_sensors = self.cfg.num_sensors
        self.mesh_ids_array = mesh_ids_array
        # print(f"Camera initialized with {len(mesh_ids_array)} meshes: {mesh_ids_array.numpy()}")

        self.width = self.cfg.width
        self.height = self.cfg.height

        self.horizontal_fov = math.radians(self.cfg.horizontal_fov_deg)
        self.far_plane = self.cfg.max_range
        self.calculate_depth = self.cfg.calculate_depth
        self.device = device

        self.camera_position_array = None
        self.camera_orientation_array = None
        self.graph = None
    
        self.initialize_camera_matrices()

        self.additional_camera_rotation = additional_rotation
        if len(self.additional_camera_rotation.shape) == 2:
            if self.additional_camera_rotation.shape[0] == (self.num_envs * self.num_sensors):
                self.additional_camera_rotation = self.additional_camera_rotation.reshape(self.num_envs, self.num_sensors, 4)
            elif self.num_envs == 1 and self.additional_camera_rotation.shape[0] == self.num_sensors:
                self.additional_camera_rotation = self.additional_camera_rotation.reshape(1, self.num_sensors, 4)
            elif self.num_sensors == 1 and self.additional_camera_rotation.shape[0] == self.num_envs:
                self.additional_camera_rotation = self.additional_camera_rotation.reshape(self.num_envs, 1, 4)
            else:
                raise ValueError(
                    f"additional_rotation shape {tuple(self.additional_camera_rotation.shape)} "
                    f"is incompatible with num_envs={self.num_envs}, num_sensors={self.num_sensors}"
                )
        elif len(self.additional_camera_rotation.shape) == 3:
            expected_shape = (self.num_envs, self.num_sensors, 4)
            if tuple(self.additional_camera_rotation.shape) != expected_shape:
                raise ValueError(
                    f"additional_rotation shape {tuple(self.additional_camera_rotation.shape)} "
                    f"must be {expected_shape}"
                )
        else:
            raise ValueError(
                f"additional_rotation must be rank-2 or rank-3, got rank={len(self.additional_camera_rotation.shape)}"
            )

        
    def initialize_camera_matrices(self):
        # Calculate camera params
        W = self.width
        H = self.height
        (u_0, v_0) = (W / 2, H / 2)
        f = W / 2 * 1 / math.tan(self.horizontal_fov / 2)

        vertical_fov = 2 * math.atan(H / (2 * f))
        alpha_u = u_0 / math.tan(self.horizontal_fov / 2)
        alpha_v = v_0 / math.tan(vertical_fov / 2)

        # print(f"Camera resolution: {W}x{H}")
        # print(f"FOV: {math.degrees(self.horizontal_fov):.1f}° (h) x {math.degrees(vertical_fov):.1f}° (v)")
        # print(f"Far plane: {self.far_plane}")

        # simple pinhole model
        self.K = wp.mat44(
            alpha_u,
            0.0,
            u_0,
            0.0,
            0.0,
            alpha_v,
            v_0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        )
        self.K_inv = wp.inverse(self.K)

        self.c_x = int(u_0)
        self.c_y = int(v_0)

    def create_render_graph_pointcloud(self, debug=False):
        if not debug:
            wp.capture_begin(device=self.device)
        # with wp.ScopedTimer("render"):
        if self.cfg.segmentation_camera == True:
            wp.launch(
                kernel=DepthCameraWarpKernels.draw_optimized_kernel_pointcloud_segmentation,
                dim=(self.num_envs, self.num_sensors, self.width, self.height),
                inputs=[
                    self.mesh_ids_array,
                    self.camera_position_array,
                    self.camera_orientation_array,
                    self.K_inv,
                    self.far_plane,
                    self.pixels,
                    self.segmentation_pixels,
                    self.c_x,
                    self.c_y,
                    self.pointcloud_in_world_frame,
                ],
                device=self.device,
            )
        else:
            wp.launch(
                kernel=DepthCameraWarpKernels.draw_optimized_kernel_pointcloud,
                dim=(self.num_envs, self.num_sensors, self.width, self.height),
                inputs=[
                    self.mesh_ids_array,
                    self.camera_position_array,
                    self.camera_orientation_array,
                    self.K_inv,
                    self.far_plane,
                    self.pixels,
                    self.c_x,
                    self.c_y,
                    self.pointcloud_in_world_frame,
                ],
                device=self.device,
            )
        if not debug:
            self.graph = wp.capture_end(device=self.device)

    def create_render_graph_depth_range(self, debug=False):
        if not debug:
            wp.capture_begin(device=self.device)
        # with wp.ScopedTimer("render"):
        if self.cfg.segmentation_camera == True:
            wp.launch(
                kernel=DepthCameraWarpKernels.draw_optimized_kernel_depth_range_segmentation,
                dim=(self.num_envs, self.num_sensors, self.width, self.height),
                inputs=[
                    self.mesh_ids_array,
                    self.camera_position_array,
                    self.camera_orientation_array,
                    self.K_inv,
                    self.far_plane,
                    self.pixels,
                    self.segmentation_pixels,
                    self.c_x,
                    self.c_y,
                    self.calculate_depth,
                ],
                device=self.device,
            )
        else:
            wp.launch(
                kernel=DepthCameraWarpKernels.draw_optimized_kernel_depth_range,
                dim=(self.num_envs, self.num_sensors, self.width, self.height),
                inputs=[
                    self.mesh_ids_array,
                    self.camera_position_array,
                    self.camera_orientation_array,
                    self.additional_camera_rotation_array,
                    self.K_inv,
                    self.far_plane,
                    self.pixels,
                    self.c_x,
                    self.c_y,
                    self.calculate_depth,
                ],
                device=self.device,
            )
        if not debug:
            self.graph = wp.capture_end(device=self.device)

    def set_image_tensors(self, pixels, segmentation_pixels=None):
        # init buffers. None when uninitialized
        if self.cfg.return_pointcloud:
            self.pixels = wp.from_torch(pixels, dtype=wp.vec3)
            self.pointcloud_in_world_frame = self.cfg.pointcloud_in_world_frame
        else:
            self.pixels = wp.from_torch(pixels, dtype=wp.float32)

        if self.cfg.segmentation_camera == True:
            self.segmentation_pixels = wp.from_torch(segmentation_pixels, dtype=wp.int32)
        else:
            self.segmentation_pixels = segmentation_pixels

    def set_pose_tensor(self, positions, orientations):
        self.camera_position_array = wp.from_torch(positions, dtype=wp.vec3)
        self.camera_orientation_array = wp.from_torch(orientations, dtype=wp.quat)
        self.additional_camera_rotation_array = wp.from_torch(self.additional_camera_rotation, dtype=wp.quat)

    # @nvtx.annotate()
    def capture(self, debug=False):
        if self.cfg.return_pointcloud:
            self.create_render_graph_pointcloud(debug=debug)
        else:
            self.create_render_graph_depth_range(debug=debug)
        wp.capture_launch(self.graph)

        return wp.to_torch(self.pixels)
