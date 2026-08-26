#!/usr/bin/env python3
"""ROS node: virtual depth camera, substituting for a real one on hardware.

Subscribes to the drone's live pose in the same frame the gate geometry is
defined in. Prefers /drone0/mavros/vision_pose/pose (e.g. mocap) if it's
actually publishing, falling back to /drone0/mavros/local_position/pose
otherwise -- same detection the vision policy scripts use for gate metrics.
Renders a synthetic depth image with GateDepthCamera (render_depth_warp.py,
same SceneManager/gate builder/mount quaternion the policy was trained
against), and republishes it mono8-encoded on the topic + scale the vision
policy scripts expect:

    depth_m = (pixel / 255) * depth_cam_far      (see depthCb in
    policy_vel_combined_gate_traversal_vision_new_binary.py)

so this can be pointed at the same ~depth_topic a policy_vel_combined_*
vision script subscribes to, letting you closed-loop test the vision policy
on real hardware without an actual depth camera.

Gate position/orientation default to the checkpoint's own training config
(window_location_center_range midpoint / window_orientation), overridable
via ~gate_pos / ~gate_roll_deg for a physical gate placed elsewhere.
"""
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from render_depth_warp import (
    GateDepthCamera, load_config, camera_config_from, gate_config_from,
    gate_quat_from_roll, mount_quat_pitched,
)


def gate_pose_from_config(config_params):
    """Fixed gate pose for this run: midpoint of the training sampling range
    (or the plain window_location for older configs), matching how the
    policy_vel_combined_*_vision scripts pick window_position for display."""
    if "window_location_center_range" in config_params:
        _range = config_params["window_location_center_range"]
        gate_pos = [(_lo + _hi) / 2.0 for _lo, _hi in _range]
    else:
        gate_pos = config_params["window_location"]
    # config_params["window_orientation"] is stale for this checkpoint family (0.0) --
    # policy_vel_combined_gate_traversal_vision_new(_binary).py both ignore it and
    # hardcode 40.0 (see their __main__, "for now hardcoding this"). Match that here
    # too, rather than the config value, so the rendered gate matches what the policy
    # scripts actually assume; override with ~gate_roll_deg if that ever changes.
    gate_roll_deg = float(config_params["window_orientation"])

    return np.asarray(gate_pos, dtype=np.float32), gate_roll_deg


class VirtualDepthCameraNode:
    def __init__(self):
        # Default matches the checkpoint policy_vel_combined_gate_traversal_vision_new(_binary).py
        # currently point at (policy_file = "20260724-101933").
        cfg = load_config(rospy.get_param("~config", "20260724-101933"))

        gate_pos_cfg, gate_roll_cfg = gate_pose_from_config(vars(cfg))
        #TODO:overriding
        gate_roll_cfg = 60.0
        gate_pos = np.asarray(rospy.get_param("~gate_pos", gate_pos_cfg.tolist()), dtype=np.float32)
        gate_roll_deg = float(rospy.get_param("~gate_roll_deg", gate_roll_cfg))
        # world -> sim, same convention as policy_vel_combined_gate_traversal_offset.py;
        # zero unless the physical drone flies from a different origin than the gate config.
        self.scene_offset = np.asarray(rospy.get_param("~scene_offset", [0.0, 0.0, 0.0]), dtype=np.float32)

        pitch_deg = float(rospy.get_param("~pitch_deg", 0.0))
        device = rospy.get_param("~device", "cuda:0")

        self.cam = GateDepthCamera(
            gate_centers=[gate_pos],
            gate_quats=gate_quat_from_roll(gate_roll_deg),
            camera_config=camera_config_from(cfg),
            gate=gate_config_from(cfg),
            mount_quat=mount_quat_pitched(pitch_deg),
            device=device,
        )
        # Kept independent from the checkpoint's max_range so it can be tuned to match
        # whatever depth_cam_far the *policy* script's depthCb was configured with.
        self.depth_cam_far = float(rospy.get_param("~depth_cam_far", self.cam.max_range))
        rospy.loginfo("[virtual_depth_camera] gate @ %s roll=%.1fdeg  camera %dx%d fov=%.1fdeg far=%.1fm",
                      np.round(gate_pos, 3).tolist(), gate_roll_deg,
                      self.cam.camera_config['width'], self.cam.camera_config['height'],
                      self.cam.camera_config['horizontal_fov_deg'], self.depth_cam_far)

        self.bridge = CvBridge()
        self.pos = None
        self.quat = None
        # Debug-only: render immediately from a fixed pose instead of waiting for a real
        # pose message, to isolate whether the render/publish path itself works.
        if rospy.get_param("~debug_assume_pose", False):
            self.pos = np.asarray(rospy.get_param("~debug_pos", [-1.5, 0.0, 1.5]), dtype=np.float32)
            self.quat = np.asarray(rospy.get_param("~debug_quat", [0.0, 0.0, 0.0, 1.0]), dtype=np.float32)
            rospy.logwarn("[virtual_depth_camera] DEBUG: assuming fixed pose %s quat %s",
                          self.pos.tolist(), self.quat.tolist())

        # Prefer /drone0/mavros/vision_pose/pose (e.g. mocap) if it's actually publishing,
        # matching the same detection used for gate metrics in the vision policy scripts.
        vision_pose_topic = rospy.get_param("~vision_pose_topic", "/drone0/mavros/vision_pose/pose")
        fallback_pose_topic = rospy.get_param("~fallback_pose_topic", "/drone0/mavros/local_position/pose")
        try:
            rospy.wait_for_message(vision_pose_topic, PoseStamped, timeout=2.0)
            pose_topic = vision_pose_topic
            rospy.loginfo("[virtual_depth_camera] Detected active %s - using it for pose", vision_pose_topic)
        except rospy.ROSException:
            pose_topic = fallback_pose_topic
            rospy.loginfo("[virtual_depth_camera] %s has no incoming messages - using %s for pose",
                          vision_pose_topic, fallback_pose_topic)

        depth_topic = rospy.get_param("~depth_topic", "/agent001/stereo_left_depth")
        self.frame_id = rospy.get_param("~frame_id", "camera_depth_optical_frame")
        rate = float(rospy.get_param("~rate", 90.0))

        self.pose_topic = pose_topic
        self.pose_sub_ = rospy.Subscriber(pose_topic, PoseStamped, self.poseCb, queue_size=5)
        self.depth_pub_ = rospy.Publisher(depth_topic, Image, queue_size=1)
        rospy.loginfo("[virtual_depth_camera] pose <- %s | depth -> %s @ %.1f Hz",
                      pose_topic, depth_topic, rate)

        self.timer_ = rospy.Timer(rospy.Duration(1.0 / rate), self.renderCb)

    def poseCb(self, msg):
        self.pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
                            dtype=np.float32) + self.scene_offset
        self.quat = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                              msg.pose.orientation.z, msg.pose.orientation.w], dtype=np.float32)

    def renderCb(self, event):
        if self.pos is None:
            rospy.logwarn_throttle(2.0, "[virtual_depth_camera] renderCb ticking but no pose received yet on %s",
                                   self.pose_topic)
            return
        depth = self.cam.render([self.pos], [self.quat])   # (1,H,W) metres; misses = NO_HIT_RAY_VAL (1000.0)
        d = depth[0].detach().cpu().numpy()
        d = np.clip(d, 0.0, self.depth_cam_far)
        img = np.round(d / self.depth_cam_far * 255.0).astype(np.uint8)

        msg = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        self.depth_pub_.publish(msg)
        # print(f"[virtual_depth_camera] published depth image @ {msg.header.stamp.to_sec():.3f} "
        #       f"frame_id={self.frame_id} pos={np.round(self.pos, 3).tolist()} "
        #       f"quat={np.round(self.quat, 3).tolist()} depth_cam_far={self.depth_cam_far:.2f}m")


if __name__ == "__main__":
    rospy.init_node("virtual_depth_camera_node")
    VirtualDepthCameraNode()
    rospy.spin()
