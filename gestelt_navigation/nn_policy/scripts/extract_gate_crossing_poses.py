#!/usr/bin/env python3
"""
Extract the drone's pose at the moment it crosses x = X_THRESH, for every
run recorded in a single rosbag.

A "run" is one rising-edge crossing of x_thresh. After a crossing, detection
is re-armed only once x drops back below (x_thresh - reset_margin), so a
single noisy pass through the threshold isn't counted twice.

Usage:
    python3 extract_gate_crossing_poses.py <bag_file>
    python3 extract_gate_crossing_poses.py <bag_file> --x-thresh 1.5 --out poses.csv
"""

import argparse
import csv

import numpy as np
import rosbag

from plotting_scripts import compute_gate_metrics


def read_poses(bag_path, topic):
    times, positions, quats = [], [], []
    with rosbag.Bag(bag_path) as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            p = msg.pose.position
            q = msg.pose.orientation
            times.append(msg.header.stamp.to_sec())
            positions.append([p.x, p.y, p.z])
            quats.append([q.x, q.y, q.z, q.w])

    if not times:
        raise RuntimeError(f"No messages found on topic '{topic}'")

    return np.array(times), np.array(positions), np.array(quats)


def find_run_segments(positions, x_thresh, reset_margin):
    """Return a list of (seg_start, seg_end, crossing_idx) per run.

    crossing_idx is the rising-edge index where x[i-1] < x_thresh <= x[i].
    seg_start/seg_end bound that run so compute_gate_metrics' nearest-to-gate
    search stays local to this run instead of the whole bag. Re-armed (i.e.
    a new run starts) only once x drops back below x_thresh - reset_margin.
    """
    x = positions[:, 0]
    segments = []
    armed = True
    seg_start = 0
    crossing_idx = None

    for i in range(1, len(x)):
        if armed and x[i - 1] < x_thresh <= x[i]:
            crossing_idx = i
            armed = False
        elif not armed and x[i] < x_thresh - reset_margin:
            segments.append((seg_start, i, crossing_idx))
            seg_start = i
            crossing_idx = None
            armed = True

    if crossing_idx is not None:
        segments.append((seg_start, len(x) - 1, crossing_idx))

    return segments


def nearest_pose(times, positions, quats, i, x_thresh):
    """Pick whichever of samples i-1, i has x closest to x_thresh."""
    if abs(positions[i - 1, 0] - x_thresh) <= abs(positions[i, 0] - x_thresh):
        j = i - 1
    else:
        j = i

    return times[j], positions[j], quats[j]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Path to the rosbag file")
    parser.add_argument("--topic", default="/drone0/mavros/vision_pose/pose")
    parser.add_argument("--x-thresh", type=float, default=1.5)
    parser.add_argument("--reset-margin", type=float, default=0.5,
                        help="x must drop below x_thresh - margin before the next crossing counts as a new run")
    parser.add_argument("--gate-x", type=float, default=1.5)
    parser.add_argument("--gate-y", type=float, default=0.0)
    parser.add_argument("--gate-z", type=float, default=2.0)
    parser.add_argument("--gate-angle-deg", type=float, default=60.0)
    parser.add_argument("--out", default="gate_crossing_poses.csv")
    args = parser.parse_args()

    times, positions, quats = read_poses(args.bag, args.topic)
    segments = find_run_segments(positions, args.x_thresh, args.reset_margin)

    gate_center = np.array([args.gate_x, args.gate_y, args.gate_z])
    gate_angle_rad = np.radians(args.gate_angle_deg)
    window_quat = np.array([np.sin(gate_angle_rad / 2), 0.0, 0.0, np.cos(gate_angle_rad / 2)])

    lin_vel = np.gradient(positions, times, axis=0)
    vel6 = np.concatenate([np.zeros_like(lin_vel), lin_vel], axis=1)  # (T,6) [ang, lin]

    print(f"Read {len(times)} poses on '{args.topic}'")
    print(f"Found {len(segments)} run(s) crossing x={args.x_thresh}")

    rows = []
    for run_idx, (seg_start, seg_end, crossing_idx) in enumerate(segments):
        t, pos, quat = nearest_pose(times, positions, quats, crossing_idx, args.x_thresh)
        print(f"\n--- Run {run_idx} ---")
        print(f"  Nearest sample  t={t:.3f}  pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})  "
              f"quat=({quat[0]:.3f}, {quat[1]:.3f}, {quat[2]:.3f}, {quat[3]:.3f})")

        t_star, pos_at_gate, pos_err, vel_at_gate, vel_err, euler_at_gate, x_dot, z_dot = \
            compute_gate_metrics(positions[seg_start:seg_end + 1],
                                 vel6[seg_start:seg_end + 1],
                                 quats[seg_start:seg_end + 1],
                                 window_quat, gate_center=gate_center, target_vel=None)

        rows.append({
            "run": run_idx, "t": t,
            "x": pos[0], "y": pos[1], "z": pos[2],
            "qx": quat[0], "qy": quat[1], "qz": quat[2], "qw": quat[3],
            "pos_err": pos_err, "vel_err": vel_err,
            "roll": euler_at_gate[0], "pitch": euler_at_gate[1], "yaw": euler_at_gate[2],
            "x_dot": x_dot, "z_dot": z_dot,
        })

    fieldnames = ["run", "t", "x", "y", "z", "qx", "qy", "qz", "qw",
                  "pos_err", "vel_err", "roll", "pitch", "yaw", "x_dot", "z_dot"]
    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"\nSaved {len(rows)} poses to {args.out}")


if __name__ == "__main__":
    main()
