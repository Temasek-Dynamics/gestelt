#!/usr/bin/env python3
"""Offline GIF regeneration for a saved sim2real test run.

save_outputs() in the *_sim2real*.py test scripts skips trajectory_xy.gif /
trajectory_xz.gif at record time (matplotlib animation saving is slow and
was cut to keep back-to-back cases fast). This reads each case's already-saved
trajectory_sim.npy / obstacles.npz / metadata.yaml and regenerates the exact
same GIFs after the fact, matching GazeboPolicyTester.save_xy_gif/save_xz_gif
in policy_vel_combined_cmdp_gru_test_deepset_sim2real_realdrone_obstacle_centered.py
pixel-for-pixel (same layout, colors, limits, animation).
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np
import yaml


def save_xy_gif(trajectory, start_pos_sim, target_pos_sim, obstacle_positions, obstacle_radii, save_path):
    fig, ax = plt.subplots(figsize=(7, 7))
    all_xy = np.vstack((trajectory[:, :2], start_pos_sim[:2], target_pos_sim[:2], obstacle_positions[:, :2]))
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
    for idx, (pos, radius) in enumerate(zip(obstacle_positions, obstacle_radii), start=1):
        if radius <= 0.0:
            continue
        ax.add_patch(plt.Circle((pos[0], pos[1]), radius, color="orange", alpha=0.28))
        ax.text(pos[0], pos[1], f"obs{idx}", fontsize=8)
    ax.plot(start_pos_sim[0], start_pos_sim[1], "g*", markersize=14, label="start")
    ax.plot(target_pos_sim[0], target_pos_sim[1], "r*", markersize=14, label="target")
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


def save_xz_gif(trajectory, start_pos_sim, target_pos_sim, obstacle_positions, obstacle_radii, save_path):
    fig, ax = plt.subplots(figsize=(8, 5))
    x_values = np.concatenate((trajectory[:, 0], [start_pos_sim[0], target_pos_sim[0]], obstacle_positions[:, 0]))
    z_values = np.concatenate((trajectory[:, 2], [start_pos_sim[2], target_pos_sim[2], 0.0, 3.0]))
    x_pad = max(0.5, 0.05 * max(np.ptp(x_values), 1.0))
    ax.set_xlim(np.min(x_values) - x_pad, np.max(x_values) + x_pad)
    ax.set_ylim(np.min(z_values) - 0.2, np.max(z_values) + 0.2)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Z (m)")
    ax.set_title("Gazebo Policy Test - XZ")
    ax.grid(True, alpha=0.3)
    z_min, z_max = ax.get_ylim()
    for idx, (pos, radius) in enumerate(zip(obstacle_positions, obstacle_radii), start=1):
        if radius <= 0.0:
            continue
        ax.add_patch(plt.Rectangle((pos[0] - radius, z_min), 2.0 * radius, z_max - z_min, color="orange", alpha=0.22))
        ax.text(pos[0], z_max, f"obs{idx}", fontsize=8, ha="center", va="top")
    ax.plot(start_pos_sim[0], start_pos_sim[2], "g*", markersize=14, label="start")
    ax.plot(target_pos_sim[0], target_pos_sim[2], "r*", markersize=14, label="target")
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


def plot_case(case_dir):
    with open(os.path.join(case_dir, "metadata.yaml"), "r") as f:
        meta = yaml.safe_load(f)
    start_pos_sim = np.asarray(meta["start_pos_sim"], dtype=np.float32)
    target_pos_sim = np.asarray(meta["target_pos_sim"], dtype=np.float32)
    trajectory = np.load(os.path.join(case_dir, "trajectory_sim.npy"))
    obstacles = np.load(os.path.join(case_dir, "obstacles.npz"))
    obstacle_positions = obstacles["positions"].astype(np.float32)
    obstacle_radii = obstacles["radii"].astype(np.float32)

    xy_path = os.path.join(case_dir, "trajectory_xy.gif")
    xz_path = os.path.join(case_dir, "trajectory_xz.gif")
    save_xy_gif(trajectory, start_pos_sim, target_pos_sim, obstacle_positions, obstacle_radii, xy_path)
    save_xz_gif(trajectory, start_pos_sim, target_pos_sim, obstacle_positions, obstacle_radii, xz_path)
    print(f"  saved {xy_path}")
    print(f"  saved {xz_path}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("run_dir", help="Output root of a completed test run, e.g. "
                                    "logs/vel_tracking/<policy>/realdrone_obstacle_centered_test_<timestamp>")
    args = ap.parse_args()

    case_dirs = []
    for layout_name in sorted(os.listdir(args.run_dir)):
        layout_dir = os.path.join(args.run_dir, layout_name)
        if not (layout_name.startswith("layout") and os.path.isdir(layout_dir)):
            continue
        for case_name in sorted(os.listdir(layout_dir)):
            case_dir = os.path.join(layout_dir, case_name)
            if case_name.startswith("case_") and os.path.isdir(case_dir):
                case_dirs.append(case_dir)
    if not case_dirs:
        raise RuntimeError(f"No layoutN/case_NNN found under {args.run_dir}")

    for case_dir in case_dirs:
        print(f"[{os.path.relpath(case_dir, args.run_dir)}]")
        plot_case(case_dir)
    print(f"Done: {len(case_dirs)} cases.")
