#!/usr/bin/env python3
"""
Plot all recorded gate-traversal trajectories from a warp_data pickle file.

Usage:
    python3 plot_warp_data.py                   # loads the latest run
    python3 plot_warp_data.py <run_index>        # loads warp_data/<index>/data.pkl
    python3 plot_warp_data.py <path/to/data.pkl> # loads a specific file
"""

import sys
import os
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import rospkg

from plotting_scripts import gate_geometry, quat_to_rotmat


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def find_data_path(arg=None):
    rospack = rospkg.RosPack()
    warp_dir = os.path.join(rospack.get_path('nn_policy'), "warp_data")

    if arg is None:
        # Latest run
        folders = sorted(
            [f for f in os.listdir(warp_dir) if os.path.isdir(os.path.join(warp_dir, f))],
            key=lambda x: int(x) if x.isdigit() else -1
        )
        if not folders:
            raise FileNotFoundError(f"No runs found in {warp_dir}")
        path = os.path.join(warp_dir, folders[-1], "data.pkl")
    elif os.path.isfile(arg):
        path = arg
    else:
        path = os.path.join(warp_dir, str(arg), "data.pkl")

    print(f"Loading: {path}")
    return path


def split_episodes(data):
    """Split the flat lists into per-episode lists using None as separator."""
    inner = data[0]
    episodes = []
    current = []

    for i, p in enumerate(inner["position"]):
        if p is None:
            if current:
                episodes.append(current)
                current = []
        else:
            current.append({
                "position": p,
                "rotation": inner["rotation"][i],
                "velocity": inner["velocity"][i],
                "omega":    inner["omega"][i],
                "action":   inner["action"][i],
                "time":     inner["time_stamp"][i],
            })

    if current:          # last episode if no trailing None
        episodes.append(current)

    return episodes


def episode_positions(ep):
    return np.array([s["position"] for s in ep])  # (T, 3)

def episode_rotations(ep):
    return np.array([s["rotation"] for s in ep])  # (T, 4)


# ---------------------------------------------------------------------------
# Plots
# ---------------------------------------------------------------------------

def plot_all(episodes):
    gate_world = gate_geometry(30)          # 30 deg tilt, same as the script
    n_ep = len(episodes)
    cmap = plt.cm.get_cmap("tab10", min(n_ep, 10))

    # ── 1. XY top-down ──────────────────────────────────────────────────────
    fig_xy, ax_xy = plt.subplots(figsize=(8, 6))
    ax_xy.set_title(f"All Trajectories — XY top-down  ({n_ep} episodes)")
    ax_xy.set_xlabel("X (m)")
    ax_xy.set_ylabel("Y (m)")
    ax_xy.set_aspect("equal")
    ax_xy.grid(True)

    # Gate footprint in XY
    ax_xy.plot(gate_world[:, 0], gate_world[:, 1], "k-", linewidth=2, label="gate")

    for i, ep in enumerate(episodes):
        pos = episode_positions(ep)
        color = cmap(i % 10)
        ax_xy.plot(pos[:, 0], pos[:, 1], color=color, linewidth=1, alpha=0.8,
                   label=f"ep {i}" if i < 10 else None)
        ax_xy.scatter(pos[0, 0],  pos[0, 1],  color=color, marker="o", s=20, zorder=5)
        ax_xy.scatter(pos[-1, 0], pos[-1, 1], color=color, marker="x", s=20, zorder=5)

    ax_xy.legend(loc="upper left", fontsize=7, ncol=2)

    # Compute shared Z limits for XZ and YZ plots
    all_pos = np.concatenate([episode_positions(ep) for ep in episodes], axis=0)
    pad = 0.3
    z_min = min(all_pos[:, 2].min(), gate_world[:, 2].min()) - pad
    z_max = max(all_pos[:, 2].max(), gate_world[:, 2].max()) + pad
    y_min = min(all_pos[:, 1].min(), gate_world[:, 1].min()) - pad
    y_max = max(all_pos[:, 1].max(), gate_world[:, 1].max()) + pad
    yz_range = max(y_max - y_min, z_max - z_min)
    y_mid = (y_min + y_max) / 2
    z_mid = (z_min + z_max) / 2
    shared_y = [y_mid - yz_range / 2, y_mid + yz_range / 2]
    shared_z = [z_mid - yz_range / 2, z_mid + yz_range / 2]

    # ── 2. XZ side view ─────────────────────────────────────────────────────
    fig_xz, ax_xz = plt.subplots(figsize=(8, 5))
    ax_xz.set_title(f"All Trajectories — XZ side view  ({n_ep} episodes)")
    ax_xz.set_xlabel("X (m)")
    ax_xz.set_ylabel("Z (m)")
    ax_xz.set_ylim(shared_z)
    ax_xz.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax_xz.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax_xz.xaxis.set_minor_locator(ticker.MultipleLocator(0.1))
    ax_xz.yaxis.set_minor_locator(ticker.MultipleLocator(0.1))
    ax_xz.grid(True, which="major", linewidth=0.8)
    ax_xz.grid(True, which="minor", linewidth=0.3, linestyle=":")

    # Gate footprint in XZ
    ax_xz.plot(gate_world[:, 0], gate_world[:, 2], "k-", linewidth=2, label="gate")

    for i, ep in enumerate(episodes):
        pos = episode_positions(ep)
        color = cmap(i % 10)
        ax_xz.plot(pos[:, 0], pos[:, 2], color=color, linewidth=1, alpha=0.8,
                   label=f"ep {i}" if i < 10 else None)
        ax_xz.scatter(pos[0, 0],  pos[0, 2],  color=color, marker="o", s=20, zorder=5)
        ax_xz.scatter(pos[-1, 0], pos[-1, 2], color=color, marker="x", s=20, zorder=5)

    ax_xz.legend(loc="upper left", fontsize=7, ncol=2)

    # ── 3. YZ front view ────────────────────────────────────────────────────
    fig_yz, ax_yz = plt.subplots(figsize=(6, 6))
    ax_yz.set_title(f"All Trajectories — YZ front view  ({n_ep} episodes)")
    ax_yz.set_xlabel("Y (m)")
    ax_yz.set_ylabel("Z (m)")
    ax_yz.set_xlim(shared_y)
    ax_yz.set_ylim(shared_z)
    ax_yz.set_aspect("equal")
    ax_yz.xaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax_yz.yaxis.set_major_locator(ticker.MultipleLocator(0.5))
    ax_yz.xaxis.set_minor_locator(ticker.MultipleLocator(0.1))
    ax_yz.yaxis.set_minor_locator(ticker.MultipleLocator(0.1))
    ax_yz.grid(True, which="major", linewidth=0.8)
    ax_yz.grid(True, which="minor", linewidth=0.3, linestyle=":")

    # Gate footprint in YZ
    ax_yz.plot(gate_world[:, 1], gate_world[:, 2], "k-", linewidth=2, label="gate")

    for i, ep in enumerate(episodes):
        pos = episode_positions(ep)
        color = cmap(i % 10)
        ax_yz.plot(pos[:, 1], pos[:, 2], color=color, linewidth=1, alpha=0.8,
                   label=f"ep {i}" if i < 10 else None)
        ax_yz.scatter(pos[0, 1],  pos[0, 2],  color=color, marker="o", s=20, zorder=5)
        ax_yz.scatter(pos[-1, 1], pos[-1, 2], color=color, marker="x", s=20, zorder=5)

    ax_yz.legend(loc="upper left", fontsize=7, ncol=2)

    # ── 4. 3D ───────────────────────────────────────────────────────────────
    fig_3d = plt.figure(figsize=(10, 7))
    ax_3d = fig_3d.add_subplot(111, projection="3d")
    ax_3d.set_title(f"All Trajectories — 3D  ({n_ep} episodes)")
    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Z (m)")

    ax_3d.plot(gate_world[:, 0], gate_world[:, 1], gate_world[:, 2],
               "k-", linewidth=2, label="gate")

    for i, ep in enumerate(episodes):
        pos = episode_positions(ep)
        color = cmap(i % 10)
        ax_3d.plot(pos[:, 0], pos[:, 1], pos[:, 2],
                   color=color, linewidth=1, alpha=0.8,
                   label=f"ep {i}" if i < 10 else None)
        ax_3d.scatter(*pos[0],  color=color, marker="o", s=20)
        ax_3d.scatter(*pos[-1], color=color, marker="x", s=20)

    ax_3d.legend(loc="upper left", fontsize=7, ncol=2)

    # ── 5. Summary stats ────────────────────────────────────────────────────
    print(f"\n{'='*45}")
    print(f"  Total episodes : {n_ep}")
    for i, ep in enumerate(episodes):
        pos = episode_positions(ep)
        print(f"  ep {i:3d}  steps={len(ep):4d}  "
              f"start=({pos[0,0]:.2f},{pos[0,1]:.2f},{pos[0,2]:.2f})  "
              f"end=({pos[-1,0]:.2f},{pos[-1,1]:.2f},{pos[-1,2]:.2f})")
    print(f"{'='*45}\n")

    plt.tight_layout()
    plt.show()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    path = find_data_path(arg)

    with open(path, "rb") as f:
        data = pickle.load(f)

    episodes = split_episodes(data)
    print(f"Found {len(episodes)} episodes.")

    if not episodes:
        print("No episode data to plot.")
        sys.exit(0)

    plot_all(episodes)
