#!/usr/bin/env python3
"""
Combine multiple warp_data pickle files into one.

Usage:
    python3 combine_warp_data.py 0 1 2              # combine runs 0, 1, 2 (latest saved next)
    python3 combine_warp_data.py 0 1 -o merged.pkl  # specify output path
    python3 combine_warp_data.py path/a.pkl path/b.pkl
"""

import sys
import os
import pickle
import argparse
import shutil
import rospkg


KEYS = ["time_stamp", "position", "velocity", "rotation", "omega", "action"]


def find_data_path(arg, warp_dir):
    if os.path.isfile(arg):
        return arg
    return os.path.join(warp_dir, str(arg), "data.pkl")


def load(path):
    print(f"  Loading: {path}")
    with open(path, "rb") as f:
        return pickle.load(f)


def count_episodes(data):
    inner = data[0]
    return sum(1 for v in inner["position"] if v is None) + (
        1 if inner["position"] and inner["position"][-1] is not None else 0)


def combine(datasets):
    merged = {0: {k: [] for k in KEYS}}
    for data in datasets:
        for k in KEYS:
            merged[0][k].extend(data[0][k])
        # Add a None separator between files (unless the file already ends with one)
        if data[0]["position"] and data[0]["position"][-1] is not None:
            for k in KEYS:
                merged[0][k].append(None)
    return merged


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("runs", nargs="+", help="Run indices or .pkl file paths to combine")
    parser.add_argument("-o", "--output", default=None, help="Output .pkl path")
    args = parser.parse_args()

    rospack = rospkg.RosPack()
    warp_dir = os.path.join(rospack.get_path('nn_policy'), "warp_data")

    print("Loading files:")
    datasets = []
    config_src = None
    for r in args.runs:
        path = find_data_path(r, warp_dir)
        data = load(path)
        ep_count = count_episodes(data)
        print(f"    -> {ep_count} episodes, {len(data[0]['position'])} samples")
        datasets.append(data)
        if config_src is None:
            candidate = os.path.join(os.path.dirname(path), "training_config.yaml")
            if os.path.isfile(candidate):
                config_src = candidate

    merged = combine(datasets)
    total_ep = count_episodes(merged)
    total_samples = sum(1 for v in merged[0]["position"] if v is not None)
    print(f"\nMerged: {total_ep} episodes, {total_samples} samples total")

    if args.output:
        out_path = args.output
    else:
        # Save as next numbered run in warp_data
        folders = [f for f in os.listdir(warp_dir) if os.path.isdir(os.path.join(warp_dir, f))]
        next_idx = len(folders)
        out_dir = os.path.join(warp_dir, str(next_idx))
        os.makedirs(out_dir, exist_ok=True)
        out_path = os.path.join(out_dir, "data.pkl")

    with open(out_path, "wb") as f:
        pickle.dump(merged, f)
    print(f"Saved to: {out_path}")

    if config_src:
        config_dst = os.path.join(os.path.dirname(out_path), "training_config.yaml")
        shutil.copy2(config_src, config_dst)
        print(f"Config copied from: {config_src}")


if __name__ == "__main__":
    main()
