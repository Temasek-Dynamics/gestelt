#!/usr/bin/env python3
"""
Convert warp_data pickle files from the old flat format:
    {"time_stamp": [], "position": [], ...}
to the new indexed format:
    {0: {"time_stamp": [], "position": [], ...}}

Usage:
    python3 convert_warp_data.py              # converts all runs in warp_data/
    python3 convert_warp_data.py 0 1 2        # converts specific run indices
    python3 convert_warp_data.py path/a.pkl   # converts a specific file
"""

import sys
import os
import pickle
import rospkg

KEYS = ["time_stamp", "position", "velocity", "rotation", "omega", "action"]


def find_warp_dir():
    rospack = rospkg.RosPack()
    return os.path.join(rospack.get_path('nn_policy'), "warp_data")


def find_data_path(arg, warp_dir):
    if os.path.isfile(arg):
        return arg
    return os.path.join(warp_dir, str(arg), "data.pkl")


def is_old_format(data):
    return isinstance(data, dict) and "position" in data


def convert(data):
    return {0: {k: data[k] for k in KEYS}}


def process(path):
    with open(path, "rb") as f:
        data = pickle.load(f)

    if not is_old_format(data):
        print(f"  Skipping (already new format): {path}")
        return

    converted = convert(data)
    with open(path, "wb") as f:
        pickle.dump(converted, f)
    print(f"  Converted: {path}")


def main():
    warp_dir = find_warp_dir()
    args = sys.argv[1:]

    if not args:
        # Convert all runs
        folders = sorted(
            [f for f in os.listdir(warp_dir) if os.path.isdir(os.path.join(warp_dir, f))],
            key=lambda x: int(x) if x.isdigit() else -1
        )
        paths = [os.path.join(warp_dir, f, "data.pkl") for f in folders]
    else:
        paths = [find_data_path(a, warp_dir) for a in args]

    for path in paths:
        if not os.path.isfile(path):
            print(f"  Not found, skipping: {path}")
            continue
        process(path)

    print("Done.")


if __name__ == "__main__":
    main()
