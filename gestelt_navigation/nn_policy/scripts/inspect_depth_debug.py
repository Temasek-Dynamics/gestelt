#!/usr/bin/env python3
"""Inspect the saved policy depth frames (nn_policy/depth_debug/<run>/*.npy).

The .npy files hold exactly what the vision policy sees: the depth AFTER
normalize + invert + 64x64 resize, i.e. values in ~[0, 1] where

    value = 1 - depth_m / max_range        (invert_depth=True)

so 1.0 = nearest, 0.0 = far-clip. This script converts back to metres
(depth_m = (1 - value) * max_range), reports per-frame stats, localizes the
nearest object (bounding box / centroid), and can print a coarse ASCII view.

Usage:
    python inspect_depth_debug.py depth_debug/20260718-104917
    python inspect_depth_debug.py depth_debug/20260718-104917 --ascii        # view every frame
    python inspect_depth_debug.py depth_debug/20260718-104917/depth_000019.npy --ascii
    python inspect_depth_debug.py <run> --max-range 20 --near-thresh 15
"""

import argparse
import glob
import os
import numpy as np


def resolve_path(path):
    """Find the run folder / .npy whether the user gives a full path, a path
    relative to nn_policy (e.g. depth_debug/<run>), or just a run name.
    The script lives in nn_policy/scripts/, so the package root is its parent."""
    pkg_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # nn_policy/
    candidates = [
        path,                                              # as given (cwd-relative / absolute)
        os.path.join(pkg_root, path),                      # relative to nn_policy/
        os.path.join(pkg_root, "depth_debug", path),       # just a run name
        os.path.join(pkg_root, "depth_debug", os.path.basename(path.rstrip("/"))),
    ]
    for c in candidates:
        if os.path.isdir(c) or (os.path.isfile(c) and c.endswith(".npy")):
            return c
    return path  # fall through so the caller reports a clear error


def to_metres(value, max_range):
    """policy input [0,1] (1=near) -> depth in metres."""
    return (1.0 - np.clip(value, 0.0, 1.0)) * max_range


def object_stats(depth_m, near_thresh):
    """Locate pixels nearer than near_thresh (the 'object'). Returns a dict or None."""
    mask = depth_m < near_thresh
    n = int(mask.sum())
    if n == 0:
        return None
    ys, xs = np.where(mask)
    return {
        "n": n,
        "frac": n / depth_m.size,
        "rows": (int(ys.min()), int(ys.max())),
        "cols": (int(xs.min()), int(xs.max())),
        "centroid": (float(ys.mean()), float(xs.mean())),
        "min_m": float(depth_m[mask].min()),
        "mean_m": float(depth_m[mask].mean()),
    }


def ascii_view(depth_m, max_range, target_rows=28):
    """Coarse render: near = bright (@), far = blank. Auto-downsamples so both
    64x64 and 360x640 frames fit the console."""
    step = max(1, round(depth_m.shape[0] / target_rows))
    ds = depth_m[::step, ::step]
    chars = np.array(list(" .:-=+*#%@"))
    idx = np.clip((max_range - ds) / max_range * (len(chars) - 1), 0, len(chars) - 1).astype(int)
    return "\n".join("".join(chars[row]) for row in idx)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="a depth_debug run folder, or a single .npy file")
    ap.add_argument("--max-range", type=float, default=20.0, help="far-clip in metres (config max_range)")
    ap.add_argument("--near-thresh", type=float, default=15.0,
                    help="pixels nearer than this [m] count as the object")
    ap.add_argument("--ascii", action="store_true", help="print a coarse depth view per frame")
    ap.add_argument("--frame", type=int, default=None,
                    help="only analyze this 1-based frame index (folder mode)")
    ap.add_argument("--raw", action="store_true",
                    help="inspect the raw full-res frames (depth_*_raw.npy, already in metres) "
                         "instead of the processed 64x64 policy input")
    args = ap.parse_args()

    path = resolve_path(args.path)
    if os.path.isdir(path):
        if args.raw:
            files = sorted(glob.glob(os.path.join(path, "*_raw.npy")))
        else:  # processed only: exclude the *_raw.npy frames
            files = sorted(f for f in glob.glob(os.path.join(path, "*.npy"))
                           if not f.endswith("_raw.npy"))
    elif os.path.isfile(path) and path.endswith(".npy"):
        files = [path]
    else:
        raise SystemExit(f"Not a run folder or .npy file: {args.path}")
    if not files:
        which = "raw (depth_*_raw.npy)" if args.raw else "processed (depth_*.npy)"
        raise SystemExit(f"No {which} frames found in {path}")
    if args.frame is not None:
        tag = f"{args.frame:06d}_raw.npy" if args.raw else f"{args.frame:06d}.npy"
        files = [f for f in files if f.endswith(tag)] or files[args.frame - 1:args.frame]

    MR = args.max_range
    # raw frames are already depth in metres; processed frames are the inverted
    # policy input in [0,1] (1=near) and must be converted back.
    is_raw = args.raw or (len(files) == 1 and files[0].endswith("_raw.npy"))
    kind = "raw depth in metres" if is_raw else "policy input (1=near, 0=far)"
    print(f"{len(files)} frame(s) | max_range={MR:.0f} m | value = {kind}\n")
    print(f"{'frame':<24}{'nearest_m':>10}{'med_m':>8}{'obj_px':>8}{'far%':>7}  object bbox / centroid(r,c)")

    nearest_all = []
    for f in files:
        v = np.load(f)
        depth_m = np.clip(v, 0.0, MR) if is_raw else to_metres(v, MR)
        med = float(np.median(depth_m))
        far = float((depth_m > 0.98 * MR).mean()) * 100.0   # fraction at the far-clip
        obj = object_stats(depth_m, args.near_thresh)
        if obj:
            nearest_all.append(obj["min_m"])
            loc = (f"rows {obj['rows'][0]}-{obj['rows'][1]} cols {obj['cols'][0]}-{obj['cols'][1]}  "
                   f"c=({obj['centroid'][0]:.0f},{obj['centroid'][1]:.0f})")
            near_str = f"{obj['min_m']:10.2f}"
            obj_px = obj["n"]
        else:
            loc = "(no object within near-thresh)"
            near_str = f"{'--':>10}"
            obj_px = 0
        print(f"{os.path.basename(f):<24}{near_str}{med:8.2f}{obj_px:8d}{far:7.1f}  {loc}")
        if args.ascii:
            print(ascii_view(depth_m, MR))
            print()

    if nearest_all:
        print(f"\nnearest object across {len(nearest_all)} frame(s): "
              f"min={min(nearest_all):.2f} m, mean={np.mean(nearest_all):.2f} m")
    else:
        print("\nno object found within --near-thresh in any frame (scene is all far-clip)")


if __name__ == "__main__":
    main()
