#!/usr/bin/env python3
"""
Standalone inference speed benchmark.

Usage:
    python3 benchmark_inference.py                      # uses policy_file defined below
    python3 benchmark_inference.py <policy_file>        # e.g. 20260513-185035
    python3 benchmark_inference.py <policy_file> --n 2000
"""
import sys
import os
import time
import argparse
import yaml
import numpy as np
import torch
import rospkg

sys.path.insert(0, os.path.dirname(__file__))
from modules.policy_simple_nwu_global_gate_traversal import TrackVelGate, TrackVelGRU

DEFAULT_POLICY_FILE = "20260513-185035"
N_WARMUP = 100
N_RUNS   = 1000


def load_policy(policy_file):
    rospack = rospkg.RosPack()
    base = os.path.join(rospack.get_path('nn_policy'), "logs/gate_traversal", policy_file)
    config_path = os.path.join(base, "training_config.yaml")
    policy_path = os.path.join(base, "policy.pth")

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)

    use_gru             = cfg.get("use_gru", False)
    include_actions     = cfg.get("gru_include_prev_action", False)
    gru_action_extra    = 4 if (use_gru and include_actions) else 0
    input_dims          = 16 + gru_action_extra

    if use_gru:
        policy = TrackVelGRU(input_dim=input_dims)
    else:
        policy = TrackVelGate(input_dim=16)

    policy.load_state_dict(torch.load(policy_path, map_location='cpu'))
    policy.eval()
    return policy, use_gru, cfg


def make_dummy_input(use_gru, include_actions):
    # matches evaluate_() with pc=True:
    # x = cat(diff_pos(3), pos(3), att(4), qd(6)) = (1,16)
    x = torch.randn(1, 16)
    h = None
    prev_action = torch.zeros(1, 4) if include_actions else None
    return x, h, prev_action


def run_benchmark(policy, use_gru, include_actions, n_warmup, n_runs):
    x, h, _ = make_dummy_input(use_gru, include_actions)

    print(f"Warming up ({n_warmup} iters)...")
    with torch.no_grad():
        for _ in range(n_warmup):
            if use_gru:
                _, h = policy(x, h)
            else:
                policy(x)

    print(f"Benchmarking ({n_runs} iters)...")
    times = []
    h = None
    with torch.no_grad():
        for _ in range(n_runs):
            t0 = time.perf_counter()
            if use_gru:
                _, h = policy(x, h)
            else:
                policy(x)
            times.append((time.perf_counter() - t0) * 1e3)  # ms

    times = np.array(times)
    print(f"\n{'='*40}")
    print(f"  n_runs   : {n_runs}")
    print(f"  mean     : {times.mean():.4f} ms")
    print(f"  std      : {times.std():.4f} ms")
    print(f"  min      : {times.min():.4f} ms")
    print(f"  max      : {times.max():.4f} ms")
    print(f"  p95      : {np.percentile(times, 95):.4f} ms")
    print(f"  p99      : {np.percentile(times, 99):.4f} ms")
    print(f"{'='*40}")
    print(f"  Target timestep : 10.00 ms  ({'OK' if times.mean() < 10.0 else 'SLOW'})")
    print(f"{'='*40}\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("policy_file", nargs="?", default=DEFAULT_POLICY_FILE)
    parser.add_argument("--n", type=int, default=N_RUNS)
    args = parser.parse_args()

    print(f"Policy: {args.policy_file}")
    policy, use_gru, cfg = load_policy(args.policy_file)
    include_actions = cfg.get("gru_include_prev_action", False)
    print(f"use_gru={use_gru}  include_actions={include_actions}")
    print(policy)

    run_benchmark(policy, use_gru, include_actions, N_WARMUP, args.n)
