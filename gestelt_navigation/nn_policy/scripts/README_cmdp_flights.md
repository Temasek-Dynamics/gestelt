# CMDP obstacle-avoidance flights (Gazebo + real drone)

Replays the training-environment test sets (`logs/docs/primal_dual*`) on the
DeepSets-GRU CMDP policy, in Gazebo or on the real drone, and saves the data in
the same per-case format as the training-side tests so the two can be compared
offline.

Obstacles are **never placed physically** (Gazebo or real). They only exist as
the policy's obstacle-avoidance input, exactly like in training. The drone
flies in an empty room.

## Scripts

| Script | Use | Node name |
|---|---|---|
| `policy_vel_combined_cmdp_gru_test_deepset_sim2real.py` | Gazebo, all cases in `primal_dual2_blocked` | `cmdp_gru_policy_gazebo_test` |
| `policy_vel_combined_cmdp_gru_test_deepset_sim2real_realdrone.py` | Real drone, launch point pinned to a fixed spot | `cmdp_gru_policy_realdrone_test` |
| `policy_vel_combined_cmdp_gru_test_deepset_sim2real_realdrone_obstacle_centered.py` | Real drone, obstacle centroid pinned to the Vicon centre | `cmdp_gru_policy_realdrone_obstacle_centered_test` | - This is for the final flight test in the paper
| `plot_case_gifs.py` | Offline: regenerate per-case XY/XZ GIFs from a saved run | (not a ROS node) |

### What `_realdrone_obstacle_centered.py` does

Flies the `primal_dual2_difficult` cases on the real drone with the CMDP
DeepSets-GRU policy, with the **obstacle layout centred on the Vicon room
centre**:

1. For each case, a shift is computed so the mean position of its 4 obstacles
   is at (0, 0). Start and target get the same shift, so the drone flies the
   same relative path around the same obstacle layout as in training.
2. Obstacles are only fed to the policy as input (never placed physically);
   the drone flies in an empty room.
3. Per case: fly to the start, hand over to the policy, stop within 0.3 m of the
   target or after 15 s, save, fly to the next case's start (no node restart).
   After the last case it holds at the final target.

Difference from `_realdrone.py`: that one pins where the drone **launches**
(`(-2.5, 0)`) and flies 10 hand-picked cases from `primal_dual2_blocked`; this
one pins where the **obstacles** are and flies all 12 cases of the difficult
set. It also has the fix for the drone flying back to the previous start
between cases (see Gotchas), which the other two scripts don't have yet.

Each script has its own node name so private params (`~start_case_num`, ...)
don't leak between scripts on the ROS parameter server.

Policy: `logs/vel_tracking/20260826-011726/policy.pth` (+ `training_config.yaml`),
set by `DEFAULT_POLICY_DIR` in each script. `logs/` is gitignored, so a copy of
the same two files is kept in the repo at `models/cmdp/20260826-011726/`
(`policy.pth`, ~150 KB, and `training_config.yaml`). The scripts still read from
`logs/`; on a fresh clone, either copy `models/cmdp/20260826-011726/` to
`logs/vel_tracking/20260826-011726/` or point `DEFAULT_POLICY_DIR` at the
`models/` copy. Test-case sets (`logs/docs/primal_dual2*`) and saved flight
runs are also under `logs/` and are not in git.

### Data backup

The full `logs/vel_tracking/20260826-011726/` folder (about 1.1 GB: the policy
plus all Gazebo and real-drone flight runs) is backed up on SharePoint:
[ICRA_2027_CMDP / Primal_Dual_Policy / 20260826-011726](https://nusu.sharepoint.com/:f:/r/sites/Bio-inspiredLargeScaleManeuvering/Shared%20Documents/04-03%20Yan%20Rui/ICRA_2027_CMDP/Primal_Dual_Policy/20260826-011726?d=wd43758631fed4626b11afa8c4e1363c7&csf=1&web=1&e=O5EUex)
(NUS access required). The test-case sets (`logs/docs/primal_dual2`,
`primal_dual2_blocked`, `primal_dual2_difficult`) are stored there too.

## Test case format

`<test_cases_dir>/layoutN/case_NNN/` with `metadata.yaml` (`start_pos_sim`,
`target_pos_sim`, `start_quat_xyzw`, ...) and `obstacles.npz` (`positions`,
`radii`). Obstacles are fixed per layout; start/target vary per case.

Available sets (`logs/docs/`):
- `primal_dual2`, `primal_dual2_blocked`: 5 layouts x 20 cases.
- `primal_dual2_difficult`: layout0, 12 cases forming a closed loop around the
  room (each case's target is the next case's start).

## Coordinate frames and offsets

`sim = world + scene_offset`, so `world = sim - scene_offset`. All saved
trajectories/obstacles are in the **sim** (training) frame. The `scene_offset`
stored in the test sets' `metadata.yaml` is 0 and is ignored by the real-drone
scripts, which recompute it per case:

- `_realdrone.py`: offset maps each case's **start** to `REAL_ROOM_FIXED_START_XY`
  (currently `(-2.5, 0.0)`). Cases come from `SELECTED_REALDRONE_CASES`
  (10 short `primal_dual2_blocked` cases whose full path fits the room).
- `_realdrone_obstacle_centered.py`: offset maps each case's **obstacle
  centroid** (mean of the 4 obstacles) to `VICON_ROOM_CENTER_XY = (0, 0)`.
  Uses `primal_dual2_difficult`. For that set the centroid is (5,5), so the
  4 obstacles land at (+/-1.21, +/-1.21) in the Vicon frame, radius 0.71 m.
  Vicon origin is *assumed* to be the room centre.
- z is never offset.

### Selected cases: `_realdrone.py` (fixed launch point)

From `primal_dual2_blocked`, shortest 10 whose full recorded path fits the room.
Flown in this order (`_start_case_num` / `_end_case_num` index this list, 1-based). "World" = Vicon/room
frame with launch at `(-2.5, 0)`; all 10 stay within x <= 2.04, y in [-3.64, 3.60].

| # | Case | Start (sim) | Target (sim) | Target (world) | Path (m) |
|---|---|---|---|---|---|
| 1 | layout2/case_006 | (0.31, 7.85) | (3.97, 7.73) | (1.16, -0.12) | 4.19 |
| 2 | layout2/case_014 | (7.41, 6.93) | (7.98, 3.30) | (-1.93, -3.64) | 4.36 |
| 3 | layout1/case_019 | (2.75, 9.17) | (6.20, 7.04) | (0.95, -2.12) | 4.37 |
| 4 | layout4/case_015 | (2.98, 6.71) | (6.88, 4.94) | (1.41, -1.77) | 4.60 |
| 5 | layout2/case_007 | (1.32, 6.31) | (3.86, 9.91) | (0.04, 3.60) | 4.64 |
| 6 | layout3/case_002 | (3.52, 6.26) | (7.03, 8.44) | (1.01, 2.18) | 4.68 |
| 7 | layout3/case_014 | (3.98, 2.90) | (7.72, 5.37) | (1.23, 2.47) | 4.75 |
| 8 | layout3/case_004 | (4.78, 6.13) | (7.54, 2.51) | (0.26, -3.62) | 5.03 |
| 9 | layout4/case_000 | (3.67, 6.61) | (8.21, 4.06) | (2.04, -2.55) | 5.43 |
| 10 | layout0/case_018 | (1.33, 4.66) | (4.88, 8.26) | (1.05, 3.60) | 5.65 |

Flight altitude is the recorded z (0.84-1.18 m), unchanged.
Obstacles for these are virtual; some fall outside the room (e.g. for
layout3/case_002, obstacles 1 and 3 are at x < -3), which is harmless.

### Selected cases: `_realdrone_obstacle_centered.py`

All 12 cases of `primal_dual2_difficult/layout0` (`case_000` ... `case_011`),
flown in order. 8 fit the room with the 0.3 m margin; `case_000`, `case_005`,
`case_006`, `case_011` do not (see below).

Real room assumed: x in [-3, 3], y in [-5, 5]. Case selection checked the full
recorded `trajectory_sim.npy` (not just start/target) against these bounds with
a 0.3 m margin. In `SELECTED_REALDRONE_CASES` of the obstacle-centred script,
cases 000, 005, 006, 011 exceed that margin by up to ~0.3 m in x; they are
flown anyway (explicit choice) and are **not verified safe**.

## Running

```bash
# all selected cases
rosrun nn_policy policy_vel_combined_cmdp_gru_test_deepset_sim2real_realdrone_obstacle_centered.py

# resume / subset (1-indexed, inclusive). Pass the ORIGINAL output_root
# (full path) to keep saving into the same run folder.
rosrun nn_policy <script>.py _start_case_num:=3 _end_case_num:=6 \
    _output_root:=<full path of the interrupted run>
```

Params: `~test_cases_dir`, `~output_root`, `~start_case_num`, `~end_case_num`
(the realdrone scripts; the Gazebo script always runs all cases).

Default output root: `logs/vel_tracking/<policy>/{real_sim2real_test|realdrone_sim2real_test|realdrone_obstacle_centered_test}_<timestamp>/`.

## Per-case run flow

1. Fly (PVA hold) to the case's start; wait for dist < 0.20 m, speed < 0.15 m/s,
   yaw error < 5 deg, held 1 s.
2. Switch to NN control (attitude/body-rate mode) and record.
3. Stop when within **0.3 m** of the target, or after `test_max_duration`
   (default `500 * delta_time` = 15 s).
4. Save, then fly to the next case's start without restarting the node.
5. After the last case, hold at the final target.

## Saved files (per case)

`trajectory_sim.npy`, `actions.npy`, `body_rates.npy`, `obstacles.npz`,
`metadata.yaml`, `policy_runtime_obs.csv`, plus `body_rates.png` and
`throttle_height.png`. `costs.npz` / `metrics.yaml` from the training-side
format are **not** computed at flight time; compute them offline.

GIFs are skipped at record time (slow). Generate afterwards:

```bash
python3 scripts/plot_case_gifs.py logs/vel_tracking/<policy>/<run_folder>
```
Needs only numpy/matplotlib/yaml/Pillow, no ROS.

## Gotchas found the hard way

- **Stale rosparams**: params persist on the ROS param server between runs of
  the same node name. If it starts at the wrong case, run
  `rosparam delete /<node_name>` or pass `_start_case_num:=1` explicitly.
- **Drone flew back to the previous start between cases**: `traj_server.cpp`
  only accepts a new PVA position (`exec_trajectory`) while its own mode is
  PVA, and the mode switch arrives on a separate topic. Meanwhile it re-sends
  `last_mission_pos_` at 25 Hz, so a slow `save_outputs()` (matplotlib) after the
  mode switch left it holding the old start. Fix (obstacle-centred script only
  so far): publish the next start to `planner_adaptor/pos_update` (not gated on
  mode) and switch mode *before* `save_outputs()`, and re-send the mode command
  every tick while in PVA hold. `_realdrone.py` and `_sim2real.py` do not have
  this fix yet.
- Log/CSV writes are guarded by a lock: the case-transition thread and the
  policy-evaluation thread are separate `rospy.Timer` threads.
- The `checkpoint` here is a DeepSets-GRU (`TrackVelDeepSetsGRU`), state = att +
  angvel + (desired_vel - vel), 5-D obstacle features
  `[rel_x, rel_y, radius, dist_xy, clearance] / max_range`.
