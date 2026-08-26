#!/usr/bin/env python3
"""Estimate the collective throttle -> collective thrust mapping of the drone.

Model (this is what the training sim assumes -- a *linear* map):

    specific_force_z (m/s^2) = thr2acc * u                 # u = normalized throttle in [0, 1]
    collective_thrust (N)    = mass * specific_force_z
                             = (mass * thr2acc) * u

How it is measured:
    For a thrust-only quadrotor the body-frame accelerometer reads the specific
    force along body z, which equals thrust/mass *regardless of tilt*:

        imu.linear_acceleration.z  ==  F / mass  ==  thr2acc * u

    So we never need the attitude to recover thrust -- we just read the body-z
    accelerometer and pair it with the throttle that was commanded ~40 ms earlier
    (the actuation + sensing delay, same window px4ctrl uses: 35-45 ms).

Data sources (pick one):
    * A rosbag:   --bag flight.bag        (recommended: repeatable, offline)
    * Live ROS:   (default)               collect while flying, fit on Ctrl-C

Outputs:
    * printed report: thr2acc, F_max = mass*thr2acc, hover throttle, TWR, R^2
    * <out>.png  scatter of (u, a) with the fitted curves
    * <out>.npz  the raw paired (u, a) samples for re-fitting later

Fit only data where the drone is actually flying (throttle above --min-throttle),
otherwise ground contact / idle corrupts the fit.
"""

import argparse
import sys
import numpy as np


# --------------------------------------------------------------------------- #
# Core: pairing + fitting (pure numpy, no ROS needed)
# --------------------------------------------------------------------------- #
def pair_throttle_accel(thr_t, thr_u, imu_t, imu_az, delay=0.04, tol=0.02):
    """Pair each IMU sample with the throttle commanded `delay` seconds earlier.

    Args:
        thr_t, thr_u : (Nt,) command timestamps [s] and normalized throttle [0,1]
        imu_t, imu_az: (Ni,) imu timestamps [s] and body-z accel [m/s^2]
        delay        : actuation+sensing delay to compensate [s]
        tol          : max allowed |t_cmd - (t_imu - delay)| to accept a pair [s]

    Returns:
        u, a : matched arrays of throttle and specific-force-z (m/s^2)
    """
    thr_t = np.asarray(thr_t, dtype=float)
    thr_u = np.asarray(thr_u, dtype=float)
    imu_t = np.asarray(imu_t, dtype=float)
    imu_az = np.asarray(imu_az, dtype=float)

    order = np.argsort(thr_t)
    thr_t, thr_u = thr_t[order], thr_u[order]

    u_list, a_list = [], []
    for t, az in zip(imu_t, imu_az):
        t_target = t - delay
        idx = np.searchsorted(thr_t, t_target)
        # nearest of the two neighbours
        best, best_dt = None, tol
        for j in (idx - 1, idx):
            if 0 <= j < len(thr_t):
                dt = abs(thr_t[j] - t_target)
                if dt <= best_dt:
                    best, best_dt = j, dt
        if best is not None:
            u_list.append(thr_u[best])
            a_list.append(az)
    return np.asarray(u_list), np.asarray(a_list)


def _r2(a, a_pred):
    ss_res = np.sum((a - a_pred) ** 2)
    ss_tot = np.sum((a - np.mean(a)) ** 2)
    return 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")


def fit_models(u, a, mass, gravity=9.81):
    """Fit u -> a with three models and derive thrust quantities.

    Returns a dict of results.
    """
    if len(u) < 10:
        raise ValueError(f"Not enough paired samples to fit ({len(u)}).")

    # 1) linear through the origin: a = s * u   (the sim-consistent form)
    s = float(np.sum(u * a) / np.sum(u * u))
    r2_origin = _r2(a, s * u)

    # 2) affine: a = m1*u + b0
    m1, b0 = np.polyfit(u, a, 1)
    r2_affine = _r2(a, m1 * u + b0)

    # 3) quadratic: a = c2*u^2 + c1*u + c0
    c2, c1, c0 = np.polyfit(u, a, 2)
    r2_quad = _r2(a, np.polyval([c2, c1, c0], u))

    return {
        "n": len(u),
        "mass": mass,
        "gravity": gravity,
        # linear-through-origin (recommended)
        "thr2acc": s,                      # m/s^2 per unit throttle
        "F_max": mass * s,                 # N at u = 1.0
        "hover_throttle": gravity / s,     # u such that a = g
        "twr": (mass * s) / (mass * gravity),
        "r2_origin": r2_origin,
        # affine
        "affine": (m1, b0),
        "r2_affine": r2_affine,
        # quadratic
        "quad": (c2, c1, c0),
        "r2_quad": r2_quad,
    }


def rls_thr2acc(u, a, rho=0.998, P0=1e6, s0=None):
    """px4ctrl-style recursive least squares for the linear-through-origin gain.

    Provided as a cross-check of the batch `thr2acc`; processes samples in order.
    """
    s = s0 if s0 is not None else (np.sum(u * a) / np.sum(u * u))
    P = P0
    for ui, ai in zip(u, a):
        gamma = 1.0 / (rho + ui * P * ui)
        K = gamma * P * ui
        s = s + K * (ai - ui * s)
        P = (1.0 - K * ui) * P / rho
    return float(s)


def report(res, rls=None):
    m, g = res["mass"], res["gravity"]
    print("\n================  Throttle -> Thrust mapping  ================")
    print(f"  paired samples      : {res['n']}")
    print(f"  mass                : {m:.4f} kg   gravity: {g:.3f} m/s^2")
    print("  --- linear through origin (recommended, matches sim) ---")
    print(f"    a = thr2acc * u")
    print(f"    thr2acc           : {res['thr2acc']:.4f} m/s^2  per unit throttle")
    print(f"    F_collective(u)   : {res['F_max']:.4f} * u   [N]   (F_max at u=1)")
    print(f"    per-motor max (x4): {res['F_max']/4.0:.4f} N")
    print(f"    hover throttle    : {res['hover_throttle']:.4f}")
    print(f"    thrust-to-weight  : {res['twr']:.3f}")
    print(f"    R^2               : {res['r2_origin']:.4f}")
    if rls is not None:
        print(f"    thr2acc (RLS)     : {rls:.4f}   (online cross-check)")
    m1, b0 = res["affine"]
    print("  --- affine  a = m1*u + b0 ---")
    print(f"    m1={m1:.4f}  b0={b0:.4f}   R^2={res['r2_affine']:.4f}")
    c2, c1, c0 = res["quad"]
    print("  --- quadratic  a = c2*u^2 + c1*u + c0 ---")
    print(f"    c2={c2:.4f}  c1={c1:.4f}  c0={c0:.4f}   R^2={res['r2_quad']:.4f}")
    print("=============================================================\n")


def save_plot(u, a, res, out_png):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as e:
        print(f"[plot skipped: {e}]")
        return
    uu = np.linspace(0, max(1.0, u.max()), 200)
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.scatter(u, a, s=6, alpha=0.3, label=f"data (n={res['n']})")
    ax.plot(uu, res["thr2acc"] * uu, "r-", lw=2,
            label=f"linear-origin (R²={res['r2_origin']:.3f})")
    m1, b0 = res["affine"]
    ax.plot(uu, m1 * uu + b0, "g--", lw=1.5,
            label=f"affine (R²={res['r2_affine']:.3f})")
    c2, c1, c0 = res["quad"]
    ax.plot(uu, np.polyval([c2, c1, c0], uu), "b:", lw=1.5,
            label=f"quadratic (R²={res['r2_quad']:.3f})")
    ax.axhline(res["gravity"], color="k", lw=0.8, ls=":")
    ax.axvline(res["hover_throttle"], color="k", lw=0.8, ls=":")
    ax.set_xlabel("normalized collective throttle  u")
    ax.set_ylabel("specific force / body-z accel  a  [m/s²]")
    ax.set_title("Throttle → thrust calibration")
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print(f"[saved plot: {out_png}]")


# --------------------------------------------------------------------------- #
# Data source: rosbag
# --------------------------------------------------------------------------- #
def read_from_bag(bag_path, imu_topic, thr_topic):
    import rosbag
    thr_t, thr_u, imu_t, imu_az = [], [], [], []
    with rosbag.Bag(bag_path, "r") as bag:
        for topic, msg, t in bag.read_messages(topics=[imu_topic, thr_topic]):
            stamp = msg.header.stamp.to_sec() if msg._has_header else t.to_sec()
            if topic == imu_topic:
                imu_t.append(stamp)
                imu_az.append(msg.linear_acceleration.z)
            elif topic == thr_topic:
                thr_t.append(stamp)
                thr_u.append(float(msg.thrust))
    if not imu_t or not thr_t:
        raise RuntimeError(f"Bag missing data: {len(imu_t)} imu, {len(thr_t)} thrust msgs. "
                           f"Check --imu-topic / --thrust-topic.")
    return np.array(thr_t), np.array(thr_u), np.array(imu_t), np.array(imu_az)


# --------------------------------------------------------------------------- #
# Data source: live ROS node
# --------------------------------------------------------------------------- #
def collect_live(imu_topic, thr_topic):
    import rospy
    from sensor_msgs.msg import Imu
    from mavros_msgs.msg import AttitudeTarget

    thr_t, thr_u, imu_t, imu_az = [], [], [], []

    def thr_cb(msg):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0 else rospy.Time.now().to_sec()
        thr_t.append(stamp)
        thr_u.append(float(msg.thrust))

    def imu_cb(msg):
        stamp = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() > 0 else rospy.Time.now().to_sec()
        imu_t.append(stamp)
        imu_az.append(msg.linear_acceleration.z)

    rospy.init_node("throttle_thrust_calibrator", anonymous=True)
    rospy.Subscriber(thr_topic, AttitudeTarget, thr_cb, queue_size=50)
    rospy.Subscriber(imu_topic, Imu, imu_cb, queue_size=100)
    print(f"Collecting live. Fly a throttle sweep (hover + gentle climbs/descents).")
    print(f"  imu   : {imu_topic}\n  thrust: {thr_topic}")
    print("Press Ctrl-C when done to fit.\n")
    rospy.spin()
    return np.array(thr_t), np.array(thr_u), np.array(imu_t), np.array(imu_az)


# --------------------------------------------------------------------------- #
def load_npz_samples(path):
    """Load paired (u, a) samples from an npz produced by this script (u/a keys)
    or by thrust_model_estimator_node.py (samples_u/samples_a keys)."""
    d = np.load(path)
    if "samples_u" in d and "samples_a" in d:
        return np.asarray(d["samples_u"]), np.asarray(d["samples_a"])
    if "u" in d and "a" in d:
        return np.asarray(d["u"]), np.asarray(d["a"])
    raise KeyError(f"{path}: no (u,a) or (samples_u,samples_a) arrays found.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bag", nargs="+", default=None,
                    help="one or more rosbags to read + pair (offline). Omit for live mode.")
    ap.add_argument("--npz", nargs="+", default=None,
                    help="one or more npz files of already-paired samples (from the online "
                         "node or a prior run). Combined with any --bag data.")
    ap.add_argument("--mass", type=float, required=True, help="vehicle mass [kg]")
    ap.add_argument("--imu-topic", default="/drone0/mavros/imu/data")
    ap.add_argument("--thrust-topic", default="/drone0/mavros/setpoint_raw/target_attitude",
                    help="mavros_msgs/AttitudeTarget topic carrying the commanded normalized thrust")
    ap.add_argument("--delay", type=float, default=0.04, help="actuation+sensing delay [s]")
    ap.add_argument("--tol", type=float, default=0.02, help="pairing time tolerance [s]")
    ap.add_argument("--min-throttle", type=float, default=0.05,
                    help="ignore samples below this throttle (ground/idle)")
    ap.add_argument("--gravity", type=float, default=9.81)
    ap.add_argument("--out", default="throttle_thrust_fit", help="output prefix (.png/.npz)")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    # Aggregate paired (u, a) from every source (many bags + many npz), fit once.
    u_all, a_all = [], []

    for bag in (args.bag or []):
        thr_t, thr_u, imu_t, imu_az = read_from_bag(bag, args.imu_topic, args.thrust_topic)
        u_b, a_b = pair_throttle_accel(thr_t, thr_u, imu_t, imu_az, delay=args.delay, tol=args.tol)
        print(f"  {bag}: {len(u_b)} paired samples")
        u_all.append(u_b); a_all.append(a_b)

    for npz in (args.npz or []):
        u_n, a_n = load_npz_samples(npz)
        print(f"  {npz}: {len(u_n)} samples")
        u_all.append(u_n); a_all.append(a_n)

    if not args.bag and not args.npz:
        # live single session
        thr_t, thr_u, imu_t, imu_az = collect_live(args.imu_topic, args.thrust_topic)
        u_l, a_l = pair_throttle_accel(thr_t, thr_u, imu_t, imu_az, delay=args.delay, tol=args.tol)
        u_all.append(u_l); a_all.append(a_l)

    if not u_all:
        print("No data collected.")
        sys.exit(1)

    u = np.concatenate(u_all)
    a = np.concatenate(a_all)

    # keep only in-flight samples with plausible accel
    mask = (u >= args.min_throttle) & np.isfinite(a) & (a > 0.0)
    u, a = u[mask], a[mask]
    print(f"Total usable paired samples across all runs: {len(u)}")
    if len(u) < 10:
        print(f"Only {len(u)} usable pairs after filtering; need >=10. "
              f"Check topics/inputs, --delay, --min-throttle.")
        sys.exit(1)

    res = fit_models(u, a, mass=args.mass, gravity=args.gravity)
    rls = rls_thr2acc(u, a)
    report(res, rls=rls)

    np.savez(args.out + ".npz", u=u, a=a, **{k: v for k, v in res.items()
                                             if np.isscalar(v)})
    print(f"[saved samples: {args.out}.npz]")
    if not args.no_plot:
        save_plot(u, a, res, args.out + ".png")


if __name__ == "__main__":
    main()
