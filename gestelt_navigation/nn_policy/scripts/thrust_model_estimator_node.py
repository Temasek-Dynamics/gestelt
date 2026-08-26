#!/usr/bin/env python3
"""Standalone ROS node: online throttle -> thrust (thr2acc) estimation during flight.

Runs completely decoupled from the policy node. Subscribe it to the IMU and the
commanded-throttle topic and it estimates, live, the linear thrust model:

    specific_force_z (m/s^2) = thr2acc * u          # u = normalized throttle in [0,1]
    collective_thrust (N)    = mass * thr2acc * u

Method (px4ctrl LinearControl::estimateThrustModel):
    The body-z accelerometer reads specific force along body z = thrust/mass,
    independent of tilt. Each commanded throttle is time-stamped; when an IMU
    sample arrives we pair it with the throttle sent ~35-45 ms earlier
    (actuation+sensing delay) and run one recursive-least-squares update.

It self-gates: throttle is only queued while a real thrust command is being sent,
so during position hold nothing is paired and the fit is not corrupted (for
ExecTrajectory you can additionally require a specific type_mask).

Live outputs (private topics, plot them in rqt/PlotJuggler):
    ~thr2acc         std_msgs/Float32
    ~hover_throttle  std_msgs/Float32
Console: a 1 Hz summary. On shutdown: <out>.npz (+ <out>.png) with a batch fit.

Usage: start this node on the side, then fly as many gate-traversal runs as you
like with the policy script. It keeps collecting across all of them; when you
Ctrl-C it, the shutdown batch fit and <out>.npz cover every run in the session.
    rosrun nn_policy thrust_model_estimator_node.py \
        _imu_topic:=/drone0/mavros/imu/data \
        _thrust_topic:=/drone0/mavros/setpoint_raw/attitude \
        _thrust_msg:=attitude_target \
        _mass:=0.234 _out:=/tmp/gate_thrust_fit
"""

import collections
import numpy as np
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as Rot


class OnlineThrustEstimator(object):
    """Recursive-least-squares estimate of the linear thrust gain thr2acc."""

    def __init__(self, gravity=9.81, mass=None, hover_throttle=0.30, rho=0.998, P0=1e6):
        self.g = float(gravity)
        self.mass = mass
        self.thr2acc = self.g / float(hover_throttle)   # initial guess
        self.P = float(P0)
        self.rho = float(rho)
        self.timed_thrust = collections.deque(maxlen=200)  # (rospy.Time, thr)

    def record_thrust(self, thr, stamp):
        self.timed_thrust.append((stamp, float(thr)))

    def update(self, acc_z, now):
        """Consume the throttle from ~40 ms ago and do one RLS step.
        Returns the paired throttle (float) on success, else None."""
        while self.timed_thrust:
            t_stamp, thr = self.timed_thrust[0]
            dt = (now - t_stamp).to_sec()
            if dt > 0.045:                       # too old -> drop, keep looking
                self.timed_thrust.popleft()
                continue
            if dt < 0.035:                       # too fresh -> wait for next IMU
                return None
            gamma = 1.0 / (self.rho + thr * self.P * thr)
            K = gamma * self.P * thr
            self.thr2acc += K * (acc_z - thr * self.thr2acc)
            self.P = (1.0 - K * thr) * self.P / self.rho
            self.timed_thrust.popleft()
            return thr
        return None

    @property
    def hover_throttle(self):
        return self.g / self.thr2acc if self.thr2acc > 1e-6 else float("nan")

    @property
    def F_max(self):
        return None if self.mass is None else self.mass * self.thr2acc


class ThrustEstimatorNode(object):
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/drone0/mavros/imu/data")
        # Default: the outgoing FCU attitude setpoint (AttitudeTarget.thrust). This is
        # the authoritative applied thrust with the least added latency -- preferred
        # over the /target_attitude echo (round-trips the FCU) or ExecTrajectory.
        self.thrust_topic = rospy.get_param("~thrust_topic", "/drone0/mavros/setpoint_raw/attitude")
        # "attitude_target" (mavros AttitudeTarget.thrust) or "exec_trajectory" (gestelt ExecTrajectory.throttle)
        self.thrust_msg = rospy.get_param("~thrust_msg", "attitude_target")
        # For ExecTrajectory only: require this type_mask (throttle applied). -1 disables the gate.
        self.require_type_mask = int(rospy.get_param("~require_type_mask", 1))
        self.min_throttle = float(rospy.get_param("~min_throttle", 0.05))
        self.hover_throttle0 = float(rospy.get_param("~hover_throttle", 0.30))
        self.gravity = float(rospy.get_param("~gravity", 9.81))
        mass = float(rospy.get_param("~mass", -1.0))
        self.mass = mass if mass > 0 else None
        self.out = rospy.get_param("~out", "throttle_thrust_online")

        # Rate-controller first-order lag (alpha) estimation:
        #   omega[k+1] = alpha*omega_cmd[k] + (1-alpha)*omega[k]   (per body axis)
        # alpha is defined at the sim control step dt_sim; we sample measured rate
        # at ~dt_sim so the estimate matches directly (and also report tau/alpha_sim).
        self.estimate_rate_lag = bool(rospy.get_param("~estimate_rate_lag", True))
        self.dt_sim = float(rospy.get_param("~dt_sim", 0.02))
        self.cmd_fresh = float(rospy.get_param("~cmd_fresh", 0.1))  # max rate-cmd age to trust [s]
        # Frame of the commanded body rates. The default thrust source
        # (setpoint_raw/attitude) carries MAVLink SET_ATTITUDE_TARGET body-frame
        # rates, so no rotation is needed ("body"). Set to "world" only if your
        # command topic publishes the policy's world-frame rates unrotated, in
        # which case they are rotated into body via the IMU orientation before the fit.
        self.cmd_frame = rospy.get_param("~cmd_frame", "body")

        self.est = OnlineThrustEstimator(gravity=self.gravity, mass=self.mass,
                                         hover_throttle=self.hover_throttle0)
        self.history = []   # (t, thr2acc, hover_throttle, acc_z)
        self.samples = []   # (u, a) for the shutdown batch fit
        self.n_imu = 0      # received-message counters (diagnostics)
        self.n_thrust = 0

        # rate-lag state: commanded rate is stored in its native frame (world by
        # default); rotated to body per-sample using the IMU orientation.
        self.last_cmd_rate = np.zeros(3)   # commanded body rate [rad/s], cmd_frame
        self.last_cmd_time = None
        self.prev_omega = None             # measured body rate at previous dt_sim tick
        self.prev_cmd_body = None          # commanded rate in BODY frame at that tick
        self.last_rate_sample_t = None
        self.rate_pairs = []               # (e[3], d[3], dt): e=cmd_body-omega, d=Δomega
        # for the joint transport-delay + alpha search (shift cmd, re-fit):
        self.cmd_hist = collections.deque(maxlen=500000)  # (t_sec, cmd_rate[3])
        self.rate_ticks = []               # (t_prev_sec, omega[3], d_omega[3])

        # thrust command subscriber (message type depends on source)
        if self.thrust_msg == "attitude_target":
            from mavros_msgs.msg import AttitudeTarget
            rospy.Subscriber(self.thrust_topic, AttitudeTarget, self.thr_att_cb, queue_size=50)
        else:
            from gestelt_msgs.msg import ExecTrajectory
            rospy.Subscriber(self.thrust_topic, ExecTrajectory, self.thr_exec_cb, queue_size=50)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=100)

        self.thr2acc_pub = rospy.Publisher("~thr2acc", Float32, queue_size=5)
        self.hover_pub = rospy.Publisher("~hover_throttle", Float32, queue_size=5)
        rospy.Timer(rospy.Duration(1.0), self.print_cb)
        rospy.on_shutdown(self.on_shutdown)

        rospy.loginfo("thrust_model_estimator: imu=%s thrust=%s (%s) mass=%s",
                      self.imu_topic, self.thrust_topic, self.thrust_msg, str(self.mass))

    # ---- thrust command callbacks ----
    # NOTE: use rospy.Time.now() for BOTH thrust and IMU (as px4ctrl does) so the
    # 35-45 ms pairing window compares timestamps on a single, consistent clock.
    # Mixing message header.stamp (different clocks per topic) breaks the pairing.
    def thr_att_cb(self, msg):
        now = rospy.Time.now()
        self.n_thrust += 1
        self.est.record_thrust(msg.thrust, now)
        self.last_cmd_rate = np.array([msg.body_rate.x, msg.body_rate.y, msg.body_rate.z])
        self.last_cmd_time = now
        self.cmd_hist.append((now.to_sec(), self.last_cmd_rate.copy()))

    def thr_exec_cb(self, msg):
        if self.require_type_mask >= 0 and msg.type_mask != self.require_type_mask:
            return
        now = rospy.Time.now()
        self.n_thrust += 1
        self.est.record_thrust(msg.throttle, now)
        self.last_cmd_rate = np.array([msg.angular_rates.angular.x,
                                       msg.angular_rates.angular.y,
                                       msg.angular_rates.angular.z])
        self.last_cmd_time = now
        self.cmd_hist.append((now.to_sec(), self.last_cmd_rate.copy()))

    # ---- rate-controller first-order lag sampling ----
    def _sample_rate_lag(self, msg, now):
        omega = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        # Only trust intervals where a fresh rate command is streaming (self-gates
        # to actual flight; avoids differencing across a gap between runs).
        fresh = (self.last_cmd_time is not None and
                 (now - self.last_cmd_time).to_sec() < self.cmd_fresh)
        if not fresh:
            self.prev_omega = None
            self.last_rate_sample_t = None
            return

        # Commanded rate in BODY frame at this instant (rotate world->body with the
        # IMU orientation, mirroring the sim's q_inv * target). Quaternion is x,y,z,w.
        if self.cmd_frame == "body":
            cmd_body = self.last_cmd_rate.copy()
        else:
            q = np.array([msg.orientation.x, msg.orientation.y,
                          msg.orientation.z, msg.orientation.w])
            if np.linalg.norm(q) < 1e-6:
                return  # no valid attitude yet
            cmd_body = Rot.from_quat(q).apply(self.last_cmd_rate, inverse=True)  # R^T * world

        if self.last_rate_sample_t is None:
            self.prev_omega = omega
            self.prev_cmd_body = cmd_body
            self.last_rate_sample_t = now
            return

        dt = (now - self.last_rate_sample_t).to_sec()
        if dt < self.dt_sim:
            return
        if self.prev_omega is not None and self.prev_cmd_body is not None:
            e = self.prev_cmd_body - self.prev_omega   # rate error at interval start
            d = omega - self.prev_omega                # measured Δomega over dt
            self.rate_pairs.append((e, d, dt, self.prev_omega.copy()))
            # t at interval start (= time of prev_omega) for the delay search
            self.rate_ticks.append((self.last_rate_sample_t.to_sec(),
                                    self.prev_omega.copy(), d.copy()))
        # start next interval
        self.prev_omega = omega
        self.prev_cmd_body = cmd_body
        self.last_rate_sample_t = now

    # ---- IMU -> RLS update ----
    def imu_cb(self, msg):
        self.n_imu += 1
        now = rospy.Time.now()
        if self.estimate_rate_lag:
            self._sample_rate_lag(msg, now)
        acc_z = msg.linear_acceleration.z
        u = self.est.update(acc_z, now)
        if u is None:
            return
        self.history.append((now.to_sec(), float(self.est.thr2acc),
                             float(self.est.hover_throttle), float(acc_z)))
        if u >= self.min_throttle:
            self.samples.append((float(u), float(acc_z)))
        self.thr2acc_pub.publish(Float32(self.est.thr2acc))
        self.hover_pub.publish(Float32(self.est.hover_throttle))

    def print_cb(self, _event):
        est = self.est
        line = ("[thr2acc] %7.3f m/s^2/u | hover_thr %6.3f | n=%d | rx imu=%d thr=%d | queued=%d"
                % (est.thr2acc, est.hover_throttle, len(self.history),
                   self.n_imu, self.n_thrust, len(self.est.timed_thrust)))
        if est.F_max is not None:
            line += " | F_max %6.3f N" % est.F_max
        rospy.loginfo(line)

    # ---- shutdown: save + batch fit ----
    def on_shutdown(self):
        save = {}
        if self.samples:
            u = np.array([s[0] for s in self.samples])
            a = np.array([s[1] for s in self.samples])
            save.update(samples_u=u, samples_a=a, history=np.array(self.history),
                        final_thr2acc=self.est.thr2acc,
                        final_hover_throttle=self.est.hover_throttle,
                        mass=(self.mass if self.mass is not None else np.nan))
        else:
            rospy.logwarn("No thrust samples collected.")
        if self.rate_pairs:
            save.update(rate_e=np.array([p[0] for p in self.rate_pairs]),
                        rate_d=np.array([p[1] for p in self.rate_pairs]),
                        rate_dt=np.array([p[2] for p in self.rate_pairs]),
                        rate_omega=np.array([p[3] for p in self.rate_pairs]))
        if save:
            np.savez(self.out + ".npz", **save)
            rospy.loginfo("Saved -> %s.npz", self.out)
        if self.samples:
            self._batch_report(u, a)
            self._save_plot(u, a)
        if self.estimate_rate_lag:
            self._fit_rate_lag()

    def _fit_rate_lag(self):
        if len(self.rate_pairs) < 20:
            rospy.logwarn("Only %d rate-lag samples; skipping fit.", len(self.rate_pairs))
            return
        E = np.array([p[0] for p in self.rate_pairs])   # (N,3) rate error (body)
        D = np.array([p[1] for p in self.rate_pairs])   # (N,3) delta-omega (body)
        dts = np.array([p[2] for p in self.rate_pairs])
        O = np.array([p[3] for p in self.rate_pairs])   # (N,3) measured omega (body)
        CMD = E + O                                      # (N,3) commanded rate in body frame
        dt_mean = float(np.mean(dts))
        names = ["x/roll ", "y/pitch", "z/yaw  "]
        print("\n===========  Rate-controller first-order lag (alpha)  ===========")
        print("  samples: %d   mean dt=%.4f s   dt_sim=%.4f s   cmd_frame=%s"
              % (len(dts), dt_mean, self.dt_sim, self.cmd_frame))
        print("  model: omega[k+1] = alpha*omega_cmd[k] + (1-alpha)*omega[k]  (body frame)")
        for i in range(3):
            e = E[:, i]; d = D[:, i]
            denom = float(np.sum(e * e))
            if denom < 1e-9:
                print("  %s: (insufficient excitation)" % names[i]); continue
            alpha = float(np.sum(d * e) / denom)        # alpha at the sampling dt
            ss_res = np.sum((d - alpha * e) ** 2); ss_tot = np.sum((d - d.mean()) ** 2)
            r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
            if 0.0 < alpha < 1.0:                       # convert to dt_sim-consistent alpha
                tau = -dt_mean / np.log(1.0 - alpha)
                alpha_sim = 1.0 - np.exp(-self.dt_sim / tau)
            else:
                tau = float("nan"); alpha_sim = alpha
            print("  %s: alpha=%.4f (@dt=%.3f) | tau=%.4f s | alpha_sim=%.4f (@%.3f) | R^2=%.3f | n=%d"
                  % (names[i], alpha, dt_mean, tau, alpha_sim, self.dt_sim, r2, len(e)))

        # ---- frame/sign diagnostics ----
        def corr(x, y):
            return (np.corrcoef(x, y)[0, 1]
                    if np.std(x) > 1e-6 and np.std(y) > 1e-6 else float("nan"))
        print("  --- diagnostics ---")
        print("  excitation  std(cmd_body) / std(omega)  [rad/s]:")
        for i in range(3):
            print("    %s: %.3f / %.3f" % (names[i], np.std(CMD[:, i]), np.std(O[:, i])))
        print("  corr(cmd_body[row], omega[col])  (diagonal should dominate & be +ve;")
        print("       -ve diagonal => axis sign flip; off-diagonal => axis swap/rotation):")
        print("            omega_x  omega_y  omega_z")
        for i in range(3):
            print("    cmd %s [%+6.2f  %+6.2f  %+6.2f]"
                  % (names[i].strip(), corr(CMD[:, i], O[:, 0]),
                     corr(CMD[:, i], O[:, 1]), corr(CMD[:, i], O[:, 2])))
        print("  corr(cmd_body[row], cmd_body[col])  (high off-diagonal => the policy")
        print("       commands multiple axes together; explains benign cross-terms above):")
        print("            cmd_x    cmd_y    cmd_z")
        for i in range(3):
            print("    cmd %s [%+6.2f  %+6.2f  %+6.2f]"
                  % (names[i].strip(), corr(CMD[:, i], CMD[:, 0]),
                     corr(CMD[:, i], CMD[:, 1]), corr(CMD[:, i], CMD[:, 2])))
        print("=================================================================\n")
        self._fit_rate_lag_delay()
        self._save_rate_plot(E, D)

    def _fit_rate_lag_delay(self):
        """Joint transport-delay + alpha per axis: sweep a dead-time applied to the
        command, re-fit the first-order model at each, and keep the delay with the
        best R^2. Uses the raw command history (assumed body-frame, cmd_frame=body)
        interpolated to (t_tick - delay)."""
        if len(self.rate_ticks) < 30 or len(self.cmd_hist) < 30:
            rospy.logwarn("Not enough data for delay search.")
            return
        if self.cmd_frame == "world":
            rospy.logwarn("delay search uses the raw (unrotated) command; run with cmd_frame=body.")
        T = np.array([t[0] for t in self.rate_ticks])
        OM = np.array([t[1] for t in self.rate_ticks])   # omega at tick start
        D = np.array([t[2] for t in self.rate_ticks])    # delta-omega over tick
        TC = np.array([c[0] for c in self.cmd_hist])
        CMD = np.array([c[1] for c in self.cmd_hist])
        order = np.argsort(TC); TC = TC[order]; CMD = CMD[order]
        # mean tick dt (for tau / alpha_sim conversion)
        dd = np.diff(T); dd = dd[(dd > 0) & (dd < 3 * self.dt_sim)]
        dt_mean = float(np.median(dd)) if dd.size else self.dt_sim
        delays = np.arange(0.0, 0.0805, 0.005)   # 0..80 ms
        names = ["x/roll ", "y/pitch", "z/yaw  "]
        print("======  Rate-loop transport delay + alpha (joint per axis)  ======")
        print("  delay grid 0..%.0f ms  |  tick dt=%.4f s  |  dt_sim=%.4f s"
              % (delays[-1] * 1000, dt_mean, self.dt_sim))
        for i in range(3):
            best = None  # (delay, alpha, r2)
            for dly in delays:
                cmd_i = np.interp(T - dly, TC, CMD[:, i])
                e = cmd_i - OM[:, i]; d = D[:, i]
                denom = float(np.sum(e * e))
                if denom < 1e-9:
                    continue
                alpha = float(np.sum(d * e) / denom)
                ss_res = np.sum((d - alpha * e) ** 2); ss_tot = np.sum((d - d.mean()) ** 2)
                r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else -np.inf
                if best is None or r2 > best[2]:
                    best = (dly, alpha, r2)
            if best is None:
                print("  %s: (insufficient excitation)" % names[i]); continue
            dly, alpha, r2 = best
            if 0.0 < alpha < 1.0:
                tau = -dt_mean / np.log(1.0 - alpha)
                alpha_sim = 1.0 - np.exp(-self.dt_sim / tau)
            else:
                tau = float("nan"); alpha_sim = alpha
            print("  %s: delay=%4.0f ms | alpha=%.4f | tau=%.4f s | alpha_sim=%.4f | R^2=%.3f"
                  % (names[i], dly * 1000, alpha, tau, alpha_sim, r2))
        print("==================================================================\n")

    def _save_rate_plot(self, E, D):
        """Per-axis Δω vs rate-error scatter with the fitted line. A clean
        first-order axis is a tight positive line through the origin; overshoot
        shows a loop/negative slope, noise shows a cloud."""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            rospy.logwarn("rate plot skipped: %s", e)
            return
        names = ["x / roll", "y / pitch", "z / yaw"]
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        for i in range(3):
            e = E[:, i]; d = D[:, i]; ax = axes[i]
            ax.scatter(e, d, s=6, alpha=0.25)
            denom = float(np.sum(e * e))
            if denom > 1e-9:
                alpha = float(np.sum(d * e) / denom)
                ss_res = np.sum((d - alpha * e) ** 2); ss_tot = np.sum((d - d.mean()) ** 2)
                r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
                xx = np.array([e.min(), e.max()])
                ax.plot(xx, alpha * xx, "r-", lw=2,
                        label="alpha=%.3f  R^2=%.2f" % (alpha, r2))
                ax.legend()
            ax.axhline(0, color="k", lw=0.5); ax.axvline(0, color="k", lw=0.5)
            ax.set_title(names[i]); ax.set_xlabel("rate error e = cmd - omega [rad/s]")
            ax.set_ylabel("delta omega [rad/s]"); ax.grid(alpha=0.3)
        fig.suptitle("Rate-loop first-order fit  (Delta-omega = alpha * e)")
        fig.tight_layout()
        fig.savefig(self.out + "_ratelag.png", dpi=130)
        rospy.loginfo("Saved rate-lag plot -> %s_ratelag.png", self.out)

    def _batch_report(self, u, a):
        if len(u) < 10:
            rospy.logwarn("Only %d samples; skipping batch fit.", len(u))
            return
        s = float(np.sum(u * a) / np.sum(u * u))               # linear through origin
        ss_res = np.sum((a - s * u) ** 2)
        ss_tot = np.sum((a - a.mean()) ** 2)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
        c2, c1, c0 = np.polyfit(u, a, 2)                        # quadratic
        print("\n===========  Batch throttle -> thrust (offline cross-check)  ===========")
        print("  samples            : %d" % len(u))
        print("  thr2acc (origin)   : %.4f m/s^2/u   (R^2=%.4f)" % (s, r2))
        print("  hover throttle     : %.4f" % (self.gravity / s))
        if self.mass is not None:
            print("  F_max = m*thr2acc  : %.4f N   (per-motor/4 = %.4f N)"
                  % (self.mass * s, self.mass * s / 4.0))
        print("  quadratic fit      : a = %.4f u^2 + %.4f u + %.4f" % (c2, c1, c0))
        print("  online final thr2acc: %.4f (RLS)" % self.est.thr2acc)
        print("========================================================================\n")

    def _save_plot(self, u, a):
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as e:
            rospy.logwarn("plot skipped: %s", e)
            return
        s = float(np.sum(u * a) / np.sum(u * u))
        uu = np.linspace(0, max(1.0, u.max()), 200)
        fig, ax = plt.subplots(figsize=(7, 5))
        ax.scatter(u, a, s=6, alpha=0.3, label="paired data")
        ax.plot(uu, s * uu, "r-", lw=2, label="linear-origin thr2acc=%.2f" % s)
        ax.plot(uu, self.est.thr2acc * uu, "m--", lw=1.5,
                label="online RLS=%.2f" % self.est.thr2acc)
        ax.axhline(self.gravity, color="k", lw=0.8, ls=":")
        ax.set_xlabel("normalized throttle u")
        ax.set_ylabel("body-z accel a [m/s^2]")
        ax.set_title("Online throttle -> thrust")
        ax.legend(); ax.grid(alpha=0.3); fig.tight_layout()
        fig.savefig(self.out + ".png", dpi=130)
        rospy.loginfo("Saved plot -> %s.png", self.out)


def main():
    rospy.init_node("thrust_model_estimator")
    ThrustEstimatorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
