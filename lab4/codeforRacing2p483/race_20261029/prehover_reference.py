#!/usr/bin/env python3
"""Publish a C2 reference with a measured-state gate before race release.

Only desired states are published. Actual pose, clock, gates and physics are
never written. ROS imports occur only in the runtime entry point.
"""
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import shutil
import threading
import time


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(values, size=3):
    if not isinstance(values, (list, tuple)) or len(values) != size or not all(finite(v) for v in values):
        raise ValueError("expected finite vector of size %d" % size)
    return tuple(float(v) for v in values)


def norm(values):
    return math.sqrt(sum(v*v for v in values))


class StableHoverGate:
    def __init__(self, target, hold_s=.7, position_m=.12, speed_mps=.25,
                 tilt_deg=8.0, angular_rate_rad_s=.3, fresh_s=.12):
        self.target = vector(target)
        self.limits = dict(hold_s=hold_s, position_m=position_m, speed_mps=speed_mps,
                           tilt_deg=tilt_deg, angular_rate_rad_s=angular_rate_rad_s, fresh_s=fresh_s)
        if any(not finite(v) or v <= 0 for v in self.limits.values()):
            raise ValueError("positive finite stability limits required")
        self.latest = None
        self.stable_since = None
        self.errors = []

    def observe(self, stamp, position, velocity, quaternion_xyzw, angular_velocity):
        try:
            if not finite(stamp) or stamp < 0:
                raise ValueError("invalid odometry timestamp")
            p, v, q, w = vector(position), vector(velocity), vector(quaternion_xyzw, 4), vector(angular_velocity)
            qn = norm(q)
            if qn < 1e-9:
                raise ValueError("zero odometry quaternion")
            q = tuple(x/qn for x in q)
            previous = self.latest
            raw = (p, v, q, w)
            if previous is not None and stamp <= previous["stamp_sim_s"]:
                if stamp == previous["stamp_sim_s"] and raw == previous["raw"]:
                    return
                raise ValueError("odometry clock rollback or conflicting timestamp")
            cosine_tilt = max(-1., min(1., 1-2*(q[0]*q[0]+q[1]*q[1])))
            tilt = math.degrees(math.acos(cosine_tilt))
            metrics = dict(position_error_m=norm([x-y for x, y in zip(p, self.target)]),
                           speed_mps=norm(v), tilt_deg=tilt, angular_rate_rad_s=norm(w))
            inside = (metrics["position_error_m"] <= self.limits["position_m"] and
                      metrics["speed_mps"] <= self.limits["speed_mps"] and
                      metrics["tilt_deg"] <= self.limits["tilt_deg"] and
                      metrics["angular_rate_rad_s"] <= self.limits["angular_rate_rad_s"])
            gap = previous is not None and stamp-previous["stamp_sim_s"] > self.limits["fresh_s"]
            if not inside:
                self.stable_since = None
            elif self.stable_since is None or gap:
                self.stable_since = float(stamp)
            self.latest = dict(stamp_sim_s=float(stamp), raw=raw, inside_limits=inside, **metrics)
        except (TypeError, ValueError) as error:
            self.stable_since = None
            self.errors.append(str(error))

    def snapshot(self, now):
        if not finite(now) or now < 0:
            raise ValueError("invalid simulator time")
        latest = self.latest
        age = None if latest is None else now-latest["stamp_sim_s"]
        fresh = age is not None and -1e-6 <= age <= self.limits["fresh_s"]
        # Count only distinct measured odometry timestamps, never wall waiting.
        span = (latest["stamp_sim_s"]-self.stable_since
                if latest is not None and self.stable_since is not None else 0.)
        result = dict(ready=bool(fresh and not self.errors and latest["inside_limits"] and
                                 span+1e-9 >= self.limits["hold_s"]),
                      fresh=bool(fresh), sample_age_sim_s=age, stable_sampled_span_s=span,
                      limits=self.limits, data_errors=list(self.errors), target_ros_m=self.target)
        if latest is not None:
            result["actual"] = {k: v for k, v in latest.items() if k != "raw"}
        return result


class ReleaseClock:
    def __init__(self, start_sim_s, boundary_s=5., max_wait_s=6.):
        if any(not finite(v) or v < 0 for v in (start_sim_s, boundary_s, max_wait_s)) or boundary_s == 0:
            raise ValueError("invalid release clock configuration")
        self.start, self.boundary, self.max_wait = float(start_sim_s), float(boundary_s), float(max_wait_s)
        self.previous = self.start
        self.released_at = None
        self.phase = "PREPARING"

    def advance(self, now, stable_ready):
        if not finite(now) or now < self.previous-1e-9:
            self.phase = "FAILED"
            raise ValueError("simulator clock moved backwards or became invalid")
        self.previous = float(now)
        elapsed = max(0., now-self.start)
        if self.released_at is not None:
            return self.boundary+max(0., now-self.released_at)
        if elapsed < self.boundary:
            return elapsed
        self.phase = "WAITING_FOR_STABILITY"
        waited = elapsed-self.boundary
        if waited > self.max_wait+1e-9:
            self.phase = "FAILED"
            raise RuntimeError("actual hover stability not reached within bounded extra wait")
        if stable_ready is True:
            self.released_at = float(now)
            self.phase = "RACING"
        return self.boundary

    def snapshot(self, now, effective_time):
        return dict(phase=self.phase, reference_start_sim_s=self.start,
                    hover_release_plan_s=self.boundary, release_sim_s=self.released_at,
                    extra_wait_sim_s=max(0., (self.released_at if self.released_at is not None else now)-self.start-self.boundary),
                    effective_plan_time_s=effective_time, maximum_extra_wait_sim_s=self.max_wait)


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False)+"\n")
    os.replace(str(temporary), str(path))


def main():
    import rospy
    from geometry_msgs.msg import Transform, Twist
    from nav_msgs.msg import Odometry
    from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
    rospy.init_node("sprint_prehover_reference")
    if rospy.rostime.is_wallclock():
        raise RuntimeError("prehover requires the actual simulator clock")
    source = Path(rospy.get_param("~plan")).resolve()
    boundary = float(rospy.get_param("~hover_release_t", 5.))
    extra_wait = float(rospy.get_param("~maximum_hover_wait_s", 6.))
    dump = Path(rospy.get_param("~dump_file")).resolve()
    out = dump.parent
    for name in ("prehover_status.json", "prehover_events.jsonl", "prehover_release.json"):
        if (out/name).exists():
            raise RuntimeError("refusing to overwrite prehover output: "+name)
    helper_path = Path(__file__).resolve().parent.parent/"scripts/plan_reference.py"
    spec = importlib.util.spec_from_file_location("preserved_plan_reference", helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    rows = helper.load_plan(source)
    ref = helper.Reference(rows)
    if not 0 < boundary < ref.times[-1]:
        raise ValueError("hover boundary is outside the reference")
    target, hover_v, hover_a = ref.sample(boundary)
    if norm(hover_v) > 1e-7 or norm(hover_a) > 1e-6:
        raise ValueError("release boundary must have zero velocity and acceleration")
    gate = StableHoverGate(target)
    lock = threading.RLock()
    def received(msg):
        p, v, q, w = msg.pose.pose.position, msg.twist.twist.linear, msg.pose.pose.orientation, msg.twist.twist.angular
        with lock:
            gate.observe(msg.header.stamp.to_sec(), [p.x,p.y,p.z], [v.x,v.y,v.z], [q.x,q.y,q.z,q.w], [w.x,w.y,w.z])
    current = rospy.wait_for_message("/current_state", Odometry, timeout=10)
    p = current.pose.pose.position
    if norm([value-rows[0][key] for value,key in zip((p.x,p.y,p.z),("x","y","z"))]) > .08:
        raise ValueError("actual ground start differs from the verified new plan")
    received(current)
    subscriber = rospy.Subscriber("/current_state", Odometry, received, queue_size=1000)
    q = current.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    if source != dump:
        shutil.copyfile(source, dump)
    publisher = rospy.Publisher("/desired_state", MultiDOFJointTrajectoryPoint, queue_size=1)
    rospy.sleep(.1)
    start = rospy.Time.now().to_sec()
    rospy.set_param("/sprint_reference_start", start)
    clock = ReleaseClock(start, boundary, extra_wait)
    events = (out/"prehover_events.jsonl").open("x", encoding="utf-8")
    previous_phase = None
    state = {}
    effective = 0.
    release_written = False
    plan_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    rate = rospy.Rate(100)
    try:
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            with lock:
                stable = gate.snapshot(now)
            if stable["data_errors"]:
                raise RuntimeError("invalid measured state before release: "+str(stable["data_errors"]))
            effective = min(ref.times[-1], clock.advance(now, stable["ready"]))
            position, velocity, acceleration = ref.sample(effective)
            msg = MultiDOFJointTrajectoryPoint()
            msg.time_from_start = rospy.Duration(effective)
            transform = Transform()
            transform.translation.x, transform.translation.y, transform.translation.z = position
            transform.rotation.z, transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
            vel, acc = Twist(), Twist()
            vel.linear.x, vel.linear.y, vel.linear.z = velocity
            acc.linear.x, acc.linear.y, acc.linear.z = acceleration
            msg.transforms, msg.velocities, msg.accelerations = [transform], [vel], [acc]
            publisher.publish(msg)
            state = dict(clock.snapshot(now, effective), stability=stable, plan_sha256=plan_sha,
                         plan=str(source), actual_sim_time_s=now, observer_monotonic_s=time.monotonic())
            if clock.phase != previous_phase:
                events.write(json.dumps(state, allow_nan=False)+"\n")
                events.flush()
                previous_phase = clock.phase
            if clock.released_at is not None and not release_written:
                atomic_json(out/"prehover_release.json", state)
                release_written = True
            atomic_json(out/"prehover_status.json", state)
            rate.sleep()
    except Exception as error:
        if rospy.is_shutdown():
            return  # Owned normal SIGINT cleanup must not relabel a completed run.
        state.update(phase="FAILED", failure=type(error).__name__+": "+str(error))
        events.write(json.dumps(state, allow_nan=False)+"\n")
        events.flush()
        atomic_json(out/"prehover_status.json", state)
        raise
    finally:
        subscriber.unregister()
        events.close()


if __name__ == "__main__":
    main()
