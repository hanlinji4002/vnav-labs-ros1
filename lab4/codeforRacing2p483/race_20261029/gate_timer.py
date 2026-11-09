#!/usr/bin/env python3
"""Read-only, simulated-time stopwatch for ordered actual gate passages.

The counted point must cross a gate plane forward inside the conservative
fully-transparent square established from the unchanged official texture.
This is a center-point rule, not a swept-body certificate or Unity scorer.
The ROS adapter starts after reset and before the reference publisher. It
never changes a controller, gate, simulator clock, or physical parameter.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import threading
import time


EVIDENCE_SHA256 = "a49ec0d1461f13979648f995bcdebbb299daf1bc94dfa0631320321ddae8dfe8"
RULE = "ordered forward center-plane crossings inside verified transparent point square"


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(values, size=3):
    if not isinstance(values, (list, tuple)) or len(values) != size or not all(finite(x) for x in values):
        raise ValueError("expected {} finite coordinates".format(size))
    return tuple(float(x) for x in values)


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def minus(a, b):
    return tuple(x-y for x, y in zip(a, b))


def quaternion_axes(value):
    x, y, z, w = vector(value, 4)
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n < 1e-12:
        raise ValueError("zero gate quaternion")
    x, y, z, w = (v/n for v in (x, y, z, w))
    return ((1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)),
            (2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)),
            (2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y)))


def load_geometry_evidence(path):
    raw = Path(path).read_bytes()
    actual = hashlib.sha256(raw).hexdigest()
    if actual != EVIDENCE_SHA256:
        raise ValueError("gate geometry evidence SHA-256 differs from audited asset extraction")
    source = json.loads(raw)
    region = source["conservative_centered_point_region"]
    if region["max_alpha_in_region"] != 0 or region["half_extent_m"] != .75:
        raise ValueError("unexpected visual-hole evidence")
    gates = []
    for item in source["gates"]:
        gates.append({"index": item["index"],
                      "center": vector(item["root_world_position_ros_m"]),
                      "normal": vector(item["gate_normal_ros"]),
                      "horizontal": vector(item["gate_horizontal_axis_ros"]),
                      "vertical": vector(item["gate_vertical_axis_ros"])})
    if [g["index"] for g in gates] != list(range(12)):
        raise ValueError("expected actual gate indices 0..11")
    return gates, region["half_extent_m"], actual


def verify_ros_gates(poses, expected, tolerance_m=1e-4):
    """Reject wrong scenes, reordered poses, and orientation convention errors."""
    if len(poses) != len(expected):
        raise ValueError("ROS gate count does not match official geometry evidence")
    for pose, gate in zip(poses, expected):
        center = vector(pose["position"])
        axes = quaternion_axes(pose["quaternion"])
        for value, actual in ((gate["center"], center), (gate["normal"], axes[0]),
                              (gate["horizontal"], axes[1]), (gate["vertical"], axes[2])):
            if max(abs(a-b) for a, b in zip(value, actual)) > tolerance_m:
                raise ValueError("ROS pose differs from verified gate {}".format(gate["index"]+1))


class GateTimer:
    """Incremental stopwatch; all times are actual odometry simulation stamps."""
    def __init__(self, gates, half_extent_m=.75, max_gap_s=.1, epsilon_m=1e-6, armed=False):
        if not gates or not finite(half_extent_m) or half_extent_m <= 0:
            raise ValueError("finite gate aperture and nonempty geometry required")
        if not finite(max_gap_s) or max_gap_s <= 0 or not finite(epsilon_m) or epsilon_m <= 0:
            raise ValueError("positive finite gap and sign tolerances required")
        self.gates = []
        for i, item in enumerate(gates):
            if item["index"] != i:
                raise ValueError("gate order must be contiguous from zero")
            gate = {"index": i}
            for key in ("center", "normal", "horizontal", "vertical"):
                gate[key] = vector(item[key])
            for key in ("normal", "horizontal", "vertical"):
                if abs(dot(gate[key], gate[key])-1) > 1e-5:
                    raise ValueError("gate basis must be unit length")
            for a, b in (("normal", "horizontal"), ("normal", "vertical"), ("horizontal", "vertical")):
                if abs(dot(gate[a], gate[b])) > 1e-5:
                    raise ValueError("gate basis must be orthogonal")
            self.gates.append(gate)
        self.half_extent = float(half_extent_m)
        self.max_gap, self.epsilon = float(max_gap_s), float(epsilon_m)
        self.previous = None
        self.planes = None
        self.events, self.credited, self.errors, self.collisions = [], [], [], []
        self.latest_time = None
        self.samples, self.duplicate_samples = 0, 0
        self.armed = bool(armed)
        self.arm_sim_s, self.arm_monotonic_s = None, None

    def arm(self, sim_time_s, monotonic_s=None):
        if self.armed:
            return
        if not finite(sim_time_s) or sim_time_s < 0 or (monotonic_s is not None and not finite(monotonic_s)):
            raise ValueError("invalid reference arming time")
        self.armed = True
        self.arm_sim_s, self.arm_monotonic_s = float(sim_time_s), monotonic_s
        self.previous, self.planes = None, None

    def side(self, distance):
        return -1 if distance < -self.epsilon else (1 if distance > self.epsilon else 0)

    def reset_planes(self, point):
        self.planes = [{"side": self.side(dot(g["normal"], minus(point, g["center"]))),
                        "pending": None} for g in self.gates]

    def fault(self, reason, sim_time_s=None):
        self.errors.append({"reason": reason, "sim_time_s": sim_time_s})
        self.previous, self.planes = None, None

    def observe(self, sim_time_s, point, monotonic_s=None):
        if not finite(sim_time_s) or sim_time_s < 0:
            self.fault("nonfinite or negative odometry simulation stamp")
            return []
        try:
            point = vector(point)
        except ValueError:
            self.fault("invalid odometry point", sim_time_s)
            return []
        now = float(sim_time_s)
        if monotonic_s is not None and not finite(monotonic_s):
            self.fault("invalid odometry receipt monotonic stamp", now)
            return []
        self.samples += 1
        if self.previous is not None:
            old_t, old_p, old_wall = self.previous
            if now == old_t and point == old_p:
                self.duplicate_samples += 1
                return []
            if now <= old_t:
                self.fault("odometry time rollback or conflicting duplicate", now)
                return []
            if old_wall is not None and monotonic_s is not None and monotonic_s <= old_wall:
                self.fault("odometry receipt monotonic time did not advance", now)
                return []
        if self.latest_time is not None and now < self.latest_time:
            self.fault("simulation timestamp rolled backwards", now)
            return []
        self.latest_time = now
        if not self.armed:
            # Reset, reposition and unrelated pre-reference motion cannot start
            # a race. Readiness may be reported while this stopwatch is unarmed.
            return []
        if self.previous is None:
            self.previous = (now, point, monotonic_s)
            self.reset_planes(point)
            return []
        old_t, old_p, old_wall = self.previous
        dt = now-old_t
        if dt > self.max_gap:
            self.errors.append({"reason": "odometry gap not interpolated", "start_sim_s": old_t,
                                "sim_time_s": now, "gap_s": dt})
            self.previous = (now, point, monotonic_s)
            self.reset_planes(point)
            return []
        found = []
        for gate, state in zip(self.gates, self.planes):
            d0 = dot(gate["normal"], minus(old_p, gate["center"]))
            d1 = dot(gate["normal"], minus(point, gate["center"]))
            if (d0 < 0 <= d1) or (d0 > 0 >= d1):
                direction = 1 if d1 > d0 else -1
                if state["pending"] is not None and state["pending"]["direction"] != direction:
                    state["pending"] = None
                if state["side"] == -direction:
                    alpha = -d0/(d1-d0)
                    intersection = tuple(a+alpha*(b-a) for a, b in zip(old_p, point))
                    local = minus(intersection, gate["center"])
                    h, v = dot(local, gate["horizontal"]), dot(local, gate["vertical"])
                    state["pending"] = {"gate_index": gate["index"], "gate_number": gate["index"]+1,
                                        "sim_time_s": old_t+alpha*dt, "direction": direction,
                                        "monotonic_estimate_s": (old_wall+alpha*(monotonic_s-old_wall)
                                                                 if old_wall is not None and monotonic_s is not None else None),
                                        "intersection_world_m": intersection,
                                        "horizontal_offset_m": h, "vertical_offset_m": v,
                                        "within_point_square": abs(h) <= self.half_extent and abs(v) <= self.half_extent,
                                        "sample_interval_s": dt, "interpolation_fraction": alpha}
            new_side = self.side(d1)
            if new_side and new_side != state["side"]:
                event = state["pending"]
                if event is not None:
                    event["confirmed_at_sim_s"] = now
                    found.append(event)
                state["side"], state["pending"] = new_side, None
            elif new_side:
                state["pending"] = None
        self.previous = (now, point, monotonic_s)
        found.sort(key=lambda e: (e["sim_time_s"], e["gate_index"]))
        for event in found:
            target = len(self.credited)
            event["credited"] = (event["gate_index"] == target and event["direction"] == 1
                                 and event["within_point_square"])
            if event["credited"]:
                self.credited.append(event)
            self.events.append(event)
        return found

    def collision(self, sim_time_s, is_collision, object_name=""):
        if not finite(sim_time_s) or sim_time_s < 0 or not isinstance(is_collision, bool):
            self.fault("invalid collision timestamp or flag")
            return None
        row = {"sim_time_s": float(sim_time_s), "is_collision": is_collision,
               "object_name": str(object_name)}
        self.collisions.append(row)
        return row

    def snapshot(self, current_sim_s=None, current_monotonic_s=None):
        start = self.credited[0]["sim_time_s"] if self.credited else None
        finish = self.credited[-1]["sim_time_s"] if len(self.credited) == len(self.gates) else None
        current = self.latest_time
        if finite(current_sim_s) and current is not None and current_sim_s >= current:
            current = float(current_sim_s)
        elapsed = 0.0 if start is None else max(0.0, (finish if finish is not None else current)-start)
        start_wall = self.credited[0]["monotonic_estimate_s"] if self.credited else None
        finish_wall = self.credited[-1]["monotonic_estimate_s"] if finish is not None else None
        wall_now = (current_monotonic_s if finite(current_monotonic_s) else
                    (self.previous[2] if self.previous is not None else None))
        elapsed_wall = (max(0.0, (finish_wall if finish_wall is not None else wall_now)-start_wall)
                        if start_wall is not None and wall_now is not None else None)
        between = sum(c["is_collision"] and start is not None and c["sim_time_s"] >= start
                      and (finish is None or c["sim_time_s"] <= finish) for c in self.collisions)
        return {"schema_version": 1, "status": "FINISHED" if finish is not None else ("RUNNING" if start is not None else "WAITING"),
                "valid_data": not self.errors, "time_basis": "simulation seconds from actual odometry stamps",
                "armed": self.armed, "arm_sim_s": self.arm_sim_s,
                "arm_monotonic_s": self.arm_monotonic_s,
                "rule": RULE, "start_gate_number": 1, "finish_gate_number": len(self.gates),
                "gates_passed": len(self.credited), "gates_total": len(self.gates),
                "start_sim_s": start, "finish_sim_s": finish, "elapsed_sim_s": elapsed,
                "start_monotonic_estimate_s": start_wall, "finish_monotonic_estimate_s": finish_wall,
                "elapsed_wall_estimate_s": elapsed_wall,
                "wall_estimate_note": "Interpolated odometry callback receipt monotonic stamps; callback/network delay remains. Frozen at last accepted gate.",
                "latest_odometry_sim_s": self.latest_time, "display_sim_s": current,
                "point_square_half_extent_m": self.half_extent,
                "point_rule_complete": finish is not None,
                "whole_body_clearance_verified": False, "official_score": None,
                "samples": self.samples, "duplicate_samples_ignored": self.duplicate_samples,
                "data_errors": self.errors, "collision_messages": len(self.collisions),
                "true_collision_messages": sum(c["is_collision"] for c in self.collisions),
                "true_collision_messages_first_to_last": between,
                "credited_crossings": self.credited}


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name+".tmp")
    tmp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    os.replace(str(tmp), str(path))


def ros_main(args):
    import rospy
    from geometry_msgs.msg import PoseArray
    from nav_msgs.msg import Odometry
    from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
    from tesse_ros_bridge.msg import CollisionStats
    if not rospy.get_param("/use_sim_time", False):
        raise RuntimeError("stopwatch requires the already-enabled actual simulator clock")
    gates, extent, sha = load_geometry_evidence(args.geometry_evidence)
    rospy.init_node("lab4_gate_timer", anonymous=False)
    poses = rospy.wait_for_message("/desired_traj_vertices", PoseArray, timeout=10)
    convert = lambda p: {"position": [p.position.x, p.position.y, p.position.z],
                         "quaternion": [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}
    verify_ros_gates([convert(p) for p in poses.poses], gates)
    timer, lock = GateTimer(gates, extent, args.max_gap_s), threading.RLock()
    out = args.out_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    for name in ("gate_timer.json", "gate_timer_events.jsonl", "gate_timer_collisions.jsonl", "gate_timer.ready.json"):
        if (out/name).exists():
            raise RuntimeError("refusing to overwrite previous timer output: "+name)
    event_file = (out/"gate_timer_events.jsonl").open("x", encoding="utf-8")
    collision_file = (out/"gate_timer_collisions.jsonl").open("x", encoding="utf-8")
    def append(handle, item):
        handle.write(json.dumps(item, ensure_ascii=False, allow_nan=False)+"\n")
        handle.flush()
    def received(msg):
        p = msg.pose.pose.position
        with lock:
            received_wall = time.monotonic()
            for row in timer.observe(msg.header.stamp.to_sec(), [p.x, p.y, p.z], received_wall):
                row["callback_monotonic_s"] = received_wall
                append(event_file, row)
    def reference(msg):
        with lock:
            timer.arm(rospy.Time.now().to_sec(), time.monotonic())
    def contact(msg):
        with lock:
            row = timer.collision(msg.header.stamp.to_sec(), bool(msg.is_collision), msg.object_name)
            if row is not None:
                row["callback_monotonic_s"] = time.monotonic()
                append(collision_file, row)
    def updated_poses(msg):
        with lock:
            try:
                verify_ros_gates([convert(p) for p in msg.poses], gates)
            except ValueError as exc:
                timer.fault(str(exc))
    subscribers = [rospy.Subscriber("/current_state", Odometry, received, queue_size=2000),
                   rospy.Subscriber("/tesse/collision", CollisionStats, contact, queue_size=2000),
                   rospy.Subscriber("/desired_traj_vertices", PoseArray, updated_poses, queue_size=10),
                   rospy.Subscriber("/desired_state", MultiDOFJointTrajectoryPoint, reference, queue_size=10)]
    window = None
    if args.show_window:
        import tkinter as tk
        window = tk.Tk()
        window.title("LAB4 Gate 1 to 12 Timer")
        window.geometry(args.window_geometry)
        window.configure(bg="#111827")
        window.attributes("-topmost", True)
        window.resizable(False, False)
        display = tk.Label(window, font=("DejaVu Sans", 26, "bold"), fg="#40e99c", bg="#111827")
        display.pack(expand=True)
        label = tk.Label(window, font=("DejaVu Sans", 10), fg="white", bg="#111827")
        label.pack(pady=(0, 6))
        window.protocol("WM_DELETE_WINDOW", window.withdraw)
    def save():
        with lock:
            state = timer.snapshot(rospy.Time.now().to_sec(), time.monotonic())
        state.update(geometry_evidence_sha256=sha, source="/current_state header.stamp; verified /desired_traj_vertices",
                     observer_monotonic_s=time.monotonic(), live_window_requested=args.show_window)
        atomic_json(out/"gate_timer.json", state)
        return state
    try:
        wait_end = time.monotonic()+5
        while timer.latest_time is None and time.monotonic() < wait_end and not rospy.is_shutdown():
            time.sleep(.02)
        if timer.latest_time is None:
            raise RuntimeError("no actual odometry received before stopwatch readiness")
        atomic_json(out/"gate_timer.ready.json", {"ready": True, "geometry_evidence_sha256": sha,
                    "gates": len(gates), "time_basis": "simulation", "rule": RULE})
        while not rospy.is_shutdown():
            state = save()
            if window is not None:
                real = state["elapsed_wall_estimate_s"]
                display.configure(text="SIM {:.3f} s".format(state["elapsed_sim_s"]))
                quality = state["status"] if state["valid_data"] else "DATA ERROR"
                real_text = "--" if real is None else "{:.3f} s".format(real)
                hits = state["true_collision_messages_first_to_last"]
                color = "#ff6666" if hits or not state["valid_data"] else "#40e99c"
                display.configure(fg=color)
                label.configure(fg=color, text="{} | GATES {}/{} | REAL {} | HITS {}".format(
                    quality, state["gates_passed"], state["gates_total"], real_text, hits))
                window.update()
            time.sleep(.04)
    finally:
        save()
        for sub in subscribers:
            sub.unregister()
        event_file.close()
        collision_file.close()
        if window is not None:
            window.destroy()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--geometry-evidence", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--max-gap-s", type=float, default=.1)
    parser.add_argument("--show-window", action="store_true", help="separate live Tk window; not assumed included in Unity-only capture")
    parser.add_argument("--window-geometry", default="400x110+560+0")
    args = parser.parse_args(argv)
    ros_main(args)


if __name__ == "__main__":
    main()
