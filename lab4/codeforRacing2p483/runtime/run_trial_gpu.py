#!/usr/bin/env python3
"""Bounded native WSLg adapter; the copied algorithms/assets are never edited.

Only this adapter launches processes. Absolute ROS topics are isolated by a
fresh, loopback-only master and exclusive TESSE ports, not misleading remaps.
An ordered gate-plane screen is NOT an official score or body-clearance proof.
"""
import argparse
import csv
import errno
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import zipfile

PLAN_REL = "optimized/balanced_t494_h25_v25_clean.csv"
PLAN_SHA256 = "e1883baf87bba068a60445275db3e70d00999303eac137dcc0b0d83486c7dad2"
ZIP_SHA256 = "27e4fe0d2905eb82730dfb11b6eacb8a6adb12d4741ede3ee4e6f65615504915"
EXACT = dict(kx=36.0, kv=12.0, kr=60.0, kw=4.0, signed=True,
             model_inertia=True, no_ff=False, free_heading=True,
             log_error=False, ff_gain=1, ff_tau=.02, omega_limit=6,
             preview=0, old_controller=False, trajectory_mode="offline_csv")
TOPICS = ["/current_state", "/desired_state", "/desired_traj_vertices", "/clock",
          "/rotor_speed_cmds", "/tesse/collision", "/sprint_acceleration_preview", "/rosout"]
DISCLAIMER = "Not an official race score or a whole-body swept-clearance certificate."
CAPTURE_READY_SECONDS = 5.0
CAPTURE_WAIT_SECONDS = 25.0
CAPTURE_FINALIZATION_RESERVE = 25.0


def digest(path):
    with Path(path).open("rb") as stream:
        return hash_stream(stream)


def hash_stream(stream):
    result = hashlib.sha256()
    for block in iter(lambda: stream.read(1024 * 1024), b""):
        result.update(block)
    return result.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def safe_relative(name):
    value = PurePosixPath(name)
    if (not name or not value.parts or value.is_absolute() or ".." in value.parts or "\\" in name
            or ":" in value.parts[0] or value.as_posix() != name):
        raise ValueError("Unsafe manifest/archive path: " + name)
    return value


def load_exact_profile(root):
    profile = json.loads((Path(root) / "original_parameters/profile.json").read_text())
    for key, expected in EXACT.items():
        actual = profile.get(key)
        if actual != expected or isinstance(actual, bool) != isinstance(expected, bool):
            raise ValueError("final_balanced494 profile mismatch: {}={!r}".format(key, actual))
    if profile.get("name") != "final_balanced494" or profile.get("offline_plan_sha256") != PLAN_SHA256:
        raise ValueError("Wrong source profile or saved-reference identity")
    return profile


def verify_plan(path):
    if digest(path) != PLAN_SHA256:
        raise ValueError("Saved reference SHA256 mismatch")
    with Path(path).open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    keys = ("t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az")
    numbers = [[float(row[key]) for key in keys] for row in rows]
    if len(numbers) < 2 or any(not all(math.isfinite(v) for v in row) for row in numbers):
        raise ValueError("Non-finite or missing saved-reference data")
    if abs(numbers[0][0]) > 1e-9 or any(b[0] - a[0] < 1e-6 for a, b in zip(numbers, numbers[1:])):
        raise ValueError("Invalid saved-reference clock knots")
    return {"sha256": PLAN_SHA256, "rows": len(rows), "duration_s": numbers[-1][0]}


def verify_manifest(root):
    root = Path(root).resolve()
    manifest = json.loads((root / "MANIFEST.json").read_text())
    checked = {}
    for item in manifest["files"]:
        relative = safe_relative(item["path"])
        path = root / relative
        if not path.is_file() or path.is_symlink() or root not in path.resolve().parents:
            raise ValueError("Missing or redirected payload file: " + str(relative))
        actual = digest(path)
        if actual != item["sha256"]:
            raise ValueError("Copied source/data changed: " + str(relative))
        checked[str(relative)] = actual
    return checked


def find_simulator_zip(root, explicit=None):
    if explicit:
        return Path(explicit).resolve()
    for path in (Path(root) / "lab4_simulator.zip", Path(root) / "vendor/lab4_simulator.zip",
                 Path(root) / "simulator/lab4_simulator.zip"):
        if path.is_file():
            return path
    raise ValueError("Keep the verified official ZIP and pass --simulator-zip PATH")


def verify_simulator(root, archive):
    if digest(archive) != ZIP_SHA256:
        raise ValueError("Official simulator ZIP SHA256 mismatch")
    folder = Path(root) / "simulator/lab4"
    checked = {}
    with zipfile.ZipFile(archive) as zipped:
        players = [n for n in zipped.namelist() if n.endswith("lab4.x86_64")]
        if len(players) != 1:
            raise ValueError("Unexpected official ZIP player layout")
        prefix = players[0][:-len("lab4.x86_64")]
        for item in zipped.infolist():
            if item.is_dir():
                continue
            safe_relative(item.filename)
            if not item.filename.startswith(prefix):
                raise ValueError("Unexpected file outside simulator ZIP prefix")
            relative = safe_relative(item.filename[len(prefix):])
            destination = folder / relative
            if not destination.is_file() or destination.is_symlink() or folder.resolve() not in destination.resolve().parents:
                raise ValueError("Missing or redirected official asset: " + str(relative))
            with zipped.open(item) as stream:
                expected = hash_stream(stream)
            if digest(destination) != expected:
                raise ValueError("Official simulator asset differs: " + str(relative))
            checked[str(relative)] = expected
    player = folder / "lab4.x86_64"
    with player.open("rb") as stream:
        header = stream.read(20)
    if len(header) < 20 or header[:6] != b"\x7fELF\x02\x01" or struct.unpack("<H", header[18:20])[0] != 62:
        raise ValueError("Official player must be native little-endian x86-64 ELF")
    if not os.access(player, os.X_OK):
        raise ValueError("Official native player is not executable; verify extraction permissions")
    return checked


def renderer_report(text):
    lines = [line.strip() for line in text.splitlines()
             if re.search(r"\bRenderer\s*:", line, re.I)]
    joined = "\n".join(lines).lower()
    software = any(token in text.lower() for token in ("llvmpipe", "softpipe", "software rasterizer", "swrast"))
    return {"renderer_lines": lines, "software_renderer_detected": software,
            "hardware_renderer_verified": bool(lines) and "d3d12" in joined and "nvidia" in joined and not software}


def assert_ports_free():
    # Binding a temporary socket is only a collision check, not a server launch.
    checks = [(11311, socket.SOCK_STREAM)]
    checks += [(port, kind) for port in (9000, 9001, 9002, 9004, 9005, 9006, 9007)
               for kind in (socket.SOCK_STREAM, socket.SOCK_DGRAM)]
    for port, kind in checks:
        with socket.socket(socket.AF_INET, kind) as handle:
            try:
                # Ignore eligible closed TCP TIME_WAIT connections, not live
                # listeners. Never relax UDP exclusivity or use SO_REUSEPORT.
                if kind == socket.SOCK_STREAM:
                    handle.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                handle.bind(("0.0.0.0", port))
            except OSError as exc:
                raise RuntimeError("Port {} {} already in use; refusing to share/kill it".format(
                    port, "TCP" if kind == socket.SOCK_STREAM else "UDP")) from exc
    if socket.has_ipv6:
        for port, kind in checks:
            try:
                with socket.socket(socket.AF_INET6, kind) as handle:
                    handle.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 1)
                    if kind == socket.SOCK_STREAM:
                        handle.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    handle.bind(("::", port))
            except OSError as exc:
                if exc.errno in (errno.EAFNOSUPPORT, errno.EPROTONOSUPPORT, errno.EADDRNOTAVAIL):
                    continue  # IPv6 is disabled on this WSL instance.
                raise RuntimeError("IPv6 port {} already in use; refusing to share/kill it".format(port)) from exc
    if Path("/proc").is_dir():
        for directory in Path("/proc").iterdir():
            if not directory.name.isdigit() or int(directory.name) == os.getpid():
                continue
            try:
                executable = (directory / "cmdline").read_bytes().split(b"\0", 1)[0]
            except (OSError, PermissionError):
                continue
            if Path(os.fsdecode(executable)).name.lower() == "lab4.x86_64":
                raise RuntimeError("An existing official player is running; refusing to share/kill it")


def build_commands(root, out, profile, width=960, height=600, rviz=False,
                   capture=False, sim_seconds=12.0,
                   single_threaded_rendering=False, glcore33=False):
    root, out = Path(root), Path(out)
    sim = root / "simulator/lab4"
    commands = {
        "roscore": ["roscore", "-p", "11311"],
        "player": [str(sim / "lab4.x86_64"), "-force-glcore33" if glcore33 else "-force-glcore", "-noaudio", "-screen-fullscreen", "0",
                   "-screen-width", str(width), "-screen-height", str(height),
                   "-screen-quality", "Very Low", "-logFile", str(out / "player_renderer.log")],
        # Let roslaunch reap its separately grouped nodes before our 3s SIGINT
        # grace expires; default 15s/2s would outlive the owning launch process.
        "bridge": ["roslaunch", "tesse_ros_bridge", "tesse_quadrotor_bridge.launch",
                   "--sigint-timeout=0.5", "--sigterm-timeout=0.5", "ns:=tesse",
                   "publish_imu:=false", "publish_collisions:=true", "enable_collision:=true",
                   "use_sim_time:=true", "publish_clock:=true", "enable_step_mode:=false",
                   "initial_scene:=1", "sim_ip:=127.0.0.1", "self_ip:=127.0.0.1", "use_broadcast:=true"],
        "clock": [sys.executable, "-B", str(root / "scripts/clock_observer.py"), str(out / "clock_wall.jsonl")],
        "bag": ["rosbag", "record", "-b", "64", "-O", str(out / "race.bag")] + TOPICS,
        "controller": ["rosrun", "controller_sprint", "controller_sprint_node",
                       "_kx:=36.0", "_kv:=12.0", "_kr:=60.0", "_komega:=4.0", "_control_hz:=1000",
                       "_use_rate_feedforward:=true", "_use_accel_feedforward:=false",
                       "_signed_attitude:=true", "_free_heading:=true", "_use_attitude_preview:=false",
                       "_rate_feedforward_gain:=1", "_omega_d_filter_tau:=0.02", "_omega_d_limit:=6",
                       "_log_attitude_error:=false", "_Jx:=0.1", "_Jy:=0.2", "_Jz:=0.1",
                       "_feedforward_reference_only:=true", "_signed_attitude_hysteresis:=0.2",
                       "_accel_feedforward_gain:=1", "_omega_dot_d_limit:=40",
                       "_omega_dot_d_filter_tau:=0.04", "_derivative_min_dt:=0.0005", "_derivative_reset_dt:=0.2"],
        "reference": [sys.executable, "-B", str(root / "scripts/plan_reference.py"),
                      "_plan:=" + str(root / PLAN_REL), "_dump_file:=" + str(out / "plan.csv"),
                      "_attitude_preview_s:=0"],
        "gates": ["rosrun", "planner_pkg", "traj_vertices_publisher",
                  "_simulator_data_directory:=" + str(sim / "lab4_Data")],
    }
    if single_threaded_rendering:
        commands["player"].append("-force-gfx-direct")
    if rviz:
        commands["rviz"] = ["rviz"]
    if capture:
        commands["capture"] = [sys.executable, "-B", str(root / "runtime/capture_x11_window.py"),
                               "--window-name", "tesse_vnav_scenes",
                               "--output", str(out / "scene.mp4"),
                               "--duration", str(min(30.0, sim_seconds + 3.0)),
                               "--fps", "60", "--ready-file", str(out / "capture.ready.json"),
                               "--backing-path", "/mnt/c", "--encode-timeout", "20"]
    # Reject callers trying to route a different controller profile through here.
    if any(profile.get(key) != value for key, value in EXACT.items()):
        raise ValueError("Only final_balanced494 is supported")
    return commands


def _capture_regular(path, description):
    path = Path(path)
    if path.is_symlink() or not path.is_file() or path.stat().st_size == 0:
        raise ValueError("Missing, empty or redirected " + description + ": " + str(path))
    return path


def load_capture_ready(path, video_path):
    """Wait-safe readiness parser; partial JSON never starts the reference."""
    path, video_path = Path(path), Path(video_path)
    if not path.exists():
        return None
    if path.is_file() and not path.is_symlink() and path.stat().st_size == 0:
        return None
    _capture_regular(path, "capture readiness file")
    try:
        report = json.loads(path.read_text())
    except json.JSONDecodeError:
        return None  # Helper closes its exclusive JSON file immediately after writing.
    if not isinstance(report, dict) or report.get("ready") is not True:
        raise ValueError("Capture helper did not report a successful first frame")
    if Path(report.get("video_path", "")).resolve() != video_path.resolve():
        raise ValueError("Capture readiness refers to another video")
    stamp = report.get("first_frame_monotonic_ns")
    if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
        raise ValueError("Capture readiness has no valid monotonic first-frame timestamp")
    artifacts = Path(str(video_path) + ".capture")
    expected = artifacts / "frames/frame_000000.ppm"
    if artifacts.is_symlink() or expected.parent.is_symlink():
        raise ValueError("Redirected capture frame directory")
    if Path(report.get("first_frame_file", "")).resolve() != expected.resolve():
        raise ValueError("Capture readiness refers to another first frame")
    _capture_regular(expected, "first real captured frame")
    return report


def validate_capture_report(video_path, returncode):
    """A helper exit and container FPS label alone cannot certify a video."""
    if isinstance(returncode, bool) or returncode != 0:
        raise ValueError("Capture helper exited unsuccessfully: " + str(returncode))
    video_path = _capture_regular(video_path, "captured MP4")
    artifacts = Path(str(video_path) + ".capture")
    if artifacts.is_symlink():
        raise ValueError("Redirected capture artifacts directory")
    report_path = _capture_regular(artifacts / "capture_report.json", "capture validation report")
    report = json.loads(report_path.read_text())
    if not isinstance(report, dict) or report.get("status") != "complete":
        raise ValueError("Capture report is not complete")
    if Path(report.get("video_path", "")).resolve() != video_path.resolve():
        raise ValueError("Capture report refers to another video")
    if report.get("duration_verified") is not True or report.get("frame_timestamps_verified") is not True:
        raise ValueError("Capture duration and frame PTS were not both verified")
    span, duration = report.get("capture_span_s"), report.get("ffprobe_duration_s")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0
           for v in (span, duration)) or abs(span - duration) > .10:
        raise ValueError("Capture/report durations do not match monotonic capture span")
    frames, decoded = report.get("frame_count"), report.get("decoded_frame_count")
    if (isinstance(frames, bool) or not isinstance(frames, int) or frames <= 0
            or isinstance(decoded, bool) or not isinstance(decoded, int) or decoded != frames + 1):
        raise ValueError("Capture frame count does not include exactly one end-boundary sentinel")
    pts_error = report.get("maximum_frame_timestamp_error_s")
    if (isinstance(pts_error, bool) or not isinstance(pts_error, (int, float))
            or not math.isfinite(pts_error) or not 0 <= pts_error <= .010):
        raise ValueError("Capture frame PTS numerical error is missing or exceeds 10 ms")
    return report


def capture_wall_remaining(args, status):
    # The existing hard watchdog fires two seconds before wall_timeout.
    return status["start_monotonic_s"] + args.wall_timeout - 2.0 - time.monotonic()


def check_capture_exit(child):
    result = child.poll()
    if result is not None and result != 0:
        raise RuntimeError("Optional capture helper exited nonzero ({}); inspect capture.log and .capture report".format(result))
    return result


def start_capture(out, args, commands, processes, status):
    duration = min(30.0, args.sim_seconds + 3.0)
    needed = CAPTURE_READY_SECONDS + duration + CAPTURE_WAIT_SECONDS + CAPTURE_FINALIZATION_RESERVE
    remaining = capture_wall_remaining(args, status)
    if remaining < needed:
        raise RuntimeError("Insufficient total wall budget for requested capture: {:.2f}s remains, {:.2f}s reserved for capture/encoding/finalization".format(remaining, needed))
    status["capture"] = {"requested": True, "status": "STARTING", "essential": False,
                         "duration_s": duration, "fps": 60,
                         "remaining_wall_s_at_start": remaining,
                         "post_collection_wait_limit_s": CAPTURE_WAIT_SECONDS}
    child = processes.start("capture", commands["capture"], essential=False)
    deadline = time.monotonic() + CAPTURE_READY_SECONDS
    while time.monotonic() < deadline:
        processes.check()
        result = check_capture_exit(child)
        if result == 0:
            raise RuntimeError("Capture ended before first-frame readiness/reference start")
        ready = load_capture_ready(out / "capture.ready.json", out / "scene.mp4")
        if ready:
            status["capture"].update(status="CAPTURING", ready=ready)
            write_json(out / "video_capture.json", status["capture"])
            return child
        time.sleep(.05)
    raise RuntimeError("Timed out after 5s waiting for capture's first real frame; reference was not started")


def finish_capture(out, args, child, processes, status):
    status["capture"]["status"] = "WAITING_FOR_NORMAL_CAPTURE_AND_ENCODING"
    available = capture_wall_remaining(args, status) - CAPTURE_FINALIZATION_RESERVE
    wait_seconds = min(CAPTURE_WAIT_SECONDS, max(0.0, available))
    started = time.monotonic()
    deadline = started + wait_seconds
    while True:
        processes.check()
        result = check_capture_exit(child)
        if result == 0:
            report = validate_capture_report(out / "scene.mp4", result)
            # Successful helper has already joined its encoder and probe; do
            # not send the generic 3s cleanup escalation to its finished group.
            processes.closed_groups.add(child.pid)
            status["capture"].update(status="VERIFIED", helper_exit_code=result,
                                     waited_after_collection_wall_s=time.monotonic() - started,
                                     helper_report=report)
            status["video_recorded"] = True
            write_json(out / "video_capture.json", status["capture"])
            return report
        if time.monotonic() >= deadline:
            if available < CAPTURE_WAIT_SECONDS:
                raise RuntimeError("Total wall budget leaves only {:.2f}s for normal capture/encoding while retaining finalization reserve; video not certified".format(wait_seconds))
            raise RuntimeError("Capture/encoding did not finish within 25s after collection; video not certified")
        time.sleep(.05)


class Processes:
    def __init__(self, out, environment):
        self.out, self.environment = Path(out), environment
        self.children, self.logs, self.commands = [], [], []
        self.closed_groups = set()

    def start(self, name, command, essential=True, cwd=None):
        log = (self.out / (name + ".log")).open("w", encoding="utf-8")
        self.logs.append(log)
        child = subprocess.Popen(command, cwd=cwd, env=self.environment, stdout=log,
                                 stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append((name, child, essential))
        self.commands.append(dict(name=name, argv=command, pid=child.pid, pgid=child.pid))
        write_json(self.out / "processes.json", self.commands)
        return child

    def check(self):
        for name, child, essential in self.children:
            if essential and child.poll() is not None:
                raise RuntimeError("Owned subprocess {} exited early ({})".format(name, child.returncode))

    def signal_groups(self, signum, selected=None):
        for name, child, _ in reversed(self.children):
            if selected is not None and name not in selected:
                continue
            if child.pid in self.closed_groups:
                continue
            # Only PGIDs created by our own start_new_session Popen calls.
            try:
                os.killpg(child.pid, signum)
            except ProcessLookupError:
                pass

    def stop(self, selected=None):
        self.signal_groups(signal.SIGINT, selected)
        chosen = [(n, p) for n, p, _ in self.children if selected is None or n in selected]
        for sig, limit in ((signal.SIGINT, 3.0), (signal.SIGTERM, 1.0), (signal.SIGKILL, .5)):
            if sig != signal.SIGINT:
                self.signal_groups(sig, selected)
            deadline = time.monotonic() + limit
            for _, child in chosen:
                try:
                    child.wait(timeout=max(.001, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
        self.closed_groups.update(child.pid for _, child in chosen)

    def run(self, name, command, timeout=8):
        child = self.start(name, command, essential=False)
        try:
            result = child.wait(timeout=timeout)
            self.closed_groups.add(child.pid)
            return result
        except subprocess.TimeoutExpired as exc:
            self.stop({name})
            raise RuntimeError("{} exceeded its finite timeout".format(name)) from exc

    def close(self):
        for log in self.logs:
            log.close()


def wait_until(predicate, processes, seconds, description):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        processes.check()
        result = predicate()
        if result:
            return result
        time.sleep(.05)
    raise RuntimeError("Timed out waiting for " + description)


def master_ready():
    try:
        with socket.create_connection(("127.0.0.1", 11311), timeout=.2):
            return True
    except OSError:
        return False


def initialize_sim_observer(rospy_module, processes):
    # rospy chooses its time source during init_node, not on later parameter changes.
    # This command uses our isolated ROS_MASTER_URI and is bounded/logged like all
    # other owned children. No observer may sample wall time before /clock exists.
    result = processes.run("use_sim_time", ["rosparam", "set", "/use_sim_time", "true"], 5)
    if result != 0:
        raise RuntimeError("Could not enable simulator time before observer initialization")
    rospy_module.init_node("lab4_gpu_trial_observer", anonymous=True, disable_signals=True)
    if rospy_module.rostime.is_wallclock():
        raise RuntimeError("Observer initialized with wall time instead of simulator time")


def collect(root, out, args, profile, commands, processes, status):
    import rospy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseArray
    from rosgraph_msgs.msg import Clock
    from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
    from tesse.env import Env
    from tesse.msgs import MetadataRequest, PropSpeeds, Respawn, Reposition

    # Recheck immediately before launching, after the potentially slower asset audit.
    assert_ports_free()
    processes.start("roscore", commands["roscore"])
    wait_until(master_ready, processes, 8, "our ROS master")
    initialize_sim_observer(rospy, processes)
    processes.start("player", commands["player"], cwd=root / "simulator/lab4")
    env = Env(simulation_ip="127.0.0.1", own_ip="127.0.0.1")
    metadata_end = time.monotonic() + 15
    while time.monotonic() < metadata_end:
        processes.check()
        response = env.request(MetadataRequest(), timeout=.5)
        if response is not None:
            metadata = response.metadata
            if isinstance(metadata, bytes):
                metadata = metadata.decode("utf-8", errors="replace")
            (out / "initial_metadata.xml").write_text(metadata, encoding="utf-8")
            break
    else:
        raise RuntimeError("Official player metadata did not become ready")
    def renderer_ready():
        path = out / "player_renderer.log"
        if path.is_file():
            report = renderer_report(path.read_text(errors="replace"))
            if report["renderer_lines"]:
                return report
        return None
    renderer = wait_until(renderer_ready, processes, 5, "Unity renderer log")
    write_json(out / "renderer.json", renderer)
    if not renderer["hardware_renderer_verified"]:
        raise RuntimeError("Unity renderer must identify D3D12/NVIDIA with no software renderer")
    processes.start("bridge", commands["bridge"])
    rospy.wait_for_message("/clock", Clock, timeout=12)
    processes.check()
    rospy.wait_for_message("/current_state", Odometry, timeout=8)
    processes.start("clock", commands["clock"])
    processes.start("bag", commands["bag"])
    time.sleep(.7)
    processes.check()
    # Keep the exact original trial.py initialization sequence and quaternion.
    resets = []
    for _ in range(3):
        env.send(PropSpeeds(0, 0, 0, 0)); time.sleep(.05)
    resets.append(dict(operation="PropSpeeds(0,0,0,0) x3", wall_monotonic_s=time.monotonic()))
    env.send(Respawn()); time.sleep(.5)
    resets.append(dict(operation="Respawn", wall_monotonic_s=time.monotonic()))
    env.send(Reposition(.4, .22, 8, 0, .9929119, 0, .1188524)); time.sleep(.6)
    env.send(PropSpeeds(0, 0, 0, 0))
    resets.append(dict(operation="Reposition", arguments=[.4, .22, 8, 0, .9929119, 0, .1188524],
                       wall_monotonic_s=time.monotonic()))
    write_json(out / "initialization.json", {"original_trial_sequence": resets,
               "bag_includes_reset_and_start_contacts": True})
    current = rospy.wait_for_message("/current_state", Odometry, timeout=5)
    (out / "initial_odometry.txt").write_text(str(current) + "\n")
    for key, value in (("kx", 36), ("kv", 12), ("kr", 60), ("komega", 4), ("control_hz", 1000)):
        rospy.set_param("/" + key, value)
    # A newly-owned master has no old private namespace to delete or reuse.
    processes.start("gates", commands["gates"])
    gates = rospy.wait_for_message("/desired_traj_vertices", PoseArray, timeout=8)
    if len(gates.poses) != 12:
        raise RuntimeError("Expected the official 12 gate poses, received {}".format(len(gates.poses)))
    (out / "gates.txt").write_text(str(gates) + "\n")
    progress = {"first": None, "last": None, "reference_t": None, "clock": None}
    clocks = []
    def clock_received(message):
        now = message.clock.to_sec()
        if progress["clock"] is not None and now < progress["clock"] - 1e-8:
            clocks.append("Simulator clock moved backwards")
        progress["clock"] = now
    def desired(message):
        now = rospy.Time.now().to_sec()
        if progress["first"] is None:
            progress["first"] = now
        progress["last"], progress["reference_t"] = now, message.time_from_start.to_sec()
    subscribers = [rospy.Subscriber("/clock", Clock, clock_received, queue_size=100),
                   rospy.Subscriber("/desired_state", MultiDOFJointTrajectoryPoint, desired, queue_size=10)]
    if args.rviz:
        processes.start("rviz", commands["rviz"], essential=False)
    processes.start("controller", commands["controller"])
    capture_child = None
    try:
        wait_until(lambda: rospy.has_param("/controller_sprint/kx"), processes, 4, "controller private parameters")
        if processes.run("start_parameters", ["rosparam", "dump", str(out / "start_parameters.yaml")], 5) != 0:
            raise RuntimeError("Could not record initial controller/bridge parameters")
        if args.capture:
            capture_child = start_capture(out, args, commands, processes, status)
        # No reference can start until the optional real-window first frame is
        # complete. Default/no-capture keeps the original launch sequence.
        processes.start("reference", commands["reference"])
        def first_reference_ready():
            if capture_child is not None:
                check_capture_exit(capture_child)
            return progress["first"] is not None
        wait_until(first_reference_ready, processes, 10, "first saved-reference state")
        end = time.monotonic() + min(40, args.wall_timeout - 15)
        if args.capture:
            end = min(end, time.monotonic() + max(0, capture_wall_remaining(args, status) - CAPTURE_FINALIZATION_RESERVE))
        last_advance, last_clock = time.monotonic(), progress["clock"]
        while time.monotonic() < end:
            processes.check()
            if capture_child is not None:
                check_capture_exit(capture_child)  # essential=False still must not hide failure.
            if clocks:
                raise RuntimeError(clocks[0])
            now = progress["clock"]
            if now is not None and now != last_clock:
                last_advance, last_clock = time.monotonic(), now
            if time.monotonic() - last_advance > 3:
                raise RuntimeError("Simulator clock stopped advancing")
            if now is not None and now - progress["first"] >= args.sim_seconds:
                break
            time.sleep(.05)
        else:
            raise RuntimeError("Collection did not reach the requested simulated duration within the remaining wall budget")
        status["collection_complete"] = True
        write_json(out / "collection.json", {"progress": progress, "requested_sim_seconds": args.sim_seconds,
                   "source_start_sim": rospy.get_param("/sprint_reference_start", None), "clock_errors": clocks,
                   "collision_reporting_parameters": {"publish_collisions": True, "enable_collision": True},
                   "all_clock_and_start_contacts_retained": True})
        if capture_child is not None:
            # Leave reference, controller, bag and clock alive through capture
            # completion and its bounded post-encoding interval. Generic 3s
            # cleanup is NOT used as a normal way to end video encoding.
            finish_capture(out, args, capture_child, processes, status)
            if clocks:
                raise RuntimeError(clocks[0])
        if processes.run("parameters", ["rosparam", "dump", str(out / "parameters.yaml")], 5) != 0:
            raise RuntimeError("Could not record same-run ROS parameters")
        if processes.run("nodes", ["rosnode", "list"], 5) != 0:
            raise RuntimeError("Could not record same-run ROS node list")
    except BaseException as error:
        if args.capture:
            status.setdefault("capture", {"requested": True})
            status["capture"].update(failure=type(error).__name__ + ": " + str(error))
            if not status["video_recorded"]:
                status["capture"]["status"] = "FAILED_OR_UNVERIFIED"
            write_json(out / "video_capture.json", status["capture"])
        raise
    finally:
        # Even a failed optional capture must finalize bag/clock while the
        # terminal controller still holds, before any motor-off cleanup.
        processes.stop({"bag", "clock"})
        processes.stop({"reference", "controller", "gates", "rviz"})
        env.send(PropSpeeds(0, 0, 0, 0))
        for subscriber in subscribers:
            subscriber.unregister()


def ensure_environment(root):
    if os.environ.get("LAB4_GPU_ENV_READY") == str(root):
        if (os.environ.get("CMAKE_PREFIX_PATH", "").split(":") != [str(root / "ros_ws/devel"), "/opt/ros/noetic"]
                or os.environ.get("ROS_MASTER_URI") != "http://127.0.0.1:11311"
                or os.environ.get("ROS_IP") != "127.0.0.1"):
            raise RuntimeError("Unexpected ROS overlay/master after env.sh; use a fresh adapter process")
        return
    script = Path(__file__).resolve()
    environment = os.environ.copy()
    environment["LAB4_GPU_ROOT"] = str(root)
    os.execve("/bin/bash", ["bash", "--noprofile", "--norc", "-c",
              'source "$1"; exec /usr/bin/python3 -B "$2" "${@:3}"',
              "lab4-gpu", str(script.with_name("env.sh")), str(script)] + sys.argv[1:], environment)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("name", help="fresh run name; never overwritten")
    parser.add_argument("--root", type=Path, default=Path(os.environ.get("LAB4_GPU_ROOT", Path(__file__).resolve().parent.parent)))
    parser.add_argument("--simulator-zip", type=Path)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=600)
    parser.add_argument("--sim-seconds", type=float, choices=(12.0, 18.0), default=12.0)
    parser.add_argument("--wall-timeout", type=float, default=105.0)
    parser.add_argument("--rviz", action="store_true")
    parser.add_argument("--capture", action="store_true", help="optional real Unity-window VFR recording; default remains no video")
    parser.add_argument("--single-threaded-rendering", action="store_true",
                        help="optional Unity -force-gfx-direct rendering-thread compatibility test; default unchanged")
    parser.add_argument("--glcore33", action="store_true",
                        help="optional -force-glcore33 instead of automatic -force-glcore; default unchanged")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9_-]+", args.name):
        parser.error("Use a unique alphanumeric/_/- run name")
    if not 320 <= args.width <= 3840 or not 240 <= args.height <= 2160:
        parser.error("Window dimensions out of range")
    if args.capture and (args.width > 960 or args.height > 600 or args.width % 2 or args.height % 2):
        parser.error("--capture requires unchanged even window dimensions at most 960x600")
    if not 60 <= args.wall_timeout <= 110:
        parser.error("Total wall timeout must be 60..110 seconds (<120 including emergency cleanup)")
    root = args.root.resolve()
    if platform.system() != "Linux" or platform.machine().lower() not in ("x86_64", "amd64"):
        parser.error("Actual runs require native x86-64 Linux/WSL; stdlib tests work on Mac")
    ensure_environment(root)
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("No WSLg DISPLAY: validate the actual graphical environment first")
    for variable in ("GALLIUM_DRIVER", "MESA_LOADER_DRIVER_OVERRIDE"):
        if any(token in os.environ.get(variable, "").lower() for token in ("llvmpipe", "softpipe", "swrast")):
            raise RuntimeError("Refusing software-rendering environment " + variable)
    assert_ports_free()
    out = root / "runs" / args.name
    if shutil.disk_usage(root).free < 1024 ** 3:
        raise RuntimeError("Below the 1 GiB free-disk safety floor")
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_HOME"] = str(out / "ros_home")
    environment = os.environ.copy()
    environment["ROS_HOME"] = str(out / "ros_home")
    processes = Processes(out, environment)
    status = dict(status="INITIALIZING", collection_complete=False, audit_exit=None,
                  competition_pass=None, whole_body_clearance_verified=None, video_recorded=False,
                  disclaimer=DISCLAIMER, start_monotonic_s=time.monotonic())
    def emergency():
        status.update(status="HARD_WALL_TIMEOUT", failure="Total bounded wall budget exhausted",
                      elapsed_wall_s=time.monotonic() - status["start_monotonic_s"])
        processes.signal_groups(signal.SIGKILL)
        write_json(out / "run_status.json", status)
        os._exit(124)
    watchdog = threading.Timer(args.wall_timeout - 2, emergency)
    watchdog.daemon = True
    watchdog.start()
    old_handlers = {}
    def interrupted(signum, frame):
        raise KeyboardInterrupt("Received signal {}".format(signum))
    for signum in (signal.SIGINT, signal.SIGTERM):
        old_handlers[signum] = signal.signal(signum, interrupted)
    result = 2
    try:
        profile = load_exact_profile(root)
        source_hashes = verify_manifest(root)
        plan = verify_plan(root / PLAN_REL)
        archive = find_simulator_zip(root, args.simulator_zip)
        asset_hashes = verify_simulator(root, archive)
        commands = build_commands(root, out, profile, args.width, args.height, args.rviz,
                                  args.capture, args.sim_seconds,
                                  args.single_threaded_rendering, args.glcore33)
        shutil.copyfile(Path(__file__), out / "run_adapter.py")
        shutil.copyfile(Path(__file__).with_name("env.sh"), out / "env_adapter.sh")
        for name in ("profile.json", "parameters.yaml"):
            shutil.copyfile(root / "original_parameters" / name, out / ("original_" + name))
        binaries = {}
        for relative in ("ros_ws/devel/lib/controller_sprint/controller_sprint_node",
                         "ros_ws/devel/lib/planner_pkg/traj_vertices_publisher"):
            path = root / relative
            if not path.is_file() or not os.access(path, os.X_OK):
                raise RuntimeError("Missing compiled native node: " + relative)
            binaries[relative] = digest(path)
        hash_report = {"payload": source_hashes, "compiled_nodes": binaries,
                   "official_assets": asset_hashes, "official_zip_sha256": ZIP_SHA256,
                   "adapter_sha256": digest(Path(__file__)), "env_sha256": digest(Path(__file__).with_name("env.sh"))}
        capture_helper = None
        if args.capture:
            capture_helper = _capture_regular(root / "runtime/capture_x11_window.py", "capture helper source")
            for command in ("ffmpeg", "ffprobe"):
                if not shutil.which(command):
                    raise RuntimeError("--capture requires " + command + "; default core trial remains available without it")
            hash_report["capture_helper_sha256"] = digest(capture_helper)
            shutil.copyfile(capture_helper, out / "capture_helper.py")
        write_json(out / "source_hashes.json", hash_report)
        write_json(out / "profile.json", {"source_profile": profile, "platform_adapter": dict(vars(args), root=str(root), simulator_zip=str(archive)),
                   "saved_reference": plan, "clock_speed_or_physics_changed": False,
                   "recording_starts_before_original_respawn_reposition": True,
                   "video_capture_requested": args.capture,
                   "video_capture_starts_after_controller_parameters_before_reference": args.capture,
                   "disclaimer": DISCLAIMER})
        if processes.run("glxinfo", ["glxinfo", "-B"], 8) != 0:
            raise RuntimeError("glxinfo failed; validate WSLg before running")
        glx = (out / "glxinfo.log").read_text(errors="replace").lower()
        if "nvidia" not in glx or "d3d12" not in glx or re.search(r"accelerated:\s*yes", glx) is None or "llvmpipe" in glx:
            raise RuntimeError("glxinfo does not verify accelerated D3D12/NVIDIA rendering")
        collect(root, out, args, profile, commands, processes, status)
        processes.stop()
        bag = out / "race.bag"
        if not bag.is_file() or (out / "race.bag.active").exists():
            raise RuntimeError("rosbag did not close cleanly; audit cannot be certified")
        if processes.run("bag_info", ["rosbag", "info", "--yaml", str(bag)], 6) != 0:
            raise RuntimeError("Could not read finalized bag")
        audit = [sys.executable, "-B", str(root / "scripts/sprint_evaluate.py"), str(bag),
                 "--audit-module", str(root / "scripts/audit_race.py"),
                 "--telemetry", str(out / "clock_wall.jsonl"), "--out", str(out / "summary.json"), "--compact"]
        status["audit_exit"] = processes.run("audit", audit, 12)
        status["status"] = "COLLECTED_PLANE_AUDIT_COMPLETE" if status["audit_exit"] == 0 else "AUDIT_REJECTED_OR_FAILED"
        if (out / "summary.json").is_file():
            summary = json.loads((out / "summary.json").read_text())
            status["screen_observations"] = {
                "ordered_planes_complete": summary["plane_sequence"]["complete"],
                "credited_planes": summary["plane_sequence"]["credited_count"],
                "start_to_last_gate_sim_s": summary["timing"]["start_to_last_gate_sim_s"],
                "within_5_sim_s": summary["timing"]["within_target_start_to_last_sim"],
                "terminal_hold_observed": summary["terminal_hold"]["observed"],
                "collision_messages_all_phases": summary["collisions"]["true_messages"],
                "collision_reporting_enabled_verified": None,
                "collisions_between_first_and_last_gate": summary["collisions"]["by_phase"].get(
                    "between_first_gate_and_finish", {}).get("message_count", 0),
                "data_errors": summary["data_quality"]["errors"],
            }
            observations = status["screen_observations"]
            elapsed = observations["start_to_last_gate_sim_s"]
            observations["strict_under_5_sim_s"] = isinstance(elapsed, (int, float)) and math.isfinite(elapsed) and elapsed < 5.0
            candidate_observed = (observations["ordered_planes_complete"] is True
                                  and observations["strict_under_5_sim_s"] is True
                                  and observations["terminal_hold_observed"] is True
                                  and not observations["data_errors"]
                                  and observations["collisions_between_first_and_last_gate"] == 0)
            status["observed_candidate_screen"] = candidate_observed
            if status["audit_exit"] == 0:
                status["status"] = "COLLECTED_CANDIDATE_SCREEN" if candidate_observed else "COLLECTED_CANDIDATE_NOT_MET"
        if verify_manifest(root) != source_hashes or digest(root / PLAN_REL) != PLAN_SHA256:
            raise RuntimeError("Source/reference bytes changed during the run")
        if capture_helper is not None and digest(capture_helper) != hash_report["capture_helper_sha256"]:
            raise RuntimeError("Capture helper source changed during the run")
        result = status["audit_exit"]
        if result == 0 and status.get("observed_candidate_screen") is not True:
            result = 1
    except KeyboardInterrupt as exc:
        status.update(status="INTERRUPTED", failure=str(exc))
        result = 130
    except Exception as exc:
        status.update(status="FAILED", failure="{}: {}".format(type(exc).__name__, exc))
        print(status["failure"], file=sys.stderr, flush=True)
    finally:
        processes.stop()
        processes.close()
        status["elapsed_wall_s"] = time.monotonic() - status["start_monotonic_s"]
        write_json(out / "run_status.json", status)
        watchdog.cancel()
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
    print(json.dumps(status, sort_keys=True), flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
