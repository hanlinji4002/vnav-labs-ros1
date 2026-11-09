"""Stdlib-only contract tests; never launch ROS, processes, or network clients.

Run beside this file with: python3 -B -m unittest -v test_run_trial_gpu
"""
import copy
import errno
import hashlib
import importlib.util
import io
import json
from pathlib import Path, PurePosixPath
import re
import tempfile
from types import SimpleNamespace
import unittest
from unittest import mock


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("lab4_run_trial_gpu", HERE / "run_trial_gpu.py")
adapter = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(adapter)


def exact_profile():
    profile = dict(adapter.EXACT)
    profile.update(name="final_balanced494", offline_plan_sha256=adapter.PLAN_SHA256)
    return profile


class PortPreflightTests(unittest.TestCase):
    def run_preflight(self, occupied=None):
        records = []

        class Probe:
            def __init__(self, family, kind):
                self.family, self.kind, self.options = family, kind, []

            def __enter__(self):
                return self

            def __exit__(self, *args):
                return False

            def setsockopt(self, level, option, value):
                self.options.append((level, option, value))

            def bind(self, address):
                records.append((self.family, self.kind, address, list(self.options)))
                if (self.family, self.kind, address[1]) == occupied:
                    raise OSError(errno.EADDRINUSE, "active listener or bound UDP socket")

        with mock.patch.object(adapter.socket, "socket", side_effect=Probe), \
                mock.patch.object(adapter.socket, "has_ipv6", True), \
                mock.patch.object(adapter.Path, "is_dir", return_value=False):
            adapter.assert_ports_free()
        return records

    def test_only_tcp_reuses_addresses_and_all_ports_are_still_bound(self):
        records = self.run_preflight()
        self.assertEqual(len(records), 30)
        for family, kind, address, options in records:
            with self.subTest(family=family, kind=kind, address=address):
                expected = []
                if family == adapter.socket.AF_INET6:
                    expected.append((adapter.socket.IPPROTO_IPV6, adapter.socket.IPV6_V6ONLY, 1))
                if kind == adapter.socket.SOCK_STREAM:
                    expected.append((adapter.socket.SOL_SOCKET, adapter.socket.SO_REUSEADDR, 1))
                self.assertEqual(options, expected)

    def test_active_tcp_and_bound_udp_conflicts_still_fail_for_both_families(self):
        for family in (adapter.socket.AF_INET, adapter.socket.AF_INET6):
            for kind in (adapter.socket.SOCK_STREAM, adapter.socket.SOCK_DGRAM):
                with self.subTest(family=family, kind=kind):
                    with self.assertRaisesRegex(RuntimeError, "already in use"):
                        self.run_preflight(occupied=(family, kind, 9004))

    def test_occupied_ros_master_is_not_ignored(self):
        with self.assertRaisesRegex(RuntimeError, "11311 TCP already in use"):
            self.run_preflight(occupied=(adapter.socket.AF_INET, adapter.socket.SOCK_STREAM, 11311))


class ObserverInitializationTests(unittest.TestCase):
    def fake_dependencies(self, result=0, wallclock=False):
        events = []

        def run(name, command, timeout):
            events.append(("run", name, command, timeout))
            return result

        def init_node(name, **kwargs):
            events.append(("init", name, kwargs))

        def is_wallclock():
            events.append(("clock_check",))
            return wallclock

        rospy = SimpleNamespace(init_node=init_node,
                                rostime=SimpleNamespace(is_wallclock=is_wallclock))
        return events, rospy, SimpleNamespace(run=run)

    def test_sim_time_is_set_before_rospy_initialization_with_bounded_owned_command(self):
        events, rospy, processes = self.fake_dependencies()
        adapter.initialize_sim_observer(rospy, processes)
        self.assertEqual(events, [
            ("run", "use_sim_time", ["rosparam", "set", "/use_sim_time", "true"], 5),
            ("init", "lab4_gpu_trial_observer", {"anonymous": True, "disable_signals": True}),
            ("clock_check",),
        ])

    def test_parameter_failure_never_initializes_a_wall_clock_observer(self):
        events, rospy, processes = self.fake_dependencies(result=1)
        with self.assertRaisesRegex(RuntimeError, "before observer initialization"):
            adapter.initialize_sim_observer(rospy, processes)
        self.assertEqual(len(events), 1)

    def test_unexpected_wall_clock_mode_fails_explicitly(self):
        _, rospy, processes = self.fake_dependencies(wallclock=True)
        with self.assertRaisesRegex(RuntimeError, "wall time"):
            adapter.initialize_sim_observer(rospy, processes)


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.path = self.root / "original_parameters/profile.json"
        self.path.parent.mkdir()

    def save(self, profile):
        self.path.write_text(json.dumps(profile), encoding="utf-8")

    def test_exact_profile_is_accepted(self):
        profile = exact_profile()
        profile["ignored_planning_cli"] = {"acc": 40, "down": 9, "order": 4}
        self.save(profile)
        self.assertEqual(adapter.load_exact_profile(self.root), profile)

    def test_each_locked_parameter_rejects_tampering(self):
        for key, value in adapter.EXACT.items():
            with self.subTest(parameter=key):
                profile = exact_profile()
                profile[key] = (not value if isinstance(value, bool) else
                                value + 1 if isinstance(value, (int, float)) else "changed")
                self.save(profile)
                with self.assertRaises(ValueError):
                    adapter.load_exact_profile(self.root)

    def test_boolean_and_numeric_equality_cannot_bypass_profile_lock(self):
        for key, changed in (("signed", 1), ("no_ff", 0), ("ff_gain", True)):
            with self.subTest(parameter=key):
                profile = exact_profile()
                profile[key] = changed
                self.save(profile)
                with self.assertRaises(ValueError):
                    adapter.load_exact_profile(self.root)

    def test_wrong_identity_hash_or_missing_parameter_is_rejected(self):
        for changed in ({"name": "another_run"}, {"offline_plan_sha256": "0" * 64},
                        {"kx": None}):
            with self.subTest(changed=changed):
                profile = exact_profile()
                profile.update(changed)
                if profile.get("kx") is None:
                    profile.pop("kx")
                self.save(profile)
                with self.assertRaises(ValueError):
                    adapter.load_exact_profile(self.root)


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "payload with spaces"
        self.out = self.root / "runs/fresh_trial"
        self.commands = adapter.build_commands(self.root, self.out, exact_profile(), 1280, 720)

    def test_controller_locks_final_saved_reference_settings(self):
        command = self.commands["controller"]
        self.assertEqual(command[:3], ["rosrun", "controller_sprint", "controller_sprint_node"])
        settings = dict(token.split(":=", 1) for token in command[3:])
        expected = {
            "_kx": "36.0", "_kv": "12.0", "_kr": "60.0", "_komega": "4.0",
            "_control_hz": "1000",
            "_Jx": "0.1", "_Jy": "0.2", "_Jz": "0.1",
            "_use_rate_feedforward": "true", "_use_accel_feedforward": "false",
            "_signed_attitude": "true", "_free_heading": "true",
            "_use_attitude_preview": "false", "_rate_feedforward_gain": "1",
            "_omega_d_filter_tau": "0.02", "_omega_d_limit": "6",
            "_log_attitude_error": "false",
            "_feedforward_reference_only": "true", "_signed_attitude_hysteresis": "0.2",
            "_accel_feedforward_gain": "1", "_omega_dot_d_limit": "40",
            "_omega_dot_d_filter_tau": "0.04", "_derivative_min_dt": "0.0005",
            "_derivative_reset_dt": "0.2",
        }
        self.assertEqual(settings, expected)
        self.assertEqual(len(settings), len(command) - 3, "Duplicate private parameters")

    def test_bridge_keeps_real_clock_collisions_and_official_launch(self):
        command = self.commands["bridge"]
        self.assertEqual(command[:3], ["roslaunch", "tesse_ros_bridge", "tesse_quadrotor_bridge.launch"])
        arguments = [token for token in command[3:] if ":=" in token]
        settings = dict(token.split(":=", 1) for token in arguments)
        self.assertEqual(settings, {
            "ns": "tesse", "publish_imu": "false", "publish_collisions": "true",
            "enable_collision": "true", "use_sim_time": "true", "publish_clock": "true",
            "enable_step_mode": "false", "initial_scene": "1", "sim_ip": "127.0.0.1",
            "self_ip": "127.0.0.1", "use_broadcast": "true",
        })
        self.assertEqual(len(settings), len(arguments), "Duplicate bridge parameters")

    def test_roslaunch_child_shutdown_finishes_inside_owned_sigint_grace(self):
        options = [token for token in self.commands["bridge"][3:] if ":=" not in token]
        self.assertEqual(options, ["--sigint-timeout=0.5", "--sigterm-timeout=0.5"])
        self.assertLess(sum(float(token.split("=", 1)[1]) for token in options), 3.0)

    def test_reference_and_gate_paths_are_root_relative_without_shell_splitting(self):
        reference = self.commands["reference"]
        self.assertIn(str(self.root / "scripts/plan_reference.py"), reference)
        self.assertIn("_plan:=" + str(self.root / adapter.PLAN_REL), reference)
        self.assertIn("_dump_file:=" + str(self.out / "plan.csv"), reference)
        self.assertIn("_attitude_preview_s:=0", reference)
        self.assertIn("_simulator_data_directory:=" +
                      str(self.root / "simulator/lab4/lab4_Data"), self.commands["gates"])
        self.assertIn(str(self.root / "scripts/clock_observer.py"), self.commands["clock"])
        self.assertIn(str(self.out / "clock_wall.jsonl"), self.commands["clock"])

    def test_native_gpu_commands_have_no_old_container_or_emulation_routes(self):
        old_root = re.compile(r"(?:^|:=)/(?:sprint|race|workspace|github)(?:/|$)")
        forbidden = ("box64", "xvfb", "libgl_always_software", "llvmpipe", "softpipe")
        for name, command in self.commands.items():
            for token in command:
                with self.subTest(command=name, token=token):
                    self.assertIsNone(old_root.search(token))
                    self.assertFalse(any(word in token.lower() for word in forbidden))
        player = self.commands["player"]
        self.assertEqual(player[0], str(self.root / "simulator/lab4/lab4.x86_64"))
        self.assertEqual(player[player.index("-screen-width") + 1], "1280")
        self.assertEqual(player[player.index("-screen-height") + 1], "720")
        self.assertNotIn("-nographics", player)
        self.assertNotIn("rviz", self.commands)

    def test_optional_rviz_and_changed_profile(self):
        enabled = adapter.build_commands(self.root, self.out, exact_profile(), rviz=True)
        self.assertEqual(enabled["rviz"], ["rviz"])
        profile = exact_profile()
        profile["kx"] = 25
        with self.assertRaises(ValueError):
            adapter.build_commands(self.root, self.out, profile)


class RendererTests(unittest.TestCase):
    def test_d3d12_nvidia_renderer_is_verified(self):
        report = adapter.renderer_report("GfxDevice: OpenGL\n  Renderer: D3D12 (NVIDIA GeForce RTX 4080)\n")
        self.assertTrue(report["hardware_renderer_verified"])
        self.assertFalse(report["software_renderer_detected"])
        self.assertEqual(len(report["renderer_lines"]), 1)

    def test_software_renderer_is_rejected_even_with_nvidia_line(self):
        for software in ("llvmpipe", "softpipe", "Software Rasterizer", "swrast"):
            with self.subTest(renderer=software):
                report = adapter.renderer_report("Renderer: D3D12 (NVIDIA RTX 4080)\n" + software)
                self.assertFalse(report["hardware_renderer_verified"])
                self.assertTrue(report["software_renderer_detected"])

    def test_missing_renderer_or_non_target_gpu_is_not_verified(self):
        for text in ("", "Vendor: NVIDIA; D3D12 supported", "Renderer: D3D12 (AMD Radeon)",
                     "Renderer: NVIDIA RTX 4080 OpenGL"):
            with self.subTest(log=text):
                self.assertFalse(adapter.renderer_report(text)["hardware_renderer_verified"])


class IntegrityTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def write_manifest(self, name="scripts/example.py", content=b"unchanged source\n"):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        expected = hashlib.sha256(content).hexdigest()
        manifest = {"files": [{"path": name, "sha256": expected}]}
        (self.root / "MANIFEST.json").write_text(json.dumps(manifest), encoding="utf-8")
        return path, manifest, expected

    def test_safe_relative_accepts_regular_paths_and_rejects_escape_routes(self):
        self.assertEqual(adapter.safe_relative("scripts/example.py"), PurePosixPath("scripts/example.py"))
        for name in ("", "/tmp/escape", "../escape", "scripts/../../escape", "scripts\\escape",
                     "C:/escape", "./scripts/example.py", "scripts//example.py"):
            with self.subTest(path=name):
                with self.assertRaises(ValueError):
                    adapter.safe_relative(name)

    def test_manifest_verifies_actual_hash_and_rejects_changed_source(self):
        path, _, expected = self.write_manifest()
        self.assertEqual(adapter.verify_manifest(self.root), {"scripts/example.py": expected})
        path.write_bytes(b"changed source\n")
        with self.assertRaises(ValueError):
            adapter.verify_manifest(self.root)

    def test_manifest_rejects_wrong_declared_hash_and_traversal(self):
        _, manifest, _ = self.write_manifest()
        for name, declared in (("scripts/example.py", "0" * 64), ("../escape", "0" * 64)):
            with self.subTest(path=name):
                invalid = copy.deepcopy(manifest)
                invalid["files"][0].update(path=name, sha256=declared)
                (self.root / "MANIFEST.json").write_text(json.dumps(invalid), encoding="utf-8")
                with self.assertRaises(ValueError):
                    adapter.verify_manifest(self.root)

    def test_manifest_rejects_missing_file(self):
        path, _, _ = self.write_manifest()
        path.unlink()
        with self.assertRaises(ValueError):
            adapter.verify_manifest(self.root)

    def test_plan_wrong_sha_is_rejected_before_csv_parsing(self):
        path = self.root / "changed.csv"
        path.write_bytes(b"not the saved reference\n")
        with self.assertRaisesRegex(ValueError, "SHA256"):
            adapter.verify_plan(path)

    def test_original_saved_csv_passes_without_absolute_host_paths(self):
        candidates = (HERE.parent / adapter.PLAN_REL,
                      HERE.parents[1] / "竞速冲刺5秒" / adapter.PLAN_REL)
        source = next((path for path in candidates if path.is_file()), None)
        if source is None:
            self.skipTest("Original saved CSV is not present in this source checkout/payload")
        report = adapter.verify_plan(source)
        self.assertEqual(report["sha256"], adapter.PLAN_SHA256)
        self.assertGreater(report["rows"], 2)
        self.assertAlmostEqual(report["duration_s"], 5.91499890810647, places=8)

    def test_stream_hash_uses_all_bytes(self):
        content = b"x" * (1024 * 1024 + 13)
        self.assertEqual(adapter.hash_stream(io.BytesIO(content)), hashlib.sha256(content).hexdigest())


if __name__ == "__main__":
    unittest.main()
