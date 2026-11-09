#!/usr/bin/env python3
"""New, explicit candidate runner reusing the preserved native WSL adapter.

Gate 1 -> gate 12 timing is separate from the old reference-start screen.
No simulator assets, old source, physical parameters or time scale are edited.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROUTE_SET = "route_optimization_20261030/candidates"
RATE_ROUTE_SET = "route_optimization_20261030/rate_limited_candidates"
UPRIGHT_ROUTE_SET = "route_optimization_20261030/upright_candidates"
FEEDBACK_ROUTE_SET = "route_optimization_20261030/feedback_candidates"
ONE_GATE_ROUTE_SET = "route_optimization_20261030/one_gate_candidates"
LOCAL_TIMING_ROUTE_SET = "route_optimization_20261030/local_timing_candidates"
EXTENSION_ROUTE_SET = "route_optimization_20261030/extension_candidates"
ALTITUDE_ROUTE_SET = "route_optimization_20261030/altitude_candidates"
ALTITUDE_MARGIN_ROUTE_SET = "route_optimization_20261030/altitude_margin_candidates"
ALTITUDE_REFINED_ROUTE_SET = "route_optimization_20261030/altitude_refined_candidates"
ORIGINAL_REFINED_ROUTE_SET = "route_optimization_20261030/original_refined_candidates"
ALTITUDE_GATE8_ROUTE_SET = "route_optimization_20261030/altitude_gate8_candidates"
TIMING_ROUTE_SET = "speed_optimization_20261105/timing_candidates"
SURROGATE_ROUTE_SET = "speed_optimization_20261105/surrogate_candidates"
SURROGATE_LOCKED_ROUTE_SET = "speed_optimization_20261105/surrogate_locked_candidates"
SURROGATE_LOCKED2_ROUTE_SET = "speed_optimization_20261105/surrogate_locked2_candidates"
SURROGATE_MODPHYS_ROUTE_SET = "speed_optimization_20261105/surrogate_modphys14_candidates"
SURROGATE_MODPHYS_B_ROUTE_SET = "speed_optimization_20261105/surrogate_modphys14b_candidates"
SURROGATE_MODPHYS_C_ROUTE_SET = "speed_optimization_20261105/surrogate_modphys14c_candidates"
MODPHYS_SETS = (SURROGATE_MODPHYS_ROUTE_SET, SURROGATE_MODPHYS_B_ROUTE_SET, SURROGATE_MODPHYS_C_ROUTE_SET)
SURROGATE_SETS = (SURROGATE_ROUTE_SET, SURROGATE_LOCKED_ROUTE_SET, SURROGATE_LOCKED2_ROUTE_SET)+MODPHYS_SETS
CANDIDATE_SETS = ("runup_candidates", "runup_y9p3_candidates", "hover_candidates", "hover_refined_candidates", ROUTE_SET, RATE_ROUTE_SET, UPRIGHT_ROUTE_SET, FEEDBACK_ROUTE_SET, ONE_GATE_ROUTE_SET, LOCAL_TIMING_ROUTE_SET, EXTENSION_ROUTE_SET, ALTITUDE_ROUTE_SET, ALTITUDE_MARGIN_ROUTE_SET, ALTITUDE_REFINED_ROUTE_SET, ORIGINAL_REFINED_ROUTE_SET, ALTITUDE_GATE8_ROUTE_SET, TIMING_ROUTE_SET)+SURROGATE_SETS


SPEED_PROFILE_FILE = "speed_optimization_20261105/control_profiles.json"


def profile_path(profile):
    """Resolve the one declared file holding a named profile; no wildcard search."""
    if profile in ("preview20", "preview40", "preview20_xy"):
        return HERE/"attitude_optimization_20261030/preview_profiles.json"
    speed = HERE/SPEED_PROFILE_FILE
    if speed.is_file() and profile in json.loads(speed.read_text()).get("profiles", {}):
        return speed
    return HERE/"attitude_optimization_20261030/control_profiles.json"


def control_overrides(profile):
    """Load a named, bounded experiment without altering the stable default."""
    if profile == "baseline":
        return {}, None
    path = profile_path(profile)
    document = json.loads(path.read_text())
    profiles = document["profiles"]
    selected = profiles.get(profile)
    if not isinstance(selected, dict):
        raise ValueError("Unknown control profile")
    values = selected["ros_private_parameters"]
    bounds = {"kx": (12, 50), "kv": (6, 20), "kr": (20, 90), "komega": (2, 8),
              "rate_feedforward_gain": (.5, 1.5), "omega_d_limit": (5, 30),
              "omega_d_filter_tau": (.005, .05), "accel_feedforward_gain": (0, .5),
              "omega_dot_d_limit": (10, 60), "omega_dot_d_filter_tau": (.02, .1),
              "state_wait_ms": (2, 19)}
    flags = {"log_attitude_error", "use_accel_feedforward", "use_attitude_preview", "control_on_state"}
    if not isinstance(values, dict) or not values:
        raise ValueError("Control profile must contain explicit overrides")
    for name, value in values.items():
        if name in flags:
            if type(value) is not bool:
                raise ValueError("Control flag must be Boolean")
        elif (name not in bounds or type(value) not in (int, float) or
              not math.isfinite(value) or not bounds[name][0] <= value <= bounds[name][1]):
            raise ValueError("Control override outside bounded experiment: "+name)
    return dict(values), path


def reference_options(profile, profile_path):
    if profile_path is None:
        return {}
    selected = json.loads(profile_path.read_text())["profiles"][profile]
    options = selected.get("reference_private_parameters", {})
    if options:
        if (set(options) not in ({"attitude_preview_s"}, {"attitude_preview_s", "preview_vertical_weight"}) or
                type(options["attitude_preview_s"]) not in (int, float) or
                options["attitude_preview_s"] not in (.02, .04) or
                selected.get("ros_private_parameters", {}).get("use_attitude_preview") is not True or
                selected.get("reference_entry") != "attitude_optimization_20261030/preview_reference.py"):
            raise ValueError("Unexpected reference experiment")
        if "preview_vertical_weight" in options and (type(options["preview_vertical_weight"]) not in (int, float) or options["preview_vertical_weight"] not in (0.0, 1.0)):
            raise ValueError("Unexpected preview vertical weight")
    return options


def override_command(command, values):
    result = list(command)
    for key, value in values.items():
        prefix = "_"+key+":="
        encoded = str(value).lower() if type(value) is bool else str(value)
        matches = [i for i, arg in enumerate(result) if arg.startswith(prefix)]
        if len(matches) != 1:
            raise ValueError("Expected one existing private parameter: "+key)
        result[matches[0]] = prefix+encoded
    return result


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def select_plan(root, factor, base, family="scaled", candidate_set=None):
    baseline = root / base.PLAN_REL
    base.verify_plan(baseline)
    if factor == 1.0 and family == "scaled":
        return baseline, {"speed_factor": 1.0, "csv_sha256": base.PLAN_SHA256,
                          "name": "preserved_baseline"}
    if family not in ("scaled", "hover", "runup"):
        raise ValueError("Unknown plan family")
    folder_name = {"scaled": "candidates", "hover": "hover_candidates", "runup": "runup_candidates"}[family]
    if candidate_set is not None:
        allowed = {"runup": ("runup_candidates", "runup_y9p3_candidates"),
                   "hover": CANDIDATE_SETS[2:]}
        if candidate_set not in allowed.get(family, ()):
            raise ValueError("Unknown candidate set for this family")
        folder_name = candidate_set
    folder = root / "race_20261029" / folder_name
    manifest = json.loads((folder / "CANDIDATES.json").read_text())
    if manifest["baseline_sha256"] != base.PLAN_SHA256:
        raise ValueError("Candidate derives from a different baseline")
    matches = [r for r in manifest["candidates"] if abs(r["speed_factor"]-factor) < 1e-10]
    if len(matches) != 1:
        raise ValueError("Speed factor not uniquely present in the candidate manifest")
    row = matches[0]
    relative = base.safe_relative(row["csv"])
    path = folder / relative
    if path.is_symlink() or base.digest(path) != row["csv_sha256"]:
        raise ValueError("Candidate CSV identity mismatch")
    maker = load_module("race_candidate_math", HERE / "make_candidates.py")
    maker.read_rows(path, verify_identity=False)
    return path, row


def gate_result(summary, timer):
    events = summary["aperture_screen"]["gates"]
    def finite(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
    complete = (summary["plane_sequence"]["complete"] is True and
                summary["plane_sequence"]["credited_count"] == 12 and len(events) == 12 and
                [e.get("gate") for e in events] == list(range(12)) and
                all(e.get("ordered_forward_crossing") is True and finite(e.get("sim_time_s"))
                    for e in events) and
                all(b["sim_time_s"] > a["sim_time_s"] for a, b in zip(events, events[1:])))
    aperture = complete and all(finite(e.get("horizontal_offset_m")) and
                               finite(e.get("vertical_offset_m")) and
                               abs(e["horizontal_offset_m"]) <= .75 and
                               abs(e["vertical_offset_m"]) <= .75 for e in events)
    duration = summary["timing"]["first_to_last_gate_sim_s"]
    timer_duration = timer.get("elapsed_sim_s")
    live_events = timer.get("credited_crossings", [])
    agreement = (complete and timer.get("status") == "FINISHED" and
                 timer.get("gates_passed") == 12 and timer.get("gates_total") == 12 and
                 len(live_events) == 12 and finite(timer_duration) and finite(duration) and
                 duration > 0 and timer_duration > 0 and
                 abs(duration-(events[-1]["sim_time_s"]-events[0]["sim_time_s"])) <= 1e-6 and
                 abs(timer_duration-duration) <= .03 and
                 all(e.get("gate_index") == i and e.get("direction") == 1 and
                     e.get("credited") is True and e.get("within_point_square") is True and
                     finite(e.get("sim_time_s")) and
                     abs(e["sim_time_s"]-events[i]["sim_time_s"]) <= .03
                     for i, e in enumerate(live_events)))
    collisions = summary["collisions"]["by_phase"].get(
        "between_first_gate_and_finish", {}).get("message_count", 0)
    valid = (complete and aperture and agreement and timer.get("valid_data") is True
             and not timer.get("data_errors") and not summary["data_quality"]["errors"] and
             collisions == 0 and timer.get("true_collision_messages_first_to_last") == 0 and
             summary["terminal_hold"]["observed"] is True)
    return {"rule": "ordered forward actual centre crossings: first gate to last gate",
            "first_to_last_sim_s": duration,
            "first_to_last_wall_estimate_s": summary["wall_telemetry"].get(
                "first_to_last_gate_wall_elapsed_estimate_s"),
            "live_timer_sim_s": timer_duration,
            "live_timer_wall_estimate_s": timer.get("elapsed_wall_estimate_s"),
            "online_offline_agreement_within_30ms": agreement,
            "ordered_12_gates": complete, "centre_in_visual_hole_square": aperture,
            "visual_point_half_extent_m": .75,
            "collision_messages_first_to_last": collisions,
            "data_errors": summary["data_quality"]["errors"],
            "terminal_hold_observed": summary["terminal_hold"]["observed"],
            "valid_measured_run": valid,
            "whole_body_clearance_verified": False,
            "official_judge_acceptance": None}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--speed-factor", type=float, default=1.0)
    parser.add_argument("--timer-window", action="store_true")
    parser.add_argument("--plan-family", choices=("scaled", "hover", "runup"), default="scaled")
    parser.add_argument("--candidate-set", choices=CANDIDATE_SETS)
    parser.add_argument("--control-profile", default="baseline")
    parser.add_argument("--omega-limit", type=float, choices=(6.0, 8.0, 10.0), default=6.0)
    parser.add_argument("--ff-tau", type=float, choices=(.01, .02), default=.02)
    parser.add_argument("--modified-physics", type=int, choices=(10, 14, 20),
                        help="STUDY ONLY: swap in a declared globalgamemanagers with a different Unity max angular speed for this run; the result is tagged MODIFIED_PHYSICS and is not an official-physics result")
    own, remaining = parser.parse_known_args(argv)
    overrides, control_profile_path = control_overrides(own.control_profile)
    reference_parameters = reference_options(own.control_profile, control_profile_path)
    if own.modified_physics is None and overrides.get("omega_d_limit", 0) > 10:
        raise ValueError("omega_d_limit above 10 rad/s is only meaningful with --modified-physics")
    if own.modified_physics is None and own.candidate_set in MODPHYS_SETS:
        raise ValueError("This candidate set was searched under modified physics; pass --modified-physics")
    modified = None
    if own.modified_physics is not None:
        manifest_path = HERE/"speed_optimization_20261105/modified_physics/MODIFIED_PHYSICS_MANIFEST.json"
        manifest = json.loads(manifest_path.read_text())
        variant = manifest["variants"][str(own.modified_physics)]
        patched = manifest_path.parent/variant["file"]
        if hashlib.sha256(patched.read_bytes()).hexdigest() != variant["sha256"]:
            raise ValueError("Modified physics file does not match its manifest")
        modified = dict(manifest=manifest_path, patched=patched, variant=variant,
                        original_sha256=manifest["source_sha256"], relative=manifest["source_file"])
    if overrides and own.plan_family not in ("hover", "runup"):
        raise ValueError("Control experiments require the measured stable prehover entry")
    prepared = own.plan_family in ("hover", "runup")
    if prepared and not any(v.startswith("--sim-seconds") for v in remaining):
        remaining += ["--sim-seconds", "18"]
    # The original adapter re-execs its own __file__; enter its environment here
    # first, preserving THIS entry point and its explicit candidate arguments.
    probe = argparse.ArgumentParser(add_help=False)
    probe.add_argument("name")
    probe.add_argument("--root", type=Path, default=HERE.parent)
    core, _ = probe.parse_known_args(remaining)
    root = core.root.resolve()
    if os.environ.get("LAB4_GPU_ENV_READY") != str(root):
        env = os.environ.copy()
        env["LAB4_GPU_ROOT"] = str(root)
        args = list(sys.argv[1:] if argv is None else argv)
        os.execve("/bin/bash", ["bash", "--noprofile", "--norc", "-c",
                  'source "$1"; exec /usr/bin/python3 -B "$2" "${@:3}"',
                  "lab4-race", str(root/"runtime/env.sh"), str(Path(__file__).resolve())] + args, env)
    base = load_module("preserved_native_adapter", root/"runtime/run_trial_gpu.py")
    plan, metadata = select_plan(root, own.speed_factor, base, own.plan_family, own.candidate_set)
    metadata = dict(metadata, plan_family=own.plan_family, candidate_set=plan.parent.name, commanded_controller={"omega_d_limit_rad_s": own.omega_limit,
                                                  "omega_d_filter_tau_s": own.ff_tau,
                                                  "feedback_gains": [36, 12, 60, 4]})
    metadata["candidate_set_path"] = own.candidate_set
    metadata["control_profile"] = own.control_profile
    metadata["control_overrides"] = overrides
    metadata["reference_overrides"] = reference_parameters
    metadata["commanded_controller"]["feedback_gains"] = [overrides.get(k, d) for k, d in zip(("kx", "kv", "kr", "komega"), (36, 12, 60, 4))]
    metadata["commanded_controller"]["omega_d_limit_rad_s"] = overrides.get("omega_d_limit", own.omega_limit)
    metadata["commanded_controller"]["omega_d_filter_tau_s"] = overrides.get("omega_d_filter_tau", own.ff_tau)
    if prepared:
        metadata["commanded_controller"].update(hover_feedback_gains=[4, 3, 12, 1],
                                                gain_schedule="quintic 0.3s at race entry and terminal stop")
    out = root / "runs" / core.name
    geometry = HERE / "gate_geometry_evidence.json"
    supplemental = [Path(__file__), HERE/"gate_timer.py", HERE/"make_candidates.py",
                    HERE/"capture_with_timer.py", geometry]
    if control_profile_path is not None:
        supplemental.append(control_profile_path)
    if reference_parameters:
        supplemental.append(HERE/"attitude_optimization_20261030/preview_reference.py")
    if own.speed_factor != 1.0 or prepared:
        supplemental.extend([plan, plan.parent/"CANDIDATES.json"])
    if own.candidate_set and own.candidate_set.startswith("route_optimization_20261030/"):
        supplemental.append(plan.with_suffix(".json"))
        supplemental.extend(sorted((HERE/"route_optimization_20261030").glob("optimize*.py")))
        supplemental.extend(sorted((HERE/"route_optimization_20261030").glob("make*.py")))
    if own.candidate_set == TIMING_ROUTE_SET or own.candidate_set in SURROGATE_SETS:
        supplemental.extend([plan.with_suffix(".json"), HERE/"speed_optimization_20261105/make_timing_candidates.py",
                             HERE/"route_optimization_20261030/make_local_adjustments.py",
                             HERE/"route_optimization_20261030/optimize_route.py"])
    if own.candidate_set in SURROGATE_SETS:
        supplemental.extend([HERE/"speed_optimization_20261105/make_surrogate_candidates.py",
                             HERE/"speed_optimization_20261105/surrogate_bias_20261105.json",
                             HERE/"speed_optimization_20261105/surrogate_search.py",
                             HERE/"speed_optimization_20261105/es_search.py",
                             HERE/"speed_optimization_20261105/closed_loop_surrogate.py"])
    if prepared:
        supplemental.extend([HERE/"prehover_reference.py", HERE/"make_flying_start.py"])
        if own.plan_family == "runup":
            supplemental.append(HERE/"make_runup_start.py")
        package = HERE/"controller_prehover"
        for source in package.rglob("*"):
            if source.is_file():
                deployed = root/"ros_ws/src/controller_prehover"/source.relative_to(package)
                if not deployed.is_file() or base.digest(source) != base.digest(deployed):
                    raise ValueError("Prehover controller build source differs from selected source: "+str(source))
                supplemental.extend([source, deployed])
        new_binary = root/"ros_ws/devel/lib/controller_prehover/controller_prehover_node"
        if not new_binary.is_file() or not os.access(new_binary, os.X_OK):
            raise ValueError("Build the separately preserved controller_prehover package first")
        supplemental.append(new_binary)
        base.TOPICS = list(base.TOPICS)+["/controller_sprint/gain_schedule", "/controller_sprint/pos_gain_schedule"]
    old_verify, old_plan, old_commands = base.verify_manifest, base.verify_plan, base.build_commands
    old_processes = base.Processes

    def verify(root_arg):
        report = old_verify(root_arg)
        for path in supplemental:
            if not path.is_file() or path.is_symlink() or root not in path.resolve().parents:
                raise ValueError("Missing or redirected new race source: " + str(path))
            report[str(path.relative_to(root))] = base.digest(path)
        return report

    def selected_reference(_):
        old_plan(root / base.PLAN_REL)
        if base.digest(plan) != metadata["csv_sha256"]:
            raise ValueError("Selected plan changed")
        import csv
        with plan.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
        return dict(metadata, rows=len(rows), duration_s=float(rows[-1]["t"]),
                    actual_selected_path=str(plan), preserved_baseline_sha256=base.PLAN_SHA256)

    def commands(*args, **kwargs):
        result = old_commands(*args, **kwargs)
        result["reference"] = ["_plan:="+str(plan) if v.startswith("_plan:=") else v
                               for v in result["reference"]]
        if prepared:
            result["reference"][2] = str(HERE/"prehover_reference.py")
            result["reference"] += ["_hover_release_t:=5.0"]
            if reference_parameters:
                result["reference"][2] = str(HERE/"attitude_optimization_20261030/preview_reference.py")
                result["reference"] = override_command(result["reference"], {"attitude_preview_s": reference_parameters["attitude_preview_s"]})
                if "preview_vertical_weight" in reference_parameters:
                    result["reference"].append("_preview_vertical_weight:="+str(reference_parameters["preview_vertical_weight"]))
        result["controller"] = [("_omega_d_limit:="+str(own.omega_limit) if v.startswith("_omega_d_limit:=") else
                                 "_omega_d_filter_tau:="+str(own.ff_tau) if v.startswith("_omega_d_filter_tau:=") else v)
                                for v in result["controller"]]
        if prepared:
            # New synchronous-control knobs exist only in controller_prehover; declare
            # their defaults so a profile can override them like any other parameter.
            result["controller"] += ["_control_on_state:=false", "_state_wait_ms:=8.0"]
        result["controller"] = override_command(result["controller"], overrides)
        if prepared:
            result["controller"][1:3] = ["controller_prehover", "controller_prehover_node"]
            result["controller"] += ["_prehover_kx:=4.0", "_prehover_kv:=3.0", "_prehover_kr:=12.0", "_prehover_komega:=1.0", "_hover_release_t:=5.0",
                                     "_gain_transition_s:=0.3", "_race_end_plan_s:="+str(metadata["metrics"]["complete_reference_duration_s"])]
        if "capture" in result:
            result["capture"][2] = str(HERE/"capture_with_timer.py")
            result["capture"] += ["--timer-state", str(out/"gate_timer.json")]
            if modified is not None:
                result["capture"] += ["--hud-banner", "MODIFIED PHYSICS: max rate %g rad/s" % modified["variant"]["max_angular_speed_rad_s"]]
        return result

    class TimerProcesses(old_processes):
        timer_started = False

        def start(self, name, command, essential=True, cwd=None):
            if name == "controller" and not self.timer_started:
                timer_cmd = [sys.executable, "-B", str(HERE/"gate_timer.py"),
                             "--geometry-evidence", str(geometry), "--out-dir", str(self.out)]
                if own.timer_window:
                    timer_cmd.append("--show-window")
                super().start("gate_timer", timer_cmd, essential=True)
                self.timer_started = True
                def ready():
                    file = self.out/"gate_timer.ready.json"
                    return file.is_file() and json.loads(file.read_text()).get("ready") is True
                base.wait_until(ready, self, 6, "live gate timer geometry and odometry readiness")
            return super().start(name, command, essential=essential, cwd=cwd)

    base.verify_manifest, base.verify_plan, base.build_commands = verify, selected_reference, commands
    base.Processes = TimerProcesses
    if modified is not None:
        supplemental.extend([modified["manifest"], modified["patched"]])
        target = root/"simulator/lab4"/modified["relative"]
        original_bytes = target.read_bytes()
        if hashlib.sha256(original_bytes).hexdigest() != modified["original_sha256"]:
            raise ValueError("Deployed simulator file is not the verified original; refusing to patch")
        # The adapter checks every deployed asset against the official ZIP; declare
        # the single intentional difference instead of weakening that check.
        old_verify_sim = base.verify_simulator
        def verify_simulator_modified(root_arg, archive):
            target.write_bytes(original_bytes)
            report = old_verify_sim(root_arg, archive)
            target.write_bytes(modified["patched"].read_bytes())
            if hashlib.sha256(target.read_bytes()).hexdigest() != modified["variant"]["sha256"]:
                raise ValueError("Patched physics file was not written correctly")
            report[modified["relative"]+" (MODIFIED_PHYSICS)"] = modified["variant"]["sha256"]
            return report
        base.verify_simulator = verify_simulator_modified
    try:
        result = base.main(remaining)
    finally:
        if modified is not None:
            target.write_bytes(original_bytes)
            if hashlib.sha256(target.read_bytes()).hexdigest() != modified["original_sha256"]:
                raise RuntimeError("Failed to restore the original simulator file: "+str(target))
    if (out/"summary.json").is_file() and (out/"gate_timer.json").is_file():
        summary = json.loads((out/"summary.json").read_text())
        timer = json.loads((out/"gate_timer.json").read_text())
        final = gate_result(summary, timer)
        video_verified = None
        if "--capture" in remaining:
            video_verified = False
            report_path = out/"scene.mp4.capture/capture_report.json"
            last_state = out/"scene.mp4.capture/timer_annotation_last_state.json"
            if report_path.is_file() and last_state.is_file():
                report = json.loads(report_path.read_text())
                annotated_timer = json.loads(last_state.read_text())
                annotation = report.get("timer_annotation", {})
                video_verified = (report.get("status") == "complete" and
                                  annotation.get("finish_displayed") is True and
                                  annotation.get("frame_status_counts", {}).get("FINISH", 0) > 0 and
                                  gate_result(summary, annotated_timer)["valid_measured_run"])
        final["timer_video_verified"] = video_verified
        hover_verified = None
        if prepared:
            hover_verified = False
            release_file = out/"prehover_release.json"
            if release_file.is_file():
                release = json.loads(release_file.read_text())
                stable = release.get("stability", {})
                released_at = release.get("release_sim_s")
                first_gate = summary["timing"]["first_gate_plane_sim_s"]
                hover_verified = (release.get("phase") == "RACING" and stable.get("ready") is True and
                                  stable.get("fresh") is True and not stable.get("data_errors") and
                                  isinstance(released_at, (int, float)) and math.isfinite(released_at) and
                                  isinstance(first_gate, (int, float)) and first_gate > released_at and
                                  release.get("plan_sha256") == metadata["csv_sha256"] and
                                  abs(release.get("actual_sim_time_s", -1)-released_at) <= 1e-6 and
                                  stable.get("stable_sampled_span_s", 0) >= stable.get("limits", {}).get("hold_s", 1e9))
                final["prehover_release_proof"] = release
        final["stable_hover_verified"] = hover_verified
        prep_clear = None
        if prepared:
            objects = {}
            for phase in ("before_trajectory_start", "launch_to_first_gate"):
                for name, count in summary["collisions"]["by_phase"].get(phase, {}).get("objects", {}).items():
                    objects[name] = objects.get(name, 0)+count
            prep_clear = all(name.lower().startswith("floor") for name, count in objects.items() if count)
            final["preparation_collision_objects"] = objects
        final["no_nonfloor_preparation_contacts"] = prep_clear
        final["valid_measured_run"] = (final["valid_measured_run"] and result in (0, 1) and
                                        video_verified is not False and hover_verified is not False and prep_clear is not False)
        final.update(candidate=metadata, preserved_adapter_exit=result,
                     real_flight_replayed=False, simulation_time_scale_changed=False,
                     official_physics=modified is None,
                     modified_physics=None if modified is None else dict(
                         max_angular_speed_rad_s=modified["variant"]["max_angular_speed_rad_s"],
                         file=modified["relative"], patched_sha256=modified["variant"]["sha256"],
                         original_sha256=modified["original_sha256"], original_restored=True,
                         note="MODIFIED SIMULATOR PHYSICS: not comparable to official-physics results"))
        base.write_json(out/"gate_race_result.json", final)
        print(json.dumps(final, sort_keys=True), flush=True)
        if result in (0, 1):
            result = 0 if final["valid_measured_run"] else 1
    return result


if __name__ == "__main__":
    raise SystemExit(main())
