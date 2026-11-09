#!/usr/bin/env python3
"""Generate separate pre-hover / elevated-start C4 racing references.

The ground-to-hover preparation is outside gate 1 -> gate 12 timing.  The
race is re-solved from an elevated rest state; prepending a hover to the old
ground-start CSV would not achieve this.  No environment, odometry, gate,
physical parameter or simulator clock is modified.  Values are references,
not measured flight performance.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

import make_candidates as common


HERE = Path(__file__).resolve().parent
DEFAULT_PLANNER = HERE.parents[1] / "竞速冲刺5秒/optimized_plan.py"
SMOOTH_COEFFICIENTS = (0., 0., 0., 0., 0., 126., -420., 540., -315., 70.)


def load_planner(path):
    """Reuse the established C4 minimum-snap solver, rather than fork it."""
    spec = importlib.util.spec_from_file_location("prehover_minimum_snap", str(path))
    if spec is None or spec.loader is None:
        raise ValueError("Cannot load minimum-snap planner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def smooth_step(u, order=0):
    if not math.isfinite(u) or not 0 <= u <= 1 or not isinstance(order, int) or not 0 <= order <= 4:
        raise ValueError("Smooth-step parameter/order outside its phase")
    return sum(math.factorial(j)/math.factorial(j-order)*SMOOTH_COEFFICIENTS[j]*u**(j-order)
               for j in range(order, 10))


def source_model(metadata, original, planner):
    durations = np.array(metadata["segment_durations_s"], dtype=float)
    waypoints = np.array(metadata["waypoints_ros_m"], dtype=float)
    if (durations.shape != (13,) or waypoints.shape != (14, 3) or
            not np.all(np.isfinite(durations)) or np.any(durations <= 0) or
            not np.all(np.isfinite(waypoints))):
        raise ValueError("Expected 13 finite baseline segments and 14 waypoints")
    t = np.array([r["t"] for r in original])
    supplied = np.array([[r[k] for k in ("x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az")]
                         for r in original])
    solver = planner.MinimumSnap(durations)
    recomputed = np.hstack([matrix@waypoints for matrix in solver.maps(t)[:3]])
    residual = float(np.max(np.abs(recomputed-supplied)))
    if residual > 2e-5:
        raise ValueError("Baseline metadata does not reproduce pinned CSV: "+str(residual))
    return durations, waypoints, residual


def prelude_sample(t, ground, hover, climb_s, hold_s):
    if not 0 <= t <= climb_s+hold_s:
        raise ValueError("Outside prelude")
    if t >= climb_s:
        return hover.copy(), np.zeros(3), np.zeros(3)
    u = t/climb_s
    delta = hover-ground
    return tuple(ground+delta*smooth_step(u) if order == 0 else
                 delta*smooth_step(u, order)/climb_s**order for order in range(3))


def generate(original, metadata, planner, factor, climb_s=3., hold_s=2., dt=.002,
             hover_z=None):
    if not math.isfinite(factor) or not 1.0 <= factor <= 1.65:
        raise ValueError("Candidate factor must be within 1.0..1.65")
    if not all(math.isfinite(x) and x > 0 for x in (climb_s, hold_s, dt)) or dt > .02:
        raise ValueError("Finite positive prelude durations and dt<=.02 required")
    durations, points, source_residual = source_model(metadata, original, planner)
    ground = points[0].copy()
    hover = ground.copy()
    hover[2] = points[1, 2] if hover_z is None else float(hover_z)
    if not math.isfinite(hover[2]) or not ground[2] < hover[2] < 4.0:
        raise ValueError("Hover height must be above spawn and below 4m")
    # Retain the known spawn x/y: no unverified backwards excursion towards y14.
    points[0] = hover
    race_durations = durations/factor
    solver = planner.MinimumSnap(race_durations)
    race_start = climb_s+hold_s
    boundaries = np.r_[0., climb_s, race_start+solver.boundaries]
    times = planner.sampling_grid(np.unique(boundaries), dt)
    values = [np.zeros((len(times), 3)) for _ in range(3)]
    race_mask = times >= race_start
    for i, t in enumerate(times[~race_mask]):
        for derivative, value in enumerate(prelude_sample(t, ground, hover, climb_s, hold_s)):
            values[derivative][i] = value
    race_times = times[race_mask]-race_start
    race_values = [matrix@points for matrix in solver.maps(race_times)]
    for order in range(3):
        values[order][race_mask] = race_values[order]
    rows = []
    yaw = original[0]["yaw"]
    for i, t in enumerate(times):
        entries = np.r_[t, values[0][i], values[1][i], values[2][i], yaw]
        rows.append(dict(zip(common.FIELDS, map(float, entries))))
    common.validate_rows(rows)
    coefficients = solver.coefficients(points)
    continuity = {}
    for derivative in range(5):
        at_start = planner.derivative_basis([0.], derivative)@coefficients[0]/race_durations[0]**derivative
        target = hover if derivative == 0 else np.zeros(3)
        continuity["prelude_to_race_order_"+str(derivative)] = float(np.max(np.abs(at_start-target)))
        joins = [float(np.max(np.abs(planner.derivative_basis([1.], derivative)@coefficients[i]/race_durations[i]**derivative
                                    -planner.derivative_basis([0.], derivative)@coefficients[i+1]/race_durations[i+1]**derivative)))
                 for i in range(12)]
        continuity["race_join_order_"+str(derivative)] = max(joins)
    race_first = race_start+solver.boundaries[1]
    race_last = race_start+solver.boundaries[12]
    # First gate is nominally crossed forward at boundary1, and preparation
    # stays at the original known spawn x/y on its approach side.
    first_gate_y = points[1, 1]
    pre_first = values[0][times < race_first-1e-7]
    monotonic_prefix = bool(np.all(pre_first[:, 1] > first_gate_y-1e-6))
    if not monotonic_prefix:
        raise ValueError("Reference crosses the first gate plane during preparation")
    if np.min(values[0][:, 2]) < ground[2]-.005:
        raise ValueError("Reference drops below original ground spawn")
    stats = planner.metrics(race_times, race_values, solver.boundaries[12])
    report = {
        "family": "prehover_replanned", "speed_factor": factor,
        "baseline_reconstruction_max_abs_pva": source_residual,
        "race_start_time_s": race_start, "climb_end_time_s": climb_s,
        "hover_release_time_s": race_start, "nominal_hover_duration_s": hold_s,
        "ground_position_ros_m": ground.tolist(), "hover_target_ros_m": hover.tolist(),
        "race_segment_durations_s": race_durations.tolist(),
        "race_waypoints_ros_m": points.tolist(),
        "race_normalized_polynomial_coefficients": coefficients.tolist(),
        "continuity_residuals": continuity,
        "no_prep_first_gate_crossing_sampled": monotonic_prefix,
        "required_runtime_release": {"actual_feedback_required": True,
            "description": "Hold reference progress at hover_release_time_s until real position/speed/tilt/rate thresholds remain satisfied; timeout must fail rather than start anyway."},
        "metrics": {"complete_reference_duration_s": rows[-1]["t"],
                    "nominal_first_gate_absolute_s": float(race_first),
                    "nominal_last_gate_absolute_s": float(race_last),
                    "nominal_first_to_last_gate_s": float(race_last-race_first),
                    "sampled_peak_speed_mps": stats["peak_speed_mps"],
                    "sampled_peak_acceleration_mps2": stats["peak_acceleration_mps2"],
                    "sampled_force_axis_rate_peak_rad_s": stats["force_axis_rate_peak_rad_s"],
                    "flight_result": None},
        "actual_flight_validated": False, "new_simulator_or_physical_parameters": False,
        "scope": "Elevated rest start after preparation; not yet a nonzero-speed flying start at the race-start boundary. Gate timing still starts only at actual first crossing."
    }
    return rows, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=common.DEFAULT_BASE)
    parser.add_argument("--planner", type=Path, default=DEFAULT_PLANNER)
    parser.add_argument("--out-dir", type=Path, default=HERE/"hover_candidates")
    parser.add_argument("--factors", type=float, nargs="+", default=[1.16, 1.20, 1.30, 1.40, 1.50, 1.60])
    parser.add_argument("--climb-s", type=float, default=3.)
    parser.add_argument("--hold-s", type=float, default=2.)
    parser.add_argument("--dt", type=float, default=.002)
    args = parser.parse_args()
    original, baseline = common.read_rows(args.baseline)
    source_meta = args.baseline.with_suffix(".json").read_bytes()
    metadata = json.loads(source_meta)
    planner = load_planner(args.planner)
    manifest = {"family": "prehover_replanned", "baseline_sha256": common.sha256(baseline),
                "baseline_metadata_sha256": common.sha256(source_meta),
                "reused_planner_sha256": common.sha256(args.planner.read_bytes()),
                "generation_only_not_actual_flight": True, "candidates": []}
    pending = []
    for factor in args.factors:
        rows, report = generate(original, metadata, planner, factor, args.climb_s, args.hold_s, args.dt)
        name = "prehover_speed_"+format(factor, ".3f").replace(".", "p")
        data = common.encode_csv(rows)
        report.update(name=name, csv_sha256=common.sha256(data), baseline_sha256=manifest["baseline_sha256"])
        pending += [(args.out_dir/(name+".csv"), data),
                    (args.out_dir/(name+".json"), (json.dumps(report, indent=2, sort_keys=True)+"\n").encode())]
        manifest["candidates"].append({"name": name, "csv": name+".csv", "csv_sha256": report["csv_sha256"],
            "speed_factor": factor, "metrics": report["metrics"], "family": report["family"],
            "race_start_time_s": report["race_start_time_s"], "climb_end_time_s": report["climb_end_time_s"],
            "hover_release_time_s": report["hover_release_time_s"], "hover_target_ros_m": report["hover_target_ros_m"]})
    pending.append((args.out_dir/"CANDIDATES.json", (json.dumps(manifest, indent=2, sort_keys=True)+"\n").encode()))
    destinations = [p for p, data in pending]
    if len(set(destinations)) != len(destinations) or any(p.exists() for p in destinations):
        raise ValueError("Fresh output directory and distinct factors required")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for path, data in pending:
        common.write_new(path, data)
    for row in manifest["candidates"]:
        print(row["name"], "nominal_gate1_to_gate12_s", row["metrics"]["nominal_first_to_last_gate_s"],
              "peak_force_axis_rate", row["metrics"]["sampled_force_axis_rate_peak_rad_s"], row["csv_sha256"])


if __name__ == "__main__":
    main()
