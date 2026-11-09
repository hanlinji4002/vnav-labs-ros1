#!/usr/bin/env python3
"""Create separate, derivative-consistent LAB4 reference-speed candidates.

This changes commanded reference motion, never simulator time, physics, gates,
odometry or video timestamps. The historical CSV and parameter profile stay
untouched. Candidate metadata contains predictions, not new flight results.
"""
import argparse
import bisect
import csv
import hashlib
import io
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
BASE_NAME = "balanced_t494_h25_v25_clean.csv"
PORTABLE_BASE = HERE.parent / "optimized" / BASE_NAME
PROJECT_BASE = HERE.parents[1] / "竞速冲刺5秒/optimized" / BASE_NAME
# The native WSL payload keeps the original bytes under root/optimized; the
# Mac source tree keeps them in LAB4/竞速冲刺5秒. Both remain hash-pinned.
DEFAULT_BASE = PORTABLE_BASE if PORTABLE_BASE.is_file() else PROJECT_BASE
BASE_SHA256 = "e1883baf87bba068a60445275db3e70d00999303eac137dcc0b0d83486c7dad2"
FIELDS = ("t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "yaw")
VELOCITY = ("vx", "vy", "vz")
ACCELERATION = ("ax", "ay", "az")


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def read_rows(path, verify_identity=True):
    data = Path(path).read_bytes()
    if verify_identity and sha256(data) != BASE_SHA256:
        raise ValueError("Historical baseline identity mismatch")
    reader = csv.DictReader(io.StringIO(data.decode("utf-8-sig")))
    if tuple(reader.fieldnames or ()) != FIELDS:
        raise ValueError("Expected exact t,p,v,a,yaw reference schema")
    rows = [{key: float(row[key]) for key in FIELDS} for row in reader]
    validate_rows(rows)
    return rows, data


def validate_rows(rows):
    if len(rows) < 2 or abs(rows[0]["t"]) > 1e-9:
        raise ValueError("Reference needs at least two knots starting at zero")
    if any(not all(math.isfinite(row[key]) for key in FIELDS) for row in rows):
        raise ValueError("Non-finite reference knot")
    if any(b["t"]-a["t"] < 1e-6 for a, b in zip(rows, rows[1:])):
        raise ValueError("Reference clock must increase by at least one microsecond")


def scale_rows(rows, factor):
    if not math.isfinite(factor) or not 1.0 <= factor <= 1.3:
        raise ValueError("Speed factor must be finite and within 1.0..1.3")
    result = []
    for old in rows:
        row = dict(old)
        row["t"] /= factor
        for key in VELOCITY:
            row[key] *= factor
        for key in ACCELERATION:
            row[key] *= factor**2
        result.append(row)
    validate_rows(result)
    return result


def encode_csv(rows):
    handle = io.StringIO(newline="")
    writer = csv.writer(handle, lineterminator="\n")
    writer.writerow(FIELDS)
    for row in rows:
        writer.writerow([format(row[key], ".17g") for key in FIELDS])
    return handle.getvalue().encode("ascii")


def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def norm(a):
    return math.sqrt(sum(x*x for x in a))


def analytic_jerk(t, metadata):
    durations = metadata["segment_durations_s"]
    bounds = [0.0]
    for duration in durations:
        bounds.append(bounds[-1]+duration)
    segment = min(len(durations)-1, max(0, bisect.bisect_right(bounds, t)-1))
    duration = durations[segment]
    u = min(1.0, max(0.0, (t-bounds[segment])/duration))
    coefficients = metadata["normalized_polynomial_coefficients"][segment]
    return [sum(k*(k-1)*(k-2)*coefficients[k][axis]*u**(k-3)
                for k in range(3, len(coefficients)))/duration**3 for axis in range(3)]


def candidate_metrics(original, rows, factor, metadata):
    speeds, accelerations, rates, forces = [], [], [], []
    for old, row in zip(original, rows):
        acceleration = [row[k] for k in ACCELERATION]
        force = [acceleration[0], acceleration[1], acceleration[2]+9.81]
        jerk = [factor**3*x for x in analytic_jerk(old["t"], metadata)]
        magnitude = norm(force)
        rates.append(norm(cross(force, jerk))/max(1e-12, magnitude*magnitude))
        speeds.append(norm([row[k] for k in VELOCITY]))
        accelerations.append(norm(acceleration))
        forces.append(magnitude)
    durations = metadata["segment_durations_s"]
    return {
        "complete_reference_duration_s": rows[-1]["t"],
        "nominal_reference_to_last_gate_s": sum(durations[:12])/factor,
        "nominal_first_to_last_gate_s": sum(durations[1:12])/factor,
        "sampled_peak_speed_mps": max(speeds),
        "sampled_peak_acceleration_mps2": max(accelerations),
        "sampled_force_axis_rate_peak_rad_s": max(rates),
        "sampled_min_force_per_mass_mps2": min(forces),
        "rate_definition": "||F cross jerk||/||F||^2, F=a+9.81*e3; analytic source-polynomial jerk; evaluated at CSV knots",
        "flight_result": None,
    }


def write_new(path, data):
    with Path(path).open("xb") as handle:
        handle.write(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--out-dir", type=Path, default=HERE/"candidates")
    parser.add_argument("--factors", type=float, nargs="+", default=[1.04, 1.08, 1.12, 1.16, 1.20])
    args = parser.parse_args()
    rows, source = read_rows(args.baseline)
    metadata_data = args.baseline.with_suffix(".json").read_bytes()
    metadata = json.loads(metadata_data)
    pending = []
    manifest = {"baseline_path": str(args.baseline), "baseline_sha256": sha256(source),
                "baseline_metadata_sha256": sha256(metadata_data),
                "time_rule": "FIRST_GATE_TO_LAST_GATE, actual odometry; predictions below are reference-only",
                "historical_windows_gpu_video_04_actual_first_to_last_sim_s": 4.0952090955156235,
                "altered_simulator_clock_or_physics": False, "candidates": []}
    for factor in args.factors:
        scaled = scale_rows(rows, factor)
        name = "balanced_speed_"+format(factor, ".3f").replace(".", "p")
        csv_data = encode_csv(scaled)
        report = {"name": name, "speed_factor": factor,
                  "baseline_sha256": sha256(source), "csv_sha256": sha256(csv_data),
                  "scaling": "t_new=t/factor; p_new=p; v_new=factor*v; a_new=factor^2*a; yaw unchanged",
                  "metrics": candidate_metrics(rows, scaled, factor, metadata),
                  "simulator_or_gate_assets_changed": False, "live_validated": False}
        pending += [(args.out_dir/(name+".csv"), csv_data),
                    (args.out_dir/(name+".json"), (json.dumps(report, indent=2, sort_keys=True)+"\n").encode())]
        manifest["candidates"].append(dict(name=name, csv=name+".csv", csv_sha256=sha256(csv_data),
                                           speed_factor=factor, metrics=report["metrics"]))
    pending.append((args.out_dir/"CANDIDATES.json", (json.dumps(manifest, indent=2, sort_keys=True)+"\n").encode()))
    paths = [path for path, _ in pending]
    if len(set(paths)) != len(paths) or any(path.exists() for path in paths):
        raise ValueError("Output already exists or duplicate factor; choose a fresh destination")
    args.out_dir.mkdir(parents=True, exist_ok=True)
    for path, data in pending:
        write_new(path, data)
    for row in manifest["candidates"]:
        m = row["metrics"]
        print("{} nominal gate1->gate12={:.6f}s, v={:.3f}m/s, a={:.3f}m/s2, force_rate={:.3f}rad/s, SHA256={}".format(
            row["name"], m["nominal_first_to_last_gate_s"], m["sampled_peak_speed_mps"],
            m["sampled_peak_acceleration_mps2"], m["sampled_force_axis_rate_peak_rad_s"], row["csv_sha256"]))


if __name__ == "__main__":
    main()
