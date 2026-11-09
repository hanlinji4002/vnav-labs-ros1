#!/usr/bin/env python3
"""Independent normalized degree-9 minimum-snap planner and bounded search.

Only reads scene/plan inputs and writes a new CSV + JSON under --out-prefix.
The endpoint-derivative formulation is an exact elimination of the equality
KKT system: fixed positions, shared internal derivatives 1..4, zero endpoint
derivatives 1..4. No ROS, Docker, controller, or scene modification is used.
"""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import re
import time

import numpy as np
from scipy import linalg, optimize

BASE = Path(__file__).resolve().parent
DEFAULT_GATES = BASE.parent / "simulator/lab4/lab4_Data/StreamingAssets/vnav-2020-lab4_static_tfs.csv"


def derivative_basis(u, order):
    u = np.atleast_1d(u)
    result = np.zeros((len(u), 10))
    for power in range(order, 10):
        result[:, power] = math.factorial(power)/math.factorial(power-order)*u**(power-order)
    return result


ENDPOINT = np.vstack([derivative_basis([side], order) for side in (0.0, 1.0) for order in range(5)])
ENDPOINT_INV = linalg.inv(ENDPOINT)
Q = np.zeros((10, 10))
for _i in range(4, 10):
    for _j in range(4, 10):
        Q[_i, _j] = (math.factorial(_i)/math.factorial(_i-4)
                     * math.factorial(_j)/math.factorial(_j-4)/(_i+_j-7))


class MinimumSnap:
    """Linear map from fixed waypoint positions to C4 min-snap polynomials."""
    def __init__(self, durations):
        self.durations = np.asarray(durations, dtype=float)
        if self.durations.ndim != 1 or not np.all(np.isfinite(self.durations)) or np.any(self.durations <= 0):
            raise ValueError("durations must be positive finite segment times")
        self.n = len(self.durations)
        self.boundaries = np.r_[0.0, np.cumsum(self.durations)]
        count = (self.n+1)*5
        hessian = np.zeros((count, count))
        transforms = []
        for segment, duration in enumerate(self.durations):
            transform = ENDPOINT_INV*np.tile(duration**np.arange(5), 2)[None, :]
            local = transform.T@Q@transform/duration**7
            ids = np.r_[np.arange(segment*5, segment*5+5), np.arange((segment+1)*5, (segment+1)*5+5)]
            hessian[np.ix_(ids, ids)] += local
            transforms.append((ids, transform))
        hessian = (hessian+hessian.T)*0.5
        free = np.array([vertex*5+order for vertex in range(1, self.n) for order in range(1, 5)], dtype=int)
        positions = np.arange(0, count, 5)
        derivative_map = np.zeros((count, self.n+1))
        derivative_map[positions, np.arange(self.n+1)] = 1.0
        if len(free):
            reduced = hessian[np.ix_(free, free)]
            scales = 1.0/np.sqrt(np.diag(reduced))
            scaled = scales[:, None]*reduced*scales[None, :]
            rhs = -scales[:, None]*hessian[np.ix_(free, positions)]
            solution = linalg.cho_solve(linalg.cho_factor(scaled, check_finite=False), rhs, check_finite=False)
            derivative_map[free] = scales[:, None]*solution
        self.coefficient_map = np.array([transform@derivative_map[ids] for ids, transform in transforms])
        self.derivative_map = derivative_map
        self.hessian = hessian

    def maps(self, sample_times):
        t = np.asarray(sample_times, dtype=float)
        segment = np.minimum(np.searchsorted(self.boundaries, t, side="right")-1, self.n-1)
        segment = np.maximum(0, segment)
        u = np.clip((t-self.boundaries[segment])/self.durations[segment], 0.0, 1.0)
        output = [np.zeros((len(t), self.n+1)) for _ in range(5)]
        for index in range(self.n):
            mask = segment == index
            if not np.any(mask):
                continue
            for order in range(5):
                output[order][mask] = derivative_basis(u[mask], order)@self.coefficient_map[index]/self.durations[index]**order
        return output

    def objective_grid(self, per_segment=45):
        return np.concatenate([self.boundaries[i]+self.durations[i]*np.linspace(0, 1, per_segment, endpoint=False)
                               for i in range(self.n)]+[self.boundaries[-1:]])

    def coefficients(self, positions):
        return np.einsum("skv,vc->skc", self.coefficient_map, positions)


def load_gates(path):
    spec = importlib.util.spec_from_file_location("optimized_geometry", BASE/"offline_geometry.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    gates = module.load_gates(path)
    return gates


def read_plan(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as handle:
        rows = list(csv.DictReader(handle))
    names = ["t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az"]
    values = np.array([[float(row[name]) for name in names] for row in rows])
    yaw = float(rows[0].get("yaw", -2.90332))
    return values, yaw


def parse_durations(path):
    text = Path(path).read_text(encoding="utf-8")
    match = re.search(r"segment times:\s*([0-9. eE+\-]+)", text)
    if not match:
        raise ValueError("no segment times in log")
    durations = np.array([float(value) for value in match.group(1).split()])
    if len(durations) != 13:
        raise ValueError("expected 13 segment durations")
    return durations


def route(gates, spawn, runout=5.0, waypoint_height=0.0, vertical_limit=0.7):
    centers = np.array([gate["center_ros_m"] for gate in gates])
    normal = np.asarray(gates[-1]["normal"])
    horizontal_normal = np.r_[normal[:2], 0.0]
    horizontal_normal /= np.linalg.norm(horizontal_normal)
    positions = np.vstack([spawn, centers, centers[-1]+runout*horizontal_normal])
    directions = np.zeros((14, 3, 24))
    initial = np.zeros(24)
    for index, gate in enumerate(gates):
        directions[index+1, :, index] = gate["horizontal"]
        directions[index+1, :, 12+index] = gate["vertical"]
        if waypoint_height > 0:
            initial[12+index] = np.clip(waypoint_height-centers[index, 2], -vertical_limit, vertical_limit)
    positions[-1, 2] += initial[-1]
    return positions, directions, initial


def fit_truncated_times(durations, positions, reference, maxiter=80):
    """The C++ logger truncates each positive duration to three decimals."""
    lower = np.asarray(durations)
    # Work on unit-box fractions for stable numerical differentiation.
    scales = np.array([1.0]*3+[0.05]*3+[0.005]*3)
    def residual(fractions):
        solver = MinimumSnap(lower+0.001*fractions)
        mapped = solver.maps(reference[:, 0])
        generated = np.hstack([mapped[order]@positions for order in range(3)])
        return ((generated-reference[:, 1:10])*scales).ravel()
    fitted = optimize.least_squares(residual, np.full(13, 0.5), bounds=(np.zeros(13), np.ones(13)),
                                    max_nfev=maxiter, ftol=1e-10, xtol=1e-10, gtol=1e-10)
    exact = lower+0.001*fitted.x
    generated = np.hstack([matrix@positions for matrix in MinimumSnap(exact).maps(reference[:, 0])[:3]])
    error = generated-reference[:, 1:10]
    return exact, {"success": bool(fitted.success), "evaluations": int(fitted.nfev),
                   "duration_logging": "positive seconds truncated to 3 decimals by std::to_string(...).substr(0,5)",
                   "position_rms_m": float(np.sqrt(np.mean(error[:, :3]**2))),
                   "position_max_m": float(np.max(np.linalg.norm(error[:, :3], axis=1))),
                   "velocity_rms_mps": float(np.sqrt(np.mean(error[:, 3:6]**2))),
                   "acceleration_rms_mps2": float(np.sqrt(np.mean(error[:, 6:9]**2)))}


def metrics(sample_times, values, last_gate_time, rate_limit=7.0):
    position, velocity, acceleration, jerk = values[:4]
    force = acceleration+np.array([0.0, 0.0, 9.81])
    norm_force = np.linalg.norm(force, axis=1)
    rates = np.linalg.norm(np.cross(force, jerk), axis=1)/np.maximum(norm_force**2, 1e-12)
    before_finish = sample_times <= last_gate_time+1e-9
    dt = np.diff(sample_times, append=sample_times[-1])
    return {"total_duration_s": float(sample_times[-1]), "start_to_last_gate_nominal_s": float(last_gate_time),
            "force_axis_rate_peak_rad_s": float(np.max(rates)),
            "force_axis_rate_p95_rad_s": float(np.percentile(rates, 95)),
            "force_axis_rate_over_limit_s": float(np.sum(dt[rates > rate_limit])),
            "force_axis_rate_pre_finish_peak_rad_s": float(np.max(rates[before_finish])),
            "force_axis_rate_pre_finish_over_limit_s": float(np.sum(dt[(rates > rate_limit)&before_finish])),
            "force_axis_rate_limit_rad_s": rate_limit,
            "peak_speed_mps": float(np.max(np.linalg.norm(velocity, axis=1))),
            "peak_acceleration_mps2": float(np.max(np.linalg.norm(acceleration, axis=1))),
            "peak_downward_acceleration_mps2": float(np.max(-acceleration[:, 2])),
            "minimum_force_per_mass_mps2": float(np.min(norm_force)),
            "minimum_height_m": float(np.min(position[:, 2])),
            "rate_note": "Analytic ||(a+g e3) cross jerk|| / ||a+g e3||^2; yaw-free instantaneous force-axis demand, not actual vehicle body rate.",
            "competition_pass": None}


class OffsetObjective:
    def __init__(self, durations, positions, directions, samples=45, rate_limit=7.0, acc_limit=100.0, down_limit=25.0):
        self.solver = MinimumSnap(durations)
        self.t = self.solver.objective_grid(samples)
        self.maps = self.solver.maps(self.t)
        self.positions, self.directions = positions, directions
        self.a0 = self.maps[2]@positions
        self.j0 = self.maps[3]@positions
        self.da = np.einsum("nv,vck->nck", self.maps[2], directions)
        self.dj = np.einsum("nv,vck->nck", self.maps[3], directions)
        self.rate_limit, self.acc_limit, self.down_limit = rate_limit, acc_limit, down_limit

    def __call__(self, offsets):
        acceleration = self.a0+np.einsum("nck,k->nc", self.da, offsets)
        jerk = self.j0+np.einsum("nck,k->nc", self.dj, offsets)
        force = acceleration+np.array([0.0, 0.0, 9.81])
        s = np.maximum(np.sum(force*force, axis=1), 1e-8)
        q = np.sum(force*jerk, axis=1)
        r = np.sum(jerk*jerk, axis=1)
        rates2 = np.maximum(r/s-q*q/s**2, 1e-12)
        rates = np.sqrt(rates2)
        excess = np.maximum(rates/self.rate_limit-1, 0.0)
        # Integrated violation plus a weak demand regularizer, both differentiable.
        weights = 10.0*excess/(self.rate_limit*rates)+0.01/self.rate_limit**2
        dforce = (-2*r[:, None]*force/s[:, None]**2-2*q[:, None]*jerk/s[:, None]**2
                  +4*q[:, None]**2*force/s[:, None]**3)*weights[:, None]
        djerk = (2*jerk/s[:, None]-2*q[:, None]*force/s[:, None]**2)*weights[:, None]
        terms = 10.0*excess**2+0.01*rates2/self.rate_limit**2
        norm_a = np.maximum(np.linalg.norm(acceleration, axis=1), 1e-9)
        if self.acc_limit > 0:
            over_acc = np.maximum(norm_a/self.acc_limit-1, 0.0)
            terms += 5*over_acc**2
            dforce += (10*over_acc/(self.acc_limit*norm_a))[:, None]*acceleration
        if self.down_limit > 0:
            over_down = np.maximum(-acceleration[:, 2]/self.down_limit-1, 0.0)
            terms += 5*over_down**2
            dforce[:, 2] -= 10*over_down/self.down_limit
        norm_f = np.sqrt(s)
        under = np.maximum(1-norm_f/2.0, 0.0)
        terms += under**2
        dforce -= (under/norm_f)[:, None]*force
        value = float(np.mean(terms)+1e-5*np.mean(offsets**2))
        gradient = (np.einsum("nc,nck->k", dforce, self.da)+np.einsum("nc,nck->k", djerk, self.dj))/len(self.t)
        gradient += 2e-5*offsets/len(offsets)
        return value, gradient


def optimize_offsets(durations, positions, directions, initial, args):
    objective = OffsetObjective(durations, positions, directions, args.objective_samples,
                                args.rate_limit, args.acc_limit, args.down_limit)
    bounds = [(-args.horizontal_limit, args.horizontal_limit)]*12+[(-args.vertical_limit, args.vertical_limit)]*12
    result = optimize.minimize(objective, initial, method="L-BFGS-B", jac=True, bounds=bounds,
                               options={"maxiter": args.iterations, "ftol": 1e-11, "maxls": 35})
    return result.x, {"phase": "offsets", "success": bool(result.success), "message": str(result.message),
                      "iterations": int(result.nit), "objective": float(result.fun)}


def optimize_times(durations, positions, directions, offsets, args):
    initial = np.array(durations)
    def objective(factors):
        times = initial*factors
        loss = OffsetObjective(times, positions, directions, args.objective_samples,
                               args.rate_limit, args.acc_limit, args.down_limit)(offsets)[0]
        return loss+0.0001*np.sum(times)
    constraint = {"type": "ineq", "fun": lambda factors: args.target_s-np.sum(initial[:12]*factors[:12])}
    result = optimize.minimize(objective, np.ones(13), method="SLSQP", bounds=[(0.65, 1.65)]*13,
                               constraints=[constraint], options={"maxiter": args.iterations, "ftol": 1e-9})
    return initial*result.x, {"phase": "times", "success": bool(result.success), "message": str(result.message),
                             "iterations": int(result.nit), "objective": float(result.fun)}


def sampling_grid(boundaries, dt):
    regular = np.arange(0.0, boundaries[-1], dt)
    keep = np.ones(len(regular), dtype=bool)
    # Preserve exact vertex instants without creating nearly-zero Hermite
    # intervals when a vertex almost coincides with the regular sample grid.
    merge_distance = max(1e-8, dt*0.05)
    for boundary in boundaries:
        keep &= np.abs(regular-boundary) > merge_distance
    return np.sort(np.r_[regular[keep], boundaries])


def export(prefix, durations, positions, directions, offsets, yaw, args, extra):
    solver = MinimumSnap(durations)
    shifted = positions+np.einsum("vck,k->vc", directions, offsets)
    t = sampling_grid(solver.boundaries, args.dt)
    maps = solver.maps(t)
    values = [matrix@shifted for matrix in maps]
    coefficients = solver.coefficients(shifted)
    residuals = {}
    for order in range(5):
        joins = [float(np.max(np.abs(derivative_basis([1.0], order)@coefficients[index]/durations[index]**order
                                    -derivative_basis([0.0], order)@coefficients[index+1]/durations[index+1]**order)))
                 for index in range(len(durations)-1)]
        residuals["internal_order_{}_max_abs".format(order)] = max(joins, default=0.0)
        if order:
            residuals["endpoint_order_{}_max_abs".format(order)] = float(max(
                np.max(np.abs(derivative_basis([0.0], order)@coefficients[0]/durations[0]**order)),
                np.max(np.abs(derivative_basis([1.0], order)@coefficients[-1]/durations[-1]**order))))
    residuals["waypoint_position_max_abs_m"] = float(np.max(np.abs(solver.maps(solver.boundaries)[0]@shifted-shifted)))
    prefix = Path(prefix)
    prefix.parent.mkdir(parents=True, exist_ok=True)
    csv_path = Path(str(prefix)+".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "ax", "ay", "az", "yaw"])
        for index, instant in enumerate(t):
            writer.writerow([format(float(value), ".17g") for value in np.r_[instant, values[0][index], values[1][index], values[2][index], yaw]])
    report = dict(extra, sampled_csv=str(csv_path), segment_durations_s=list(map(float, durations)),
                  waypoints_ros_m=shifted.tolist(), horizontal_offsets_m=offsets[:12].tolist(),
                  vertical_offsets_m=offsets[12:].tolist(), normalized_polynomial_coefficients=coefficients.tolist(),
                  equality_constraint_residuals=residuals,
                  metrics=metrics(t, values, solver.boundaries[12], args.rate_limit),
                  constraints="Degree 9, endpoint derivatives 1..4 zero, internal C4, positions fixed at shifted waypoints.",
                  aperture_note="Offsets are search bounds, not verified physical aperture/vehicle clearance.")
    Path(str(prefix)+".json").write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(csv_path), "metrics": report["metrics"], "search": extra.get("search")}, ensure_ascii=False), flush=True)
    return report


def self_test():
    solver = MinimumSnap([0.6, 0.7])
    points = np.array([[0.0, 0.0, 0.2], [1.0, 0.3, 1.0], [2.0, 0.0, 0.5]])
    c = solver.coefficients(points)
    for derivative in range(1, 5):
        assert np.max(np.abs(derivative_basis([0], derivative)@c[0]/0.6**derivative)) < 1e-5
        assert np.max(np.abs(derivative_basis([1], derivative)@c[-1]/0.7**derivative)) < 1e-5
    for derivative in range(5):
        left = derivative_basis([1], derivative)@c[0]/0.6**derivative
        right = derivative_basis([0], derivative)@c[1]/0.7**derivative
        assert np.max(np.abs(left-right)) < 1e-5
    assert np.max(np.abs(solver.maps([0.0, 0.6, 1.3])[0]@points-points)) < 1e-7
    grid = sampling_grid(np.array([0.0, 1.0+1e-12, 2.0]), 0.002)
    assert 1.0+1e-12 in grid
    assert np.min(np.diff(grid)) > 1e-4  # No nearly singular quintic intervals.
    directions = np.zeros((3, 3, 24))
    directions[1, 0, 0], directions[1, 2, 1] = 1.0, 1.0
    objective = OffsetObjective([0.6, 0.7], points, directions, samples=30)
    offsets = np.zeros(24)
    offsets[:2] = [0.13, 0.07]
    _, gradient = objective(offsets)
    for index in range(2):
        delta = np.zeros(24)
        delta[index] = 1e-6
        numerical = (objective(offsets+delta)[0]-objective(offsets-delta)[0])/(2e-6)
        assert abs(numerical-gradient[index]) < 1e-4*max(1.0, abs(numerical)), (numerical, gradient[index])
    print("self-test: endpoint derivatives, C4 joins, waypoints, objective gradient, sampling grid OK")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--gates", type=Path, default=DEFAULT_GATES)
    parser.add_argument("--timing-log", type=Path, default=BASE/"plans/a100_d25_s1.log")
    parser.add_argument("--reference", type=Path, default=BASE/"plans/a100_d25_s1.csv")
    parser.add_argument("--exact-times", type=Path, help="optional full-precision 13 segment times (whitespace or comma separated)")
    parser.add_argument("--out-prefix", type=Path, default=BASE/"optimized/minsnap_candidate")
    parser.add_argument("--mode", choices=("none", "offsets", "times", "both"), default="offsets")
    parser.add_argument("--skip-fit", action="store_true")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--iterations", type=int, default=120)
    parser.add_argument("--rounds", type=int, default=1)
    for name, default in (("runout", 5.0), ("horizontal-limit", 0.6), ("vertical-limit", 0.7),
                          ("rate-limit", 7.0), ("acc-limit", 100.0), ("down-limit", 25.0),
                          ("target-s", 4.99), ("dt", 0.005), ("height", 0.0), ("initial-height-limit", 0.7)):
        parser.add_argument("--"+name, type=float, default=default)
    parser.add_argument("--objective-samples", type=int, default=45)
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if args.dt <= 0 or args.iterations < 1 or args.rounds < 1:
        parser.error("positive dt/iterations/rounds required")
    for stem in (str(args.out_prefix), str(args.out_prefix)+"_baseline"):
        if any(Path(stem+extension).exists() for extension in (".csv", ".json")):
            parser.error("output already exists; choose a fresh --out-prefix: "+stem)
    started = time.monotonic()
    reference, yaw = read_plan(args.reference)
    gates = load_gates(args.gates)
    positions, directions, offsets = route(gates, reference[0, 1:4], args.runout, args.height, args.initial_height_limit)
    durations = parse_durations(args.timing_log)
    fitted = None
    if args.exact_times is not None:
        durations = np.array([float(item) for item in re.split(r"[\s,]+", args.exact_times.read_text().strip()) if item])
        if len(durations) != 13:
            parser.error("exact-times needs 13 values")
        fitted = {"source": str(args.exact_times), "full_precision_times": True}
    elif not args.skip_fit:
        shifted = positions+np.einsum("vck,k->vc", directions, offsets)
        durations, fitted = fit_truncated_times(durations, shifted, reference)
    initial = export(str(args.out_prefix)+"_baseline", durations, positions, directions, offsets, yaw, args,
                     {"baseline_fit": fitted, "search": []})
    print("baseline fit: "+json.dumps(fitted), flush=True)
    search = []
    for _ in range(args.rounds):
        if args.mode in ("offsets", "both"):
            offsets, info = optimize_offsets(durations, positions, directions, offsets, args)
            search.append(info)
            print(json.dumps(info), flush=True)
        if args.mode in ("times", "both"):
            durations, info = optimize_times(durations, positions, directions, offsets, args)
            search.append(info)
            print(json.dumps(info), flush=True)
    export(args.out_prefix, durations, positions, directions, offsets, yaw, args,
           {"baseline_fit": fitted, "baseline_metrics": initial["metrics"], "search": search,
            "offline_wall_elapsed_s": time.monotonic()-started})


if __name__ == "__main__":
    main()
