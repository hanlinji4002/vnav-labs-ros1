#!/usr/bin/env python3
"""Closed-loop surrogate of the race: Lee controller (as controller_prehover)
plus a Unity-style 50 Hz rigid body with the 7 rad/s angular speed cap.

It is a screening model for route design, calibrated against archived real
flights; it is NOT the simulator and its predictions are never a result.
"""
import csv
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
RACE = HERE.parent
G = 9.81
MASS = 1.0
J = np.diag([0.1, 0.2, 0.1])
UNITY_MAX_ANGULAR_SPEED = 7.0     # Rigidbody.maxAngularVelocity default; never set by the simulator
PHYSICS_DT = 0.02                 # Unity fixedDeltaTime default; matches 50 Hz /current_state


def load_csv(path):
    if isinstance(path, tuple):
        return path
    with Path(path).open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    t = np.array([float(r['t']) for r in rows])
    cols = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']
    data = np.array([[float(r[c]) for c in cols] for r in rows])
    return t, data


def gates():
    g = json.loads((RACE/'gate_geometry_evidence.json').read_text())['gates']
    return [dict(c=np.array(r['root_world_position_ros_m']), n=np.array(r['gate_normal_ros']),
                 h=np.array(r['gate_horizontal_axis_ros']), v=np.array(r['gate_vertical_axis_ros'])) for r in g]


GATES = gates()


def smooth(u):
    u = min(1.0, max(0.0, u))
    return min(1.0, max(0.0, u*u*u*(10.0+u*(-15.0+6.0*u))))


def hat(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def vee(m):
    return np.array([m[2, 1], m[0, 2], m[1, 0]])


def transport(force, frame):
    """Rotate frame so its z axis points along force (Eigen FromTwoVectors)."""
    n = np.linalg.norm(force)
    if n < 1e-6:
        return None
    target = force/n
    a = frame[:, 2]
    c = float(np.dot(a, target))
    axis = np.cross(a, target)
    s = np.linalg.norm(axis)
    if s < 1e-12:
        if c > 0:
            return frame.copy()
        # antiparallel: rotate pi about any perpendicular axis
        perp = np.cross(a, [1, 0, 0])
        if np.linalg.norm(perp) < 1e-6:
            perp = np.cross(a, [0, 1, 0])
        rot = Rotation.from_rotvec(math.pi*perp/np.linalg.norm(perp)).as_matrix()
        return rot@frame
    rot = Rotation.from_rotvec(axis/s*math.atan2(s, c)).as_matrix()
    return rot@frame


def rotation_distance(a, b):
    return math.acos(max(-1.0, min(1.0, (np.trace(a.T@b)-1.0)*0.5)))


def bounded(v, m):
    n = np.linalg.norm(v)
    return v*(m/n) if n > m else v


class Controller:
    def __init__(self, kx=36., kv=12., kr=60., kw=4., omega_limit=7., tau=.01,
                 preview_s=.02, preview_xy=True, hover=(4., 3., 12., 1.), release=5., transition=.3,
                 end=60., signed=True, hysteresis=.2, er_rate_cap=None, eomega_cap=None):
        # er_rate_cap: cap |kr*eR| so the attitude P term never asks for more than
        # kw*er_rate_cap of body rate (the simulator clamps |omega| at 7 rad/s).
        self.er_rate_cap, self.eomega_cap = er_rate_cap, eomega_cap
        self.kx, self.kv, self.kr, self.kw = kx, kv, kr, kw
        self.omega_limit, self.tau = omega_limit, tau
        self.preview_s, self.preview_xy = preview_s, preview_xy
        self.hover, self.release, self.transition, self.end = hover, release, transition, end
        self.signed, self.hysteresis = signed, hysteresis
        self.ff_frame = None
        self.prev_rd = None
        self.prev_t = None
        self.omega_d = np.zeros(3)
        self.branch = 1
        self.branch_init = False
        self.derivative_ready = False

    def gains(self, plan_t):
        rising = smooth((plan_t-self.release)/self.transition)
        falling = 1.0-smooth((plan_t-(self.end-self.transition))/self.transition)
        w = min(rising, falling)
        hx, hv, hr, hw = self.hover
        return (hx+w*(self.kx-hx), hv+w*(self.kv-hv), hr+w*(self.kr-hr), hw+w*(self.kw-hw))

    def step(self, t, plan_t, x, v, R, omega_body, xd, vd, ad, a_preview):
        kx, kv, kr, kw = self.gains(plan_t)
        e3 = np.array([0., 0., 1.])
        force = -kx*(x-xd)-kv*(v-vd)+MASS*G*e3+MASS*ad
        att_acc = ad.copy()
        if a_preview is not None:
            att_acc = a_preview.copy()
            if self.preview_xy:
                att_acc[2] = ad[2]
        attitude_force = force+MASS*(att_acc-ad)
        positive = transport(attitude_force, R)
        if positive is None:
            return None
        if self.signed:
            negative = transport(-attitude_force, R)
            pc, nc = rotation_distance(R, positive), rotation_distance(R, negative)
            selected = self.branch
            if not self.branch_init:
                selected = 1 if pc <= nc else -1
            elif self.branch > 0 and nc+self.hysteresis < pc:
                selected = -1
            elif self.branch < 0 and pc+self.hysteresis < nc:
                selected = 1
            if self.branch_init and selected != self.branch:
                self.derivative_ready = False
            self.branch, self.branch_init = selected, True
            rd = positive if selected > 0 else negative
        else:
            rd = positive
        # reference-only feedforward frame
        nominal = self.branch*MASS*(G*e3+att_acc)
        if self.ff_frame is None:
            self.ff_frame = R.copy()
        rate_rotation = transport(nominal, self.ff_frame)
        valid = rate_rotation is not None
        if valid:
            self.ff_frame = rate_rotation
            self.update_rates(rate_rotation, t)
        else:
            self.derivative_ready = False
            self.omega_d = np.zeros(3)
        eR = 0.5*vee(rd.T@R-R.T@rd)
        if self.er_rate_cap is not None:
            eR = bounded(eR, kw*self.er_rate_cap/max(kr, 1e-9))
        rate_to_body = R.T@(self.prev_rd if self.prev_rd is not None else R)
        wd = bounded(self.omega_d, self.omega_limit) if valid else np.zeros(3)
        ref_rate = rate_to_body@wd
        e_omega = omega_body-ref_rate
        if self.eomega_cap is not None:
            e_omega = bounded(e_omega, self.eomega_cap)
        thrust = float(force@(R@e3))
        torque = -kr*eR-kw*e_omega+np.cross(omega_body, J@omega_body)-J@(np.cross(omega_body, ref_rate))
        return thrust, torque, rd

    def update_rates(self, rd, now):
        dt = (now-self.prev_t) if (self.derivative_ready and self.prev_t is not None) else 0.
        if not self.derivative_ready or dt < 0 or dt > .2:
            self.prev_rd, self.prev_t = rd, now
            self.omega_d = np.zeros(3)
            self.derivative_ready = True
            return
        if dt < 5e-4:
            return
        inc = Rotation.from_matrix(self.prev_rd.T@rd)
        raw = bounded(inc.as_rotvec()/dt, self.omega_limit)
        tr = rd.T@self.prev_rd
        old = tr@self.omega_d
        wgt = -math.expm1(-dt/self.tau) if self.tau > 0 else 1.0
        self.omega_d = bounded(old+wgt*(raw-old), self.omega_limit)
        self.prev_rd, self.prev_t = rd, now


def simulate(csv_path, release_plan_t=5.0, end_t=None, controller_kwargs=None, latency_steps=1,
             control_substeps=1, angular_cap=UNITY_MAX_ANGULAR_SPEED, gyroscopic=False, seed=None,
             ref_lag_s=0.0, preview_extra_s=0.0, timewarp=None):
    """timewarp=dict(e0=0.3, k=1.5, rmin=0.6): reference time rate = clip(1-k*max(err-e0,0), rmin, 1)
    using the measured position error (same lag as the controller's state)."""
    """Integrate from the hover release (plan time 5 s, vehicle at rest at the
    hover target) to the end of the plan. Returns per-gate crossing events."""
    times, data = load_csv(csv_path)
    if end_t is None:
        end_t = float(times[-1])
    controller = Controller(end=end_t, **(controller_kwargs or {}))

    def sample(pt):
        pt = min(max(pt, times[0]), times[-1])
        row = np.array([np.interp(pt, times, data[:, i]) for i in range(9)])
        return row[:3], row[3:6], row[6:9]

    x0, _, _ = sample(release_plan_t)
    x, v = x0.copy(), np.zeros(3)
    R = np.eye(3)
    omega_world = np.zeros(3)
    # measured state visible to the controller lags by latency_steps physics steps
    history = [(x.copy(), v.copy(), R.copy(), omega_world.copy())]*(latency_steps+1)
    t = 0.0
    events = []
    prev_x = x.copy()
    log = []
    thrust, torque = MASS*G, np.zeros(3)
    n_steps = int(round((end_t-release_plan_t)/PHYSICS_DT/0.5))+5
    virtual = 0.0
    warp_log = []
    for k in range(n_steps):
        mx, mv, mR, mw = history[-1-latency_steps]
        if timewarp is not None:
            xd_now, _, _ = sample(release_plan_t+virtual-ref_lag_s)
            err = float(np.linalg.norm(mx-xd_now))
            rate = min(1.0, max(timewarp.get('rmin', .6), 1.0-timewarp.get('k', 1.5)*max(err-timewarp.get('e0', .3), 0.)))
            warp_log.append(rate)
        else:
            rate = 1.0
        plan_t = release_plan_t+virtual
        if n_steps > 0 and k > 0:
            pass
        xd, vd, ad = sample(plan_t-ref_lag_s)
        _, _, ap = sample(plan_t-ref_lag_s+controller.preview_s+preview_extra_s)
        if timewarp is not None:
            # consistent slowed traversal: d/dt of x(virtual) = v*rate, a*rate^2 (rate-dot neglected)
            vd, ad, ap = vd*rate, ad*rate*rate, ap*rate*rate
        out = controller.step(t, plan_t, mx, mv, mR, mR.T@mw, xd, vd, ad, ap)
        if out is not None:
            thrust, torque, _ = out
        # Unity/PhysX semi-implicit Euler at 50 Hz, body torque applied in world axes
        acc = thrust*R[:, 2]/MASS-G*np.array([0., 0., 1.])
        v = v+acc*PHYSICS_DT
        x_new = x+v*PHYSICS_DT
        Jw = R@J@R.T
        tau_world = R@torque
        if gyroscopic:
            tau_world = tau_world-np.cross(omega_world, Jw@omega_world)
        omega_world = omega_world+np.linalg.solve(Jw, tau_world)*PHYSICS_DT
        omega_world = bounded(omega_world, angular_cap)
        R = Rotation.from_rotvec(omega_world*PHYSICS_DT).as_matrix()@R
        # gate crossings between x and x_new
        for gi, g in enumerate(GATES):
            s0 = float((x-g['c'])@g['n'])
            s1 = float((x_new-g['c'])@g['n'])
            if s0 < 0 <= s1:
                f = s0/(s0-s1)
                p = x+f*(x_new-x)
                events.append(dict(gate=gi+1, t=t+f*PHYSICS_DT, h=float((p-g['c'])@g['h']), v=float((p-g['c'])@g['v']),
                                   speed=float(np.linalg.norm(v))))
        x = x_new
        history.append((x.copy(), v.copy(), R.copy(), omega_world.copy()))
        tilt = math.degrees(math.acos(max(-1., min(1., R[2, 2]))))
        log.append(dict(t=t, x=x.copy(), v=v.copy(), tilt=tilt, omega=float(np.linalg.norm(omega_world)), err=float(np.linalg.norm(x-xd)), rate=rate))
        t += PHYSICS_DT
        virtual += PHYSICS_DT*rate
        if plan_t >= end_t and t > (end_t-release_plan_t)+1.0:
            break
    return events, log


def ordered_credits(events):
    """First forward crossing of each gate in order, like the real timer."""
    credited, nxt, last_t = [], 1, -1.0
    for e in sorted(events, key=lambda e: e['t']):
        if e['gate'] == nxt and e['t'] > last_t:
            credited.append(e)
            nxt += 1
            last_t = e['t']
    return credited


def summarize(csv_path, **kw):
    events, log = simulate(csv_path, **kw)
    cred = ordered_credits(events)
    result = dict(gates=len(cred), first_to_last=(cred[-1]['t']-cred[0]['t']) if len(cred) == 12 else None,
                  offsets=[(e['gate'], round(e['h'], 3), round(e['v'], 3)) for e in cred],
                  min_margin=min(.75-max(abs(e['h']), abs(e['v'])) for e in cred) if cred else None,
                  peak_tilt=max(l['tilt'] for l in log), peak_err=max(l['err'] for l in log),
                  gate1_speed=cred[0]['speed'] if cred else None,
                  min_rate=min(l['rate'] for l in log), warp_slowdown_s=float(sum((1-l['rate'])*PHYSICS_DT for l in log)),
                  omega_sat_fraction=float(np.mean([l['omega'] >= UNITY_MAX_ANGULAR_SPEED-1e-3 for l in log])))
    return result


if __name__ == '__main__':
    import sys
    for p in sys.argv[1:]:
        r = summarize(p)
        print(Path(p).name, json.dumps({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}))
