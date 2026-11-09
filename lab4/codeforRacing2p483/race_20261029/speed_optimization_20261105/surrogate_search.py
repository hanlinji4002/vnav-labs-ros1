#!/usr/bin/env python3
"""Route search against the closed-loop surrogate (7 rad/s Unity cap in the loop).

Variables: 24 gate-plane waypoint offsets and 13 segment durations of the
original 14-waypoint prehover route. Feasibility = ordered 12 crossings with a
surrogate point margin above a threshold for the nominal model AND two gain
perturbations. Predictions are screening only; TESSE flight decides.
"""
import copy
import csv
import io
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RACE = HERE.parent
sys.path.insert(0, str(RACE/'route_optimization_20261030'))
sys.path.insert(0, str(HERE))
import optimize_route as base  # noqa: E402
import make_local_adjustments as local  # noqa: E402
import closed_loop_surrogate as S  # noqa: E402

TEMPLATE = RACE/'hover_refined_candidates/prehover_speed_1p260.json'
ENSEMBLE = {'nominal': dict(latency_steps=1),
            'kx40': dict(latency_steps=1, controller_kwargs=dict(kx=40, kv=13)),
            'kx32': dict(latency_steps=1, controller_kwargs=dict(kx=32, kv=11))}


BIAS_FILE = HERE/'surrogate_bias_20261105.json'


def load_bias(path=BIAS_FILE):
    if not path.exists():
        return None
    b = json.loads(path.read_text())
    floor, mult = b.get('sd_floor', 0.), b.get('sd_multiplier', 1.)
    return dict(h=np.array([b['bias_h'][str(g)] for g in range(1, 13)]), v=np.array([b['bias_v'][str(g)] for g in range(1, 13)]),
                sh=mult*np.maximum(floor, [b['sd_h'][str(g)] for g in range(1, 13)]),
                sv=mult*np.maximum(floor, [b['sd_v'][str(g)] for g in range(1, 13)]))


def corrected_margin(offsets12, bias):
    """Point margin after adding the empirical bias and subtracting one sd."""
    if not offsets12 or len(offsets12) != 12:
        return -9.
    m = []
    for g, h, v in offsets12:
        i = g-1
        if bias is None:
            m.append(.75-max(abs(h), abs(v)))
        else:
            m.append(min(.75-abs(h+bias['h'][i])-bias['sh'][i], .75-abs(v+bias['v'][i])-bias['sv'][i]))
    return float(min(m))


class Search:
    def __init__(self, planned_limit=.40, threshold=.20, log=None, use_bias=True, sim_kwargs=None, bias_path=None):
        # sim_kwargs: e.g. dict(angular_cap=14, controller_kwargs=dict(omega_limit=14, kr=90)) for MODIFIED-physics studies
        self.sim_kwargs = sim_kwargs or {}
        self.bias = (load_bias(bias_path) if bias_path else load_bias()) if use_bias else None
        self.data = base.inputs()
        self.template = json.loads(TEMPLATE.read_text())
        self.base_d = np.array(self.template['race_segment_durations_s'])
        self.base_p = np.array(self.template['race_waypoints_ros_m'])
        self.centers = self.data['centers']
        self.H, self.V = self.data['horizontal'], self.data['vertical']
        self.planned_limit, self.threshold = planned_limit, threshold
        self.evals = 0
        self.log = log

    def points(self, offsets, hover_y=None):
        p = self.base_p.copy()
        if hover_y is not None:
            # Door_Left_01 was struck at y~10.3 (runup130_fullschedule_20261030_01 bag); stay well short of it.
            if not 8.0 <= hover_y <= 9.7:
                raise ValueError('hover y outside the known-free runway 8.0..9.7')
            p[0, 1] = hover_y
        for g in range(12):
            h, v = offsets[g], offsets[12+g]
            p[g+1] = self.centers[g]+h*self.H[g]+v*self.V[g]
        return p

    def base_offsets(self):
        return np.r_[[float((self.base_p[g+1]-self.centers[g])@self.H[g]) for g in range(12)],
                     [float((self.base_p[g+1]-self.centers[g])@self.V[g]) for g in range(12)]]

    def build(self, durations, offsets, name='search', hover_y=None):
        points = self.points(np.clip(offsets, -self.planned_limit, self.planned_limit), hover_y)
        raw, report, audit = local.rebuild(self.data, copy.deepcopy(self.template), np.array(durations), points, name, 1.4, {})
        report['hover_target_ros_m'] = points[0].tolist()
        rows = list(csv.DictReader(io.StringIO(raw.decode())))
        t = np.array([float(r['t']) for r in rows])
        cols = ['x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az']
        arr = np.array([[float(r[c]) for c in cols] for r in rows])
        return raw, report, (t, arr)

    def evaluate(self, durations, offsets, hover_y=None):
        self.evals += 1
        try:
            raw, report, table = self.build(durations, offsets, hover_y=hover_y)
        except (AssertionError, ValueError) as error:
            return dict(feasible=False, reason='build: '+str(error)[:80], margin=-9, scored=float(np.sum(durations[1:12])))
        results = {}
        for name, kw in ENSEMBLE.items():
            merged = dict(kw)
            merged.update({k: v for k, v in self.sim_kwargs.items() if k != 'controller_kwargs'})
            merged['controller_kwargs'] = {**kw.get('controller_kwargs', {}), **self.sim_kwargs.get('controller_kwargs', {})}
            r = S.summarize(table, **merged)
            results[name] = r
        margins = [corrected_margin(r['offsets'], self.bias) if r['gates'] == 12 else -9 for r in results.values()]
        margin = min(margins)
        nominal = results['nominal']
        predicted = np.zeros(24)
        if nominal['gates'] == 12:
            for g, h, v in nominal['offsets']:
                predicted[g-1], predicted[12+g-1] = h, v
        out = dict(feasible=margin >= self.threshold, margin=float(margin), margins={k: round(m, 3) for k, m in zip(results, margins)},
                   scored=float(np.sum(durations[1:12])), predicted_time=nominal['first_to_last'], predicted=predicted, hover_y=hover_y,
                   gate1_speed=nominal.get('gate1_speed'),
                   peak_tilt=nominal['peak_tilt'], sat=nominal['omega_sat_fraction'], raw=raw, report=report)
        if self.log is not None:
            self.log.append({k: v for k, v in out.items() if k not in ('raw', 'report', 'predicted')})
        return out

    def recenter(self, durations, offsets, rounds=4, gain=.6):
        best = self.evaluate(durations, offsets)
        for _ in range(rounds):
            if best['margin'] <= -9:
                break
            trial = np.clip(offsets-gain*best['predicted'], -self.planned_limit, self.planned_limit)
            cand = self.evaluate(durations, trial)
            if cand['margin'] > best['margin']+1e-3:
                offsets, best = trial, cand
            else:
                break
        return offsets, best

    def run(self, max_rounds=40, shrink=.02, verbose=True):
        durations = self.base_d.copy()
        offsets = self.base_offsets()
        offsets, best = self.recenter(durations, offsets)
        if verbose:
            print('start', round(best['scored'], 4), 'margin', round(best['margin'], 3), best['margins'], flush=True)
        history = [(durations.copy(), offsets.copy(), best)]
        stalled = 0
        for rnd in range(max_rounds):
            improved = False
            # 1) uniform shrink of scored segments
            trial_d = durations.copy()
            trial_d[1:12] *= (1-shrink)
            cand = self.evaluate(trial_d, offsets)
            if cand['feasible']:
                durations, best, improved = trial_d, cand, True
            else:
                # 2) best single-segment shrink (scored segments and the approach)
                options = []
                for seg in range(0, 12):
                    d = durations.copy()
                    d[seg] *= (1-2*shrink)
                    c = self.evaluate(d, offsets)
                    options.append((c['margin'], seg, d, c))
                options.sort(key=lambda o: -o[0])
                for m, seg, d, c in options:
                    if c['feasible'] and seg >= 1:
                        durations, best, improved = d, c, True
                        break
                if not improved:
                    # 3) re-centre waypoints and try the uniform shrink again
                    new_off, re = self.recenter(durations, offsets, rounds=3)
                    if re['margin'] > best['margin']+1e-3:
                        offsets, best = new_off, re
                        cand = self.evaluate(trial_d, offsets)
                        if cand['feasible']:
                            durations, best, improved = trial_d, cand, True
            if verbose:
                print('round', rnd, 'scored', round(best['scored'], 4), 'pred', round(best['predicted_time'] or 0, 4), 'margin', round(best['margin'], 3),
                      best['margins'], 'tilt', round(best['peak_tilt'], 1), 'evals', self.evals, flush=True)
            if improved:
                history.append((durations.copy(), offsets.copy(), best))
                stalled = 0
            else:
                stalled += 1
                if stalled >= 2:
                    break
        return history


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--threshold', type=float, default=.20)
    ap.add_argument('--out', type=Path, default=HERE/'surrogate_search_20261105.json')
    a = ap.parse_args()
    log = []
    search = Search(threshold=a.threshold, log=log)
    start = time.monotonic()
    history = search.run()
    final_d, final_o, final = history[-1]
    doc = dict(threshold=a.threshold, ensemble={k: {kk: vv for kk, vv in v.items()} for k, v in ENSEMBLE.items()},
               evaluations=search.evals, wall_s=time.monotonic()-start,
               final=dict(durations=final_d.tolist(), offsets=final_o.tolist(), scored=final['scored'], predicted_time=final['predicted_time'],
                          margin=final['margin'], margins=final['margins'], peak_tilt=final['peak_tilt']),
               steps=[dict(scored=h[2]['scored'], margin=h[2]['margin'], durations=h[0].tolist(), offsets=h[1].tolist()) for h in history],
               evaluation_log=log, scope='surrogate screening only; not a flight result')
    a.out.write_text(json.dumps(doc, indent=1)+'\n')
    print('done', round(doc['wall_s'], 1), 's; final scored', round(final['scored'], 4), 'margin', round(final['margin'], 3))


if __name__ == '__main__':
    main()
