#!/usr/bin/env python3
"""Export selected surrogate-search points as a runner candidate set.

Each candidate is the original 14-waypoint prehover route with declared
per-gate plane offsets and segment durations from the search history. Nominal
and surrogate numbers are predictions; only a TESSE flight is a result.
"""
import copy
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
RACE = HERE.parent
sys.path.insert(0, str(RACE/'route_optimization_20261030'))
sys.path.insert(0, str(HERE))
import optimize_route as base  # noqa: E402
import make_local_adjustments as local  # noqa: E402
import surrogate_search as SS  # noqa: E402
import closed_loop_surrogate as S  # noqa: E402

OUT = HERE/'surrogate_candidates'


def pick(history, targets, max_tilt=120.):
    """For each target scored time pick the history point with the highest
    margin among those within +-0.012 s whose surrogate peak tilt stays below
    max_tilt (the inverted-thrust branch is not trusted); fall back to closest."""
    chosen = []
    history = [h for h in history if (h.get('peak_tilt') or 0) <= max_tilt]
    for target in targets:
        near = [h for h in history if abs(h['scored']-target) <= .012]
        if not near:
            near = sorted(history, key=lambda h: abs(h['scored']-target))[:1]
        chosen.append(max(near, key=lambda h: h['margin']))
    return chosen


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--search', type=Path, default=HERE/'es_search_20261105_stage2.json')
    ap.add_argument('--targets', type=float, nargs='+', default=[3.148, 3.05, 3.004, 2.947, 2.90])
    ap.add_argument('--first-id', type=float, default=1.401)
    ap.add_argument('--out', type=Path, default=OUT)
    ap.add_argument('--prefix', default='surrogate_t')
    ap.add_argument('--max-tilt', type=float, default=120.)
    ap.add_argument('--modified-physics-note', default=None)
    ap.add_argument('--bias-file', type=Path, default=None)
    a = ap.parse_args()
    doc = json.loads(a.search.read_text())
    history = [h for h in doc['history'] if 'durations' in h]
    history.append(dict(doc['best'], evals=doc['evaluations'], pred=doc['best']['predicted_time']))
    search = SS.Search(threshold=doc['threshold'], bias_path=a.bias_file)
    data = search.data
    template = search.template
    candidates = []
    for i, h in enumerate(pick(history, a.targets, a.max_tilt)):
        d = np.array(h['durations'])
        o = np.array(h['offsets'])
        points = search.points(np.clip(o, -search.planned_limit, search.planned_limit))
        base_d = np.array(template['race_segment_durations_s'])
        base_o = search.base_offsets()
        name = a.prefix+'%d' % round(h['scored']*1000)
        changes = dict(base='prehover_speed_1p260 (uniform 1.26)', method='closed_loop_surrogate evolution search',
                       search_file=a.search.name, search_evaluation=h.get('evals'),
                       segment_time_factors={str(k): float(d[k]/base_d[k]) for k in range(13) if abs(d[k]-base_d[k]) > 1e-12},
                       gate_plane_offsets_m={str(g+1): [float(o[g]), float(o[12+g])] for g in range(12)},
                       gate_plane_shifts_m={str(g+1): [float(o[g]-base_o[g]), float(o[12+g]-base_o[12+g])] for g in range(12)},
                       net_scored_time_change_s=float(d[1:12].sum()-base_d[1:12].sum()),
                       surrogate=dict(margin=h['margin'], margins=h['margins'], predicted_first_to_last_s=h.get('pred'),
                                      peak_tilt_deg=h.get('peak_tilt'), ensemble=list(SS.ENSEMBLE)),
                       note='Surrogate margin is a screening number calibrated on archived flights; it is not a flight result.',
                       modified_physics=a.modified_physics_note)
        raw, report, audit = local.rebuild(data, copy.deepcopy(template), d, points, name, round(a.first_id+i*.001, 3), changes)
        report['speed_factor_semantics'] = 'unique manifest selection ID; see declared_changes'
        # re-run the surrogate on the exported bytes to confirm identity of what will fly
        import csv, io
        rows = list(csv.DictReader(io.StringIO(raw.decode())))
        t = np.array([float(r['t']) for r in rows])
        arr = np.array([[float(r[c]) for c in ('x', 'y', 'z', 'vx', 'vy', 'vz', 'ax', 'ay', 'az')] for r in rows])
        sim_kwargs = {}
        if doc.get('modified_physics'):
            sim_kwargs = dict(angular_cap=doc['modified_physics']['angular_cap'], controller_kwargs=doc['modified_physics']['controller_kwargs'])
        check = S.summarize((t, arr), latency_steps=1, **sim_kwargs)
        report['declared_changes']['surrogate']['export_recheck'] = dict(gates=check['gates'], first_to_last=check['first_to_last'], min_margin=check['min_margin'])
        candidates.append((raw, report, audit))
        print(name, 'scored', round(h['scored'], 4), 'margin', round(h['margin'], 3), 'recheck', check['gates'], round(check['min_margin'] or -9, 3))
    local.write_set(data, a.out, candidates)


if __name__ == '__main__':
    main()
