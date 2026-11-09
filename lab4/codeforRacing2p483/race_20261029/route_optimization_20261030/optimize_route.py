#!/usr/bin/env python3
"""Offline C4 route redesign. Predictions are NOT TESSE flight results.

Gate points move only in the original fixed planes. The first unscored leg,
scored inter-gate times and final braking leg are optimized independently.
The exact 3 s vertical climb + 2 s hover stays unchanged. Runtime must still
wait for the existing actual-feedback stable-hover release.
"""
import argparse
import json
from pathlib import Path
import sys
import time
import numpy as np
from scipy import optimize

HERE = Path(__file__).resolve().parent
RACE = HERE.parent
sys.path.insert(0, str(RACE))
import make_flying_start as flying
common = flying.common


def inputs():
    rows, raw = common.read_rows(common.DEFAULT_BASE)
    meta = json.loads(common.DEFAULT_BASE.with_suffix('.json').read_text())
    planner = flying.load_planner(flying.DEFAULT_PLANNER)
    duration, original, residual = flying.source_model(meta, rows, planner)
    geometry = json.loads((RACE/'gate_geometry_evidence.json').read_text())['gates']
    centers = np.array([r['root_world_position_ros_m'] for r in geometry])
    horizontal = np.array([r['gate_horizontal_axis_ros'] for r in geometry])
    vertical = np.array([r['gate_vertical_axis_ros'] for r in geometry])
    normal = np.array([r['gate_normal_ros'] for r in geometry])
    points = original.copy()
    ground = points[0].copy()
    points[0, 2] = points[1, 2]
    positions = points.copy()
    positions[1:13] = centers
    directions = np.zeros((14, 3, 24))
    for i in range(12):
        directions[i+1,:,i] = horizontal[i]
        directions[i+1,:,12+i] = vertical[i]
    offsets = np.r_[np.sum((points[1:13]-centers)*horizontal,axis=1),
                    np.sum((points[1:13]-centers)*vertical,axis=1)]
    source = RACE/'evidence/runs_v2/live_20261030_020808_9f252a/summary.json'
    summary = json.loads(source.read_text())
    actual = np.array([[r['horizontal_offset_m'], r['vertical_offset_m']]
                       for r in summary['aperture_screen']['gates']])
    bias = np.r_[actual[:,0],actual[:,1]]-offsets
    return dict(rows=rows, raw=raw, metadata=meta, planner=planner, duration=duration/1.2,
                ground=ground, positions=positions, directions=directions, offsets=offsets,
                centers=centers,horizontal=horizontal,vertical=vertical,normal=normal,bias=bias,
                source_residual=residual, source_summary=source,
                reference_points=points)


class Objective:
    def __init__(self, data, durations, bias_weight=.5, samples=35):
        self.data = data
        self.base = data['planner'].OffsetObjective(durations,data['positions'],data['directions'],
                                                    samples,7.0,125.0,7.0)
        self.bias_weight = bias_weight
        self.dv = np.einsum('nv,vck->nck',self.base.maps[1],data['directions'])
        self.v0 = self.base.maps[1]@data['positions']
        self.prediction = .55*data['bias']

    def __call__(self,x):
        value,grad = self.base(x)
        # Excess predicted offsets are a screening heuristic, not a clearance
        # promise: changes to the plan/gains alter tracking errors.
        predicted = x+self.prediction
        excess = np.maximum(np.abs(predicted)-.48,0)
        value += self.bias_weight*np.mean(excess**2)
        grad += self.bias_weight*2*np.sign(predicted)*excess/len(x)
        # Stronger centre-seeking only in the most limited historical gates.
        w = np.zeros(24);w[1]=.035;w[7]=.015
        value += np.sum(w*predicted**2)
        grad += 2*w*predicted
        # Weak route-length pressure; force-axis demand dominates.
        velocity = self.v0+np.einsum('nck,k->nc',self.dv,x)
        speeds = np.maximum(np.linalg.norm(velocity,axis=1),1e-8)
        value += .00025*np.mean(speeds)
        grad += .00025*np.einsum('nc,nck->k',velocity/speeds[:,None],self.dv)/len(speeds)
        return value,grad


def redesign(data,target,limit=.35,rounds=3):
    durations = data['duration'].copy()
    durations[1:12] *= target/durations[1:12].sum()
    durations[0] = 1.0
    durations[12] = max(.85,durations[12])
    offsets = np.clip(data['offsets'], -limit, limit)
    records=[]
    # The geometry can only change within each fixed gate's hole.
    for outer in range(rounds):
        obj = Objective(data,durations)
        fit = optimize.minimize(obj,offsets,jac=True,method='L-BFGS-B',bounds=[(-limit,limit)]*24,
                                options=dict(maxiter=180,ftol=1e-12,maxls=40))
        offsets=fit.x
        records.append(dict(phase='gate_points',round=outer,success=bool(fit.success),iterations=int(fit.nit),loss=float(fit.fun)))
        scale = durations.copy()
        def objective(factors):
            d = scale*factors
            # Unscored first leg can grow, but should not become needlessly slow.
            return Objective(data,d)(offsets)[0]+.0001*(d[0]+d[12])
        bounds=[(.8/scale[0],1.35/scale[0])]+[(.65,1.5)]*11+[(.8/scale[-1],1.5/scale[-1])]
        fit=optimize.minimize(objective,np.ones(13),method='SLSQP',bounds=bounds,
              constraints=[dict(type='eq',fun=lambda f: np.sum(scale[1:12]*f[1:12])-target)],
              options=dict(maxiter=95,ftol=2e-9))
        durations=scale*fit.x
        records.append(dict(phase='segment_times',round=outer,success=bool(fit.success),iterations=int(fit.nit),loss=float(fit.fun)))
    return durations,offsets,records


def diagnostic(data,durations,points):
    planner=data['planner'];solver=planner.MinimumSnap(durations)
    times=planner.sampling_grid(solver.boundaries,.002)
    vals=[m@points for m in solver.maps(times)]
    stats=planner.metrics(times,vals,solver.boundaries[12],rate_limit=7.)
    mask=(times>=solver.boundaries[1])&(times<=solver.boundaries[12])
    stats['scored_path_length_m']=float(np.sum(np.linalg.norm(np.diff(vals[0][mask],axis=0),axis=1)))
    stats['nominal_first_to_last_gate_s']=float(durations[1:12].sum())
    stats['min_reference_forward_gate_velocity_mps']=float(min(
        np.dot((solver.maps(np.array([solver.boundaries[i+1]]))[1]@points)[0],data['normal'][i]) for i in range(12)))
    return solver,stats


def export_candidate(data,durations,offsets,records,out,candidate_id,label):
    planner=data['planner']
    points=data['positions']+np.einsum('vck,k->vc',data['directions'],offsets)
    solver,stats=diagnostic(data,durations,points)
    boundaries=np.unique(np.r_[0.,3.,5.+solver.boundaries])
    times=planner.sampling_grid(boundaries,.002)
    vals=[np.zeros((len(times),3)) for _ in range(3)]
    race=times>=5
    for i,t in enumerate(times[~race]):
        for d,v in enumerate(flying.prelude_sample(t,data['ground'],points[0],3.,2.)):vals[d][i]=v
    for d,m in enumerate(solver.maps(times[race]-5.)[:3]):vals[d][race]=m@points
    rows=[dict(zip(common.FIELDS,map(float,np.r_[t,vals[0][i],vals[1][i],vals[2][i],data['rows'][0]['yaw']]))) for i,t in enumerate(times)]
    common.validate_rows(rows)
    coeff=solver.coefficients(points)
    continuity={}
    for order in range(5):
        residuals=[np.max(np.abs(planner.derivative_basis([1.],order)@coeff[i]/durations[i]**order-planner.derivative_basis([0.],order)@coeff[i+1]/durations[i+1]**order)) for i in range(12)]
        continuity['race_join_order_'+str(order)]=float(max(residuals))
        start=(planner.derivative_basis([0.],order)@coeff[0]/durations[0]**order)[0]
        continuity['prelude_to_race_order_'+str(order)]=float(np.max(np.abs(start-(points[0] if order==0 else 0))))
    projected_plane=np.sum((points[1:13]-data['centers'])*data['normal'],axis=1)
    before_first=vals[0][times<5.+durations[0]-1e-7]
    no_early=bool(np.all((before_first-data['centers'][0])@data['normal'][0]<1e-5))
    if not no_early or max(continuity.values())>1e-3 or abs(projected_plane).max()>1e-8:
        raise ValueError('Mathematical route validation failed')
    name='route_'+label
    csv=common.encode_csv(rows)
    report=dict(name=name,family='prehover_replanned',speed_factor=candidate_id,
      speed_factor_semantics='unique manifest selection ID, NOT a uniform speed multiplier or measured speed',
      baseline_sha256=common.sha256(data['raw']),csv_sha256=common.sha256(csv),
      actual_flight_validated=False,generation_only_not_actual_flight=True,
      baseline_reconstruction_max_abs_pva=data['source_residual'],
      race_start_time_s=5.,climb_end_time_s=3.,hover_release_time_s=5.,nominal_hover_duration_s=2.,
      ground_position_ros_m=data['ground'].tolist(),hover_target_ros_m=points[0].tolist(),
      race_segment_durations_s=durations.tolist(),race_waypoints_ros_m=points.tolist(),
      race_normalized_polynomial_coefficients=coeff.tolist(),continuity_residuals=continuity,
      horizontal_offsets_m=offsets[:12].tolist(),vertical_offsets_m=offsets[12:].tolist(),
      historical_tracking_bias_m=data['bias'].tolist(),
      bias_heuristic='55% of one baseline actual-minus-reference offset; optimization screen only, not a validated error model',
      no_prep_first_gate_crossing_sampled=no_early,gate_plane_max_abs_residual_m=float(abs(projected_plane).max()),
      required_runtime_release=dict(actual_feedback_required=True),
      new_simulator_or_physical_parameters=False,search=records,
      metrics=dict(complete_reference_duration_s=rows[-1]['t'],nominal_first_gate_absolute_s=5.+durations[0],
        nominal_last_gate_absolute_s=5.+sum(durations[:12]),nominal_first_to_last_gate_s=stats['nominal_first_to_last_gate_s'],
        sampled_peak_speed_mps=stats['peak_speed_mps'],sampled_peak_acceleration_mps2=stats['peak_acceleration_mps2'],
        sampled_force_axis_rate_peak_rad_s=stats['force_axis_rate_peak_rad_s'],flight_result=None),
      full_metrics=stats,
      scope='New gate offsets and segment times; unchanged fixed gate geometry, prehover and stop positions. Same actual crossing judge must pass in TESSE.')
    out.mkdir(parents=True,exist_ok=True)
    common.write_new(out/(name+'.csv'),csv)
    common.write_new(out/(name+'.json'),(json.dumps(report,indent=2,sort_keys=True)+'\n').encode())
    entry={k:report[k] for k in ('name','csv_sha256','speed_factor','metrics','family','race_start_time_s','climb_end_time_s','hover_release_time_s','hover_target_ros_m','speed_factor_semantics')}
    entry['csv']=name+'.csv'
    print(name,json.dumps(stats),flush=True)
    return entry,report


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--out',type=Path,default=HERE/'candidates');ap.add_argument('--targets',type=float,nargs='+',default=[3.25,3.10,2.95]);ap.add_argument('--limit',type=float,default=.35);ap.add_argument('--rounds',type=int,default=3)
    args=ap.parse_args()
    if args.out.exists():raise ValueError('Use a fresh directory')
    if not 0<args.limit<=.45 or any(not 2.7<=x<=3.5 for x in args.targets):raise ValueError('Unsupported search bounds')
    data=inputs();start=time.monotonic();entries=[];reports=[]
    _,baseline=diagnostic(data,data['duration'],data['reference_points'])
    print('BASELINE',json.dumps(baseline),flush=True)
    for i,target in enumerate(args.targets):
        d,x,search=redesign(data,target,args.limit,args.rounds)
        entry,report=export_candidate(data,d,x,search,args.out,1.201+i*.001,'t'+str(round(target*1000))+'_h'+str(round(args.limit*100)))
        entries.append(entry);reports.append(report)
    manifest=dict(family='prehover_replanned',baseline_sha256=common.sha256(data['raw']),generation_only_not_actual_flight=True,
      source_geometry_sha256=common.sha256((RACE/'gate_geometry_evidence.json').read_bytes()),
      historical_actual_summary_sha256=common.sha256(data['source_summary'].read_bytes()),
      original_planner_sha256=common.sha256(flying.DEFAULT_PLANNER.read_bytes()),
      generator_sha256=common.sha256(Path(__file__).read_bytes()),baseline_metrics=baseline,
      gate_offset_bound_m=args.limit,wall_elapsed_s=time.monotonic()-start,candidates=entries)
    common.write_new(args.out/'CANDIDATES.json',(json.dumps(manifest,indent=2,sort_keys=True)+'\n').encode())
    print('COMPLETE',manifest['wall_elapsed_s'],flush=True)

if __name__=='__main__':main()
