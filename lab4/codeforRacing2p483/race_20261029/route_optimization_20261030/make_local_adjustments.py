#!/usr/bin/env python3
"""Narrow waypoint/time variations of feedback candidates; no scene edits."""
import copy
import json
from pathlib import Path
import numpy as np
import optimize_route as base
import make_feedback_candidates as feedback
import verify_candidates


def rebuild(data,template,durations,points,name,candidate_id,changes):
    planner=data['planner'];solver,stats=base.diagnostic(data,durations,points)
    times=planner.sampling_grid(np.unique(np.r_[0.,3.,5.+solver.boundaries]),.002)
    values=[np.zeros((len(times),3)) for i in range(3)];mask=times>=5.
    for i,t in enumerate(times[~mask]):
        for d,v in enumerate(base.flying.prelude_sample(t,data['ground'],points[0],3.,2.)):values[d][i]=v
    for d,m in enumerate(solver.maps(times[mask]-5.)[:3]):values[d][mask]=m@points
    rows=[dict(zip(base.common.FIELDS,map(float,np.r_[t,values[0][i],values[1][i],values[2][i],data['rows'][0]['yaw']]))) for i,t in enumerate(times)]
    base.common.validate_rows(rows);coeff=solver.coefficients(points);continuity={}
    for order in range(5):
        continuity['race_join_order_'+str(order)]=float(max(np.max(np.abs(planner.derivative_basis([1.],order)@coeff[i]/durations[i]**order-planner.derivative_basis([0.],order)@coeff[i+1]/durations[i+1]**order)) for i in range(12)))
        first=(planner.derivative_basis([0.],order)@coeff[0]/durations[0]**order)[0]
        continuity['prelude_to_race_order_'+str(order)]=float(np.max(abs(first-(points[0] if order==0 else 0))))
    if max(continuity.values())>1e-3:raise ValueError('C4 validation failed')
    raw=base.common.encode_csv(rows);r=copy.deepcopy(template)
    r.update(name=name,speed_factor=candidate_id,csv_sha256=base.common.sha256(raw),race_segment_durations_s=durations.tolist(),
      race_waypoints_ros_m=points.tolist(),race_normalized_polynomial_coefficients=coeff.tolist(),continuity_residuals=continuity,
      declared_changes=changes,full_metrics=stats)
    r['metrics']=dict(complete_reference_duration_s=rows[-1]['t'],nominal_first_gate_absolute_s=5.+durations[0],nominal_last_gate_absolute_s=5.+sum(durations[:12]),
      nominal_first_to_last_gate_s=float(sum(durations[1:12])),sampled_peak_speed_mps=stats['peak_speed_mps'],
      sampled_peak_acceleration_mps2=stats['peak_acceleration_mps2'],sampled_force_axis_rate_peak_rad_s=stats['force_axis_rate_peak_rad_s'],flight_result=None)
    delta=points[1:13]-data['centers']
    r['horizontal_offsets_m']=np.sum(delta*data['horizontal'],axis=1).tolist()
    r['vertical_offsets_m']=np.sum(delta*data['vertical'],axis=1).tolist()
    r['gate_plane_max_abs_residual_m']=float(abs(np.sum(delta*data['normal'],axis=1)).max())
    r['scope']='Only explicitly declared waypoint/time changes to the original prehover route. New reference only; actual flight required.'
    audit=verify_candidates.audit(r,data)
    return raw,r,audit


def write_set(data,out,candidates):
    if out.exists():raise ValueError('Fresh directory required')
    out.mkdir();entries=[];audits=[]
    for raw,r,audit in candidates:
        base.common.write_new(out/(r['name']+'.csv'),raw)
        base.common.write_new(out/(r['name']+'.json'),(json.dumps(r,indent=2,sort_keys=True)+'\n').encode())
        entry={k:r[k] for k in ('name','csv_sha256','speed_factor','metrics','family','race_start_time_s','climb_end_time_s','hover_release_time_s','hover_target_ros_m','speed_factor_semantics')};entry['csv']=r['name']+'.csv'
        entries.append(entry);audits.append(audit)
        print(r['name'],r['speed_factor'],r['metrics']['nominal_first_to_last_gate_s'],flush=True)
    manifest=dict(family='prehover_replanned',baseline_sha256=base.common.sha256(data['raw']),generation_only_not_actual_flight=True,
      source_geometry_sha256=base.common.sha256((base.RACE/'gate_geometry_evidence.json').read_bytes()),
      generator_sha256=base.common.sha256(Path(__file__).read_bytes()),candidates=entries)
    base.common.write_new(out/'CANDIDATES.json',(json.dumps(manifest,indent=2,sort_keys=True)+'\n').encode())
    base.common.write_new(out/'reference_crossing_validation.json',(json.dumps(dict(passed=True,candidates=audits),indent=2)+'\n').encode())


def main():
    data=base.inputs();one=[]
    for factor,key in ((1.24,1.251),(1.26,1.252),(1.28,1.253)):
        _,_,template,_=feedback.generate(data,factor,key)
        p=np.array(template['race_waypoints_ros_m']);p[8]=data['reference_points'][8]
        d=np.array(template['race_segment_durations_s'])
        template['waypoint_changes']=template['waypoint_changes'][:1]
        one.append(rebuild(data,template,d,p,'one_gate_speed_'+format(factor,'.3f').replace('.','p'),key,
           dict(base_uniform_speed_factor=factor,gate2_horizontal_m=-.45,gate8='restored unchanged original waypoint',all_times='unchanged')))
    write_set(data,base.HERE/'one_gate_candidates',one)
    _,_,template,_=feedback.generate(data,1.28,1.233)
    d=np.array(template['race_segment_durations_s']);p=np.array(template['race_waypoints_ros_m'])
    local=[]
    for key,label,changes in ((1.241,'plus040',{7:.02,8:.02}),(1.242,'plus050_minus020',{7:.03,8:.02,3:-.01,10:-.01}),
                              (1.243,'plus080_minus040',{7:.04,8:.04,3:-.02,10:-.02})):
        shifted=d.copy()
        for i,v in changes.items():shifted[i]+=v
        local.append(rebuild(data,template,shifted,p,'local_timing_'+label,key,
          dict(base_uniform_speed_factor=1.28,all_waypoints='unchanged feedback 1.28',segment_duration_changes_s=changes,net_scored_time_change_s=sum(changes.values()))))
    write_set(data,base.HERE/'local_timing_candidates',local)

if __name__=='__main__':main()
