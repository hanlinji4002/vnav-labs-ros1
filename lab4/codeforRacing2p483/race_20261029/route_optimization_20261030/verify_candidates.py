#!/usr/bin/env python3
"""Independent polynomial crossing audit of generated references, not flights."""
import json
from pathlib import Path
import sys
import numpy as np
import optimize_route as model


def audit(report,data):
    coefficients=np.array(report['race_normalized_polynomial_coefficients'])
    durations=np.array(report['race_segment_durations_s'])
    bounds=np.r_[0.,np.cumsum(durations)]
    previous=-1e-9;events=[];all_roots=[]
    for gate in range(12):
        normal=data['normal'][gate];center=data['centers'][gate]
        roots=[]
        for segment,poly in enumerate(coefficients):
            plane_poly=poly@normal;plane_poly=plane_poly.copy();plane_poly[0]-=center@normal
            for u in np.polynomial.polynomial.polyroots(plane_poly):
                if abs(u.imag)>1e-6 or not -1e-7<=u.real<=1+1e-7:continue
                u=float(np.clip(u.real,0,1));t=bounds[segment]+durations[segment]*u
                velocity=(model.flying.load_planner(model.flying.DEFAULT_PLANNER).derivative_basis([u],1)@poly)[0]/durations[segment]
                forward=float(velocity@normal)
                if forward<1e-6:continue
                point=np.polynomial.polynomial.polyval(u,poly)
                h=float((point-center)@data['horizontal'][gate]);v=float((point-center)@data['vertical'][gate])
                roots.append(dict(gate=gate,reference_time_s=5.+t,race_time_s=t,horizontal_m=h,vertical_m=v,forward_mps=forward,segment=segment))
        roots=sorted(roots,key=lambda r:r['race_time_s']);dedup=[]
        for event in roots:
            if not dedup or event['race_time_s']-dedup[-1]['race_time_s']>1e-5:dedup.append(event)
        all_roots.append(dedup)
        subsequent=[r for r in dedup if r['race_time_s']>previous+1e-6]
        if not subsequent:raise AssertionError('Missing ordered forward crossing '+str(gate))
        event=subsequent[0]
        if max(abs(event['horizontal_m']),abs(event['vertical_m']))>.450001:raise AssertionError('First forward crossing outside planned gate margin '+str(event))
        if abs(event['race_time_s']-bounds[gate+1])>1e-5:raise AssertionError('Unexpected early crossing '+str(event))
        events.append(event);previous=event['race_time_s']
    endpoint={}
    planner=data['planner']
    for d in range(1,5):
        a=(planner.derivative_basis([0.],d)@coefficients[0])/durations[0]**d
        b=(planner.derivative_basis([1.],d)@coefficients[-1])/durations[-1]**d
        endpoint[str(d)]=float(max(np.abs(a).max(),np.abs(b).max()))
    if max(endpoint.values())>1e-3:raise AssertionError('Endpoint rest failed')
    return dict(name=report['name'],passed=True,scope='Analytic degree-9 reference roots, not actual flight',ordered_first_forward_crossings=events,
      all_forward_roots=all_roots,endpoint_rest_max_abs_by_derivative=endpoint,
      actual_flight_validated=False)


def main():
    data=model.inputs();folders=[Path(x) for x in sys.argv[1:]] or [model.HERE/'candidates',model.HERE/'rate_limited_candidates']
    result=[]
    for folder in folders:
        manifest=json.loads((folder/'CANDIDATES.json').read_text())
        for row in manifest['candidates']:
            report=json.loads((folder/(row['name']+'.json')).read_text())
            raw=(folder/row['csv']).read_bytes()
            if model.common.sha256(raw)!=row['csv_sha256']:raise AssertionError('CSV hash differs')
            result.append(audit(report,data))
    print(json.dumps(dict(passed=True,candidates=result),indent=2))

if __name__=='__main__':main()
