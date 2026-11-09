#!/usr/bin/env python3
"""(1+lambda) evolution search on the surrogate: maximise ensemble margin, then
trade margin for scored time. Screening only."""
import json, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
import surrogate_search as SS

def merit(r, threshold):
    if r['margin'] < threshold:
        return r['margin']-10
    return -r['scored']

def main():
    import argparse
    ap=argparse.ArgumentParser(); ap.add_argument('--threshold',type=float,default=.18); ap.add_argument('--budget',type=int,default=400)
    ap.add_argument('--seed',type=int,default=1); ap.add_argument('--out',type=Path,default=Path(__file__).parent/'es_search_20261105.json')
    ap.add_argument('--sigma-off',type=float,default=.08); ap.add_argument('--sigma-dur',type=float,default=.02)
    ap.add_argument('--init',type=Path)
    ap.add_argument('--lock-segments',type=int,nargs='*',default=[],help='scored segments that may not be shorter than the 1.26 reference (hairpins)')
    ap.add_argument('--min-factor',type=float,default=.7)
    ap.add_argument('--hover-y-range',type=float,nargs=2,help='search the hover/start y within this range (runway is free only to ~9.7)')
    ap.add_argument('--seg0-range',type=float,nargs=2,default=[.45,.9])
    ap.add_argument('--modified-physics-cap',type=float,help='MODIFIED simulator study only: Unity max angular speed')
    ap.add_argument('--kr',type=float); ap.add_argument('--kw',type=float)
    ap.add_argument('--bias-file',type=Path)
    a=ap.parse_args(); rng=np.random.default_rng(a.seed)
    sim_kwargs={}
    if a.modified_physics_cap:
        ck={'omega_limit':a.modified_physics_cap}
        if a.kr: ck['kr']=a.kr
        if a.kw: ck['kw']=a.kw
        sim_kwargs=dict(angular_cap=a.modified_physics_cap,controller_kwargs=ck)
    s=SS.Search(threshold=a.threshold,sim_kwargs=sim_kwargs,bias_path=a.bias_file)
    d=s.base_d.copy(); o=s.base_offsets()
    if a.init:
        prev=json.loads(a.init.read_text())['best']; d=np.array(prev['durations']); o=np.array(prev['offsets'])
    hy=None
    if a.init:
        hy=prev.get('hover_y')
    if a.hover_y_range and hy is None: hy=float(s.base_p[0,1])
    best=s.evaluate(d,o,hover_y=hy); bm=merit(best,a.threshold)
    print('start scored',round(best['scored'],4),'margin',round(best['margin'],3),best['margins'],flush=True)
    t0=time.monotonic(); hist=[]
    sig_o, sig_d = a.sigma_off, a.sigma_dur
    since=0
    while s.evals < a.budget:
        od=d.copy(); oo=o.copy()
        # perturb a random subset of offsets and scored durations
        mask_o=rng.random(24)<.35; oo[mask_o]+=rng.normal(0,sig_o,mask_o.sum())
        oo=np.clip(oo,-s.planned_limit,s.planned_limit)
        mask_d=rng.random(11)<.4; od[1:12][mask_d]*=np.exp(rng.normal(0,sig_d,mask_d.sum()))
        ohy=hy
        if a.hover_y_range:
            if rng.random()<.5: ohy=float(np.clip(hy+rng.normal(0,.15),a.hover_y_range[0],a.hover_y_range[1]))
            if rng.random()<.5: od[0]=float(np.clip(od[0]*np.exp(rng.normal(0,2*sig_d)),a.seg0_range[0],a.seg0_range[1]))
        elif rng.random()<.3: od[0]*=np.exp(rng.normal(0,sig_d))
        lower=a.min_factor*s.base_d[1:12].copy()
        for seg in a.lock_segments: lower[seg-1]=s.base_d[seg]
        od[1:12]=np.clip(od[1:12],lower,1.3*s.base_d[1:12])
        r=s.evaluate(od,oo,hover_y=ohy); m=merit(r,a.threshold)
        if m>bm:
            d,o,hy,best,bm=od,oo,ohy,r,m; since=0
            hist.append(dict(evals=s.evals,scored=r['scored'],margin=r['margin'],margins=r['margins'],pred=r['predicted_time'],peak_tilt=r['peak_tilt'],durations=od.tolist(),offsets=oo.tolist(),hover_y=ohy,seg0=float(od[0]),gate1_speed=r.get('gate1_speed')))
            print(f"eval {s.evals}: scored {r['scored']:.4f} pred {r['predicted_time'] or 0:.4f} margin {r['margin']:.3f} {r['margins']} tilt {r['peak_tilt']:.0f} hy {ohy} seg0 {od[0]:.3f} v1 {r.get('gate1_speed') or 0:.1f}",flush=True)
        else:
            since+=1
            if since%60==0: sig_o*=.7; sig_d*=.7; print('  shrink sigma',round(sig_o,3),round(sig_d,4),flush=True)
    doc=dict(threshold=a.threshold,seed=a.seed,modified_physics=sim_kwargs or None,bias_file=str(a.bias_file) if a.bias_file else 'default',lock_segments=a.lock_segments,min_factor=a.min_factor,bias_corrected=s.bias is not None,evaluations=s.evals,wall_s=time.monotonic()-t0,
             best=dict(durations=d.tolist(),offsets=o.tolist(),hover_y=hy,gate1_speed=best.get('gate1_speed'),scored=best['scored'],predicted_time=best['predicted_time'],margin=best['margin'],margins=best['margins'],peak_tilt=best['peak_tilt']),
             history=hist,scope='surrogate screening only; not a flight result')
    a.out.write_text(json.dumps(doc,indent=1)+'\n'); print('done',round(doc['wall_s']),'s')
if __name__=='__main__': main()
