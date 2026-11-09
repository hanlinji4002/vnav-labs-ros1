#!/usr/bin/env python3
"""Run one selected measured profile and save only its newly verified video."""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import uuid


def build_command(here, root, name, profile):
    command=[sys.executable,'-B',str(here/'run_race.py'),name,'--root',str(root),
             '--simulator-zip','/mnt/c/Users/<user>/lab4_gpu_transfer_20261023/lab4_simulator.zip',
             '--glcore33','--plan-family',profile['plan_family'],'--speed-factor',str(profile['speed_factor']),
             '--omega-limit',str(profile['omega_limit']),'--ff-tau',str(profile['ff_tau']),
             '--timer-window','--capture','--wall-timeout','110']
    if profile.get('candidate_set'):
        command += ['--candidate-set',profile['candidate_set']]
    command += ['--control-profile',profile.get('control_profile','baseline')]
    return command


def request_matches(profile, verdict):
    candidate=verdict.get('candidate',{})
    requested_set=profile.get('candidate_set')
    # Nested candidates require the exact recorded full path as well as the
    # basename. Legacy simple names retain their existing comparison.
    if isinstance(requested_set,str) and '/' in requested_set:
        set_matches=(candidate.get('candidate_set_path') == requested_set and
                     candidate.get('candidate_set') == Path(requested_set).name)
    else:
        set_matches=candidate.get('candidate_set') == requested_set
    return (verdict.get('valid_measured_run') is True and
            verdict.get('official_physics', True) is True and verdict.get('modified_physics') is None and
            verdict.get('timer_video_verified') is True and
            candidate.get('csv_sha256') == profile['csv_sha256'] and
            candidate.get('control_profile') == profile.get('control_profile','baseline') and
            candidate.get('control_overrides') == profile.get('control_overrides',{}) and
            candidate.get('reference_overrides') == profile.get('reference_overrides',{}) and
            candidate.get('plan_family') == profile['plan_family'] and
            set_matches and
            candidate.get('speed_factor') == profile['speed_factor'])


def main(argv=None, here=None, desktop_root=None):
    parser=argparse.ArgumentParser(description=__doc__)
    mode=parser.add_mutually_exclusive_group()
    mode.add_argument('--stable',action='store_true')
    mode.add_argument('--optimized',action='store_true')
    mode.add_argument('--fast',action='store_true')
    args=parser.parse_args(argv)
    here=Path(__file__).resolve().parent if here is None else Path(here)
    root=here.parent
    profile_name=('FAST_PROFILE_20261105.json' if args.fast else
                  'OPTIMIZED_PROFILE.json' if args.optimized else
                  'STABLE_PROFILE.json' if args.stable else 'RECOMMENDED_PROFILE.json')
    profile_path=here/profile_name
    raw=profile_path.read_bytes()
    profile=json.loads(raw)
    name='live_'+datetime.datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:6]
    command=build_command(here,root,name,profile)
    env=os.environ.copy()
    env['MESA_D3D12_DEFAULT_ADAPTER_NAME']='NVIDIA'
    print('Take off, hold steady, then race. Timer starts at gate 1 and stops at gate 12.',flush=True)
    result=subprocess.call(command,env=env)
    out=root/'runs'/name
    if not out.is_dir():return result or 2
    (out/'present_request.json').write_text(json.dumps({'profile':profile,'profile_sha256':hashlib.sha256(raw).hexdigest(),
                                                       'command':command,'child_exit':result},indent=2)+'\n')
    judged=out/'gate_race_result.json'
    if result!=0 or not judged.is_file():
        print('Run not verified; evidence retained at '+str(out),flush=True)
        return result or 2
    verdict=json.loads(judged.read_text())
    if not request_matches(profile,verdict):
        print('Run or requested profile not verified; evidence retained at '+str(out),flush=True)
        return 2
    desktop_root=Path('/mnt/c/Users/<user>/Desktop') if desktop_root is None else Path(desktop_root)
    desktop_name=('LAB4_Fast_Race_20261105' if args.fast else
                  'LAB4_Optimized_Race_20261030' if args.optimized else 'LAB4_Prehover_Race_20261030')
    destination=desktop_root/desktop_name/name
    required=('scene.mp4','gate_race_result.json','prehover_release.json','start_parameters.yaml','present_request.json')
    if not all((out/filename).is_file() for filename in required):
        print('Required evidence missing; run retained at '+str(out),flush=True)
        return 2
    destination.mkdir(parents=True,exist_ok=False)
    video=out/'scene.mp4'
    shutil.copyfile(video,destination/'Flight_With_Timer.mp4')
    for filename in ('gate_race_result.json','prehover_release.json','start_parameters.yaml','present_request.json'):
        shutil.copyfile(out/filename,destination/filename)
    sha=hashlib.sha256(video.read_bytes()).hexdigest()
    if hashlib.sha256((destination/'Flight_With_Timer.mp4').read_bytes()).hexdigest()!=sha:
        raise RuntimeError('Copied video hash mismatch')
    (destination/'SHA256SUMS.txt').write_text(sha+'  Flight_With_Timer.mp4\n')
    print('VERIFIED gate 1 -> 12: %.6f simulated seconds'%verdict['first_to_last_sim_s'],flush=True)
    print('Windows Desktop: '+desktop_name+'/'+name+'/Flight_With_Timer.mp4',flush=True)
    return 0

if __name__=='__main__':raise SystemExit(main())
