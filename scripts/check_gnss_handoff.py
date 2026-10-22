#!/usr/bin/env python3
"""Freeze measured-reset GNSS handoff, single-return-spike and expiry controls."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
import json
import math
import os
from pathlib import Path

from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_gnss_recovery import POLICY
from check_lane_change_speeds import sweep
from check_relocalization import GATES as REGIONAL_GATES, REPO, verify_epochs
from check_relocalization_map_only import freeze as freeze_inputs, materialize as map_scenario, run_case as drive_case

GATES = dict(REGIONAL_GATES, policy=POLICY, grace_s=5., gnss_return_s=16., late_gnss_return_s=18.,
             spike_window_s=[16.,16.2], spike_offset_m=[0.,.45], first_acceptance_window_s=[16.6,17.1],
             movement_window_s=[12.3,16.], minimum_pre_gnss_distance_m=5., maximum_noise_component_m=.14,
             timeout_brake_delay_s=.051, timeout_stop_margin_s=1.5)


def specifications(plants, smoke=False):
    result=[]
    for plant in plants:
        for world in ['straight','curve']:
            for seed in ([7] if smoke else [1,7,42]):
                result.append(dict(plant=plant,world=world,seed=seed,mode='guarded'))
            for mode in ['unguarded','healthy','timeout']:
                result.append(dict(plant=plant,world=world,seed=7,mode=mode))
    return result


def materialize(spec):
    source,scenario=map_scenario(dict(spec,mode='handoff'))
    scenario['name']=f"Confirmed GNSS handoff {spec['world']} {spec['mode']}"
    scenario['gnss_recovery']=copy.deepcopy(POLICY)
    if spec['mode']!='unguarded':scenario['relocalization']['confirm_gnss_handoff']=True
    if spec['mode'] in ['guarded','unguarded']:
        scenario['gnss_bias_windows']=[{'from':16.,'until':16.2,'offset':dict(x=0.,y=.45)}]
    if spec['mode']=='timeout':
        scenario['gnss_dropout_windows'][0]['until']=18.
        scenario['expected']='fault'
    return source,scenario


def freeze(output,specs):
    return freeze_inputs(output,specs,factory=materialize,gates=GATES)


def audit(run,config,ticks,spec,frozen):
    if not contains_declared(run['scenario'],frozen):raise ValueError('handoff input differs from frozen scenario')
    if {'truth','objects','sensor_timing','gnss_bias_windows','gnss_dropout_windows'} & config.keys():
        raise ValueError('synthesis labels entered the operational header')
    for key in ['relocalization','gnss_recovery']:
        if config.get(key)!=frozen[key]:raise ValueError('external handoff policy changed: '+key)
    if config['localization_map']['points']!=frozen['localization_map']['points']:raise ValueError('surveyed map changed')
    frames,summary=run['frames'],run['summary'];errors=[];yaws=[];noise_verified=0
    if len(frames)!=len(ticks) or len(ticks)!=summary['steps']:raise ValueError('full physical/control coverage required')
    delay=frozen['sensor_timing']['gnss_delay_ticks'];return_at=18. if spec['mode']=='timeout' else 16.
    for i,(frame,tick) in enumerate(zip(frames,ticks)):
        inp,out=tick['input'],tick['expected'];now=inp['time']
        if abs(now-i*.05)>1e-8 or frame['time']!=now or frame['estimate']!=out['estimate']:raise ValueError('physical/control alignment changed')
        dropout=8.-1e-9<=now<11.-1e-9
        if bool(inp.get('odometry'))==dropout or (inp.get('odometry') and inp['odometry']['stamp']!=now):raise ValueError('motion fault schedule changed')
        acq=(i-delay)*.05;due=i>=delay and (i-delay)%4==0 and (acq<8.-1e-9 or acq>=return_at-1e-9)
        if bool(inp.get('gnss'))!=due or (due and abs(inp['gnss']['stamp']-acq)>1e-9):raise ValueError('GNSS acquisition/delivery changed')
        if bool(inp.get('lidar'))!=(i%2==0) or (inp.get('lidar') and inp['lidar']['stamp']!=now) or inp['lidar_failed']:
            raise ValueError('synchronous LiDAR acquisition schedule changed')
        if due:
            pose=frames[i-delay]['truth']['pose'];mount=frozen.get('gnss_antenna_mount_m') or dict(x=0.,y=0.)
            c,s=math.cos(pose['yaw']),math.sin(pose['yaw']);offset=[0.,.45] if spec['mode'] in ['guarded','unguarded'] and 16.-1e-9<=acq<16.2-1e-9 else [0.,0.]
            expected=[pose['position']['x']+c*mount['x']-s*mount['y']+offset[0],pose['position']['y']+s*mount['x']+c*mount['y']+offset[1]]
            if max(abs(inp['gnss']['position'][key]-expected[j]) for j,key in enumerate(['x','y']))>.14000001:
                raise ValueError('GNSS bias/noise differs from its original acquisition truth')
            noise_verified+=1
        truth,estimated=frame['truth']['pose'],out['estimate']['pose']
        errors.append(math.dist([truth['position'][k] for k in ['x','y']],[estimated['position'][k] for k in ['x','y']]))
        yaws.append(abs(angle(truth['yaw']-estimated['yaw'])))
    oracle=verify_epochs(config,ticks);valid=oracle.pop('valid_indices');post=oracle['post_epoch'];reset=oracle['reset_stamp']
    if reset is None or not 11.<=reset<=15.:raise ValueError('no independently measured stopped reset')
    valid_rmse=math.sqrt(sum(errors[i]**2 for i in valid)/len(valid));valid_max=max(errors[i] for i in valid);valid_yaw=max(yaws[i] for i in valid)
    rmse=math.sqrt(sum(e*e for e in errors)/len(errors));maximum=max(errors)
    if abs(rmse-summary['localization_rmse'])>1e-10 or abs(maximum-summary['localization_max_error'])>1e-10:
        raise ValueError('global invalid-epoch errors were removed')
    explained=[f'localization max error {maximum:.3f} m exceeds 1 m'] if maximum>1. and valid_max<=1. else []
    failed=[]
    if summary['failures']!=explained or summary['passed']!=(not explained):failed.append('source_acceptance')
    if valid_rmse>.2 or valid_max>.4 or valid_yaw>.1:failed.append('valid_epoch_accuracy')
    if summary['collisions'] or summary['road_violations'] or summary['min_clearance']<1. or summary['simulated_seconds']>45.:
        failed.append('physical_clearance_deadline')
    clearance=sweep(run)
    if min(f['truth']['speed'] for f in frames if 10.5<=f['time']<11.2)>.05:failed.append('physical_stop_before_reset')
    window=[f for f in frames if 12.3<=f['time']<=16.]
    distance=sum(math.dist([a['truth']['pose']['position'][k] for k in ['x','y']],[b['truth']['pose']['position'][k] for k in ['x','y']]) for a,b in zip(window,window[1:]))
    if distance<5.:failed.append('actual_pre_gnss_motion')
    proposals=post['gnss_proposals'];accepted=[(i,p) for i,p in enumerate(proposals) if p['decision']=='Accepted'];first=accepted[0][1]['delivery'] if accepted else None
    if spec['mode']=='unguarded':
        if not accepted or accepted[0][0]<POLICY['confirmation_fixes']-1:failed.append('unconfirmed_handoff')
    elif spec['mode']=='timeout':
        expiry=reset+POLICY['max_hold_s'];first_brake=next((t['input']['time'] for t in ticks if 'GnssHandoffExpired' in t['expected']['health']),None)
        if (first is not None or not post['expired'] or post['completed_recoveries'] or not proposals
                or first_brake is None or not expiry<first_brake<=expiry+.051):failed.append('absolute_handoff_expiry')
        if summary['reached_goal'] or max(f['truth']['speed'] for f in frames if f['time']>=expiry+1.5)>.05:
            failed.append('expired_physical_stop_hold')
    elif (not accepted or accepted[0][0]<POLICY['confirmation_fixes']-1 or post['completed_recoveries']!=1
          or post['held_proposals']<2 or proposals[0]['decision']=='Accepted' or not 16.6<=first<=17.1):
        failed.append('confirmed_gnss_handoff')
    if spec['mode']!='timeout' and not summary['reached_goal']:failed.append('original_goal')
    return dict(oracle=oracle,total_ticks=len(ticks),gnss_noise_samples_verified=noise_verified,
                global_rmse_m=rmse,global_maximum_error_m=maximum,invalid_pose_ticks=len(ticks)-len(valid),
                valid_epoch_rmse_m=valid_rmse,valid_epoch_maximum_error_m=valid_max,valid_epoch_maximum_yaw_rad=valid_yaw,
                expected_run_exit_code=1 if explained else 0,source_accuracy_failures_retained=explained,
                independent_clearance=clearance,pre_gnss_distance_m=distance,first_actual_gnss_acceptance_s=first,
                first_return_proposal=proposals[0] if proposals else None,failed_gates=failed)


def run_case(output,spec,binary,replay_binary):
    row=drive_case(output,spec,binary,replay_binary,auditor=audit)
    row['quality_passed']=row['passed']
    if spec['mode']=='unguarded':
        row['expectation_met']=bool(row.get('failed_gates')==['unconfirmed_handoff'] and row['run_exit_code']==row['expected_run_exit_code']
                                    and row['replay_exit_code']==0 and row['replay']['verified'] and row['replay']['ticks']==row['total_ticks'])
    else:row['expectation_met']=row['passed']
    return row


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all');parser.add_argument('--smoke',action='store_true');parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--binary',type=Path,default=REPO/'target/release'/('rustdriving.exe' if os.name=='nt' else 'rustdriving'));parser.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release/rustdriving-rne');args=parser.parse_args()
    output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant];manifest=freeze(output,specifications(plants,args.smoke))
    if args.freeze_only:print('Frozen confirmed handoff episodes:',len(manifest['episodes']));return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()};provenance={p:dict(path=str(binaries[p]),sha256=sha(binaries[p])) for p in plants};rows=[]
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda s:run_case(output,s,binaries[s['plant']],args.binary.resolve()),manifest['episodes']):
            rows.append(row);report=dict(passed=all(x['expectation_met'] for x in rows),complete=len(rows)==len(manifest['episodes']),manifest_sha256=sha(output/'frozen-manifest.json'),binaries=provenance,gates=GATES,episodes=rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n');print(row['plant'],row['world'],row['mode'],row['seed'],row['expectation_met'],row.get('audit_error',row.get('failed_gates')),flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__=='__main__':main()
