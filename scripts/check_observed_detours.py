#!/usr/bin/env python3
"""Qualify sensor-observed stopped detours with paired maps and retained failures."""
import argparse
from collections import Counter
from concurrent.futures import ThreadPoolExecutor
import copy
import hashlib
import json
import lzma
import math
import os
from pathlib import Path

from check_forecast_consistency import physical_holds
from check_forecast_envelope import coverage
from check_gnss_covariance import contains_declared
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_hazards import check_motion_predictions,check_speed_profiles
from check_mid_edge import compare,reference
from check_navigation_relocalization import verify_navigation
from check_relocalization_map_only import freeze,run_case
from check_road_width import projector,swept_clearance,validate_inputs
from forecast_consistency_oracle import reconstruct as forecast_guard
from inspect_planning import audit as inspect_decisions
from measurement_enclosure_oracle import reconstruct as enclosure
from observed_detour_cases import GATES,REPO,materialize,specifications
from observed_detour_oracle import ObservedDetours
from planar_sweep_oracle import audit_stream
from sparse_circle_oracle import reconstruct as sparse_circles
from track_initialization_oracle import reconstruct as track_motion


def remove_observed_status(line):
    text=line.decode();marker=',"observed_obstructions":';start=text.find(marker)
    if start<0:raise ValueError('missing observed-detour output field')
    _,end=json.JSONDecoder().raw_decode(text,start+len(marker))
    if text[end:end+1]!='}':raise ValueError('observed-detour status is not the final navigation field')
    projected=text[:start]+text[end:]
    original=json.loads(text);original['navigation'].pop('observed_obstructions')
    if json.loads(projected)!=original:raise ValueError('projection changed operational values')
    return projected.encode()


def prefix(points,end):
    result=[points[0]];cursor=0.
    for a,b in zip(points,points[1:]):
        length=math.dist([a[k] for k in ['x','y']],[b[k] for k in ['x','y']])
        if cursor+length<end-1e-6:result.append(b)
        else:
            fraction=min(1.,max(0.,(end-cursor)/length))
            result.append({k:a[k]+fraction*(b[k]-a[k]) for k in ['x','y']});break
        cursor+=length
    return result


class Decisions:
    def __init__(self,config):self.config=config;self.counts=Counter();self.ticks=0
    def __call__(self,tick,active,stop,phase):
        points=active['points'] if phase=='Following' or stop is None else prefix(active['points'],min(active['length_m'],stop+1.))
        local=dict(record_planning_diagnostics=True,vehicle=self.config['vehicle'],
            route=dict(points=points,half_width=active['half_width']),
            lateral_candidate_margin_m=self.config.get('lateral_candidate_margin_m'),
            refine_lateral_candidates=self.config.get('refine_lateral_candidates',False))
        result=inspect_decisions(local,[tick]);self.counts.update(result['candidate_dispositions']);self.ticks+=1


def route_physics(run,ticks,config,navigation,observed):
    events=run.get('route_history',[]);switches=navigation['switch_events']
    initial=reference(config['navigation'],config['navigation'].get('closed_edges',[]))
    # Existing simulations include an initial event only with external updates.
    # Without it, the frozen request and first physical frame establish the route.
    if events and events[0]['time']==0.:
        compare(events[0]['plan'],initial);events=events[1:]
    if len(events)!=len(switches):raise ValueError('route history lost a reconstructed handover')
    plans=[initial];maximum_switch_speed=0.
    for index,event in enumerate(events):
        now=event['time'];closed=list(config['navigation'].get('closed_edges',[]))
        for tick in ticks:
            if tick['input']['time']>now+1e-9:break
            update=tick['input'].get('navigation_update')
            if update is not None:closed=list(update['closed_edges'])
        local=[r['evidence']['edge'] for r in observed.report['retained_edges'] if r['requested_at_control_s']<=now+1e-9]
        plan=reference(config['navigation'],list(dict.fromkeys(closed+local)))
        compare(event['plan'],plan);plans.append(plan)
        if abs(now-switches[index]['stamp'])>1e-9:raise ValueError('physical route switch clock changed')
        speed=max(abs(event[k]) for k in ['true_speed','estimated_speed']);maximum_switch_speed=max(maximum_switch_speed,speed)
        if speed>.05+1e-9:raise ValueError('physical route switch occurred before standstill')
    compare(run['navigation'],plans[-1])
    locators=[projector(p['points']) for p in plans];active=0;margins=[];confirmation=[]
    for frame,tick in zip(run['frames'],ticks):
        if active<len(events) and frame['time']>=events[active]['time']-1e-9:active+=1
        if frame['navigation']['active_edges']!=plans[active]['edge_ids']:raise ValueError('physical active road differs')
        margins.append(plans[active]['half_width']-run['vehicle']['radius']-locators[active](frame['truth']['pose']['position'])[0])
        if frame['navigation'].get('handover_hold'):confirmation.append(abs(frame['truth']['speed']))
    return dict(passed=min(margins)>=.1 and max(confirmation,default=0.)<=.05+1e-9,
        minimum_actual_road_margin_m=min(margins),samples=len(margins),
        maximum_actual_confirmation_speed_m_s=max(confirmation,default=0.),
        confirmation_samples=len(confirmation),maximum_route_switch_speed_m_s=maximum_switch_speed)


def verify_policy(config,frozen,spec):
    if config['navigation']!=frozen['navigation']:raise ValueError('operational navigation policy differs')
    if config['navigation'].get('avoid_observed_obstructions',False)!=spec['enabled']:raise ValueError('observed-detour policy lost')
    if {'objects','truth','duration','world_motion','sensor_timing','navigation_updates','full_rate_recording'}&config.keys():
        raise ValueError('world truth or schedules entered operational config')
    if config.get('forecast_envelope')!=GATES['forecast_budgets']:raise ValueError('motion error budgets changed')


def audit(run,config,ticks,spec,frozen):
    verify_policy(config,frozen,spec)
    recorded=copy.deepcopy(run['scenario'])
    # Scenario serde omits the explicitly empty external-update schedule.
    recorded.setdefault('navigation_updates',[])
    if not contains_declared(recorded,frozen):raise ValueError('world differs from frozen paired input')
    frames=run['frames'];summary=run['summary']
    if len(frames)!=len(ticks) or len(ticks)!=summary['steps']:raise ValueError('incomplete physical or acquired evidence')
    errors=[];yaws=[]
    for i,(f,t) in enumerate(zip(frames,ticks)):
        out=t['expected']
        if abs(f['time']-i*.05)>1e-9 or f['time']!=out['time'] or f['estimate']!=out['estimate'] or f['navigation']!=out['navigation']:
            raise ValueError('physical/control frame mismatch')
        errors.append(math.dist([f['truth']['pose']['position'][k] for k in ['x','y']],[out['estimate']['pose']['position'][k] for k in ['x','y']]))
        yaws.append(abs(angle(f['truth']['pose']['yaw']-out['estimate']['pose']['yaw'])))
    rmse=math.sqrt(sum(e*e for e in errors)/len(errors))
    if abs(rmse-summary['localization_rmse'])>1e-10 or abs(max(errors)-summary['localization_max_error'])>1e-10:
        raise ValueError('summary omitted physical localization errors')
    observer=ObservedDetours(config);decisions=Decisions(config)
    nav=verify_navigation(config,ticks,obstruction_observer=observer,decision_auditor=decisions)
    physical=swept_clearance(run);roads=route_physics(run,ticks,config,nav,observer);cov=coverage(run,ticks)
    folder=Path(spec['_folder']);failed=[]
    if summary['collisions'] or physical['minimum_swept_clearance_m']<1.:failed.append('physical_clearance')
    if summary['road_violations'] or not roads['passed']:failed.append('road_reserve_or_confirmation')
    if rmse>.3 or max(errors)>.6 or max(yaws)>.1:failed.append('localization_accuracy')
    if not cov['passed']:failed.append('forecast_coverage')
    if summary['simulated_seconds']>frozen['duration']+1e-9:failed.append('source_deadline')
    negative_goal=spec['mode'] in ['detour-off','external-off']
    failures=['goal not reached within duration'] if negative_goal else []
    want_goal=spec['world'] not in ['no-alternative','late'] and not negative_goal
    source_pass=(summary['reached_goal']==want_goal and summary['failures']==failures and summary['passed']==(not negative_goal))
    if not source_pass:failed.append('expected_source_outcome')
    # A deliberately unsolved control still fails its original goal gate.
    if not summary['passed']:failed.append('source_acceptance')
    retained=observer.report['retained_edges']
    expected_detour=spec['enabled'] and spec['world'] in ['detour','external']
    if len(retained)!=int(expected_detour) or nav['final_switches']!=int(expected_detour or spec['world']=='known-closure'):
        failed.append('observed_detour_outcome')
    if retained and (retained[0]['evidence']['edge']!='main' or observer.report['last_decision']!='DetourRequested'):
        raise ValueError('unexpected observed edge preference')
    if spec['enabled'] and spec['world'] in ['no-alternative','late']:
        want='NoAlternative' if spec['world']=='no-alternative' else 'UnreachableDivergence'
        if observer.report['last_decision']!=want:failed.append('negative_proposal_contract')
    hold_physics=physical_holds(run,ticks)
    if not hold_physics['passed']:failed.append('forecast_hold_physics')
    return dict(expected_run_exit_code=0 if summary['passed'] else 1,failed_gates=failed,
        expected_source_outcome=source_pass,
        observed_detours=observer.summary(),navigation_oracle=nav,
        planning_decisions=dict(passed=True,verified_ticks=decisions.ticks,candidate_dispositions=dict(decisions.counts)),
        independent_clearance=physical,road_containment=roads,coverage=cov,
        rmse_m=rmse,maximum_error_m=max(errors),maximum_yaw_rad=max(yaws),
        forecast_consistency=forecast_guard(config,ticks),forecast_hold_physics=hold_physics,
        planar_sweeps=audit_stream(config,ticks),sparse_circles=sparse_circles(config,ticks),
        track_motion=track_motion(config,ticks),measurement_enclosure=enclosure(config,ticks),
        motion_predictions=check_motion_predictions(folder/'run/sensors.jsonl'),
        speed_profiles=check_speed_profiles(folder/'run/sensors.jsonl'))


def stored_bytes(root,row,name):
    receipt=row['storage'][name];path=root/row['recordings']/receipt['path']
    if sha(path)!=receipt['stored_sha256']:raise ValueError('stored recording changed')
    data=lzma.decompress(path.read_bytes()) if receipt['encoding']=='xz' else path.read_bytes()
    if hashlib.sha256(data).hexdigest()!=receipt['sha256']:raise ValueError('decoded recording changed')
    return data


def paired_comparisons(root,rows):
    by_key={(r['plant'],r['world'],r['seed'],r['enabled']):r for r in rows};proofs=[]
    for on in rows:
        if not on['enabled']:continue
        off=by_key[(on['plant'],on['world'],on['seed'],False)]
        enabled=json.loads((root/on['scenario']).read_text());disabled=json.loads((root/off['scenario']).read_text())
        if enabled['navigation'].pop('avoid_observed_obstructions') is not True or enabled!=disabled:
            raise ValueError('paired world, sensors or acceptance changed')
        new_failures=sorted(set(on.get('failed_gates',[]))-set(off.get('failed_gates',[])))
        proof=dict(plant=on['plant'],world=on['world'],seed=on['seed'],same_inputs_except_flag=True,
            new_failed_gates=new_failures,regression_passed=not new_failures)
        if on['world'] in ['moving','late','no-alternative']:
            raw=stored_bytes(root,on,'replay/outputs.jsonl');projected=b''.join(remove_observed_status(l) for l in raw.splitlines(keepends=True))
            baseline=stored_bytes(root,off,'replay/outputs.jsonl')
            proof.update(operational_outputs_byte_identical=projected==baseline,
                projected_output_sha256=hashlib.sha256(projected).hexdigest(),baseline_output_sha256=hashlib.sha256(baseline).hexdigest())
            a=json.loads(stored_bytes(root,on,'run/run.json'));b=json.loads(stored_bytes(root,off,'run/run.json'))
            for f in a['frames']:f['navigation'].pop('observed_obstructions')
            proof['physical_frames_identical']=a['frames']==b['frames']
            proof['summary_byte_identical']=stored_bytes(root,on,'run/summary.json')==stored_bytes(root,off,'run/summary.json')
            streams=[]
            for row in [on,off]:
                records=[json.loads(l) for l in stored_bytes(root,row,'run/sensors.jsonl').splitlines()]
                streams.append([r['tick']['input'] for r in records if 'tick' in r])
            proof['acquired_inputs_identical']=streams[0]==streams[1]
            proof['negative_control_passed']=all(proof[k] for k in ['operational_outputs_byte_identical',
                'physical_frames_identical','summary_byte_identical','acquired_inputs_identical'])
        proof['passed']=proof['regression_passed'] and proof.get('negative_control_passed',True)
        proofs.append(proof)
    return dict(passed=all(p['passed'] for p in proofs),pairs=proofs)


def parent_compatibility(root,rows,parent):
    old=json.loads((parent/'report.json').read_text())
    lookup={(r['plant'],r['world'],r['seed']):r for r in old['episodes']};proofs=[]
    for row in rows:
        if row['enabled']:continue
        previous=lookup[(row['plant'],row['world'],row['seed'])]
        if row['sha256']!=previous['sha256']:raise ValueError('parent frozen input changed')
        identical={name:stored_bytes(root,row,name)==stored_bytes(parent,previous,name) for name in row['storage']}
        proofs.append(dict(plant=row['plant'],world=row['world'],seed=row['seed'],streams=identical,passed=all(identical.values())))
    return dict(passed=all(p['passed'] for p in proofs),episodes=proofs)


def finish_report(root,report,parent=None):
    rows=report['episodes']
    contract_gates={'expected_source_outcome','observed_detour_outcome','negative_proposal_contract',
        'physical_clearance','road_reserve_or_confirmation','localization_accuracy','source_deadline','forecast_hold_physics'}
    report['contract_passed']=all(not e.get('audit_error') and e.get('observed_detours',{}).get('passed') is True
        and not contract_gates&set(e.get('failed_gates',[])) and e.get('replay_exit_code')==0
        and e.get('replay',{}).get('verified') is True and e['replay']['ticks']==e['summary']['steps']
        and e.get('run_exit_code')==(0 if e['summary']['passed'] else 1) for e in rows)
    report['strict_passes']=sum(e['passed'] for e in rows)
    if report['complete'] and report['contract_passed']:
        report['paired_comparisons']=paired_comparisons(root,rows)
        if parent is not None:report['parent_compatibility']=parent_compatibility(root,rows,parent)
    pairs=report.get('paired_comparisons',{})
    report['regression_passed']=pairs.get('passed',False)
    report['negative_controls_passed']=bool(pairs) and all(p.get('negative_control_passed',True) for p in pairs['pairs'])
    report['contract_qualified']=(report['complete'] and report['contract_passed'] and report['negative_controls_passed']
        and report.get('parent_compatibility',{}).get('passed',True))
    report['passed']=report['contract_qualified'] and report['regression_passed']
    report['adoption_passed']=report['passed'] and report['strict_passes']==len(rows)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true');parser.add_argument('--only-index',type=int)
    parser.add_argument('--parent-cohort',type=Path)
    parser.add_argument('--check-contract',action='store_true',help='check the observed-route/physical contract while retaining regression and adoption failures')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants),factory=materialize,gates=GATES)
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()};fingerprints={p:sha(binaries[p]) for p in plants}
    validate_inputs(output,manifest,args.binary.resolve())
    selected=list(enumerate(manifest['episodes']))
    if args.only_index is not None:selected=[selected[args.only_index]]
    def one(item):
        index,spec=item;folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row=run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
            auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
        receipts=output/'episode-receipts';receipts.mkdir(exist_ok=True)
        (receipts/f'{index:03d}.json').write_text(json.dumps(row,indent=2)+'\n');return row
    report=dict(complete=False,passed=False,gates=GATES,binaries=fingerprints,manifest_sha256=sha(output/'frozen-manifest.json'),episodes=[])
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,selected):
            rows=report['episodes'];rows.append(row);report['complete']=len(rows)==len(selected)
            report['contract_passed']=all(not e.get('audit_error') and e.get('observed_detours',{}).get('passed') is True
                and not {'expected_source_outcome','observed_detour_outcome','negative_proposal_contract'}&set(e.get('failed_gates',[]))
                and e.get('replay_exit_code')==0 and e.get('replay',{}).get('verified') is True and e['replay']['ticks']==e['summary']['steps'] for e in rows)
            report['strict_passes']=sum(e['passed'] for e in rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['mode'],row['seed'],'goal',row.get('summary',{}).get('reached_goal'),
                  row.get('audit_error',row.get('failed_gates')),flush=True)
    if args.only_index is None:finish_report(output,report,args.parent_cohort.resolve() if args.parent_cohort else None)
    else:report.update(passed=False,adoption_passed=False,contract_qualified=False)
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    if any(sha(binaries[p])!=h for p,h in fingerprints.items()):raise ValueError('binary changed during qualification')
    raise SystemExit(0 if report['contract_qualified' if args.check_contract else 'passed'] else 1)


if __name__=='__main__':main()
