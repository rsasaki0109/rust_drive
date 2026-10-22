#!/usr/bin/env python3
"""Independently verify acquired evidence for stationary forecast blocking.

The inspector supports fixed-route planar recordings with decision capture.
It certifies the recorded lattice's status, not global route infeasibility.
"""
import argparse
import gzip
import json
import lzma
import math
from pathlib import Path

from check_road_width import projector
from check_yield_relocalization import verify_yields
from inspect_planning import audit as audit_decisions, project_output as remove_decisions


def project_output(line):
    original=json.loads(line)
    projected=remove_decisions(line)
    split=projected.rfind(b',"planning_progress":')
    if split<0:raise ValueError('missing final progress status before decision capture')
    projected=projected[:split]+b'}\n'
    original.pop('planning_diagnostics');original.pop('planning_progress')
    if json.loads(projected)!=original:raise ValueError('progress projection changed operational values')
    return projected


def xy(p):return [p['x'],p['y']]


def statuses(config,ticks):
    """Batch reference: partition actual acquisitions into anchored evidence prefixes."""
    if any(config.get(k) for k in ['navigation','lane_change','stop_lines','stop_signs',
                                   'multi_height_lidar','lidar3d','relocalization']):
        raise ValueError('progress inspector requires a fixed planar route without mapped duties or epoch reset')
    if config.get('record_planning_diagnostics') is not True:
        raise ValueError('independent progress inspection requires decision capture')
    route=config['route']['points'];locate=projector(route)
    length=sum(math.dist(xy(a),xy(b)) for a,b in zip(route,route[1:]))
    mapped_holds=[False]*len(ticks)
    if config.get('yield_intersections'):
        # Reconstruct mapped ownership from scans, forecasts and poses. Reported
        # phase/endpoint values cannot suppress a genuine obstruction diagnosis.
        records=verify_yields(config,ticks)['records']
        for i,record in enumerate(records):
            mapped_holds[i]=any(s['phase']=='Waiting' and z['stop_line']['route_s_m']-config['vehicle']['radius']-1.<length
                for z,s in zip(config['yield_intersections'],record['zones']))
    last_scan=None;fault_cutoff=None;consumed=None;prefix=[];result=[]
    for index,tick in enumerate(ticks):
        source=tick['input'];out=tick['expected'];now=out['time'];d=out['planning_diagnostics']
        if source.get('lidar_failed'):fault_cutoff=now
        scan=source.get('lidar')
        if scan is not None:
            stamp=scan['stamp']
            valid=(type(stamp) in [int,float] and math.isfinite(stamp) and 0.<=stamp<=now+1e-9
                and len(scan['points'])<=20000 and all(math.isfinite(v) for p in scan['points'] for v in xy(p))
                and all(math.hypot(*xy(p))<=200. for p in scan['points']))
            if not valid or source.get('lidar_failed'):fault_cutoff=now
            elif ('InvalidLidar' not in out['health'] and 'MotionDiscontinuity' not in out['health']
                  and (last_scan is None or stamp>last_scan) and (fault_cutoff is None or stamp>fault_cutoff)):
                last_scan=stamp;fault_cutoff=None
        scene_valid=last_scan is not None and now-last_scan<=.35+1e-9 and (consumed is None or last_scan>=consumed)
        fresh=scene_valid and (consumed is None or last_scan>consumed)
        if fresh:consumed=last_scan
        records=d['candidates'];chosen=d['selected_candidate']
        ids=sorted({r['nominal_forecast_conflict']['prediction_id'] for r in records if r['nominal_forecast_conflict']})
        pose=xy(out['estimate']['pose']['position']);progress=d['route_progress_m'];path=out['trajectory']
        supported=(scene_valid and not mapped_holds[index] and d['scene_acquisition_s']==last_scan and not out['health'] and not out['emergency']
            and d['failure'] is None and chosen is not None and 0<=chosen<len(records)<=9
            and records[chosen]['disposition']=='CheckedStop' and ids
            and all(r['disposition']!='CompletePath' for r in records)
            and type(progress) in [int,float] and math.isfinite(progress)
            and all(math.isfinite(v) for v in pose) and math.isfinite(out['estimate']['speed'])
            and abs(out['estimate']['speed'])<=.2 and length-locate(out['estimate']['pose']['position'])[1]>2.
            and out['command']['acceleration']<=0. and path['mode']=='Yield' and len(path['points'])>=2
            and all(math.isfinite(p['speed']) and 0.<=p['speed']<=.2 and math.dist(xy(p['position']),pose)<=.2
                    for p in path['points']))
        if supported and prefix and not fresh:
            if (math.dist(pose,prefix[0][1])>.2 or abs(progress-prefix[0][2])>.2 or ids!=prefix[0][3]):prefix=[]
        if not supported:prefix=[]
        elif fresh:
            item=(last_scan,pose,progress,ids)
            trial=prefix+[item]
            # Check the complete acquired prefix against its first anchor; never
            # accept a chain of small movements as continuous standstill.
            anchored=(all(math.dist(p[1],trial[0][1])<=.2 and abs(p[2]-trial[0][2])<=.2 and p[3]==trial[0][3]
                          for p in trial)
                      and all(0.<b[0]-a[0]<=.35+1e-9 for a,b in zip(trial,trial[1:])))
            prefix=trial if anchored else [item]
        duration=prefix[-1][0]-prefix[0][0] if prefix else 0.
        result.append(dict(state='StationaryForecastBlocked' if len(prefix)>=3 and duration>=2.-1e-9 else 'Observing',
            first_supporting_acquisition_s=prefix[0][0] if prefix else None,
            last_supporting_acquisition_s=prefix[-1][0] if prefix else None,
            supporting_acquisitions=len(prefix),supported_duration_s=duration,
            blocking_prediction_ids=prefix[0][3] if prefix else []))
    return result


def audit(config,ticks):
    enabled=config.get('monitor_planning_progress',False)
    if type(enabled) is not bool:raise ValueError('mistyped progress policy')
    if not enabled:
        if any(t['expected'].get('planning_progress') is not None for t in ticks):
            raise ValueError('undeclared planning progress status')
        return dict(passed=True,enabled=False,verified_control_ticks=len(ticks),blocked_control_ticks=0,intervals=[])
    audit_decisions(config,ticks)
    expected=statuses(config,ticks);intervals=[];blocked=0;active=False
    for tick,want in zip(ticks,expected):
        actual=tick['expected'].get('planning_progress')
        if not isinstance(actual,dict) or actual.keys()!=want.keys():raise ValueError('missing or changed progress schema')
        if type(actual['supporting_acquisitions']) is not int or any(type(i) is not int for i in actual['blocking_prediction_ids']):
            raise ValueError('mistyped progress count or identity')
        for name,value in want.items():
            observed=actual[name]
            if type(value) is float:
                if type(observed) not in [int,float] or not math.isfinite(observed) or abs(observed-value)>1e-9:
                    raise ValueError('different acquired progress evidence: '+name)
            elif observed!=value:raise ValueError('different acquired progress evidence: '+name)
        now=tick['expected']['time'];current=want['state']=='StationaryForecastBlocked'
        if current:
            blocked+=1
            if not active:intervals.append(dict(detected_at_control_s=now,
                first_supporting_acquisition_s=want['first_supporting_acquisition_s'],
                blocking_prediction_ids=want['blocking_prediction_ids'],cleared_at_control_s=None))
        elif active:intervals[-1]['cleared_at_control_s']=now
        active=current
    return dict(passed=True,enabled=True,verified_control_ticks=len(ticks),blocked_control_ticks=blocked,
                intervals=intervals,final_state=expected[-1]['state'] if expected else 'Observing',
                sensor_only=True,global_infeasibility_claimed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--log',type=Path,required=True)
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    opener=lzma.open if args.log.suffix=='.xz' else gzip.open if args.log.suffix=='.gz' else open
    with opener(args.log,'rt') as stream:
        config=json.loads(next(stream))['header']['config']
        ticks=[r['tick'] for line in stream if 'tick' in (r:=json.loads(line))]
    encoded=json.dumps(audit(config,ticks),indent=2)+'\n'
    if args.output:args.output.write_text(encoded)
    else:print(encoded,end='')


if __name__=='__main__':main()
