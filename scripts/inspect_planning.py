#!/usr/bin/env python3
"""Validate and summarize planner decisions on a fixed route without mapped stop duties."""
import argparse
from collections import Counter
import gzip
import json
import lzma
import math
from pathlib import Path

from check_hazards import prediction_radius_at
from check_road_width import projector

REJECTED={'NoSideRoom','OutsideTargetCorridor','OppositeSideDuringPass',
          'GeometryOutsideCorridor','UnreachableSpeedProfile',
          'RefinementForecastConflict','NoCheckedStop'}
ACCEPTED={'CompletePath','CheckedStop'}


def finite(value):
    if type(value) not in [int,float] or not math.isfinite(value):
        raise ValueError('nonfinite or mistyped planning number')
    return value


def project_output(line):
    """Remove the final optional field without reserializing any original float."""
    original=json.loads(line)
    if 'planning_diagnostics' not in original:raise ValueError('missing captured decisions')
    split=line.rfind(b',"planning_diagnostics":')
    if split<0:raise ValueError('planning diagnostics must be the final output field')
    projected=line[:split]+b'}\n'
    original.pop('planning_diagnostics')
    if json.loads(projected)!=original:raise ValueError('projection changed operational outputs')
    return projected


def audit(config,ticks):
    enabled=config.get('record_planning_diagnostics',False)
    if type(enabled) is not bool:raise ValueError('mistyped capture policy')
    if enabled and any(config.get(k) for k in ['navigation','lane_change','traffic_controls','stop_signs','intersections']):
        raise ValueError('independent inspection requires a fixed route without route-changing duties')
    locate=projector(config['route']['points'])
    side=config.get('lateral_candidate_margin_m')
    side=3.5 if side is None else min(3.5,max(0.,config['route']['half_width']-config['vehicle']['radius']-side))
    fractions=[0.,1.,-1.,.875,-.875,.75,-.75,.5,-.5] if config.get('refine_lateral_candidates',False) else [0.,1.,-1.]
    statuses=Counter();selected=Counter();blockers=Counter();conflicts=0;guarded=0;maximum=0
    for tick in ticks:
        out=tick['expected'];report=out.get('planning_diagnostics')
        if not enabled:
            if report is not None:raise ValueError('capture absent from the declared policy')
            continue
        if not isinstance(report,dict):raise ValueError('declared planner decisions disappeared')
        records=report['candidates'];maximum=max(maximum,len(records))
        failure=report['failure']
        if failure in ['InvalidInput','InvalidLocalGeometry']:
            if records or report['selected_candidate'] is not None or out['trajectory']['points']:
                raise ValueError('invalid planning call retained old candidates')
            continue
        if failure not in [None,'NoSelectableCandidate']:raise ValueError('unknown planning failure')
        if len(records)!=len(fractions):raise ValueError('candidate diagnostics were omitted or exceeded their bound')
        here=locate(out['estimate']['pose']['position'])[1]
        if abs(finite(report['route_progress_m'])-here)>1e-8:raise ValueError('different supplied-route progress')
        if not 0.<=finite(report['planning_span_m'])<=40.:raise ValueError('invalid planning span')
        stamp=report['scene_acquisition_s']
        if stamp is not None and not 0.<=finite(stamp)<=out['time']+1e-9:raise ValueError('invalid scene acquisition clock')
        if out['health']:
            if stamp is not None or out['command']['acceleration']>-5.99:raise ValueError('fault did not clear support or retain braking')
            guarded+=1
        predictions={p['id']:p for p in out['predictions']}
        for i,(record,fraction) in enumerate(zip(records,fractions)):
            if type(record['index']) is not int or record['index']!=i:raise ValueError('candidate order or identity changed')
            if abs(finite(record['lateral_target_m'])-side*fraction)>1e-10:raise ValueError('diagnostic target differs from the bounded policy')
            status=record['disposition'];statuses[status]+=1
            if status not in REJECTED|ACCEPTED:raise ValueError('unknown candidate disposition')
            conflict=record['nominal_forecast_conflict'];profile=record['profile']
            if (profile is not None)!=(status in ACCEPTED):raise ValueError('rejected candidate advertises an accepted profile')
            requires_conflict=status in ['RefinementForecastConflict','NoCheckedStop','CheckedStop']
            if (conflict is not None)!=requires_conflict:raise ValueError('forecast rejection lost its responsible prediction')
            if conflict is not None:
                if type(conflict['prediction_id']) is not int or conflict['prediction_id']<0:
                    raise ValueError('mistyped conflict identity')
                prediction=predictions.get(conflict['prediction_id'])
                if prediction is None:raise ValueError('conflict names an absent forecast')
                time=finite(conflict['relative_time_s'])
                if time<0.:raise ValueError('negative relative conflict time')
                radius=prediction_radius_at(prediction,time)
                if abs(finite(conflict['forecast_radius_m'])-radius)>1e-10*max(1.,radius):raise ValueError('conflict radius differs from the recorded forecast')
                horizon=prediction['dt']*(len(prediction['positions'])-1)
                if finite(conflict['forecast_horizon_s'])!=horizon:raise ValueError('forecast horizon changed')
                if finite(conflict['minimum_clearance_floor_m'])!=(1. if i>=3 else 0.):raise ValueError('clearance policy changed')
                conflicts+=1;blockers[str(prediction['id'])]+=1
            if profile is not None:
                if type(profile['point_count']) is not int or not 2<=profile['point_count']<=82:raise ValueError('invalid profile size')
                for k in ['end_progress_m','end_time_s','terminal_speed_m_s']:finite(profile[k])
                if profile['end_time_s']<=0. or profile['terminal_speed_m_s']<0.:raise ValueError('invalid profile extent')
                if type(profile['searched_stop']) is not bool:raise ValueError('mistyped stop search record')
                if status=='CheckedStop' and profile['terminal_speed_m_s']!=0.:raise ValueError('checked stop is still moving')
        index=report['selected_candidate']
        if index is None:
            if failure!='NoSelectableCandidate' or out['trajectory']['points']:raise ValueError('missing selected candidate')
        else:
            if type(index) is not int or not 0<=index<len(records) or failure is not None:raise ValueError('invalid selected candidate')
            record=records[index];profile=record['profile'];points=out['trajectory']['points']
            if profile is None or not points:raise ValueError('selected candidate has no profile')
            if record['lateral_target_m']!=out['trajectory']['lateral_target']:raise ValueError('selected lateral target differs from the actual proposal')
            if profile['point_count']!=len(points) or profile['end_time_s']!=points[-1]['time'] or profile['terminal_speed_m_s']!=points[-1]['speed']:
                raise ValueError('selected profile differs from the actual proposal')
            if abs(profile['end_progress_m']-locate(points[-1]['position'])[1])>1e-8:raise ValueError('selected endpoint progress differs')
            if record['disposition']=='CheckedStop':
                if len(points)<2 or points[-1]['position']!=points[-2]['position'] or abs(points[-1]['time']-points[-2]['time']-8.)>1e-8:
                    raise ValueError('checked stop lost its complete stationary tail')
            selected[record['disposition']]+=1
        support=report['refinement_confirmation']
        if support is not None:
            count=support['acquisition_count'];first=finite(support['first_acquisition_s']);last=finite(support['last_acquisition_s'])
            if type(count) is not int or not 1<=count<=3 or not 0.<=first<=last<=out['time']+1e-9:raise ValueError('invalid refinement confirmation clock/count')
            if type(support['confirmed']) is not bool or support['confirmed']!=(count>=3 and last-first>=.2-1e-9):raise ValueError('unconfirmed refinement advertised as ready')
            target=finite(support['lateral_target_m'])
            if not any(r['lateral_target_m']==target and r['disposition']=='CompletePath' for r in records[3:]):raise ValueError('confirmation refers to an unavailable refined proposal')
        if index is not None and index>=3:
            if support is None or not support['confirmed'] or support['lateral_target_m']!=records[index]['lateral_target_m']:
                raise ValueError('selected refinement lacks acquired confirmation')
        for field in ['obstruction_stop_requested','obstruction_stop_active']:
            if type(report[field]) is not bool:raise ValueError('mistyped obstruction duty')
        if report['obstruction_stop_active'] and not report['obstruction_stop_requested']:raise ValueError('active hold lacks its stopping duty')
    return dict(capture_enabled=enabled,verified_control_ticks=len(ticks),maximum_candidate_records=maximum,
                candidate_dispositions=dict(statuses),selected_profiles=dict(selected),
                nominal_forecast_conflicts=conflicts,forecast_blockers=dict(blockers),fault_braking_ticks=guarded)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    opener=lzma.open if args.log.suffix=='.xz' else gzip.open if args.log.suffix=='.gz' else open
    with opener(args.log,'rt') as stream:
        config=json.loads(next(stream))['header']['config']
        ticks=[item['tick'] for line in stream if 'tick' in (item:=json.loads(line))]
    result=audit(config,ticks);encoded=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(encoded)
    else:print(encoded,end='')


if __name__=='__main__':main()
