#!/usr/bin/env python3
"""Freeze earlier-stop search trials without waiving goal or forecast failures."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import math
from pathlib import Path

from check_absolute_heading import audit as heading_audit, materialize as heading_input
from check_forecast_envelope import audit as forecast_audit
from check_gnss_map_history import sha
from check_obstruction_stops import certify_trajectory, materialize as obstruction_input
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_road_width import (GATES as ROAD_GATES, materialize as road_input,
    projector, road_margin, swept_clearance, validate_inputs)

REPO = Path(__file__).resolve().parents[1]
GATES = dict(ROAD_GATES, minimum_emergency_step_reduction=.8,
             mission_hold_window_s=[20., 65.], maximum_actual_hold_speed_m_s=.05,
             maximum_extra_profiles_per_lateral_candidate=16,
             parent_heading_results_sha256=sha(REPO/'assets/absolute-heading-results.json'),
             parent_obstruction_results_sha256=sha(REPO/'assets/obstruction-stop-results.json'))


def specifications(plants):
    rows = []
    for plant in plants:
        for seed in [7, 42]:
            for heading in [False, True]:
                rows.append(dict(plant=plant, world='mission', seed=seed, heading=heading,
                                 family='heading' if heading else 'road', kind='base'))
            for world in ['straight', 'curve', 'right']:
                rows.append(dict(plant=plant, world=world, seed=seed, heading=True, family='heading', kind='base'))
            for world in ['blocked', 'release', 'release-fault']:
                rows.append(dict(plant=plant, world=world, seed=seed, heading=False, family='obstruction', kind='base'))
        for kind in ['missing', 'jump', 'recover-missing', 'recover-jump']:
            rows.append(dict(plant=plant, world='straight', seed=171, heading=True, family='heading', kind=kind))
    for row in rows:
        row['mode'] = ('heading-' if row['heading'] else '') + (row['kind']+'-' if row['kind']!='base' else '') + 'search'
    return rows


def materialize(spec):
    if spec['family']=='heading': source, value = heading_input(spec)
    elif spec['family']=='road': source, value = road_input(dict(spec, mode='fitted'))
    elif spec['family']=='obstruction': source, value = obstruction_input(dict(spec, mode='envelope' if spec['world']=='blocked' else 'enabled', settle=True))
    else: raise ValueError('unknown stop-search fixture family')
    value['search_obstruction_stops'] = True
    return source, value


def freeze(output, specs): return freeze_inputs(output, specs, factory=materialize, gates=GATES)


def contract(config, ticks):
    """Infer a pending stop from complete emitted geometry, not a planner latch.

    A monotonic moving brake arms retention before any emergency. An ordinary
    stopped launch on a partial approach does not. Actual future motion appears
    only in the separate physical scorer. A full clear candidate releases the
    inferred duty; missing sensing and malformed holds cannot do so.
    """
    locate = projector(config['route']['points']); length = config['route']['lengths'][-1]
    armed = False; settled = False; arms = []; releases = []; held = 0; violations = []; bounds = 0
    for tick in ticks:
        out = tick['expected']; now = out['time']; path = out['trajectory']['points']
        if not all(math.isfinite(out['command'][k]) for k in ['acceleration','steering']):
            raise ValueError('nonfinite stop-search command')
        if out['health']:
            if out['command']['acceleration'] > -5.99: violations.append(dict(time=now, reason='health_fault_brake'))
            continue
        if out['trajectory']['mode']=='Emergency':
            if path or out['command']['acceleration']>-5.99 or out['command']['steering']!=0.:
                violations.append(dict(time=now, reason='emergency_brake'))
            if out['predictions'] and not armed: armed=True; arms.append(now)
            settled = settled or (armed and out['estimate']['speed']<=.2)
            continue
        if len(path)<2: raise ValueError('stop-search trajectory lacks a complete interval')
        here = locate(out['estimate']['pose']['position'])[1]
        span = locate(path[-1]['position'])[1]-here
        complete = span >= min(40., max(0., length-here-1.))-.25
        if complete:
            if armed:
                bounds += certify_trajectory(out,config['vehicle']['radius'])['interval_bounds']
                armed=False; settled=False; releases.append(now)
            continue
        if path[-1]['speed']!=0. or abs(path[-1]['time']-path[-2]['time']-8.)>1e-8:
            violations.append(dict(time=now, reason='partial_candidate_without_complete_stop_hold'))
        braking = (out['estimate']['speed']>.2
                   and all(b['speed']<=a['speed']+1e-9 for a,b in zip(path,path[1:])))
        if out['predictions'] and braking and not armed: armed=True; arms.append(now)
        if not armed: continue
        # Validate every partial candidate, including each retained hold, with
        # accelerated-path interval bounds independent of the Rust chord sweep.
        bounds += certify_trajectory(out,config['vehicle']['radius'])['interval_bounds']
        settled = settled or out['estimate']['speed']<=.2
        if not settled: continue
        held += 1
        if (out['command']['acceleration']>-.5+1e-8
                or any(b['speed']>a['speed']+1e-9 for a,b in zip(path,path[1:]))):
            violations.append(dict(time=now, reason='restarted_on_partial_stop'))
    return dict(passed=not violations, arms=arms, releases=releases, held_ticks=held,
                violations=violations, certified_interval_bounds=bounds)


def audit(run, config, ticks, spec, frozen):
    if config.get('search_obstruction_stops',False)!=frozen.get('search_obstruction_stops',False):
        raise ValueError('stop-search replay policy differs from the frozen input')
    if not config.get('settle_obstruction_stops'): raise ValueError('stop-search retention missing')
    if spec['heading']:
        result = heading_audit(run, config, ticks, spec, frozen)
    else:
        result = forecast_audit(run, config, ticks, spec, frozen)
        result['road_containment'] = road_margin(run)
        result['independent_clearance'] = swept_clearance(run)
        if (result['road_containment']['minimum_actual_road_margin_m']<.1
                or result['independent_clearance']['minimum_swept_clearance_m']<1.):
            result['failed_gates'].append('physical_separation_road_reserve')
    duty = contract(config, ticks)
    result['stop_contract'] = duty
    if not duty['passed']: result['failed_gates'].append('settled_stop_contract')
    emergency = sum(bool(t['expected']['emergency']) for t in ticks)
    if emergency != run['summary']['emergency_steps']: raise ValueError('emergency counter differs from complete outputs')
    result['verified_emergency_steps'] = emergency
    if spec['world']=='mission':
        samples = [f for f in run['frames'] if 20.-1e-9<=f['time']<=65.+1e-9]
        result['mission_hold'] = dict(samples=len(samples), maximum_actual_speed_m_s=max(f['truth']['speed'] for f in samples),
                                     positive_control_commands=sum(f['command']['acceleration']>-.5+1e-8 for f in samples))
    elif spec['family']=='obstruction':
        lo, hi = (14.,20.) if spec['world']=='blocked' else (10.,11.5)
        maximum = max(f['truth']['speed'] for f in run['frames'] if lo-1e-9<=f['time']<=hi+1e-9)
        result['actual_hold_maximum_m_s'] = maximum
        if maximum>.05: result['failed_gates'].append('actual_standstill')
        if spec['world']!='blocked':
            release = next((t['expected']['time'] for t in ticks if t['expected']['time']>=12. and t['expected']['command']['acceleration']>0.),None)
            result['release_command_s'] = release
            if release is None or release>22. or not run['summary']['reached_goal']: result['failed_gates'].append('clear_path_release_goal')
        if spec['world']=='release-fault':
            maximum = max(f['truth']['speed'] for f in run['frames'] if 13.5-1e-9<=f['time']<=14.95+1e-9)
            result['fault_hold_maximum_m_s'] = maximum
            if maximum>.05: result['failed_gates'].append('fault_standstill')
    return result


def key(row): return tuple(row[k] for k in ['plant','world','seed','heading','kind'])


def compare(row, baseline):
    verified = (not row.get('audit_error') and row.get('replay_exit_code')==0
                and row.get('run_exit_code')==(0 if row.get('summary',{}).get('passed') else 1)
                and row.get('replay',{}).get('verified') is True
                and row['replay']['ticks']==row['summary']['steps'])
    new = sorted(set(row.get('failed_gates',[]))-set(baseline['failed_gates']))
    regression = verified and not new
    stable = None
    if row['world']=='mission':
        hold = row.get('mission_hold',{})
        stable = (verified and row.get('stop_contract',{}).get('passed') is True
                  and row.get('verified_emergency_steps',math.inf)<=.2*baseline['verified_emergency_steps']
                  and hold.get('samples',0)>=900 and hold.get('maximum_actual_speed_m_s',math.inf)<=.05
                  and hold.get('positive_control_commands',1)==0)
    return dict(regression_passed=regression, new_failed_gates=new, stable_mission_stop_passed=stable,
                baseline_emergency_steps=baseline['verified_emergency_steps'],
                baseline_failed_gates=baseline['failed_gates'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--check-regression',action='store_true')
    parser.add_argument('--baseline-results',type=Path,default=REPO/'assets/stop-search-results.json')
    parser.add_argument('--binary',type=Path,default=REPO/'target/release/rustdriving')
    parser.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args=parser.parse_args(); output=args.output.resolve()
    plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants))
    if args.freeze_only:return
    baseline={key(r):r for r in json.loads(args.baseline_results.read_text())['baselines']}
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    fingerprints={k:sha(binaries[k]) for k in plants};validate_inputs(output,manifest,args.binary.resolve());rows=[]
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            row['comparison']=compare(row,baseline[key(row)]);rows.append(row)
            report=dict(complete=len(rows)==len(manifest['episodes']),binaries=fingerprints,
                        manifest_sha256=sha(output/'frozen-manifest.json'),episodes=rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row['comparison'],row.get('audit_error'),flush=True)
    report.update(regression_passed=all(r['comparison']['regression_passed'] for r in rows),
                  stable_mission_stops_passed=all(r['comparison']['stable_mission_stop_passed'] for r in rows if r['world']=='mission'),
                  strict_qualification_passed=all(r['passed'] for r in rows))
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    assert all(sha(binaries[k])==fingerprints[k] for k in plants)
    if not (report['regression_passed'] and report['stable_mission_stops_passed'] if args.check_regression else report['strict_qualification_passed']):raise SystemExit(1)


if __name__=='__main__':main()
