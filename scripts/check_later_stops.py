#!/usr/bin/env python3
"""Qualify later checked stopping proposals with unchanged physical and forecast gates."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import copy
from functools import lru_cache
import json
import os
from pathlib import Path

from check_gnss_map_history import sha
from checked_approach_oracle import reconstruct as checked_approach
from check_road_width import verify_targets
from later_stop_oracle import reconstruct as stopping_certificates
from check_pose_forecasts import audit_dynamics
from check_relocalization_map_only import freeze,run_case
from check_road_width import validate_inputs
from check_track_initialization import compare
from cold_motion_oracle import reconstruct as cold_occupancy
from velocity_hypotheses_oracle import reconstruct as velocity_occupancy

REPO=Path(__file__).resolve().parents[1]
PARENT=REPO/'assets/checked-approaches-results.json'
GATES={'default_enabled': False,
 'maximum_later_stop_candidates': 80,
 'model': 'Before the ordinary/earlier stop search, visit every existing geometry knot after its '
          'heuristic stop in descending accumulated arc length, at most 80 endpoints from the '
          'existing 81-point path. Rebuild the reachable speed profile and check the entire '
          'collision sweep and eight-second stationary tail. No new path spacing, monotonicity '
          'assumption or binary search.',
 'ownership': 'Later search cannot override an active/braking obstruction commitment or a settled '
              'yield endpoint. Three fresh acquired confirmations retain ownership of replacing an '
              'existing stationary stop through the preceding checked-approach policy. A later '
              'checked approach does not arm the earlier-search emergency braking latch.',
 'unchanged': 'All 48 parent worlds, sensing/noise, original goals/deadlines, motion/pose/cold '
              'budgets, side width/reserve and physical accuracy/clearance/99-percent coverage '
              'gates. Only search_later_obstruction_stops is added.',
 'goal_deadline_s': 65.0,
 'minimum_physical_clearance_m': 1.0,
 'minimum_actual_road_margin_m': 0.1,
 'minimum_forecast_coverage': 0.99,
 'minimum_coverage_samples': 100,
 'forecast_horizons_s': [0.2, 1.0, 2.0, 4.0, 8.0],
 'reason': 'The initial sixteen evenly spaced distance candidates changed a proposed endpoint but '
           'not reference mission actuation or progress. Enumerating the existing geometry knots '
           'avoids introducing a coarser stopping grid than the path itself.'}


@lru_cache(maxsize=1)
def parent():return json.loads(PARENT.read_text())


def baseline(spec):return next(r for r in parent()['final_matrix']['episodes'] if r['scenario']==spec['later_baseline_scenario'])


def original(spec):return parent()['frozen_inputs'][baseline(spec)['scenario']]['input']


def specifications(plants):
    fields=['plant','world','mode','seed','enabled','cohort','family','heading','kind','parent_mode',
        'parent_scenario','baseline_scenario','sweep_mode','qualification_family','cold_baseline_scenario','velocity_baseline_scenario','approach_baseline_scenario']
    return [dict({k:r[k] for k in fields if k in r},mode=r['mode']+'-later-stops',later_baseline_scenario=r['scenario'])
        for r in parent()['final_matrix']['episodes'] if r['plant'] in plants]


def materialize(spec):
    value=copy.deepcopy(original(spec))
    if 'search_later_obstruction_stops' in value:raise ValueError('preceding checked approach policy changed')
    value['search_later_obstruction_stops']=True;return baseline(spec)['source'],value


def verify_policy(config,frozen,spec):
    for value in [config,frozen]:
        if value.get('search_later_obstruction_stops') is not True:raise ValueError('checked approach option changed')
    value=copy.deepcopy(frozen);value.pop('search_later_obstruction_stops')
    if value!=original(spec):raise ValueError('original world, sensing, deadline or policies changed')
    for key in ['forecast_envelope','forecast_pose_sigma','forecast_cold_max_speed_m_s',
            'fit_recent_track_motion','enclose_track_measurements','forecast_consistency_guard','forecast_recent_velocity_hypotheses','settle_obstruction_stops','search_obstruction_stops','lateral_candidate_margin_m','replan_checked_approaches']:
        if config.get(key)!=frozen.get(key):raise ValueError('existing motion, pose or support policy changed')


def audit(run,config,ticks,spec,frozen):
    verify_policy(config,frozen,spec);result=audit_dynamics(run,config,ticks,spec,frozen)
    result['cold_motion']=cold_occupancy(config,ticks)
    result['recent_velocity_hypotheses']=velocity_occupancy(config,ticks)
    result['checked_approach']=checked_approach(config,ticks)
    result['stopping_certificates']=stopping_certificates(config,ticks)
    result['targets']=(dict(verified_control_ticks=result['planning_decisions']['verified_ticks'],active_route_corridors=True)
        if config.get('navigation') else verify_targets(config,ticks))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true');parser.add_argument('--only-index',type=int)
    parser.add_argument('--check-contract',action='store_true',help='retain current checked approach contract while reporting physical regression failures')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants),factory=materialize,gates=GATES)
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()};fingerprints={p:sha(binaries[p]) for p in plants}
    validate_inputs(output,manifest,args.binary.resolve());selected=list(enumerate(manifest['episodes']))
    if args.only_index is not None:selected=[selected[args.only_index]]
    def one(item):
        index,spec=item;folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row=run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
        row['comparison']=compare(row,baseline(spec));receipts=output/'episode-receipts';receipts.mkdir(exist_ok=True)
        (receipts/f'{index:03d}.json').write_text(json.dumps(row,indent=2)+'\n');return row
    report=dict(complete=False,passed=False,gates=GATES,binaries=fingerprints,manifest_sha256=sha(output/'frozen-manifest.json'),
        parent_results_sha256=sha(PARENT),episodes=[])
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,selected):
            rows=report['episodes'];rows.append(row);report['complete']=len(rows)==len(selected)
            report['contract_passed']=all(not r.get('audit_error') and r.get('pose_occupancy',{}).get('passed') is True and r.get('cold_motion',{}).get('passed') is True and r.get('recent_velocity_hypotheses',{}).get('passed') is True and r.get('checked_approach',{}).get('passed') is True and r.get('stopping_certificates',{}).get('passed') is True
                and r.get('replay_exit_code')==0 and r.get('replay',{}).get('verified') is True
                and r['replay']['ticks']==r['summary']['steps'] and r.get('run_exit_code')==(0 if r['summary']['passed'] else 1) for r in rows)
            report['regression_passed']=all(r['comparison']['regression_passed'] for r in rows)
            report['strict_passes']=sum(r['passed'] for r in rows)
            report['passed']=report['complete'] and report['contract_passed'] and report['regression_passed']
            report['adoption_passed']=report['passed'] and report['strict_passes']==len(rows)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],'goal',row.get('summary',{}).get('reached_goal'),
                row.get('audit_error',row.get('failed_gates')),'new',row['comparison']['new_failed_gates'],flush=True)
    if any(sha(binaries[p])!=h for p,h in fingerprints.items()):raise ValueError('binary changed during qualification')
    raise SystemExit(0 if report['complete'] and report['contract_passed'] and (args.check_contract or report['passed']) else 1)


if __name__=='__main__':main()
