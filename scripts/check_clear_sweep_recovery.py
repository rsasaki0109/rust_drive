#!/usr/bin/env python3
"""Compare actual ray capture and stopped empty-sweep recovery on frozen inputs."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import json
import lzma
import math
import os
from pathlib import Path

from check_forecast_consistency import audit as parent_audit, materialize as parent_input, specifications as parent_specs
from check_gnss_map_history import sha
from check_relocalization_map_only import freeze, run_case
from check_road_width import validate_inputs
from check_track_initialization import compare
from planar_sweep_oracle import audit_stream

REPO=Path(__file__).resolve().parents[1]
PARENT=REPO/'assets/forecast-consistency-results.json'
ORIGINAL=REPO/'assets/standalone-enclosure-results.json'
GATES=dict(parent_results_sha256=sha(PARENT),original_results_sha256=sha(ORIGINAL),default_enabled=False,
           maximum_measured_speed_m_s=.2,minimum_acquisitions=3,minimum_span_s=.2,maximum_gap_s=.35,
           require_synchronous_measured_motion=True,require_all_rays_complete=True,require_no_returns=True,
           checked_center_radius_m=40.,minimum_resolved_disc_radius_m=.4,
           unchanged='Original world, sensors/noise, future error budgets, source goals/deadlines, physical clearance, road, accuracy and coverage gates')


@lru_cache(maxsize=2)
def packet(path):return json.loads(Path(path).read_text())


def key(row):return tuple(row[k] for k in ['cohort','plant','world','seed'])+(row.get('parent_mode',row['mode']),)


def baseline(spec, original=False):
    return next(r for r in packet(str(ORIGINAL if original else PARENT))['final_matrix']['episodes'] if key(r)==key(spec))


def recovery_policy(plant):
    return dict(lidar=dict(ray_count=720,first_angle_rad=-math.pi if plant=='reference-2d' else math.pi,
                angular_step_rad=math.tau/720 if plant=='reference-2d' else -math.tau/719,
                min_range_m=.2,max_range_m=45.,range_error_m=.05),
                checked_center_radius_m=40.,minimum_resolved_disc_radius_m=.4)


def specifications(plants):
    return [dict(s,parent_mode=s['mode'],mode=s['mode']+'-'+mode,sweep_mode=mode)
            for s in parent_specs(plants) for mode in ['capture','recover']]


def materialize(spec):
    source,value=parent_input(dict(spec,mode=spec['parent_mode']))
    old=baseline(spec)
    if value!=packet(str(PARENT))['frozen_inputs'][old['scenario']]['input']:raise ValueError('preceding physical input changed')
    value['record_planar_sweep']=True
    if spec['sweep_mode']=='recover':value['clear_sweep_recovery']=recovery_policy(spec['plant'])
    return source,value


def archive(spec,name):
    base=Path(spec['_baseline']);old=baseline(spec)
    row=next(r for r in packet(str(base/'report.json'))['episodes'] if key(r)==key(spec))
    record=row['storage'][name]
    if record['sha256']!=old['storage'][name]['sha256']:raise ValueError('preceding archived output changed')
    path=base/row['recordings']/record['path']
    if sha(path)!=record['stored_sha256']:raise ValueError('preceding compressed evidence changed')
    with lzma.open(path,'rb') as stream:raw=stream.read()
    import hashlib
    if hashlib.sha256(raw).hexdigest()!=record['sha256']:raise ValueError('preceding decoded evidence changed')
    return raw


def audit(run,config,ticks,spec,frozen):
    expected=recovery_policy(spec['plant']) if spec['sweep_mode']=='recover' else None
    if config.get('clear_sweep_recovery')!=expected or frozen.get('clear_sweep_recovery')!=expected:
        raise ValueError('declared empty-sweep recovery changed')
    result=parent_audit(run,config,ticks,spec,frozen)
    result['planar_sweeps']=audit_stream(config,ticks)
    if spec['sweep_mode']=='capture':
        old=baseline(spec);folder=Path(spec['_folder'])
        if sha(folder/'replay/outputs.jsonl')!=old['storage']['replay/outputs.jsonl']['sha256']:
            raise ValueError('capturing acquired rays changed operational output bytes')
        preceding=json.loads(archive(spec,'run/run.json'))
        if preceding['frames']!=run['frames'] or preceding['summary']!=run['summary']:
            raise ValueError('capturing acquired rays changed physical frames or summary')
        original=[json.loads(line)['tick']['input'] for line in archive(spec,'run/sensors.jsonl').splitlines() if 'tick' in json.loads(line)]
        for previous,tick in zip(original,ticks):
            current=dict(tick['input'])
            if current.get('lidar') is not None:
                current['lidar']=dict(current['lidar']);current['lidar'].pop('sweep')
            if current!=previous:raise ValueError('ray capture changed original sensor values or clocks')
        if len(original)!=len(ticks):raise ValueError('capture lost original sensor ticks')
        result['capture_compatibility']=dict(passed=True,identical_physical_frames=True,identical_summary=True,
            identical_original_sensor_values=True,operational_output_sha256=old['storage']['replay/outputs.jsonl']['sha256'])
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--baseline-output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true');parser.add_argument('--check-contract',action='store_true')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve();plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants),factory=materialize,gates=GATES)
    if args.freeze_only:return
    if not packet(str(args.baseline_output.resolve()/'report.json'))['complete']:raise ValueError('preceding physical qualification incomplete')
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()};fingerprints={p:sha(binaries[p]) for p in plants}
    validate_inputs(output,manifest,args.binary.resolve())
    report=dict(complete=False,passed=False,gates=GATES,binaries=fingerprints,
        manifest_sha256=sha(output/'frozen-manifest.json'),episodes=[])
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row=run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
            auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder),_baseline=str(args.baseline_output.resolve())),f))
        row['comparison']=compare(row,baseline(spec));row['original_comparison']=compare(row,baseline(spec,original=True))
        return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            report['episodes'].append(row);rows=report['episodes'];enabled=[r for r in rows if r['sweep_mode']=='recover']
            report['complete']=len(rows)==len(manifest['episodes'])
            report['contract_passed']=all(not r.get('audit_error') and r.get('forecast_consistency',{}).get('passed') is True
                and r.get('planar_sweeps',{}).get('passed') is True and r.get('forecast_hold_physics',{}).get('passed') is True
                and r.get('run_exit_code')==(0 if r.get('summary',{}).get('passed') else 1)
                and r.get('replay_exit_code')==0 and r.get('replay',{}).get('verified') is True
                and r['replay']['ticks']==r['summary']['steps'] for r in rows)
            report['capture_compatibility_passed']=all(r.get('capture_compatibility',{}).get('passed') is True for r in rows if r['sweep_mode']=='capture')
            report['regression_passed']=all(r['comparison']['regression_passed'] for r in enabled)
            report['original_regression_passed']=all(r['original_comparison']['regression_passed'] for r in enabled)
            report['strict_enabled_passes']=sum(r['passed'] for r in enabled)
            report['passed']=report['complete'] and report['contract_passed'] and report['capture_compatibility_passed'] and report['regression_passed']
            report['adoption_passed']=report['passed'] and report['strict_enabled_passes']==len(enabled)
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],'strict',row['passed'],row.get('audit_error'),
                'new',row['comparison']['new_failed_gates'],'original-new',row['original_comparison']['new_failed_gates'],flush=True)
    if any(sha(binaries[p])!=h for p,h in fingerprints.items()):raise ValueError('runtime changed during qualification')
    raise SystemExit(0 if report['complete'] and (report['contract_passed'] and report['capture_compatibility_passed'] if args.check_contract else report['passed']) else 1)


if __name__=='__main__':main()
