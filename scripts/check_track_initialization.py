#!/usr/bin/env python3
"""Compare confirmed cold-track motion without waiving existing qualification gates."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path

from check_forecast_envelope import GATES as FORECAST_GATES, audit as forecast_audit, materialize as forecast_input
from check_gnss_map_history import sha
from check_relocalization_map_only import freeze as freeze_inputs, run_case
from check_road_width import road_margin, validate_inputs
from check_stop_search import audit as stop_audit, materialize as stop_input, specifications as stop_specs
from track_initialization_oracle import reconstruct

REPO=Path(__file__).resolve().parents[1]
GATES=dict(FORECAST_GATES,comparison='No newly failed strict gates against identical parent inputs; all first acquisitions remain scored',
           default_enabled=False,minimum_observations=3,minimum_span_s=.2,maximum_span_s=.6,
           maximum_radius_range_m=.12,maximum_fit_rms_m=.12,maximum_speed_m_s=15.)


def specifications(plants):
    rows=[dict(s,cohort='stop-search') for s in stop_specs(plants)]
    rows += [dict(plant=p,world=w,seed=s,mode='confirmed',cohort='forecast',family='forecast')
             for p in plants for w in ['slow-crossing','traffic-lead-stop','intersection-crossing'] for s in [7,42]]
    return rows


def materialize(spec):
    if spec['cohort']=='stop-search': source,value=stop_input(spec)
    else: source,value=forecast_input(dict(spec,mode='envelope'))
    value['confirm_track_initialization']=True
    return source,value


def freeze(output,specs): return freeze_inputs(output,specs,factory=materialize,gates=GATES)


def verify_tracker_policies(config,frozen):
    policies=['confirm_track_initialization','enclose_track_measurements','fit_recent_track_motion']
    for key in policies:
        actual,declared=config.get(key,False),frozen.get(key,False)
        if type(actual) is not bool or type(declared) is not bool or actual!=declared:
            raise ValueError('declared tracker policy differs: '+key)
    if not any(config.get(key,False) for key in policies):raise ValueError('tracked measurement policy missing')


def audit(run,config,ticks,spec,frozen):
    verify_tracker_policies(config,frozen)
    result=(stop_audit if spec['cohort']=='stop-search' else forecast_audit)(run,config,ticks,spec,frozen)
    if not result.get('road_containment'):
        result['road_containment']=road_margin(run)
        if result['road_containment']['minimum_actual_road_margin_m']<.1:
            result['failed_gates'].append('physical_separation_road_reserve')
    result['initialization']=reconstruct(config,ticks)
    return result


def baseline(spec):
    rows=json.loads((REPO/'assets/track-initialization-baselines.json').read_text())['episodes']
    fields=['cohort','plant','world','seed','mode']
    row=next(r for r in rows if all(r['specification'][k]==spec[k] for k in fields))
    _,declared=materialize(spec);declared.pop('confirm_track_initialization')
    if declared!=row['frozen_input']: raise ValueError('parent world or settings differ')
    return row


def compare(row,old):
    verified=(not row.get('audit_error') and row.get('replay_exit_code')==0
              and row.get('run_exit_code')==(0 if row.get('summary',{}).get('passed') else 1)
              and row.get('replay',{}).get('verified') is True
              and row['replay']['ticks']==row['summary']['steps'])
    new=sorted(set(row.get('failed_gates',[]))-set(old['failed_gates']))
    return dict(regression_passed=verified and not new,new_failed_gates=new,baseline_failed_gates=old['failed_gates'],
                coverage={h:dict(before=old['coverage']['horizons'][h]['coverage_fraction'],after=r['coverage_fraction'],
                                 before_eligible=old['coverage']['horizons'][h]['eligible'],after_eligible=r['eligible'])
                          for h,r in (row.get('coverage') or {}).get('horizons',{}).items()})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--only-index',type=int)
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/'target/release'/('rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release'/('rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve()
    plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants))
    baselines=[baseline(s) for s in manifest['episodes']]
    frozen_baselines=output/'baselines.json';encoded=json.dumps(baselines,indent=2)+'\n'
    if frozen_baselines.exists():
        if frozen_baselines.read_text()!=encoded: raise ValueError('frozen baselines changed')
    else: frozen_baselines.write_text(encoded)
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    fingerprints={p:sha(binaries[p]) for p in plants}
    if not (output/'validation.json').exists():validate_inputs(output,manifest,args.binary.resolve())
    rows=[];selected=list(enumerate(manifest['episodes']))
    if args.only_index is not None:selected=[selected[args.only_index]]
    def one(item):
        index,spec=item
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row=run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
                     auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
        row['comparison']=compare(row,baselines[index]['episode'])
        receipts=output/'episode-receipts';receipts.mkdir(exist_ok=True)
        (receipts/f'{index:03d}.json').write_text(json.dumps(row,indent=2)+'\n')
        return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,selected):
            rows.append(row)
            report=dict(complete=len(rows)==len(selected),binaries=fingerprints,manifest_sha256=sha(output/'frozen-manifest.json'),
                        baselines_sha256=sha(frozen_baselines),episodes=rows,
                        regression_passed=all(r['comparison']['regression_passed'] for r in rows),
                        strict_passes=sum(r['passed'] for r in rows))
            (output/('report.json' if args.only_index is None else f'report-{args.only_index}.json')).write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row['passed'],row.get('audit_error'),row['comparison']['new_failed_gates'],flush=True)
    assert all(sha(binaries[p])==fingerprints[p] for p in plants)
    if not report['regression_passed']:raise SystemExit(1)


if __name__=='__main__':main()
