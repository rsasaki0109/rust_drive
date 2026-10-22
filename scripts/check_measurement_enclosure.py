#!/usr/bin/env python3
"""Freeze measured-disc preservation against the complete parent qualification."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import json
import os
from pathlib import Path

from check_gnss_map_history import sha
from check_relocalization_map_only import freeze as freeze_inputs,run_case
from check_road_width import validate_inputs
from check_track_initialization import GATES as PARENT_GATES,audit as parent_audit,compare,materialize as parent_input,specifications as parent_specs
from measurement_enclosure_oracle import reconstruct

REPO=Path(__file__).resolve().parents[1]
PARENT=REPO/'assets/track-initialization-results.json'
ORIGINAL=REPO/'assets/track-initialization-baselines.json'
GATES=dict(PARENT_GATES,minimum_actual_road_margin_m=.1,maximum_measured_disc_containment_failures=0,
           extent='Minimum radius at the filtered center enclosing the smoothed disc and the newest measured disc',
           parent_results_sha256=sha(PARENT))


@lru_cache(maxsize=1)
def parent():return json.loads(PARENT.read_text())


@lru_cache(maxsize=1)
def original_parent():return json.loads(ORIGINAL.read_text())


def initialization(spec):
    mode=spec.get('initialization_mode',spec.get('initialization','confirmed'))
    if mode not in ['confirmed','legacy']:raise ValueError('unknown initialization mode')
    return mode


def specifications(plants,*,initialization='confirmed'):
    rows=parent_specs(plants)
    if initialization=='legacy':return [dict(r,initialization='legacy') for r in rows]
    if initialization!='confirmed':raise ValueError('unknown initialization mode')
    return rows


def materialize(spec):
    source,value=parent_input(spec);value['enclose_track_measurements']=True
    if initialization(spec)=='legacy':value.pop('confirm_track_initialization')
    return source,value


def baseline(spec):
    keys=['cohort','plant','world','seed','mode']
    if initialization(spec)=='legacy':
        record=next(r for r in original_parent()['episodes'] if all(r['specification'][k]==spec[k] for k in keys))
        old,declared=record['episode'],record['frozen_input']
    else:
        old=next(r for r in parent()['final_matrix']['episodes'] if all(r[k]==spec[k] for k in keys))
        declared=parent()['frozen_inputs'][old['scenario']]['input']
    _,value=materialize(spec);value.pop('enclose_track_measurements')
    if value!=declared:raise ValueError('parent world or policies changed')
    return old


def freeze(output,specs):
    modes={initialization(s) for s in specs}
    if len(modes)!=1:raise ValueError('freeze one declared initialization mode at a time')
    gates=GATES if modes=={'confirmed'} else dict(GATES,parent_results_sha256=sha(ORIGINAL),initialization='legacy')
    return freeze_inputs(output,specs,factory=materialize,gates=gates)


def audit(run,config,ticks,spec,frozen):
    if config.get('enclose_track_measurements') is not True:raise ValueError('declared measured geometry policy lost')
    result=parent_audit(run,config,ticks,spec,frozen)
    result['measurement_enclosure']=reconstruct(config,ticks)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true')
    parser.add_argument('--only-index',type=int)
    parser.add_argument('--initialization',choices=['confirmed','legacy'],default='confirmed',
                        help='Use confirmed initial motion or qualify enclosure with ordinary alpha-beta initialization')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/'target/release'/('rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/'integrations/rne/target/release'/('rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve()
    plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    manifest=freeze(output,specifications(plants,initialization=args.initialization));old=[baseline(s) for s in manifest['episodes']]
    encoded=json.dumps(old,indent=2)+'\n';path=output/'baselines.json'
    if path.exists():
        if path.read_text()!=encoded:raise ValueError('frozen parent results changed')
    else:path.write_text(encoded)
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    fingerprints={p:sha(binaries[p]) for p in plants};validate_inputs(output,manifest,args.binary.resolve())
    rows=[];selected=list(enumerate(manifest['episodes']))
    if args.only_index is not None:selected=[selected[args.only_index]]
    def one(item):
        index,spec=item;folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        row=run_case(output,spec,binaries[spec['plant']],args.binary.resolve(),
                     auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder)),f))
        row['initialization_mode']=args.initialization
        row['comparison']=compare(row,old[index])
        receipts=output/'episode-receipts';receipts.mkdir(exist_ok=True)
        (receipts/f'{index:03d}.json').write_text(json.dumps(row,indent=2)+'\n');return row
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,selected):
            rows.append(row)
            report=dict(complete=len(rows)==len(selected),binaries=fingerprints,manifest_sha256=sha(output/'frozen-manifest.json'),
                        parent_results_sha256=sha(PARENT if args.initialization=='confirmed' else ORIGINAL),
                        initialization=args.initialization,episodes=rows,strict_passes=sum(r['passed'] for r in rows),
                        regression_passed=all(r['comparison']['regression_passed'] for r in rows),
                        measurement_contract_passed=all(r.get('measurement_enclosure',{}).get('passed') is True for r in rows))
            (output/('report.json' if args.only_index is None else f'report-{args.only_index}.json')).write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row['passed'],row.get('audit_error'),row['comparison']['new_failed_gates'],flush=True)
    assert all(sha(binaries[p])==fingerprints[p] for p in plants)
    if not(report['regression_passed'] and report['measurement_contract_passed']):raise SystemExit(1)


if __name__=='__main__':main()
