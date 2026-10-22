#!/usr/bin/env python3
"""Qualify decision recording against all 44 preceding physical input conditions."""
import argparse
from concurrent.futures import ThreadPoolExecutor
from functools import lru_cache
import hashlib
import json
import os
from pathlib import Path

from check_gnss_map_history import sha
from check_refined_lateral import audit as parent_audit
from check_relocalization_map_only import freeze,run_case
from check_road_width import validate_inputs
from inspect_planning import audit as audit_decisions,project_output
from comparison_control import Control

REPO=Path(__file__).resolve().parents[1]
PARENT=REPO/'assets/refined-lateral-results.json'


@lru_cache(maxsize=1)
def parent():return json.loads(PARENT.read_text())


def key(row):return tuple(row[k] for k in ['plant','world','mode','seed'])


def baseline(spec):return next(row for row in parent()['final_matrix']['episodes'] if key(row)==key(spec))


def specifications(plants):
    return [dict((k,row[k]) for k in ['plant','world','mode','seed','evaluation_family','refined','cohort','heading','kind','family','initialization'] if k in row)
            for row in parent()['final_matrix']['episodes'] if row['plant'] in plants]


def materialize(spec):
    old=baseline(spec);frozen=parent()['frozen_inputs'][old['scenario']]
    value=dict(frozen['input']);value['record_planning_diagnostics']=True
    return old['source'],value


def baseline_archive(spec,name):
    old=baseline(spec);control=spec['_control']
    provided=control.episode(old,parent()['frozen_inputs'][old['scenario']]['input'])
    return control.archive(provided,name)


def audit(run,config,ticks,spec,frozen):
    if config.get('record_planning_diagnostics') is not True or frozen.get('record_planning_diagnostics') is not True:
        raise ValueError('declared decision capture disappeared')
    result=parent_audit(run,config,ticks,spec,frozen)
    result['planning_decisions']=audit_decisions(config,ticks)
    old=baseline(spec);folder=Path(spec['_folder'])
    encoded,record=baseline_archive(spec,'run/run.json')
    previous=json.loads(encoded)
    digest=hashlib.sha256();count=0
    with (folder/'replay/outputs.jsonl').open('rb') as stream:
        for line in stream:digest.update(project_output(line));count+=1
    _,reference=baseline_archive(spec,'replay/outputs.jsonl')
    expected=reference['sha256']
    if digest.hexdigest()!=expected or count!=previous['summary']['steps']:
        raise ValueError('capturing decisions changed operational output bytes')
    if run['frames']!=previous['frames'] or run['summary']!=previous['summary']:
        raise ValueError('capturing decisions changed physical frames or complete outcomes')
    result['compatibility']=dict(operational_output_sha256=expected,projected_output_sha256=digest.hexdigest(),
                                 verified_output_ticks=count,physical_frames_identical=True,summary_identical=True,
                                 control_binary_sha256=spec['_control'].binaries[spec['plant']],
                                 historical_output_sha256=old['storage']['replay/outputs.jsonl']['sha256'])
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline-output',type=Path,required=True,
                        help='Complete preceding check_refined_lateral.py output with archived physical frames')
    parser.add_argument('--plant',choices=['all','reference-2d','rne-dynamic'],default='all')
    parser.add_argument('--freeze-only',action='store_true')
    suffix='.exe' if os.name=='nt' else ''
    parser.add_argument('--binary',type=Path,default=REPO/('target/release/rustdriving'+suffix))
    parser.add_argument('--native-binary',type=Path,default=REPO/('integrations/rne/target/release/rustdriving-rne'+suffix))
    args=parser.parse_args();output=args.output.resolve()
    plants=['reference-2d','rne-dynamic'] if args.plant=='all' else [args.plant]
    gates=dict(parent()['original_physical_gates'],parent_results_sha256=sha(PARENT),
               maximum_candidate_records=9,require_identical_operational_output_projection=True,
               require_identical_physical_frames=True,default_enabled=False)
    manifest=freeze(output,specifications(plants),factory=materialize,gates=gates)
    if args.freeze_only:return
    binaries={'reference-2d':args.binary.resolve(),'rne-dynamic':args.native_binary.resolve()}
    control=Control(args.baseline_output,{p:binaries[p] for p in plants})
    for spec in manifest['episodes']:
        baseline_archive(dict(spec,_control=control),'run/run.json')
    validate_inputs(output,manifest,args.binary.resolve())
    fingerprints={p:sha(binaries[p]) for p in plants}
    report=dict(complete=False,passed=False,gates=gates,manifest_sha256=sha(output/'frozen-manifest.json'),
                binaries=fingerprints,episodes=[])
    def one(spec):
        folder=output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mode']}-seed-{spec['seed']}"
        return run_case(output,spec,binaries[spec['plant']],binaries['reference-2d'],
                        auditor=lambda r,c,t,s,f:audit(r,c,t,dict(s,_folder=str(folder),_control=control),f))
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(one,manifest['episodes']):
            report['episodes'].append(row);report['complete']=len(report['episodes'])==len(manifest['episodes'])
            report['passed']=report['complete'] and all(r['passed'] for r in report['episodes'])
            (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
            print(row['plant'],row['world'],row['mode'],row['seed'],row['passed'],row.get('audit_error',row.get('strict_failed_gates')),flush=True)
    if any(sha(binaries[p])!=h for p,h in fingerprints.items()):raise ValueError('binary changed during qualification')
    raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
