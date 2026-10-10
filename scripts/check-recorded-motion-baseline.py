#!/usr/bin/env python3
"""Verify the archived first temporal failure independently from raw timestamps."""
import argparse
import bisect
import hashlib
import json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent

def main():
    p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,default=ROOT/'data/tum-fr1-xyz-motion/raw');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    baseline=ROOT/'integrations/rgbd/baselines/motion-temporal-v1'
    freeze_raw=(baseline/'freeze.json').read_bytes();freeze=json.loads(freeze_raw);manifest=json.loads((baseline/'manifest.json').read_text());failure=json.loads((baseline/'failure.json').read_text())
    digest=lambda b:hashlib.sha256(b).hexdigest()
    for key,name in [('evaluator_source_sha256','evaluator.rs'),('motion_source_sha256','motion.rs'),('independent_checker_sha256','checker.py')]:
        if freeze[key]!=digest((baseline/name).read_bytes()):raise ValueError('original archived source differs '+name)
    if freeze['manifest_sha256']!=digest((baseline/'manifest.json').read_bytes()) or failure['freeze_sha256']!=digest(freeze_raw):raise ValueError('original protocol/manifest differs')
    if failure['exit_status']!=2 or failure['physical_scoring_complete'] is not False or 'GT interpolation gap exceeds 0.02 seconds' not in (baseline/'failed-run.log').read_text():raise ValueError('failed validity outcome concealed')
    for entry in manifest['files']:
        b=(a.raw/entry['file']).read_bytes()
        if len(b)!=entry['bytes'] or digest(b)!=entry['sha256']:raise ValueError('raw source differs')
    # Read timestamps only; neither depth geometry nor mocap pose values enter.
    stamps=[float(line.split()[0]) for line in (a.raw/'groundtruth.txt').read_text().splitlines() if line and not line.startswith('#')]
    invalid=[]
    for frame in manifest['frames']:
        i=bisect.bisect_left(stamps,frame['timestamp']);gap=stamps[i]-stamps[i-1] if 0<i<len(stamps) else None
        if gap is None or gap>.02:invalid.append({'source_index':frame['source_index'],'timestamp':frame['timestamp'],'bracket_s':gap})
    if [x['source_index'] for x in invalid]!=[201,202,203]:raise ValueError('archived gap witness changed')
    result={'passed_integrity':True,'original_trial_exit_status':2,'original_trial_accuracy_pass':False,'invalid_reference_frames':invalid,'matcher_rerun':False,'numeric_interpolation_gate_s':.02,'raw_and_archived_source_hashes_verified':True}
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
