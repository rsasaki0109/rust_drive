#!/usr/bin/env python3
"""Reproduce recorded-motion evidence, retaining failed/unscorable baselines.

Exit0 means frozen integrity and known regression outcomes reproduced, not that
all physical protocols passed. Previously evaluated selections are now viewed
regressions. Their original preregistered evidence remains immutable.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
ROOT=Path(__file__).resolve().parent.parent
CASES=[('tum-fr1-xyz',4,3,11,11,0),('tum-fr1-xyz-fast',11,11,11,11,11),('tum-fr1-xyz-tight',11,9,11,11,11),('tum-fr1-xyz-motion',11,11,7,8,11),('tum-fr1-xyz-motion-v2',11,9,11,11,11)]

def preserve_original_protocol(current):
    """Audit historical sources and require identical numerical/selection gates."""
    original_path=ROOT/'assets/recorded-motion-v2-freeze.json'
    original=json.loads(original_path.read_text())
    archive=ROOT/'integrations/rgbd/baselines/public-motion-v4'
    source=json.loads((archive/'SOURCE.json').read_text())
    mapping={'evaluator.rs':'evaluator_source_sha256','motion.rs':'motion_source_sha256','checker.py':'independent_checker_sha256'}
    for name,digest in source['sha256'].items():
        actual=hashlib.sha256((archive/name).read_bytes()).hexdigest()
        if actual!=digest or (name in mapping and actual!=original[mapping[name]]):
            raise ValueError('historical public source archive changed: '+name)
    for key in ['registration_config','registration_config_sha256','preprocessing','motion_preprocessing','accuracy_gates','uncertainty','map_policy','frames','manifest_sha256','cargo_lock_sha256','matcher_source_sha256','geometry_checker_sha256']:
        if current[key]!=original[key]:raise ValueError('viewed regression changed original numerical/source gate: '+key)
    if current['kind']!='calibration_regression':raise ValueError('viewed data misrepresented as fresh')
    return hashlib.sha256(original_path.read_bytes()).hexdigest()

def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    binary=a.binary.resolve();results=[]
    for dataset,pa,ma,pr,mr,odo in CASES:
        folder=a.output/dataset;folder.mkdir(exist_ok=True);manifest=ROOT/'data'/dataset/'manifest.json';raw=manifest.parent/'raw';freeze=folder/'freeze.json'
        if not freeze.exists():subprocess.run([str(binary),'--motion','--manifest',str(manifest),'--prepare-freeze',str(freeze)],check=True)
        if dataset.endswith('-v2'):preserve_original_protocol(json.loads(freeze.read_text()))
        report=folder/'results.json';oracle=folder/'oracle.json';log=folder/'evaluator.log'
        with log.open('w') as stream:
            run=subprocess.run([str(binary),'--motion','--manifest',str(manifest),'--raw',str(raw),'--freeze',str(freeze),'--output',str(report)],stdout=stream,stderr=subprocess.STDOUT)
        expected_rc=0 if pa==11 and ma==11 and pr==11 and mr==11 else 1
        if run.returncode!=expected_rc:raise ValueError(f'{dataset}: changed evaluator validity/protocol status{run.returncode}, expected{expected_rc}')
        summary=json.loads(report.read_text())['summary'];expected={'pairs':11,'pair_accepted':pa,'pair_accurate':min(pa,pr),'map_accepted':ma,'map_accurate':min(ma,mr),'pair_reference_valid':pr,'map_reference_valid':mr,'continuous_odometry_frames':odo}
        if summary!=expected:raise ValueError(f'{dataset}: regression changed {summary}, expected{expected}')
        subprocess.run([sys.executable,str(ROOT/'scripts/check-recorded-motion.py'),'--manifest',str(manifest),'--raw',str(raw),'--freeze',str(freeze),'--report',str(report),'--output',str(oracle)],check=True)
        proof=json.loads(oracle.read_text());results.append({'dataset':dataset,'summary':summary,'evaluator_exit_status':run.returncode,'full_pair_and_map_protocol_passed':run.returncode==0,'independent_integrity_passed':proof['passed_integrity'],'mutations_rejected':len(proof['mutations_rejected']),'report_sha256':hashlib.sha256(report.read_bytes()).hexdigest(),'oracle_sha256':hashlib.sha256(oracle.read_bytes()).hexdigest(),'freeze_sha256':hashlib.sha256(freeze.read_bytes()).hexdigest()})
    subprocess.run([sys.executable,str(ROOT/'scripts/check-recorded-motion-baseline.py'),'--output',str(a.output/'first-failure-oracle.json')],check=True)
    result={'schema_version':1,'regression_integrity_passed':True,'all_physical_protocols_passed':all(r['full_pair_and_map_protocol_passed'] for r in results),'cases':results,'scope':'viewed regressions in the same indoor Kinect room; original preregistered source/evidence preserved and numerical gates unchanged; no new holdout, automotive or confidence calibration claim'}
    (a.output/'suite.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
if __name__=='__main__':main()
