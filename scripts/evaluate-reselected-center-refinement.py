#!/usr/bin/env python3
"""Compare fitted-center reselection with the fixed union on ten viewed windows."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import reselected_center_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('reselected_center_recorded_inputs','evaluate-alternating-depth-refinement.py')


previous = load('reselected_center_original_statistics','evaluate-reselected-slope-refinement.py')


def statistics(rows,controls):
    result = previous.statistics(rows,controls)
    result['independently_checked_stage_pairs'] = sum(len(stage[key]) for row in rows for stage in row['round_admission_stages'] for key in ('initial','terminal_reselected'))
    return result


def recorded(case,source):
    saved = source['recorded'][case];require([s['source_index'] for s in saved['samples']]==list(SAMPLES),'same ten viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body);require(sha(body)==saved['input_freeze_sha256'],'original input freeze')
    inventory = json.loads((parent/'files.json').read_text());require(saved['sensor_ledger_sha256']==inventory[prefix+'sensor-only.json.gz']['sha256'],'original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body)==freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']};suffix = case.replace('-','_')
    audit = load('reselected_center_raw_'+suffix,'check-reciprocal-keyframes.py').auditor()
    verifier = load('reselected_center_verify_'+suffix,'check-reselected-center-refinement.py');surfaces = {};rows = [];systems = controls = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];receipt = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0<path.stat().st_size<=4*1024*1024,'bounded original PNG')
            body = path.read_bytes();require(sha(body)==receipt['sha256'] and len(body)==receipt['bytes'],'original PNG digest')
            image = Image.open(io.BytesIO(body));require(image.size==(640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth image')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (kernel.fixed.prepare([dict(surface=kernel.fixed.base.pyramid(depth,freeze['camera']))])[0]['surface'],
                verifier.reference.prepare([dict(surface=audit.pyramid(depth,freeze['camera']))])[0]['surface'])
        return surfaces[stamp]
    for sample in saved['samples']:
        control = sample['refinement'];initial = np.asarray(control['initial_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
        prior_before = None if op_prior is None else json.dumps(op_prior.encode(),sort_keys=True)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        slopes = verifier.reference.compare_prepared(nodes,other)
        controls += verifier.reference.verify(control,other,initial,prior,audit)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,other,initial,prior,audit)
        require(report['rounds'][0]['batch']==dict(control,cap=30),'first full round exactly reproduces the prior terminal fit within30')
        for previous,next_round in zip(report['rounds'],report['rounds'][1:]):
            require(previous['reselected']['population']==next_round['batch']['frozen_population'],'reselection becomes the next fixed population without another state change')
        for key in ('population','equation','correction'):
            require(report['rounds'][0]['reselected'][key]==sample['terminal_reselected'][key],'first fresh system exactly reproduces fixed-union control')
        stages = []
        for row in report['rounds']:
            counts = dict(number=row['number'])
            for key,poses in (('initial',row['batch']['initial_poses']),('terminal_reselected',row['batch']['final_poses'])):
                selected,counts[key] = kernel.fixed.selection(nodes,np.asarray(poses),1)
                wanted,expected = verifier.reference.selection(other,np.asarray(poses),1)
                require(counts[key]==expected,'independent round raw/legacy/fitted/union stages')
                verifier.reference.check_metadata(kernel.fixed.metadata(selected),wanted)
            stages.append(counts)
        terminal = np.asarray(report['final_poses'])
        selection = kernel.transition(kernel.fixed.population(nodes,initial,1),kernel.fixed.population(nodes,terminal,1))
        expected = verifier.transition(verifier.reference.population(other,initial,1),verifier.reference.population(other,terminal,1))
        verifier.compare_transition(selection,expected)
        require(prior_before==(None if op_prior is None else json.dumps(op_prior.encode(),sort_keys=True)),'incoming nonlinear prior remains unchanged across all rounds')
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_report_sha256=sha(json.dumps(control,sort_keys=True).encode()),first_round_matches_control_except_cap=True,
            depth_slope_fields=slopes,round_admission_stages=stages,first_reselected_system_matches_control=True,initial_to_terminal_reselection=selection,incoming_prior_unchanged=True,refinement=report))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'rounds',report['association_rounds'],'updates',report['applied_iterations'],
            'selection/discrete',report['selection_converged'],report['final_original_discrete_correction']['converged'],
            'stop',report['termination'],report['guard_failure'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_systems=systems,independently_rechecked_control_systems=controls,statistics=statistics(rows,saved['samples']),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/fitted-center-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free original local input')
    freeze = json.loads((ROOT/'assets/reselected-center-refinement-v1/source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'frozen source '+name)
    analytic = load('reselected_center_recorded_analytic','check-reselected-center-refinement.py').controls()
    analytic['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('reselected_center_refinement.py','check-reselected-center-refinement.py')}
    write(out/'analytic.json',analytic)
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda case:recorded(case,source),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'unchanged source '+name)
    numerical_progression = all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values())
    require(analytic['identity_admission_agreement'] and analytic['source_proof'],'resolved analytic selection and isolated scheduler bindings')
    write(out/'summary.json',dict(data_role='already viewed development; fitted-center union in unchanged three-round scheduler',sample_indices=list(SAMPLES),
        max_rounds=3,inner_cap=30,max_corrections=90,input_fitted_center_sha256=sha(body),real_local_pose_fitting=True,
        new_trajectory_fitting=False,numeric_ground_truth_parsed=False,physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        numerical_progression_rule_pass=numerical_progression,identity_admission_agreement=True,
        eligible_for_followup_full_trajectory=numerical_progression,recorded=reports,source_sha256=freeze))


if __name__=='__main__':main()
