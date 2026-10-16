#!/usr/bin/env python3
"""Compare two-pixel interior reselection on the same ten viewed windows."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import interior_center_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


common = load('interior_center_recorded_inputs','evaluate-alternating-depth-refinement.py')


previous = load('interior_center_original_statistics','evaluate-reselected-center-refinement.py')


def statistics(rows,controls):
    first = [dict(r,refinement=r['refinement']['rounds'][0]['batch']) for r in controls]
    result = previous.statistics(rows,first)
    old = [r['refinement'] for r in controls]
    result['control_fixed_converged'] = sum(q['final_fixed_converged'] for q in old)
    result['control_original_discrete_fresh_converged'] = sum(q['final_original_discrete_correction']['converged'] for q in old)
    result['control_median_original_discrete_rms_m'] = float(np.median([q['final_original_discrete_equation']['rms_m'] for q in old]))
    initial = [r for s in rows for r in s['initial_selection']]
    result['control_initial_source_occurrences'] = sum(r['old_count'] for r in initial)
    result['retained_control_initial_source_occurrences'] = sum(r['shared_count'] for r in initial)
    result['removed_control_initial_source_occurrences'] = sum(r['removed_count'] for r in initial)
    result['initial_population_retention_vs_control'] = result['retained_control_initial_source_occurrences']/result['control_initial_source_occurrences']
    result['original_one_pixel_continuous_fresh_converged'] = sum(r['terminal_one_pixel_reselected']['correction']['converged'] for r in rows)
    result['eligible_for_followup_full_trajectory'] = bool(result['selection_converged']==len(rows)
        and result['original_one_pixel_continuous_fresh_converged']==len(rows)
        and result['original_discrete_fresh_converged']>result['control_original_discrete_fresh_converged']
        and all(r['refinement']['guard_failure'] is None for r in rows)
        and result['original_discrete_guard_failures']==0 and result['original_discrete_rms_gate_passes']==len(rows)
        and result['median_original_discrete_rms_m']<=result['control_median_original_discrete_rms_m'])
    return result


def recorded(case,source):
    saved = source['recorded'][case];require([s['source_index'] for s in saved['samples']]==list(SAMPLES),'same ten viewed windows')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(body);require(sha(body)==saved['input_freeze_sha256'],'original input freeze')
    inventory = json.loads((parent/'files.json').read_text());require(saved['sensor_ledger_sha256']==inventory[prefix+'sensor-only.json.gz']['sha256'],'original sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body)==freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']};suffix = case.replace('-','_')
    audit = load('interior_center_raw_'+suffix,'check-reciprocal-keyframes.py').auditor()
    verifier = load('interior_center_verify_'+suffix,'check-interior-center-refinement.py');surfaces = {};rows = [];systems = controls = one_pixel_systems = 0
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
        controls += verifier.previous.verify(control,other,initial,prior,audit)
        report = kernel.refine(nodes,initial,op_prior);systems += verifier.verify(report,other,initial,prior,audit)
        for previous,next_round in zip(report['rounds'],report['rounds'][1:]):
            require(previous['reselected']['population']==next_round['batch']['frozen_population'],'reselection becomes the next fixed population without another state change')
        stages = []
        for row in report['rounds']:
            counts = dict(number=row['number'])
            for key,poses in (('initial',row['batch']['initial_poses']),('terminal_reselected',row['batch']['final_poses'])):
                selected,counts[key] = kernel.fixed.selection(nodes,np.asarray(poses),1)
                wanted,expected = verifier.reference.selection(other,np.asarray(poses),1)
                require(counts[key]==expected,'independent round raw/legacy/fitted/union stages')
                verifier.reference.check_metadata(kernel.fixed.metadata(selected),wanted)
            stages.append(counts)
        original = kernel.previous.fixed.population(nodes,initial,1);chosen = kernel.population(nodes,initial,1)
        old_pairs = {(r['target'],r['source']):r for r in original}
        for row in chosen:
            old = old_pairs[row['target'],row['source']];index = np.flatnonzero(np.isin(old['source_ids'],row['source_ids']))
            for key in ('source_ids','points','normals','normalizer'):np.testing.assert_array_equal(old[key][index],row[key])
        receipt = kernel.transition(original,chosen)
        wanted = verifier.transition(verifier.previous.reference.population(other,initial,1),verifier.population(other,initial,1))
        verifier.compare_transition(receipt,wanted);require(all(r['added_count']==r['changed_normalizers_on_shared']==0 for r in receipt),'strict subset with unchanged initial metrics')
        require([{k:v for k,v in s.items() if k not in ('interior_after_probes','admitted_interior_count')} for s in stages[0]['initial']]==sample['round_admission_stages'][0]['initial'],'original center stage counts unchanged')
        terminal = np.asarray(report['final_poses'])
        selection = kernel.transition(kernel.fixed.population(nodes,initial,1),kernel.fixed.population(nodes,terminal,1))
        expected = verifier.transition(verifier.reference.population(other,initial,1),verifier.reference.population(other,terminal,1))
        verifier.compare_transition(selection,expected)
        one_pixel = kernel.previous.fixed.population(nodes,terminal,1)
        one_pixel_ind = verifier.previous.reference.population(other,terminal,1)
        verifier.reference.check_metadata(kernel.fixed.metadata(one_pixel),one_pixel_ind)
        e = kernel.fixed.equation(one_pixel,nodes,terminal,op_prior,1)
        independent = verifier.reference.system(one_pixel_ind,other,terminal,prior,1,audit)
        audit.compare_equation(kernel.fixed.base.encode_equation(e),independent);one_pixel_systems += 1
        correction = kernel.fixed.discrete.correction(e,len(nodes)-1)
        verifier.reference.reference.check_correction(correction,independent,len(nodes)-1,audit)
        require(prior_before==(None if op_prior is None else json.dumps(op_prior.encode(),sort_keys=True)),'incoming nonlinear prior remains unchanged across all rounds')
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        rows.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],prior_before=prior,
            control_report_sha256=sha(json.dumps(control,sort_keys=True).encode()),complete_first_fit_equal_to_control=report['rounds'][0]['batch']==control['rounds'][0]['batch'],initial_selection=receipt,
            depth_slope_fields=slopes,round_admission_stages=stages,initial_to_terminal_reselection=selection,
            terminal_one_pixel_reselected=dict(population=kernel.fixed.metadata(one_pixel),equation=kernel.fixed.base.encode_equation(e),correction=correction),incoming_prior_unchanged=True,refinement=report))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],'rounds',report['association_rounds'],'updates',report['applied_iterations'],
            'selection/discrete',report['selection_converged'],report['final_original_discrete_correction']['converged'],
            'stop',report['termination'],report['guard_failure'],flush=True)
    return dict(input_freeze_sha256=saved['input_freeze_sha256'],sensor_ledger_sha256=saved['sensor_ledger_sha256'],
        independently_checked_systems=systems,independently_rechecked_control_systems=controls,independently_checked_one_pixel_systems=one_pixel_systems,
        statistics=statistics(rows,saved['samples']),samples=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = common.archived(ROOT/'assets/reselected-center-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free original local input')
    freeze = json.loads((ROOT/'assets/interior-center-refinement-v1/source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'frozen source '+name)
    analytic = load('interior_center_recorded_analytic','check-interior-center-refinement.py').controls()
    analytic['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('interior_center_refinement.py','check-interior-center-refinement.py')}
    write(out/'analytic.json',analytic)
    with ThreadPoolExecutor(max_workers=2) as pool:reports = dict(zip(CASES,pool.map(lambda case:recorded(case,source),CASES),strict=True))
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes())==digest,'unchanged source '+name)
    numerical_progression = all(v['statistics']['eligible_for_followup_full_trajectory'] for v in reports.values())
    require(analytic['identity_admission_agreement'] and analytic['source_proof'],'resolved analytic selection and isolated scheduler bindings')
    write(out/'summary.json',dict(data_role='already viewed development; two-pixel interior in unchanged three-round scheduler',sample_indices=list(SAMPLES),
        max_rounds=3,inner_cap=30,max_corrections=90,input_reselected_center_sha256=sha(body),interior_radius_pixels=2,real_local_pose_fitting=True,
        new_trajectory_fitting=False,numeric_ground_truth_parsed=False,physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False,
        numerical_progression_rule_pass=numerical_progression,identity_admission_agreement=True,
        eligible_for_followup_full_trajectory=numerical_progression,recorded=reports,source_sha256=freeze))


if __name__=='__main__':main()
