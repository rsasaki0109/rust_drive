#!/usr/bin/env python3
"""Replay saved continuation poses and decompose pixel reassociation without labels."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

import depth_correspondence_stability as operational
import depth_correspondence_oracle as oracle
from frozen_depth_objective import energy
from patch_depth_normals import pyramid
from projective_depth_common import ROOT,fresh,require,sha,write

CASES = ('tum-fr1-room-temporal','tum-fr1-desk2-independent')
SAMPLES = (120,160,200,240,279)


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def archived(packet,name):
    inventory = json.loads((packet/'files.json').read_text());expected = inventory[name];body = (packet/name).read_bytes()
    require(len(body) == expected['bytes'] and sha(body) == expected['sha256'],'archived input '+str(packet/name))
    return body


def prior_object(value):
    if value is None:return None
    from reciprocal_keyframe_depth import Prior
    return Prior(value['origin'],value['ids'],[np.asarray(p) for p in value['anchors']],
        np.asarray(value['h']),np.asarray(value['g']),list(map(tuple,value['consumed_pairs'])),value['generation'])


def statistics(samples):
    rows = [r for s in samples for r in s['iterations']];totals = [r['totals'] for r in rows]
    before = sum(t['before_count'] for t in totals);after = sum(t['after_count'] for t in totals)
    stable = sum(t['stable_pairs'] for t in totals)
    return dict(iterations=len(rows),fixed_cost_decreases=sum(t['fixed_step_delta'] < 0 for t in totals),
        reassociated_cost_increases=sum(t['total_delta'] > 1e-12*max(1,abs(t['objective_before'])) for t in totals),
        reassociation_adds_cost=sum(t['association_delta'] > 1e-12 for t in totals),
        total_before_candidate_occurrences=before,total_after_candidate_occurrences=after,stable_candidate_occurrences=stable,
        retained_before_fraction=stable/before if before else None,
        aggregate_jaccard=stable/(before+after-stable) if before+after-stable else None,
        retargeted_source_occurrences=sum(t['retargeted_sources'] for t in totals),
        disappeared_source_occurrences=sum(t['disappeared_sources'] for t in totals),
        appeared_source_occurrences=sum(t['appeared_sources'] for t in totals),
        factor_gate_changes=sum(t['factor_gate_changes'] for t in totals),
        maximum_reassociated_cost_increase=max((t['total_delta'] for t in totals),default=None),
        maximum_fixed_step_delta=max((t['fixed_step_delta'] for t in totals),default=None))


def recorded(case,source):
    frozen = source['recorded'][case]
    require([s['source_index'] for s in frozen['samples']] == list(SAMPLES),'fixed recorded sample indices')
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    body = archived(parent,prefix+'freeze.json');freeze = json.loads(body)
    require(sha(body) == frozen['input_freeze_sha256'],'same original input freeze')
    manifest = json.loads((parent/'files.json').read_text())
    require(frozen['sensor_ledger_sha256'] == manifest[prefix+'sensor-only.json.gz']['sha256'],'same archived sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes()
    require(sha(body) == freeze['manifest_sha256'],'same original input manifest')
    by_stamp = {r['depth_timestamp']:r for r in json.loads(body)['frames']}
    audit = load('turnover_audit_'+case.replace('-','_'),'check-reciprocal-keyframes.py').auditor()
    checker = load('turnover_checker_'+case.replace('-','_'),'check-depth-correspondence.py')
    surfaces = {};samples = [];systems = transitions = 0
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original depth')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original depth hash')
            image = Image.open(io.BytesIO(raw))
            require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'original axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = (pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera']))
        return surfaces[stamp]
    for sample in frozen['samples']:
        prior = sample['prior_before'];op_prior = prior_object(prior);poses = np.asarray(sample['initial_poses'])
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],poses,strict=True)]
        independent = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        reports = []
        require(len(sample['trace']) == 10,'ten archived continuation updates')
        for index,step in enumerate(sample['trace']):
            require(step['scale'] == 1,'finest archived continuation only')
            np.testing.assert_array_equal(step['before'],poses)
            new = audit.audit_step(independent,prior,step);systems += 1
            edges = [];expected_edges = []
            for b in range(1,len(nodes)):
                for a in range(b):
                    old_pairs = operational.select(nodes[a],nodes[b],poses[a],poses[b],1)
                    new_pairs = operational.select(nodes[a],nodes[b],new[a],new[b],1)
                    old_ind = oracle.independent_matches([dict(independent[a],root=poses[a])],independent[b],poses[b],1,False)
                    new_ind = oracle.independent_matches([dict(independent[a],root=new[a])],independent[b],new[b],1,False)
                    actual = operational.transition(old_pairs,new_pairs,poses[a],poses[b],new[a],new[b])
                    expected = oracle.transition(old_ind,new_ind,poses[a],poses[b],new[a],new[b])
                    checker.compare(actual,expected);transitions += 1
                    edges.append(dict(actual,target=a,source=b));expected_edges.append(expected)
            pb,pa = energy([],nodes,poses,op_prior),energy([],nodes,new,op_prior)
            totals = operational.aggregate(edges,pb,pa)
            expected_totals = oracle.aggregate(expected_edges,oracle.prior_value(prior,independent,poses,audit),oracle.prior_value(prior,independent,new,audit))
            checker.compare(totals,expected_totals)
            require(abs(totals['association_delta']-(totals['added_cost_at_after']-totals['removed_cost_at_after'])) <= 1e-10,'global exact cost partition')
            reports.append(dict(iteration=index+1,edges=edges,totals=totals));poses = new
        audit.compare_equation(sample['final_equation'],audit.independent_equation(independent,poses,1,prior));systems += 1
        np.testing.assert_array_equal(sample['final_poses'],poses)
        samples.append(dict(source_index=sample['source_index'],node_timestamps=sample['node_timestamps'],iterations=reports))
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
        print(case,sample['source_index'],json.dumps(statistics([samples[-1]])),flush=True)
    return dict(input_freeze_sha256=frozen['input_freeze_sha256'],sensor_ledger_sha256=frozen['sensor_ledger_sha256'],
        independently_reconstructed_saved_systems=systems,independently_checked_pair_transitions=transitions,
        samples=samples,statistics=statistics(samples))


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    body = archived(ROOT/'assets/depth-convergence-v1','summary.json');source = json.loads(body)
    require(not source['real_numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'frozen label-free diagnostic source')
    with ThreadPoolExecutor(max_workers=2) as pool:
        reports = dict(zip(CASES,pool.map(lambda case:recorded(case,source),CASES),strict=True))
    names = set(json.loads((ROOT/'assets/descent-keyframe-depth-v1/source-freeze.json').read_text()))
    names |= set(json.loads((ROOT/'assets/depth-convergence-v1/source-freeze.json').read_text()))
    names |= {'scripts/'+name for name in ('diagnose-depth-correspondence.py','check-depth-correspondence.py','depth_correspondence_stability.py','depth_correspondence_oracle.py')}
    names.add('assets/depth-correspondence-v1/design.json')
    write(out/'summary.json',dict(data_role='already viewed development; saved continuation replay only',sample_indices=list(SAMPLES),
        input_continuation_sha256=sha(body),real_pose_fitting=False,numeric_ground_truth_parsed=False,
        tracker_state_commits=False,default_changed=False,recorded=reports,
        counts_are_overlapping_pair_occurrences=True,raw_sum_population_can_change=True,
        source_sha256={name:sha((ROOT/name).read_bytes()) for name in sorted(names)}))


if __name__ == '__main__':main()
