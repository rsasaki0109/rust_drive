#!/usr/bin/env python3
"""Audit read-only directional sweeps at four archived local search failures."""
import argparse
import copy
import importlib.util
import io
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

import depth_search_diagnostic as diagnostic
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


verifier = load('search_diagnostic_oracle','check-long-depth-refinement.py')
audit = load('search_diagnostic_raw','check-reciprocal-keyframes.py').auditor()
common = load('search_diagnostic_inputs','evaluate-alternating-depth-refinement.py')
kernel = diagnostic.kernel


def oracle_pixels(item,nodes,poses,scale):
    relative = np.linalg.solve(poses[item['target']],poses[item['source']])
    points = np.column_stack([item['points'],np.ones(len(item['points']))])@relative.T
    c = nodes[item['target']]['surface'][scale]['camera']
    image = points[:,:3]@np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]]).T
    uv = image[:,:2]/image[:,2,None]
    nearest = np.round(uv);cell = np.floor(np.where(np.abs(uv-nearest) <= 1e-10,nearest,uv)).astype(int)
    return uv,cell


def check_probe(actual,matches,nodes,poses,prior,scale,direction,g):
    alpha = actual['alpha'];candidate = np.asarray(poses).copy()
    for node,d in enumerate(direction,1):candidate[node] = audit.increment(poses[node],alpha*d)
    np.testing.assert_allclose(diagnostic.moved(poses,direction,alpha),candidate,atol=2e-14,rtol=0)
    before = verifier.objective(matches,nodes,poses,prior,scale,audit)
    try:cost = verifier.objective(matches,nodes,candidate,prior,scale,audit)
    except ValueError as error:
        require(actual['guard_failure'] == str(error),'independent probe domain guard')
        require(all(actual[k] is None for k in ('cost','cost_delta','paired_cost_delta','secant','endpoint_cell_changes',
            'max_pixel_displacement','changed_cell_residual_delta','same_cell_residual_delta','prior_delta')),'invalid probe has no partial cost or source deletion')
        require(not actual['armijo_with_slack'] and not actual['armijo_without_slack'],'invalid probe cannot pass')
        return dict(cost=None,cost_delta=None,paired_cost_delta=None)
    require(actual['guard_failure'] is None,'independently valid probe')
    changed = 0;maximum = 0.;different_terms = [];same_terms = []
    for item in matches:
        r0,_ = verifier.independent.residual(item,nodes,poses,scale)
        r1,_ = verifier.independent.residual(item,nodes,candidate,scale)
        loss0 = np.where(abs(r0) < .01,r0*r0/2,.01*abs(r0)-.00005)
        loss1 = np.where(abs(r1) < .01,r1*r1/2,.01*abs(r1)-.00005)
        uv0,cell0 = oracle_pixels(item,nodes,poses,scale);uv1,cell1 = oracle_pixels(item,nodes,candidate,scale)
        different = np.any(cell0 != cell1,axis=1)
        changed += int(different.sum());maximum = max(maximum,float(abs(uv1-uv0).max()))
        different_terms.extend((loss1-loss0)[different]);same_terms.extend((loss1-loss0)[~different])
    prior_delta = verifier.objective([],nodes,candidate,prior,scale,audit)-verifier.objective([],nodes,poses,prior,scale,audit)
    changed_delta,same_delta = math.fsum(different_terms),math.fsum(same_terms)
    paired = math.fsum((changed_delta,same_delta,prior_delta))
    require(actual['endpoint_cell_changes'] == changed,'independent endpoint cell count')
    np.testing.assert_allclose(actual['max_pixel_displacement'],maximum,atol=3e-11,rtol=1e-8)
    for key,wanted in (('cost',cost),('cost_delta',cost-before),('paired_cost_delta',paired),
        ('changed_cell_residual_delta',changed_delta),('same_cell_residual_delta',same_delta),('prior_delta',prior_delta)):
        np.testing.assert_allclose(actual[key],wanted,atol=2e-12,rtol=1e-7,err_msg=key)
    np.testing.assert_allclose(actual['secant'],actual['paired_cost_delta']/alpha,atol=0,rtol=0)
    slope = float(g@direction.ravel());limit = before+.0001*alpha*slope
    # A decision within objective implementation disagreement is recorded,
    # rather than silently asserting a reliable sign at numerical precision.
    ambiguity = 2e-12
    for key,bound in (('armijo_without_slack',limit),('armijo_with_slack',limit+1e-12*max(1.,abs(before)))):
        if abs(cost-bound) > ambiguity:require(actual[key] == (cost <= bound),'independent resolved Armijo decision')
    return dict(cost=cost,cost_delta=cost-before,paired_cost_delta=paired)


def checked_sweep(op_matches,other_matches,nodes,other,poses,op_prior,prior,direction,g,independent_g):
    rows = diagnostic.sweep(op_matches,nodes,poses,op_prior,1,direction,g)
    for exponent,row in zip(diagnostic.EXPONENTS,rows,strict=True):
        require(row['exponent'] == exponent,'predeclared ordered exponents')
        for key,sign in (('positive',1),('negative',-1)):
            item = row[key];require(item['alpha'] == sign*2.**(-exponent),'predeclared signed multiplier')
            item['oracle'] = check_probe(item,other_matches,other,poses,prior,1,direction,independent_g)
        central = None if any(row[k]['cost'] is None for k in ('positive','negative')) else (
            row['positive']['paired_cost_delta']-row['negative']['paired_cost_delta'])/(2*2.**(-exponent))
        require(row['central_secant'] == central,'central signed difference')
    return rows


def controls():
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5)
    y,x = np.indices((32,32));poses = np.array([np.eye(4),np.eye(4)])
    reports = {};fixtures = {}
    for name in ('smooth','knot','domain'):
        rho = .5+(.001*abs(x-16) if name == 'knot' else .0002*x)+.0001*y
        layer = dict(camera=c,depth=1/rho,normals=np.tile([0.,0.,1.],(32,32,1)))
        uv = np.array([[16.,12.25]]) if name == 'knot' else np.array([[12.25,12.25],[12.75,13.25],[17.25,14.25]])
        points = np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(len(uv),2.)])
        matches = [dict(target=0,source=1,points=points,normals=np.tile([0.,0.,1.],(len(points),1)),
            source_ids=np.arange(len(points)),normalizer=np.ones(len(points)))]
        nodes = [dict(timestamp=t,root=np.eye(4),surface={1:layer}) for t in (1.,1.03)]
        direction = np.array([[-.001 if name != 'domain' else 2.,0,0,0,0,0]])
        op = kernel.equation(matches,nodes,poses,None,1)
        other = verifier.system(matches,nodes,poses,None,1,audit)
        audit.compare_equation(kernel.base.encode_equation(op),other)
        rows = checked_sweep(matches,matches,nodes,nodes,poses,None,None,direction,op['g'],other['g'])
        slope = float(op['g']@direction.ravel())
        if name == 'smooth':
            np.testing.assert_allclose(rows[10]['central_secant'],slope,atol=2e-14,rtol=1e-6)
            require(rows[10]['positive']['endpoint_cell_changes'] == rows[10]['negative']['endpoint_cell_changes'] == 0,'smooth interior fixture')
        elif name == 'knot':
            require(slope < 0 < rows[10]['positive']['secant'] and rows[10]['negative']['secant'] < 0,'one-sided knot slopes distinguish modeled gradient')
            require(rows[10]['positive']['endpoint_cell_changes'] == 1,'left cell at grid knot')
        else:require(rows[0]['positive']['guard_failure'] == 'fixed source support','invalid sampling fixture')
        reports[name] = dict(direction_role='injected analytic direction, not a solved proposal',directional_derivative=slope,rows=rows)
        fixtures[name] = (matches,nodes,direction,other['g'])
    rejected = []
    for name,fixture,key,change in (
        ('wrong cost','smooth','positive',lambda p:p.update(cost=p['cost']+.01)),
        ('wrong cell count','knot','positive',lambda p:p.update(endpoint_cell_changes=0)),
        ('false invalid-probe cost','domain','positive',lambda p:p.update(cost=0.)),
        ('wrong loss partition','smooth','positive',lambda p:p.update(same_cell_residual_delta=.01))):
        item = copy.deepcopy(reports[fixture]['rows'][0][key]);change(item)
        matches,nodes,direction,g = fixtures[fixture]
        try:check_probe(item,matches,nodes,poses,None,1,direction,g)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected search diagnostic corruption: '+name)
    return dict(analytic_pass=True,injected_directions_not_solver_proposals=True,fixtures=reports,rejected_corruptions=rejected)


def recorded(packet):
    body = common.archived(ROOT/'assets/long-depth-refinement-v1','summary.json');source = json.loads(body)
    require(not source['numeric_ground_truth_parsed'] and not source['new_trajectory_fitting'],'label-free archived local states')
    case = 'tum-fr1-room-temporal';saved = source['recorded'][case]
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    freeze_body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(freeze_body)
    require(sha(freeze_body) == saved['input_freeze_sha256'],'same raw input freeze')
    inventory = json.loads((parent/'files.json').read_text())
    require(saved['sensor_ledger_sha256'] == inventory[prefix+'sensor-only.json.gz']['sha256'],'same sensor ledger')
    base = ROOT/'data'/case;manifest_body = (base/'manifest.json').read_bytes()
    require(sha(manifest_body) == freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(manifest_body)['frames']}
    surfaces = {};rows = []
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original PNG')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original PNG hash')
            image = Image.open(io.BytesIO(raw));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = kernel.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera'])
        return surfaces[stamp]
    for sample in saved['samples']:
        if sample['source_index'] not in (120,160,240,279):continue
        q = sample['refinement'];initial,poses = np.asarray(q['initial_poses']),np.asarray(q['final_poses'])
        prior = sample['prior_before'];op_prior = common.prior_object(prior)
        nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
        other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
        matches = kernel.population(nodes,initial,1);expected = verifier.population(other,initial,1)
        require(kernel.metadata(matches) == q['frozen_population'],'original initial identities and normalizers')
        verifier.check_metadata(q['frozen_population'],expected)
        e = kernel.equation(matches,nodes,poses,op_prior,1);ie = verifier.system(expected,other,poses,prior,1,audit)
        audit.compare_equation(q['final_equation'],ie);audit.compare_equation(q['final_equation'],e)
        d,_ = kernel.discrete.proposal(e,len(nodes)-1);reason,independent_d,_ = verifier.reference.failure(ie,len(nodes)-1,audit)
        require(reason is None and not verifier.reference.small(independent_d),'unchanged admissible unconverged terminal proposal')
        np.testing.assert_allclose(d,independent_d,atol=1e-10,rtol=1e-7)
        np.testing.assert_array_equal(d,q['final_correction']['delta'])
        candidate,receipt = kernel.search(matches,nodes,poses,op_prior,1,d,e['g'])
        other_candidate,other_receipt = verifier.search(expected,other,poses,prior,1,independent_d,ie['g'],audit)
        require(candidate is None and other_candidate is None and receipt == q['failed_search'],'archived bounded failure unchanged')
        verifier.compare_search(receipt,other_receipt)
        probes = checked_sweep(matches,expected,nodes,other,poses,op_prior,prior,d,e['g'],ie['g'])
        for trial,row in zip(receipt['trials'],probes[:8],strict=True):
            require(trial['cost'] == row['positive']['cost'] and trial['guard_failure'] == row['positive']['guard_failure'],'all original candidates reproduced exactly')
        distances = [abs(diagnostic.pixels(item,nodes,poses,1)[0]-np.rint(diagnostic.pixels(item,nodes,poses,1)[0])) for item in matches]
        rows.append(dict(source_index=sample['source_index'],role='domain control' if sample['source_index'] == 279 else 'valid-candidate focus',
            node_timestamps=sample['node_timestamps'],archived_applied_corrections=q['applied_iterations'],
            fixed_source_occurrences=sum(len(i['points']) for i in matches),
            population_metadata_sha256=sha(json.dumps(q['frozen_population'],sort_keys=True).encode()),
            original_prior_sha256=sha(json.dumps(prior,sort_keys=True).encode()),
            baseline_cost=kernel.energy(matches,nodes,poses,op_prior,1),directional_derivative=float(e['g']@d.ravel()),
            full_proposal=d.tolist(),snapped_coordinate_occurrences=sum(int((distance <= 1e-10).sum()) for distance in distances),
            minimum_raw_grid_distance_pixels=min(float(distance.min()) for distance in distances),probes=probes))
        for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
        print(case,sample['source_index'],'probes',2*len(probes),'complete; no poses committed',flush=True)
        surfaces = {t:surfaces[t] for t in sample['node_timestamps']}
    require([r['source_index'] for r in rows] == [120,160,240,279],'predeclared diagnostic samples')
    freeze = json.loads((packet/'source-freeze.json').read_text())
    for name,digest in freeze.items():require(sha((ROOT/name).read_bytes()) == digest,'diagnostic frozen source '+name)
    return dict(input_long_summary_sha256=sha(body),case=case,samples=rows,source_sha256=freeze,
        data_role='post-hoc diagnosis of already viewed search failures',local_pose_fitting=False,
        new_trajectory_fitting=False,numeric_ground_truth_parsed=False,physical_accuracy_scored=False,
        tracker_state_commits=False,optimizer_continued=False,default_changed=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analytic-only',action='store_true');parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    report = controls() if args.analytic_only else recorded(ROOT/'assets/depth-search-diagnostic-v1')
    write(out/'summary.json',report)


if __name__ == '__main__':main()
