#!/usr/bin/env python3
"""Read-only independent slope accounting around saved interpolation cells."""
import argparse
import copy
import importlib.util
import io
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

import depth_cell_slopes as diagnostic
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


reference = load('cell_slope_prior_oracles','diagnose-depth-search.py')
verifier,audit,common = reference.verifier,reference.audit,reference.common
kernel = diagnostic.kernel
PARENTS = ('deep-search-depth-refinement-v1','damped-depth-refinement-v1')
SAMPLES = (120,160,240,279)


def independent_measure(matches,nodes,poses,prior,direction,alpha):
    candidate = np.asarray(poses).copy()
    for node,d in enumerate(direction,1):candidate[node] = audit.increment(poses[node],alpha*d)
    flow = np.vstack([np.zeros(6),direction.copy()]);flow[1:,:3] += alpha*np.cross(direction[:,:3],direction[:,3:])
    pixels = [reference.oracle_pixels(item,nodes,candidate,1) for item in matches]
    result = dict(alpha=alpha,cost=None,slope=None,prior_slope=None,prior_cost=None,guard_failure=None,
        uv=[p[0] for p in pixels],cells=[p[1] for p in pixels],terms=None,residuals=None)
    try:
        cost = verifier.objective(matches,nodes,candidate,prior,1,audit)
        g = audit.independent_prior(prior,nodes,candidate)[1]
        prior_slope = float(np.dot(g,flow[1:].reshape(-1)))
        terms = [];residuals = []
        for item in matches:
            r,j = verifier.independent.residual(item,nodes,candidate,1)
            dr = np.einsum('ij,j->i',j,flow[item['source']]-flow[item['target']])
            influence = np.where(abs(r) <= .01,r,.01*np.sign(r))
            residuals.append(r);terms.append(influence*dr)
        result.update(cost=cost,slope=math.fsum([prior_slope]+[float(x) for row in terms for x in row]),prior_slope=prior_slope,
            prior_cost=verifier.objective([],nodes,candidate,prior,1,audit),terms=terms,residuals=residuals)
    except ValueError as error:result['guard_failure'] = str(error)
    return result


def compare_measure(actual,wanted):
    disagreements = 0;maximum = 0.
    for uv0,uv1,cell0,cell1 in zip(actual['uv'],wanted['uv'],actual['cells'],wanted['cells'],strict=True):
        error = abs(uv0-uv1);maximum = max(maximum,float(error.max()))
        require(np.all(error < 3e-11),'independent homogeneous projection agreement')
        different = np.any(cell0 != cell1,axis=1);disagreements += int(different.sum())
        # Cell switches occur at integer-1e-10 because of the frozen snap rule.
        distance0 = abs(uv0-(np.rint(uv0)-1e-10))
        distance1 = abs(uv1-(np.rint(uv1)-1e-10))
        near = np.any(np.minimum(distance0,distance1) <= 3e-11,axis=1)
        require(np.all(~different | near),'cell disagreement only at unresolved numeric switch')
    resolved = disagreements == 0
    if resolved:
        require(actual['guard_failure'] == wanted['guard_failure'],'independent complete sampling domain')
        for key in ('cost','slope','prior_slope','prior_cost'):
            if wanted[key] is None:require(actual[key] is None,'invalid support has no '+key)
            else:np.testing.assert_allclose(actual[key],wanted[key],atol=2e-12,rtol=1e-7,err_msg=key)
    return dict(resolved=resolved,endpoint_cell_disagreements=disagreements,max_pixel_implementation_difference=maximum)


def checked_grid(matches,other_matches,nodes,other,poses,op_prior,prior,direction,scale):
    alphas = diagnostic.multipliers(scale)
    actual = [diagnostic.measure(matches,nodes,poses,op_prior,direction,a) for a in alphas]
    wanted = [independent_measure(other_matches,other,poses,prior,direction,a) for a in alphas]
    baseline = actual[alphas.index(0.)];other_baseline = wanted[alphas.index(0.)]
    probes = []
    for a,b in zip(actual,wanted,strict=True):
        check = compare_measure(a,b);entry = diagnostic.encode(a,baseline)
        entry['oracle'] = diagnostic.encode(b,other_baseline);entry['comparison'] = check
        if check['resolved'] and a['cost'] is not None:
            np.testing.assert_allclose(entry['paired_cost_delta'],entry['oracle']['paired_cost_delta'],atol=2e-12,rtol=1e-7)
        probes.append(entry)
    partitions = []
    for i in range(len(alphas)-1):
        a = diagnostic.partition(matches,actual[i],actual[i+1]);b = diagnostic.partition(other_matches,wanted[i],wanted[i+1])
        for entry,measurements in ((a,actual),(b,wanted)):
            entry['paired_cost_delta'] = diagnostic.paired_delta(measurements[i],measurements[i+1])
            entry['secant'] = None if entry['paired_cost_delta'] is None else entry['paired_cost_delta']/(alphas[i+1]-alphas[i])
        resolved = probes[i]['comparison']['resolved'] and probes[i+1]['comparison']['resolved']
        if resolved:
            require(a['changed_sources'] == b['changed_sources'] and a['endpoint_changed_source_occurrences'] == b['endpoint_changed_source_occurrences'],
                'independent exact source membership of endpoint cell changes')
            for key in ('changed_cell_slope_delta','same_cell_slope_delta','prior_slope_delta','total_slope_delta','paired_cost_delta'):
                if b[key] is None:require(a[key] is None,'invalid transition has no slope partition')
                else:np.testing.assert_allclose(a[key],b[key],atol=4e-12,rtol=1e-6,err_msg=key)
        if a['total_slope_delta'] is not None:
            np.testing.assert_allclose(a['total_slope_delta'],actual[i+1]['slope']-actual[i]['slope'],atol=2e-13,rtol=1e-8)
        a.update(independently_resolved=resolved,oracle=b);partitions.append(a)
    return dict(scale_alpha=scale,probes=probes,partitions=partitions),actual,wanted


def controls():
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);y,x = np.indices((32,32))
    reports = {};fixtures = {}
    for name in ('smooth','knot','domain','rotated path'):
        rho = .5+(.001*abs(x-16) if name == 'knot' else .0002*x)+.0001*y
        layer = dict(camera=c,depth=1/rho,normals=np.tile([0.,0.,1.],(32,32,1)))
        uv = np.array([[16.,12.25]]) if name == 'knot' else np.array([[1.001 if name == 'domain' else 12.25,12.25]])
        points = np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(len(uv),2.)])
        count = 3 if name == 'rotated path' else 2;poses = np.tile(np.eye(4),(count,1,1))
        nodes = [dict(timestamp=1.+i*.03,root=np.eye(4),surface={1:layer}) for i in range(count)]
        matches = [dict(target=1 if count == 3 else 0,source=count-1,points=points,normals=np.tile([0.,0.,1.],(len(points),1)),
            source_ids=np.arange(len(points)),normalizer=np.ones(len(points)))]
        direction = np.array([[-.001 if name != 'domain' else -.01,0,0,0,0,0]])
        if count == 3:direction = np.array([[.003,-.002,.001,.004,-.003,.002],[-.004,.0015,-.001,-.002,.003,-.001]])
        scale = 1/256 if name == 'domain' else 1/64
        report,actual,wanted = checked_grid(matches,matches,nodes,nodes,poses,None,None,direction,scale)
        if name == 'knot':
            zero = next(p for p in report['probes'] if p['alpha'] == 0.)
            require(zero['slope'] < 0 and all(p['slope'] > 0 for p in report['probes'] if p['alpha'] > 0),'authored kink slope reversal')
        elif name == 'domain':require(any(p['guard_failure'] == 'fixed source support' for p in report['probes']),'authored lost-domain probe')
        else:
            alpha = .01;eps = 1e-5
            point = diagnostic.measure(matches,nodes,poses,None,direction,alpha)
            numeric = (diagnostic.measure(matches,nodes,poses,None,direction,alpha+eps)['cost']-
                diagnostic.measure(matches,nodes,poses,None,direction,alpha-eps)['cost'])/(2*eps)
            np.testing.assert_allclose(point['slope'],numeric,atol=2e-12,rtol=2e-5)
        reports[name] = report;fixtures[name] = (actual,wanted)
    rejected = []
    for name,fixture,index,change in (('wrong slope','smooth',9,lambda p:p.update(slope=99.)),
        ('fabricated invalid cost','domain',18,lambda p:p.update(cost=0.)),
        ('wrong cell','smooth',9,lambda p:p['cells'][0].__setitem__((0,0),0))):
        actual,wanted = fixtures[fixture];bad = copy.deepcopy(actual[index]);change(bad)
        try:compare_measure(bad,wanted[index])
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected cell slope corruption: '+name)
    # An injected roundoff-sized crossing is explicitly unresolved, never
    # counted as an independent slope agreement.
    actual,wanted = fixtures['knot'];a,b = copy.deepcopy(actual[9]),copy.deepcopy(wanted[9])
    a['uv'][0][0,0] = 16-1e-10-1e-13;b['uv'][0][0,0] = 16-1e-10+1e-13
    a['cells'][0][0,0] = 15;b['cells'][0][0,0] = 16
    require(not compare_measure(a,b)['resolved'],'numeric switch ambiguity retained')
    return dict(analytic_pass=True,injected_directions_not_guarded_pose_proposals=True,fixtures=reports,
        finite_difference_smooth_and_moved_target_source_pass=True,explicit_unresolved_switch_control=True,rejected_corruptions=rejected)


def recorded(packet):
    inputs = {name:common.archived(ROOT/'assets'/name,'summary.json') for name in PARENTS}
    source = {name:json.loads(body) for name,body in inputs.items()};case = 'tum-fr1-room-temporal'
    require(all(not s['numeric_ground_truth_parsed'] and not s['new_trajectory_fitting'] for s in source.values()),'label-free archived local states')
    saved = source[PARENTS[0]]['recorded'][case]
    parent = ROOT/'assets/reciprocal-keyframe-depth-v1';prefix = 'first/'+case+'-retain/'
    freeze_body = common.archived(parent,prefix+'freeze.json');freeze = json.loads(freeze_body)
    require(sha(freeze_body) == saved['input_freeze_sha256'],'same raw input freeze')
    inventory = json.loads((parent/'files.json').read_text());require(saved['sensor_ledger_sha256'] == inventory[prefix+'sensor-only.json.gz']['sha256'],'same sensor ledger')
    base = ROOT/'data'/case;body = (base/'manifest.json').read_bytes();require(sha(body) == freeze['manifest_sha256'],'original manifest')
    by_stamp = {f['depth_timestamp']:f for f in json.loads(body)['frames']};surfaces = {};rows = []
    def surface(stamp):
        if stamp not in surfaces:
            name = by_stamp[stamp]['depth_file'];expected = freeze['input_sha256'][name];path = base/'raw'/name
            require(path.is_file() and not path.is_symlink() and 0 < path.stat().st_size <= 4*1024*1024,'bounded original PNG')
            raw = path.read_bytes();require(len(raw) == expected['bytes'] and sha(raw) == expected['sha256'],'original PNG digest')
            image = Image.open(io.BytesIO(raw));require(image.size == (640,480) and image.mode in ('I;16','I;16B','I'),'axial depth PNG')
            depth = np.asarray(image,dtype=float)/5000
            surfaces[stamp] = kernel.base.pyramid(depth,freeze['camera']),audit.pyramid(depth,freeze['camera'])
        return surfaces[stamp]
    for sample_index in SAMPLES:
        for name in PARENTS:
            sample = next(s for s in source[name]['recorded'][case]['samples'] if s['source_index'] == sample_index)
            require(source[name]['recorded'][case]['input_freeze_sha256'] == saved['input_freeze_sha256'],'same images for both methods')
            q = sample['refinement'];initial,poses = np.asarray(q['initial_poses']),np.asarray(q['final_poses']);prior = sample['prior_before'];op_prior = common.prior_object(prior)
            nodes = [dict(timestamp=t,root=p.copy(),surface=surface(t)[0]) for t,p in zip(sample['node_timestamps'],initial,strict=True)]
            other = [dict(n,surface=surface(n['timestamp'])[1]) for n in nodes]
            matches = kernel.population(nodes,initial,1);expected = verifier.population(other,initial,1)
            require(kernel.metadata(matches) == q['frozen_population'],'original source identities and normalizers');verifier.check_metadata(q['frozen_population'],expected)
            e = kernel.equation(matches,nodes,poses,op_prior,1);ie = verifier.system(expected,other,poses,prior,1,audit)
            audit.compare_equation(q['final_equation'],e);audit.compare_equation(q['final_equation'],ie)
            direction,_ = kernel.discrete.proposal(e,len(nodes)-1);reason,independent_d,_ = verifier.reference.failure(ie,len(nodes)-1,audit)
            require(reason is None and not verifier.reference.small(independent_d),'original unconverged guarded terminal proposal')
            np.testing.assert_array_equal(direction,q['final_correction']['delta']);np.testing.assert_allclose(direction,independent_d,atol=1e-10,rtol=1e-7)
            last = q['trace'][-1]['line_search'];gn = last if name == PARENTS[0] else last['gauss_newton'];scale = gn['accepted_alpha'] or 2.**-20
            report,_,_ = checked_grid(matches,expected,nodes,other,poses,op_prior,prior,direction,scale)
            zero = next(p for p in report['probes'] if p['alpha'] == 0.)
            np.testing.assert_allclose(zero['cost'],q['final_cost'],atol=1e-15,rtol=0)
            np.testing.assert_allclose(zero['slope'],e['g']@direction.ravel(),atol=2e-13,rtol=1e-8)
            rows.append(dict(parent=name,source_index=sample_index,archived_applied_corrections=q['applied_iterations'],
                population_metadata_sha256=sha(json.dumps(q['frozen_population'],sort_keys=True).encode()),
                prior_sha256=sha(json.dumps(prior,sort_keys=True).encode()),full_proposal=direction.tolist(),
                fixed_source_occurrences=sum(len(i['points']) for i in matches),report=report))
            for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
            print(name,sample_index,'nineteen probes complete; no optimizer continuation',flush=True)
        surfaces = {}
    frozen = json.loads((packet/'source-freeze.json').read_text())
    for name,digest in frozen.items():require(sha((ROOT/name).read_bytes()) == digest,'frozen diagnostic source '+name)
    return dict(data_role='post-hoc read-only cell-transition slope diagnosis',case=case,input_summary_sha256={n:sha(b) for n,b in inputs.items()},
        source_sha256=frozen,anchors=rows,local_pose_fitting=False,optimizer_continued=False,new_trajectory_fitting=False,
        numeric_ground_truth_parsed=False,physical_accuracy_scored=False,tracker_state_commits=False,default_changed=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--analytic-only',action='store_true');parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output)
    write(out/'summary.json',controls() if args.analytic_only else recorded(ROOT/'assets/depth-cell-slopes-v1'))


if __name__ == '__main__':main()
