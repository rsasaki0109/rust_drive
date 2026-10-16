#!/usr/bin/env python3
"""Independent fixed-pair derivatives, spectral updates and fresh-pair rechecks."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import fixed_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


def independent_system(matches,nodes,poses,prior,audit,objective):
    h,g,delta = audit.independent_prior(prior,nodes,poses);edges = [];count = 0;squared = 0.
    consumed = set() if prior is None else set(map(tuple,prior['consumed_pairs']))
    for item in matches:
        a,b = item['target'],item['source'];pt,nt,ps,ns = item['points']
        require((nodes[a]['timestamp'],nodes[b]['timestamp']) not in consumed,'independent fixed ownership')
        relative = np.linalg.solve(poses[a],poses[b]);q = ps@relative[:3,:3].T+relative[:3,3]
        source_normal = ns@relative[:3,:3].T;summed = source_normal+nt
        length = np.sqrt(np.sum(summed*summed,axis=1));normal = summed/length[:,None]
        difference = q-pt;r = np.sum(normal*difference,axis=1)
        rot = poses[a][:3,:3]
        root_points = ps@poses[b][:3,:3].T+poses[b][:3,3];root_normals = ns@poses[b][:3,:3].T
        dq = -np.einsum('ij,njk->nik',rot.T,objective.cross_matrices(root_points))
        dnormal = -np.einsum('ij,njk->nik',rot.T,objective.cross_matrices(root_normals))
        projector = np.eye(3)[None,:,:]-np.einsum('ni,nj->nij',normal,normal)
        dn = np.einsum('nij,njk->nik',projector,dnormal)/length[:,None,None]
        rotational = np.einsum('ni,nij->nj',normal,dq)+np.einsum('ni,nij->nj',difference,dn)
        j = np.concatenate([normal@rot.T,rotational],axis=1)
        weights = np.where(abs(r) <= .01,1.,.01/np.maximum(abs(r),1e-15))
        incidence = np.array([int(i == b)-int(i == a) for i in range(1,len(nodes))])
        matrix = np.kron(incidence[None,:],np.eye(6));hh = (j*weights[:,None]).T@j;gg = j.T@(weights*r)
        h += matrix.T@hh@matrix;g += matrix.T@gg
        count += len(r);squared += float(np.dot(r,r))
        edges.append(dict(target=a,source=b,count=len(r),rms_m=float(np.sqrt(np.mean(r*r)))))
    return dict(h=h,g=g,edges=edges,count=count,rms_m=float(np.sqrt(squared/count)) if count else 0.,prior_delta=delta)


def failure(e,current,audit):
    guard = load('fixed_independent_guards','check-selective-keyframes.py')
    return guard.failure(audit,e,current)


def small(d):
    return bool(np.all(np.linalg.norm(d[:,:3],axis=1) < .00001) and np.all(np.linalg.norm(d[:,3:],axis=1) < .00001))


def expected_search(matches,nodes,poses,prior,d,g,audit,objective):
    before = objective.objective(matches,nodes,poses,prior,audit);directional = float(g@d.ravel())
    require(directional <= 0 and np.isfinite(directional),'independent descent direction')
    trials = []
    for exponent in range(8):
        alpha = 1/(2**exponent);candidate = np.asarray(poses).copy()
        for node,delta in enumerate(d,1):candidate[node] = audit.increment(poses[node],delta*alpha)
        cost = objective.objective(matches,nodes,candidate,prior,audit)
        limit = before+.0001*alpha*directional+1e-12*max(1,abs(before))
        trials.append(dict(alpha=alpha,cost=cost,limit=limit))
        if cost <= limit:return candidate,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=alpha)
    return None,None


def compare_search(actual,expected):
    require(actual['accepted_alpha'] == expected['accepted_alpha'] and len(actual['trials']) == len(expected['trials']),'fixed first passing multiplier')
    for key in ('before_cost','directional_derivative'):
        np.testing.assert_allclose(actual[key],expected[key],atol=1e-10,rtol=1e-8)
    for a,b in zip(actual['trials'],expected['trials'],strict=True):
        require(a['alpha'] == b['alpha'],'fixed candidate halving')
        np.testing.assert_allclose([a['cost'],a['limit']],[b['cost'],b['limit']],atol=1e-10,rtol=1e-8)


def check_correction(actual,e,current,audit):
    reason,d,condition = failure(e,current,audit)
    require(actual['guard_failure'] == reason and actual['converged'] == (reason is None and small(d)),'honest independently checked correction')
    if reason is not None:
        require(actual['delta'] is None and actual['condition'] is None,'inadmissible correction remains unpublished')
    else:
        np.testing.assert_allclose(actual['delta'],d,atol=1e-9,rtol=1e-7)
        np.testing.assert_allclose(actual['condition'],condition,atol=1e-6,rtol=1e-8)


def verify(report,nodes,initial,prior,audit):
    objective = load('fixed_independent_objective','check-descent-keyframes.py')
    require(report['scale'] in (4,2,1) and report['cap'] == 10,'fixed refinement scale and ten-correction bound')
    poses = np.asarray(initial).copy();np.testing.assert_array_equal(report['initial_poses'],poses)
    matches = objective.selection(nodes,poses,report['scale'])
    metadata = [{k:x[k] for k in ('target','source','count','pixel_sha256')} for x in matches]
    require(report['frozen_population'] == metadata,'single initial measured population')
    require(len(report['trace']) <= 10 and report['applied_iterations'] == len(report['trace']),'bounded honest correction count')
    np.testing.assert_allclose(report['initial_fixed_cost'],objective.objective(matches,nodes,poses,prior,audit),atol=1e-10,rtol=1e-9)
    checked = 0
    for item in report['trace']:
        np.testing.assert_array_equal(item['before'],poses)
        e = independent_system(matches,nodes,poses,prior,audit,objective);audit.compare_equation(item['equation'],e)
        reason,d,condition = failure(e,len(nodes)-1,audit)
        require(reason is None and not small(d),'no correction after failure or fixed convergence')
        np.testing.assert_allclose(item['delta'],d,atol=1e-9,rtol=1e-7)
        np.testing.assert_allclose(item['condition'],condition,atol=1e-6,rtol=1e-8)
        expected,receipt = expected_search(matches,nodes,poses,prior,d,e['g'],audit,objective)
        require(expected is not None,'applied fixed correction has a valid multiplier')
        compare_search(item['line_search'],receipt)
        np.testing.assert_allclose(item['after'],expected,atol=1e-10,rtol=0)
        np.testing.assert_array_equal(item['after'][0],np.eye(4));poses = np.asarray(item['after']).copy();checked += 1
    fixed = independent_system(matches,nodes,poses,prior,audit,objective)
    fresh = audit.independent_equation(nodes,poses,report['scale'],prior)
    audit.compare_equation(report['final_fixed_equation'],fixed);audit.compare_equation(report['final_fresh_equation'],fresh)
    reason,d,_ = failure(fixed,len(nodes)-1,audit);converged = reason is None and small(d)
    if report['guard_failure'] == 'line search':
        require(reason is None and not converged,'search failure follows admissible unconverged solve')
        after,_ = expected_search(matches,nodes,poses,prior,d,fixed['g'],audit,objective)
        require(after is None,'independently reproduce exhausted frozen search')
    else:
        require(report['guard_failure'] == reason,'independently reproduced frozen termination')
        require(reason is not None or converged or len(report['trace']) == 10,'no unexplained early frozen stop')
    require(report['fixed_converged'] == converged,'no false fixed convergence')
    check_correction(report['final_fixed_correction'],fixed,len(nodes)-1,audit)
    check_correction(report['final_fresh_correction'],fresh,len(nodes)-1,audit)
    np.testing.assert_array_equal(report['final_poses'],poses)
    np.testing.assert_allclose(report['final_fixed_cost'],objective.objective(matches,nodes,poses,prior,audit),atol=1e-10,rtol=1e-9)
    new_matches = objective.selection(nodes,poses,report['scale'])
    np.testing.assert_allclose(report['final_fresh_cost'],objective.objective(new_matches,nodes,poses,prior,audit),atol=1e-10,rtol=1e-9)
    return checked+2


def controls():
    audit = load('fixed_refinement_audit','check-reciprocal-keyframes.py').auditor()
    camera = audit.rays.CAMERA;identity = np.eye(4);depth = audit.rays.render(identity)
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);u = tracker.step(1.03,source)
    require(u['accepted'],'measured operational warm start')
    poses = np.asarray(u['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(z,camera)) for t,p,z in zip((1.,1.03),poses,(depth,source))]
    independent = [dict(n,surface=audit.pyramid(z,camera)) for n,z in zip(nodes,(depth,source))]
    report = kernel.refine(nodes,poses,None);checked = verify(report,independent,poses,None,audit)
    error = float(np.linalg.norm(np.asarray(report['final_poses'])[-1,:3,3]-truth[:3,3]))
    require(error < .004 and report['fixed_converged'],'rendered fixed local convergence and 4mm accuracy')
    # Perturb only the measured warm start to exercise multiple relinearizations.
    shifted = poses.copy();shifted[1] = audit.increment(shifted[1],np.array([.02,-.01,.005,.012,-.014,.009]))
    moved = kernel.refine(nodes,shifted,None);checked += verify(moved,independent,shifted,None,audit)
    require(len(moved['trace']) >= 2,'multiple frozen relinearizations exercised')
    # The full objective gradient is checked after movement, at the original
    # frozen population and with a nonzero transported prior on a retained node.
    third = dict(timestamp=1.06,root=poses[1].copy(),surface=nodes[1]['surface'])
    raw3 = nodes+[third];ind3 = independent+[dict(third,surface=independent[1]['surface'])]
    p3 = np.array([identity,shifted[1],poses[1]])
    prior = kernel.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    objective = load('fixed_gradient_objective','check-descent-keyframes.py')
    matches = kernel.population(raw3,p3,1);ind_matches = objective.selection(ind3,p3,1)
    current = p3.copy();current[1] = audit.increment(p3[1],np.array([.001,0,0,0,.001,0]))
    e = kernel.equation(matches,raw3,current,prior)
    audit.compare_equation(kernel.base.encode_equation(e),independent_system(ind_matches,ind3,current,prior.encode(),audit,objective))
    numeric = []
    for node in (1,2):
        for axis in range(6):
            delta = np.zeros(6);delta[axis] = 1e-6;plus,minus = current.copy(),current.copy()
            plus[node] = audit.increment(current[node],delta);minus[node] = audit.increment(current[node],-delta)
            numeric.append((objective.objective(ind_matches,ind3,plus,prior.encode(),audit)-objective.objective(ind_matches,ind3,minus,prior.encode(),audit))/2e-6)
    np.testing.assert_allclose(numeric,e['g'],atol=3e-7,rtol=2e-6)
    guarded = {}
    for name,z in [('missing support',np.zeros_like(depth)),('plane',np.full_like(depth,2.))]:
        surfaces = [dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)]
        oracle = [dict(n,surface=audit.pyramid(z,camera)) for n in surfaces]
        initial = np.array([identity,identity]);bad = kernel.refine(surfaces,initial,None)
        checked += verify(bad,oracle,initial,None,audit)
        require(not bad['trace'] and not bad['fixed_converged'] and bad['guard_failure'] is not None,'degenerate fixed data never fabricate support')
        guarded[name] = bad['guard_failure']
    rejected = []
    changes = [('changed frozen population',lambda r:r['frozen_population'][0].update(pixel_sha256='0'*64)),
        ('wrong inner system',lambda r:r['trace'][0]['equation'].update(h=(np.asarray(r['trace'][0]['equation']['h'])*2).tolist())),
        ('wrong applied multiplier',lambda r:r['trace'][0]['line_search'].update(accepted_alpha=.5)),
        ('wrong applied pose',lambda r:r['trace'][0]['after'][1][0].__setitem__(3,.1)),
        ('false fixed convergence',lambda r:r.update(fixed_converged=not r['fixed_converged'])),
        ('false fresh convergence',lambda r:r['final_fresh_correction'].update(converged=not r['final_fresh_correction']['converged'])),
        ('wrong fresh system',lambda r:r['final_fresh_equation'].update(g=(np.asarray(r['final_fresh_equation']['g'])+1).tolist())),
        ('wrong objective',lambda r:r.update(final_fixed_cost=r['final_fixed_cost']+1)),
        ('extra correction',lambda r:r.update(applied_iterations=r['applied_iterations']+1))]
    for name,change in changes:
        bad = copy.deepcopy(moved);change(bad)
        try:verify(bad,independent,shifted,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected fixed-refinement corruption: '+name)
    for node,p in zip(nodes,poses):np.testing.assert_array_equal(node['root'],p)
    return dict(analytic_pass=True,checked_systems=checked,gradient_axes=12,gradient_includes_prior=True,
        warm_start_position_error_m=error,perturbed_start_iterations=len(moved['trace']),guards=guarded,
        rejected_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in
        ('fixed_depth_refinement.py','check-fixed-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
