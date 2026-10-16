#!/usr/bin/env python3
"""Independent polynomial sampling, fixed normalizers, matrix and search checks."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import continuous_depth_refinement as kernel
from depth_correspondence_oracle import independent_matches
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


reference = load('continuous_fixed_audit','check-fixed-depth-refinement.py')
interpolation = load('continuous_polynomial_oracle','check-subpixel-keyframes.py')


def values(nodes,poses,a,b,points,normals,scale):
    relative = np.linalg.solve(poses[a],poses[b])
    q = np.column_stack([points,np.ones(len(points))])@relative.T
    ids,f,df,point = interpolation.independent_sample(nodes[a]['surface'][scale],q[:,:3])
    size = np.sqrt(np.einsum('ij,ij->i',df,df))
    require(np.isfinite(size).all() and np.all(size > 1e-12),'finite continuous gradient')
    source_normal = normals@relative[:3,:3].T
    ok = np.sqrt(np.einsum('ij,ij->i',q[ids,:3]-point,q[ids,:3]-point)) < (.12 if scale == 4 else .08)
    ok &= np.einsum('ij,ij->i',source_normal[ids],df/size[:,None]) > .8
    return ids[ok],f[ok],df[ok],size[ok]


def population(nodes,poses,scale):
    rows = []
    for b in range(1,len(nodes)):
        for a in range(b):
            pair = independent_matches([dict(nodes[a],root=poses[a])],nodes[b],poses[b],scale,False)
            if pair['count'] < 200:continue
            points,normals = pair['points'][2:];ids,_,_,size = values(nodes,poses,a,b,points,normals,scale)
            if len(ids) >= 200:rows.append(dict(target=a,source=b,points=points[ids],normals=normals[ids],
                source_ids=pair['pixel_ids'][ids,1],normalizer=size))
    return rows


def check_metadata(actual,matches):
    require(len(actual) == len(matches),'complete fixed source population')
    for item,wanted in zip(actual,matches,strict=True):
        require(set(item) == {'target','source','count','source_pixel_sha256','normalizer','normalizer_sha256'},'fixed source schema')
        require(item['target'] == wanted['target'] and item['source'] == wanted['source'] and item['count'] == len(wanted['source_ids']),'fixed source endpoints and count')
        require(item['source_pixel_sha256'] == sha(np.asarray(wanted['source_ids'],dtype='<i8').tobytes()),'independent source pixel identities')
        reported = np.asarray(item['normalizer'],dtype='<f8')
        require(item['normalizer_sha256'] == sha(reported.tobytes()),'honest fixed normalizer digest')
        np.testing.assert_allclose(reported,wanted['normalizer'],atol=1e-11,rtol=1e-9)


def residual(item,nodes,poses,scale):
    a,b = item['target'],item['source']
    ids,f,df,_ = values(nodes,poses,a,b,item['points'],item['normals'],scale)
    require(np.array_equal(ids,np.arange(len(item['source_ids']))),'fixed source support')
    r = f/item['normalizer']
    root = item['points']@poses[b][:3,:3].T+poses[b][:3,3]
    rotation = poses[a][:3,:3]
    differential = -np.einsum('ij,njk->nik',rotation.T,reference.load('continuous_cross','check-descent-keyframes.py').cross_matrices(root))
    df = df/item['normalizer'][:,None]
    j = np.concatenate([df@rotation.T,np.einsum('ni,nij->nj',df,differential)],axis=1)
    return r,j


def system(matches,nodes,poses,prior,scale,audit):
    h,g,delta = audit.independent_prior(prior,nodes,poses);edges = [];count = 0;ss = 0.
    consumed = set() if prior is None else set(map(tuple,prior['consumed_pairs']))
    for item in matches:
        a,b = item['target'],item['source']
        require((nodes[a]['timestamp'],nodes[b]['timestamp']) not in consumed,'independent continuous ownership')
        r,j = residual(item,nodes,poses,scale)
        w = np.where(abs(r) <= .01,1.,.01/np.maximum(abs(r),1e-15))
        incidence = np.array([int(i == b)-int(i == a) for i in range(1,len(nodes))])
        matrix = np.kron(incidence[None,:],np.eye(6))
        h += matrix.T@((j*w[:,None]).T@j)@matrix;g += matrix.T@(j.T@(w*r))
        count += len(r);ss += float(np.dot(r,r))
        edges.append(dict(target=a,source=b,count=len(r),rms_m=float(np.sqrt(np.mean(r*r)))))
    return dict(h=h,g=g,edges=edges,count=count,rms_m=float(np.sqrt(ss/count)) if count else 0.,prior_delta=delta)


def objective(matches,nodes,poses,prior,scale,audit):
    cost = reference.load('continuous_prior_value','depth_correspondence_oracle.py').prior_value(prior,nodes,poses,audit)
    for item in matches:
        r,_ = residual(item,nodes,poses,scale);quadratic = abs(r) <= .01
        cost += float(np.sum(r[quadratic]**2)/2+np.sum(.01*abs(r[~quadratic])-.00005))
    return cost


def search(matches,nodes,poses,prior,scale,d,g,audit):
    before = objective(matches,nodes,poses,prior,scale,audit);directional = float(g@d.ravel())
    require(np.isfinite(directional) and directional <= 0,'independent continuous descent')
    trials = []
    for exponent in range(8):
        alpha = 1/(2**exponent);candidate = np.asarray(poses).copy()
        for node,delta in enumerate(d,1):candidate[node] = audit.increment(poses[node],alpha*delta)
        limit = before+.0001*alpha*directional+1e-12*max(1.,abs(before))
        try:cost = objective(matches,nodes,candidate,prior,scale,audit);reason = None
        except ValueError as error:cost = None;reason = str(error)
        trials.append(dict(alpha=alpha,cost=cost,limit=limit,guard_failure=reason))
        if reason is None and cost <= limit:return candidate,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=alpha)
    return None,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=None)


def compare_search(actual,wanted):
    require(set(actual) == set(wanted) and actual['accepted_alpha'] == wanted['accepted_alpha'],'continuous first passing multiplier')
    require(len(actual['trials']) == len(wanted['trials']),'all continuous trial receipts')
    for key in ('before_cost','directional_derivative'):np.testing.assert_allclose(actual[key],wanted[key],atol=1e-10,rtol=1e-8)
    for item,expected in zip(actual['trials'],wanted['trials'],strict=True):
        require(set(item) == set(expected) and item['alpha'] == expected['alpha'] and item['guard_failure'] == expected['guard_failure'],'honest candidate validity')
        np.testing.assert_allclose(item['limit'],expected['limit'],atol=1e-10,rtol=1e-8)
        if expected['cost'] is None:require(item['cost'] is None,'invalid candidate cannot lower cost by dropping points')
        else:np.testing.assert_allclose(item['cost'],expected['cost'],atol=1e-10,rtol=1e-8)


def verify(report,nodes,initial,prior,audit):
    require(report['cap'] == 10 and report['scale'] in (4,2,1),'unchanged correction limit')
    poses = np.asarray(initial).copy();np.testing.assert_array_equal(report['initial_poses'],poses)
    scale = report['scale'];matches = population(nodes,poses,scale);check_metadata(report['frozen_population'],matches)
    np.testing.assert_allclose(report['initial_cost'],objective(matches,nodes,poses,prior,scale,audit),atol=1e-10,rtol=1e-8)
    require(report['applied_iterations'] == len(report['trace']) <= 10,'honest bounded continuous corrections')
    checked = 0
    for item in report['trace']:
        np.testing.assert_array_equal(item['before'],poses)
        e = system(matches,nodes,poses,prior,scale,audit);audit.compare_equation(item['equation'],e);checked += 1
        reason,d,condition = reference.failure(e,len(nodes)-1,audit)
        require(reason is None and not reference.small(d),'no update after guard or convergence')
        np.testing.assert_allclose(item['delta'],d,atol=1e-9,rtol=1e-7)
        np.testing.assert_allclose(item['condition'],condition,atol=1e-6,rtol=1e-8)
        expected,receipt = search(matches,nodes,poses,prior,scale,d,e['g'],audit)
        require(expected is not None,'applied continuous update has valid Armijo step');compare_search(item['line_search'],receipt)
        np.testing.assert_allclose(item['after'],expected,atol=1e-10,rtol=0)
        np.testing.assert_array_equal(item['after'][0],np.eye(4));poses = np.asarray(item['after'])
    final = system(matches,nodes,poses,prior,scale,audit);audit.compare_equation(report['final_equation'],final)
    discrete = audit.independent_equation(nodes,poses,scale,prior);audit.compare_equation(report['final_discrete_equation'],discrete)
    reason,d,_ = reference.failure(final,len(nodes)-1,audit);converged = reason is None and reference.small(d)
    if report['guard_failure'] == 'line search':
        require(reason is None and not converged,'failed search follows admissible unconverged solve')
        expected,receipt = search(matches,nodes,poses,prior,scale,d,final['g'],audit)
        require(expected is None,'independently reproduced continuous search exhaustion');compare_search(report['failed_search'],receipt)
    else:
        require(report['guard_failure'] == reason and report['failed_search'] is None,'independent continuous stop guard')
        require(reason is not None or converged or len(report['trace']) == 10,'no unexplained early continuous stop')
    require(report['continuous_converged'] == converged,'no false continuous convergence')
    termination = 'guard' if report['guard_failure'] is not None else ('converged' if converged else 'budget')
    require(report['termination'] == termination,'honest continuous termination')
    reference.check_correction(report['final_correction'],final,len(nodes)-1,audit)
    reference.check_correction(report['final_discrete_correction'],discrete,len(nodes)-1,audit)
    np.testing.assert_array_equal(report['final_poses'],poses)
    np.testing.assert_allclose(report['final_cost'],objective(matches,nodes,poses,prior,scale,audit),atol=1e-10,rtol=1e-8)
    return checked+2


def controls():
    interpolation_controls = interpolation.interpolation_controls()
    audit = load('continuous_rendered_audit','check-reciprocal-keyframes.py').auditor()
    identity = np.eye(4);camera = audit.rays.CAMERA;depth = audit.rays.render(identity)
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered warm start')
    initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(z,camera)) for t,p,z in zip((1.,1.03),initial,(depth,source),strict=True)]
    independent = [dict(n,surface=audit.pyramid(z,camera)) for n,z in zip(nodes,(depth,source),strict=True)]
    moved = kernel.refine(nodes,initial,None);checked = verify(moved,independent,initial,None,audit)
    estimate = np.asarray(moved['final_poses'])[-1]
    position = float(np.linalg.norm(estimate[:3,3]-truth[:3,3]))
    rotation = float(np.arccos(np.clip((np.trace(truth[:3,:3].T@estimate[:3,:3])-1)/2,-1,1)))
    require(position <= .004 and rotation <= .004,'rendered continuous local pose remains within 4mm/4mrad')
    shifted = initial.copy();shifted[1] = audit.increment(shifted[1],np.array([.02,-.01,.005,.012,-.014,.009]))
    perturbed = kernel.refine(nodes,shifted,None);checked += verify(perturbed,independent,shifted,None,audit)
    measured_guards = {}
    for name,z in [('identity',depth),('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))]:
        raw = [dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)]
        control = [dict(n,surface=audit.pyramid(z,camera)) for n in raw];poses = np.array([identity,identity])
        result = kernel.refine(raw,poses,None);checked += verify(result,control,poses,None,audit)
        require(not result['trace'],'identity/degenerate images apply no corrections')
        if name == 'identity':require(result['continuous_converged'] and result['guard_failure'] is None,'identity continuous convergence')
        else:require(result['termination'] == 'guard' and not result['continuous_converged'],'degenerate images cannot fabricate information')
        measured_guards[name] = result['guard_failure']
    # A nonzero marginal prior and moved target/source exercise all 12 axes.
    third = dict(timestamp=1.06,root=initial[1].copy(),surface=nodes[1]['surface'])
    raw3 = nodes+[third];ind3 = independent+[dict(third,surface=independent[1]['surface'])]
    poses3 = np.array([identity,initial[1],initial[1]])
    poses3[1] = audit.increment(poses3[1],np.array([.00073,-.00031,.00021,.00012,-.00014,.00009]))
    poses3[2] = audit.increment(poses3[2],np.array([-.00041,.00023,-.00011,-.00011,.00008,.00017]))
    prior = kernel.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    matches = kernel.population(raw3,poses3,1);expected_matches = population(ind3,poses3,1)
    e = kernel.equation(matches,raw3,poses3,prior,1)
    audit.compare_equation(kernel.base.encode_equation(e),system(expected_matches,ind3,poses3,prior.encode(),1,audit))
    numeric = []
    for node in (1,2):
        for axis in range(6):
            d = np.zeros(6);d[axis] = 1e-7;plus,minus = poses3.copy(),poses3.copy()
            plus[node] = audit.increment(poses3[node],d);minus[node] = audit.increment(poses3[node],-d)
            numeric.append((objective(expected_matches,ind3,plus,prior.encode(),1,audit)-objective(expected_matches,ind3,minus,prior.encode(),1,audit))/2e-7)
    np.testing.assert_allclose(numeric,e['g'],atol=3e-7,rtol=2e-6)
    # Validity cannot silently change the objective's source population.
    matches = kernel.population(nodes,initial,1);require(bool(matches),'rendered continuous measured support')
    outside = initial.copy();outside[1,0,3] += 1.
    for callback in (lambda:kernel.evaluate(matches[0],nodes,outside,1),
                     lambda:residual(expected_matches[0],ind3,np.array([identity,outside[1],poses3[2]]),1)):
        try:callback()
        except ValueError as error:require(str(error) == 'fixed source support','reproduce source-domain failure')
        else:raise ValueError('invalid fixed sources silently dropped')
    raw_identity = [dict(timestamp=t,root=identity.copy(),surface=nodes[0]['surface']) for t in (1.,1.03)]
    ind_identity = [dict(n,surface=independent[0]['surface']) for n in raw_identity];same_poses = np.array([identity,identity])
    fixed = kernel.population(raw_identity,same_poses,1);ind_fixed = population(ind_identity,same_poses,1)
    # Injected direction/gradient isolate the invalid-candidate receipt branch;
    # they are not evidence of an image-derived normal-system proposal.
    direction = np.array([[.09,0.,0.,0.,0.,0.]])
    candidate,receipt = kernel.search(fixed,raw_identity,same_poses,None,1,direction,-direction.ravel())
    expected,expected_receipt = search(ind_fixed,ind_identity,same_poses,None,1,direction,-direction.ravel(),audit)
    require(candidate is None and expected is None and len(receipt['trials']) == 8,'isolated exhausted search')
    require(any(t['guard_failure'] == 'fixed source support' for t in receipt['trials']),'invalid trial preserves fixed domain')
    compare_search(receipt,expected_receipt)
    rejected = []
    mutations = [('changed source hash',lambda r:r['frozen_population'][0].update(source_pixel_sha256='0'*64)),
        ('changed normalizer digest',lambda r:r['frozen_population'][0].update(normalizer_sha256='0'*64)),
        ('changed normalizer value',lambda r:r['frozen_population'][0]['normalizer'].__setitem__(0,2.)),
        ('wrong initial cost',lambda r:r.update(initial_cost=r['initial_cost']+1.)),
        ('false continuous convergence',lambda r:r.update(continuous_converged=not r['continuous_converged'])),
        ('false discrete convergence',lambda r:r['final_discrete_correction'].update(converged=not r['final_discrete_correction']['converged'])),
        ('wrong final matrix',lambda r:r['final_equation'].update(g=(np.asarray(r['final_equation']['g'])+1).tolist())),
        ('wrong stop cause',lambda r:r.update(termination='budget' if r['termination'] != 'budget' else 'converged'))]
    if moved['trace']:
        mutations += [('wrong applied multiplier',lambda r:r['trace'][0]['line_search'].update(accepted_alpha=.003)),
            ('wrong applied pose',lambda r:r['trace'][0]['after'][1][0].__setitem__(3,.1))]
    for name,change in mutations:
        bad = copy.deepcopy(moved);change(bad)
        try:verify(bad,independent,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected continuous refinement corruption: '+name)
    bad = copy.deepcopy(moved);bad['frozen_population'][0]['normalizer'][0] *= 1.1
    bad['frozen_population'][0]['normalizer_sha256'] = sha(np.asarray(bad['frozen_population'][0]['normalizer'],dtype='<f8').tobytes())
    try:verify(bad,independent,initial,None,audit)
    except (ValueError,AssertionError):rejected.append('coherently changed fixed normalizer')
    else:raise ValueError('coherently changed fixed metric accepted')
    bad = copy.deepcopy(receipt);bad['trials'][0].update(cost=0.,guard_failure=None)
    try:compare_search(bad,expected_receipt)
    except (ValueError,AssertionError):rejected.append('invalid candidate presented as zero cost')
    else:raise ValueError('invalid trial accepted as an improvement')
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    return dict(analytic_pass=True,independently_checked_reported_systems=checked,additional_prior_system_checks=1,
        objective_gradient_axes=12,interpolation=interpolation_controls,source_domain_failure_reproduced=True,
        rendered_continuous_converged=moved['continuous_converged'],rendered_discrete_fresh_converged=moved['final_discrete_correction']['converged'],
        rendered_applied_corrections=moved['applied_iterations'],rendered_termination=moved['termination'],rendered_guard_failure=moved['guard_failure'],
        rendered_position_error_m=position,rendered_rotation_error_rad=rotation,measured_guards=measured_guards,
        perturbed_rendered_continuous_converged=perturbed['continuous_converged'],perturbed_rendered_applied_corrections=perturbed['applied_iterations'],
        perturbed_rendered_termination=perturbed['termination'],perturbed_rendered_guard_failure=perturbed['guard_failure'],
        isolated_injected_direction_invalid_trials=sum(t['guard_failure'] is not None for t in receipt['trials']),
        isolated_injected_direction_search_exhausted=True,
        rejected_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+name:sha((ROOT/'scripts'/name).read_bytes()) for name in
        ('continuous_depth_refinement.py','check-continuous-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
