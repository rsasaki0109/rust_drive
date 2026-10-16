#!/usr/bin/env python3
"""Independent full-round audit of bounded continuous-source reselection."""
import argparse
import copy
import importlib.util
from pathlib import Path

import numpy as np

import reselected_slope_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


reference = load('reselected_fixed_audit','check-slope-admission-refinement.py')


def transition(before,after):
    def flatten(rows):
        return {(r['target'],r['source'],int(s)):(r['points'][i],r['normals'][i],r['normalizer'][i])
            for r in rows for i,s in enumerate(r['source_ids'])}
    a,b = flatten(before),flatten(after);result = []
    for target,source in sorted({key[:2] for key in a.keys()|b.keys()}):
        x = {key for key in a if key[:2]==(target,source)};y = {key for key in b if key[:2]==(target,source)};common = sorted(x&y)
        ratios = []
        for key in common:
            for i in (0,1):np.testing.assert_array_equal(a[key][i],b[key][i])
            ratios.append(b[key][2]/a[key][2])
        ids = np.array([key[2] for key in common],dtype='<i8')
        result.append(dict(target=target,source=source,old_count=len(x),new_count=len(y),shared_count=len(common),
            removed_count=len(x-y),added_count=len(y-x),shared_source_sha256=sha(ids.tobytes()),
            changed_normalizers_on_shared=sum(a[key][2]!=b[key][2] for key in common),
            normalizer_ratio_min=None if not ratios else float(min(ratios)),
            normalizer_ratio_median=None if not ratios else float(np.quantile(ratios,.5)),
            normalizer_ratio_max=None if not ratios else float(max(ratios)),source_values_preserved=True))
    return result


def compare_transition(actual,wanted):
    require(len(actual)==len(wanted),'all between-round factor transitions')
    for row,expected in zip(actual,wanted,strict=True):
        require(set(row)==set(expected),'transition schema')
        for key,value in expected.items():
            if key.startswith('normalizer_ratio_') and value is not None:np.testing.assert_allclose(row[key],value,atol=1e-10,rtol=1e-9)
            else:require(row[key]==value,'honest transition '+key)


def outcome(inner_guard,inner_converged,fresh_guard,fresh_converged,round_number):
    # Derive scheduling independently from the reconstructed inner and fresh
    # systems; never call the operational scheduler.
    if inner_guard is not None:return 'inner_guard',inner_guard
    if not inner_converged:return 'inner_budget',None
    if fresh_guard is not None:return 'selection_guard',fresh_guard
    if fresh_converged:return 'converged',None
    return ('round_budget',None) if round_number==3 else (None,None)


def verify(report,nodes,initial,prior,audit):
    require(report['max_rounds']==3 and report['inner_cap']==30 and report['max_corrections']==90,'fixed bounded outer/inner work')
    poses = np.asarray(initial).copy();np.testing.assert_array_equal(report['initial_poses'],poses)
    require(report['association_rounds']==len(report['rounds']) and 1<=len(report['rounds'])<=3,'honest association rounds')
    checked = updates = 0;termination = failure = None;scale = report['scale']
    for i,row in enumerate(report['rounds'],1):
        require(termination is None and row['number']==i,'no round after a terminal state')
        batch = row['batch'];require(batch['cap']==30 and batch['scale']==scale,'unchanged inner solver bound and scale')
        checked += reference.verify(batch,nodes,poses,prior,audit)
        before = reference.population(nodes,poses,scale);poses = np.asarray(batch['final_poses'])
        after = reference.population(nodes,poses,scale);selection = row['reselected']
        reference.check_metadata(selection['population'],after)
        e = reference.system(after,nodes,poses,prior,scale,audit);checked += 1
        audit.compare_equation(selection['equation'],e)
        reference.reference.check_correction(selection['correction'],e,len(nodes)-1,audit)
        np.testing.assert_allclose(selection['cost'],reference.objective(after,nodes,poses,prior,scale,audit),atol=1e-10,rtol=1e-8)
        compare_transition(row['transition'],transition(before,after))
        reason,d,_ = reference.reference.failure(e,len(nodes)-1,audit)
        fresh_converged = reason is None and reference.reference.small(d)
        termination,failure = outcome(batch['guard_failure'],batch['continuous_converged'],reason,fresh_converged,i)
        updates += len(batch['trace'])
    require(termination is not None and (termination,failure)==(report['termination'],report['guard_failure']),'independently reconstructed outer stop')
    require(report['applied_iterations']==updates<=90,'honest total correction count')
    require(report['selection_converged']==(termination=='converged'),'both fixed and freshly selected convergence required')
    require(report['final_fixed_converged']==batch['continuous_converged'],'honest terminal fixed convergence')
    require(report['final_reselected_correction']==selection['correction'],'terminal fresh proposal retained')
    require(report['final_original_discrete_equation']==batch['final_discrete_equation'] and
        report['final_original_discrete_correction']==batch['final_discrete_correction'],'original-discrete check never replaced')
    np.testing.assert_array_equal(report['final_poses'],poses)
    return checked


def scheduler_controls():
    cases = [(None,True,None,True,1),('line search',False,None,True,1),(None,False,None,True,1),
        (None,True,'new frame needs measured support',False,1),(None,True,None,False,1),
        (None,True,None,False,3),(None,True,None,True,3)]
    for guard,converged,fresh_guard,fresh_converged,number in cases:
        actual = kernel.stop(dict(guard_failure=guard,continuous_converged=converged),dict(guard_failure=fresh_guard,converged=fresh_converged),number)
        require(actual==outcome(guard,converged,fresh_guard,fresh_converged,number),'scheduler stop precedence')
    for invalid in (0,4,True):
        try:kernel.stop(dict(guard_failure=None,continuous_converged=True),dict(guard_failure=None,converged=True),invalid)
        except ValueError:pass
        else:raise ValueError('unbounded round accepted')
    return len(cases)


def identity_boundary_controls(audit,camera,depth):
    """Preserve both answers at exact support-cell boundaries, without repair."""
    identity = np.eye(4);poses = np.array([identity,identity]);result = {}
    for name,z in (('unquantized',depth),('quantized_2mm',np.round(depth/.002)*.002)):
        raw = kernel.fixed.prepare([dict(timestamp=t,root=identity.copy(),surface=kernel.fixed.base.pyramid(z,camera)) for t in (1.,1.03)])
        other = reference.prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw]);reference.compare_prepared(raw,other)
        actual = kernel.fixed.population(raw,poses,1);expected = reference.population(other,poses,1)
        a = {(r['target'],r['source'],int(s)) for r in actual for s in r['source_ids']}
        b = {(r['target'],r['source'],int(s)) for r in expected for s in r['source_ids']}
        require(a!=b,'authored exact-boundary selection disagreement remains visible')
        try:reference.check_metadata(kernel.fixed.metadata(actual),expected)
        except (ValueError,AssertionError):pass
        else:raise ValueError('boundary source disagreement silently accepted')
        coordinates = []
        for target,source,pixel in sorted(a^b):
            layer = raw[source]['surface'][1];point = layer['vertices'].reshape(-1,3)[pixel];c = raw[target]['surface'][1]['camera']
            coordinates.append([c['fx']*point[0]/point[2]+c['cx'],c['fy']*point[1]/point[2]+c['cy']])
        coordinates = np.asarray(coordinates);distance = float(np.max(abs(coordinates-np.rint(coordinates))))
        require(distance<=3e-11,'unresolved control is at exact initial-probe grid boundaries')
        result[name] = dict(operational_count=len(a),independent_count=len(b),
            only_operational=[list(k) for k in sorted(a-b)],only_independent=[list(k) for k in sorted(b-a)],
            max_source_projection_grid_distance_pixels=distance,auditor_rejected_disagreement=True,resolved=False)
    return result


def controls():
    inherited = reference.controls();scheduling = scheduler_controls()
    audit = load('reselected_rendered','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA
    depth = np.round(audit.rays.render(identity)/.002)*.002
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = np.round(audit.rays.render(truth)/.002)*.002
    tracker = kernel.fixed.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered warm start');initial = np.asarray(warm['revised_roots'])
    nodes = kernel.fixed.prepare([dict(timestamp=t,root=p.copy(),surface=kernel.fixed.base.pyramid(z,camera))
        for t,p,z in zip((1.,1.03),initial,(depth,source),strict=True)])
    other = reference.prepare([dict(n,surface=audit.pyramid(z,camera)) for n,z in zip(nodes,(depth,source),strict=True)])
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    require(report['association_rounds']>=2,'actual rendered reselection reaches multiple rounds')
    # The full independent inner audit rejects source/weight/matrix/search
    # corruption; outer controls additionally reject selection and scheduling.
    changes = [('false outer convergence',lambda r:r.update(selection_converged=not r['selection_converged'])),
        ('wrong round count',lambda r:r.update(association_rounds=1)),
        ('wrong total updates',lambda r:r.update(applied_iterations=r['applied_iterations']+1)),
        ('wrong fresh matrix',lambda r:r['rounds'][0]['reselected']['equation'].update(g=(np.asarray(r['rounds'][0]['reselected']['equation']['g'])+1).tolist())),
        ('wrong transition loss',lambda r:r['rounds'][0]['transition'][0].update(removed_count=999)),
        ('changed incoming round start',lambda r:r['rounds'][1]['batch']['initial_poses'][1][0].__setitem__(3,9.))]
    rejected = []
    for name,change in changes:
        bad = copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected outer corruption '+name)
    guards = {}
    for name,z in (('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))):
        raw = kernel.fixed.prepare([dict(timestamp=t,root=identity.copy(),surface=kernel.fixed.base.pyramid(z,camera)) for t in (1.,1.03)])
        ind = reference.prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw]);poses = np.array([identity,identity])
        result = kernel.refine(raw,poses,None);checked += verify(result,ind,poses,None,audit)
        require(result['association_rounds']==1 and not result['applied_iterations'],'no artificial outer progress for degenerate images')
        require(result['termination']=='inner_guard' and not result['selection_converged'],'degenerate data keep support guards')
        guards[name] = result['guard_failure']
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    prior = kernel.fixed.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    prior_before = prior.encode();poses3 = np.array([identity,initial[1],initial[1]])
    poses3[2] = audit.increment(poses3[2],np.array([-.0004,.0002,-.0001,-.0001,.00008,.00017]))
    third = dict(timestamp=1.06,root=poses3[2].copy(),surface=nodes[1]['surface'])
    three = nodes+[third];ind_three = other+[dict(third,surface=other[1]['surface'])]
    with_prior = kernel.refine(three,poses3,prior);checked += verify(with_prior,ind_three,poses3,prior_before,audit)
    require(prior.encode()==prior_before,'one original nonlinear prior remains unchanged across rounds')
    input_guards = []
    bad_origin = initial.copy();bad_origin[0,0,3]=.1
    new_prior = kernel.fixed.base.Prior(1.,[1.03],[identity],np.eye(6),np.zeros(6),[],1)
    for name,poses,bound_prior in (('pose count',initial[:1],None),('fixed origin',bad_origin,None),('new acquisition prior',initial,new_prior)):
        try:kernel.refine(nodes,poses,bound_prior)
        except ValueError:input_guards.append(name)
        else:raise ValueError('unguarded reselected input '+name)
    boundary = identity_boundary_controls(audit,camera,audit.rays.render(identity))
    return dict(analytic_pass=True,inherited_analytic_pass=inherited['analytic_pass'],inherited_prior_gradient_axes=inherited['objective_gradient_axes'],
        scheduler_cases=scheduling,rejected_corruptions=rejected,rendered_rounds=report['association_rounds'],
        rendered_applied_corrections=report['applied_iterations'],rendered_selection_converged=report['selection_converged'],
        rendered_depth_quantization_m=.002,reported_systems=checked,guards=guards,input_roots_unchanged=True,
        nonzero_prior_unchanged=True,nonzero_prior_rounds=with_prior['association_rounds'],
        input_guards=input_guards,identity_admission_agreement=False,unresolved_identity_boundaries=boundary,rendered=report,rendered_with_prior=with_prior)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('reselected_slope_refinement.py','check-reselected-slope-refinement.py')}
    write(out/'summary.json',report)


if __name__=='__main__':main()
