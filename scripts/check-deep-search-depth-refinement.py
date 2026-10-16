#!/usr/bin/env python3
"""Independently verify a search-bound-only fixed-objective extension."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import deep_search_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


# Rebind only a private verifier's search dependency. Its original population,
# fixed-objective audit and checkpoint event ordering remain unchanged.
parent = load('deep_fixed_objective_audit','check-long-depth-refinement.py')
population,check_metadata,system,objective = parent.population,parent.check_metadata,parent.system,parent.objective
compare_search,reference = parent.compare_search,parent.reference


def search(matches,nodes,poses,prior,scale,d,g,audit):
    before = objective(matches,nodes,poses,prior,scale,audit);directional = float(g@d.ravel())
    require(np.isfinite(directional) and directional <= 0,'independent continuous descent')
    trials = []
    for exponent in range(21):
        alpha = 1/(2**exponent);candidate = np.asarray(poses).copy()
        for node,delta in enumerate(d,1):candidate[node] = audit.increment(poses[node],alpha*delta)
        limit = before+.0001*alpha*directional+1e-12*max(1.,abs(before))
        try:cost = objective(matches,nodes,candidate,prior,scale,audit);reason = None
        except ValueError as error:cost = None;reason = str(error)
        trials.append(dict(alpha=alpha,cost=cost,limit=limit,guard_failure=reason))
        if reason is None and cost <= limit:return candidate,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=alpha)
    return None,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=None)


parent.search = search
verify,verify_checkpoint = parent.verify,parent.verify_checkpoint


def source_proof():
    def body(file,function):
        text = (ROOT/'scripts'/file).read_text()
        node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name == function)
        return ast.get_source_segment(text,node)
    require(body('deep_search_depth_refinement.py','refine') == body('long_depth_refinement.py','refine'),'unchanged complete refiner body')
    require(body('deep_search_depth_refinement.py','search') == body('sampling_depth_refinement.py','search').replace('range(8)','range(21)'),
        'only operational search range changes')
    require(body('check-deep-search-depth-refinement.py','search') == body('check-continuous-depth-refinement.py','search').replace('range(8)','range(21)'),
        'only independent search range changes')
    require(verify.__globals__['search'] is search,'private fixed-objective verifier uses extended independent search')
    require(kernel.snapshot is parent.kernel.snapshot and kernel.checkpoint is parent.kernel.checkpoint,'unchanged checkpoint implementation')
    for name in ('population','metadata','equation','energy','base','discrete'):
        require(getattr(kernel,name) is getattr(parent.kernel,name),'unchanged dependency '+name)
    return True


def controls():
    proof = source_proof()
    inherited = load('deep_unmodified_control_suite','check-long-depth-refinement.py').controls()
    audit = load('deep_rendered_oracle','check-reciprocal-keyframes.py').auditor()
    identity = np.eye(4);camera = audit.rays.CAMERA;depth = audit.rays.render(identity)
    truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = [dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(nodes,(depth,source),strict=True)]
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    old = parent.kernel.refine(nodes,initial,None);require(report == old,'complete unchanged rendered terminal report')
    endpoints = sum(verify_checkpoint(kernel.checkpoint(report,nodes,None,cap),report,other,initial,None,audit,cap) for cap in (10,20,30))
    # Authored image-domain fixtures use an injected direction. No solve,
    # minimum-support or physical pose accuracy claim is made for them.
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5)
    y,x = np.indices((32,32));rho = .5+.0002*x+.0001*y
    layer = dict(camera=c,depth=1/rho,normals=np.tile([0.,0.,1.],(32,32,1)))
    images = [dict(timestamp=t,root=identity.copy(),surface={1:layer}) for t in (1.,1.03)]
    poses = np.array([identity,identity]);direction = np.array([[-.01,0,0,0,0,0]])
    receipts = {}
    for name,u in (('deeper pass',1.001),('exhausted domain',1.)):
        point = np.array([[(u-c['cx'])*2/c['fx'],(12.25-c['cy'])*2/c['fy'],2.]])
        matches = [dict(target=0,source=1,points=point,normals=np.array([[0.,0.,1.]]),source_ids=np.array([0]),normalizer=np.ones(1))]
        e = kernel.equation(matches,images,poses,None,1);ie = system(matches,images,poses,None,1,audit)
        audit.compare_equation(kernel.base.encode_equation(e),ie)
        candidate,receipt = kernel.search(matches,images,poses,None,1,direction,e['g'])
        expected,wanted = search(matches,images,poses,None,1,direction,ie['g'],audit);compare_search(receipt,wanted)
        before,old_receipt = parent.kernel.search(matches,images,poses,None,1,direction,e['g'])
        require(before is None and receipt['trials'][:8] == old_receipt['trials'],'complete original failed candidate prefix')
        if name == 'deeper pass':
            require(receipt['accepted_alpha'] == 1/256 and len(receipt['trials']) == 9,'first valid ninth candidate')
            np.testing.assert_allclose(candidate,expected,atol=1e-14,rtol=0)
            require(receipt['trials'][-1]['cost'] < receipt['before_cost'],'authored deeper pass decreases cost')
        else:
            require(candidate is None and expected is None and len(receipt['trials']) == 21,'bounded exhaustion at twenty-one candidates')
            require(all(t['cost'] is None and t['guard_failure'] == 'fixed source support' for t in receipt['trials']),'invalid sources cannot receive a partial cost')
        receipts[name] = receipt
    rejected = []
    wanted = receipts['deeper pass']
    for name,change in (('wrong accepted multiplier',lambda r:r.update(accepted_alpha=1/128)),
        ('truncated failed prefix',lambda r:r['trials'].pop(0)),
        ('false valid candidate cost',lambda r:r['trials'][-1].update(cost=1.)),
        ('false candidate validity',lambda r:r['trials'][0].update(guard_failure=None))):
        bad = copy.deepcopy(wanted);change(bad)
        try:compare_search(bad,wanted)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected deeper-search corruption: '+name)
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    return dict(analytic_pass=True,source_proof=proof,inherited_long_controls=inherited,
        additional_rendered_reported_systems=checked,additional_checkpoint_systems=endpoints,
        rendered_continuous_converged=report['continuous_converged'],rendered_applied_corrections=report['applied_iterations'],
        complete_rendered_control_equal=True,authored_injected_direction_systems=2,
        authored_directions_are_not_guarded_solver_proposals=True,authored_search_receipts=receipts,
        rejected_deep_search_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('deep_search_depth_refinement.py','check-deep-search-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
