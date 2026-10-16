#!/usr/bin/env python3
"""Independent Hermite interpolation, original selection and fixed-objective audit."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import normal_hermite_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


selector = load('cubic_original_selection','check-sampling-depth-refinement.py')
fixed = load('cubic_independent_fixed_objective','check-continuous-depth-refinement.py')
reference = fixed.reference


def independent_sample(layer,points):
    c = layer['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    ids = np.flatnonzero((points[:,2]>.3)&np.isfinite(points).all(axis=1));q = points[ids];image = q@k.T;uv = image[:,:2]/image[:,2,None]
    cell = np.floor(uv).astype(int)
    inside = (cell[:,0]>=1)&(cell[:,0]<c['width']-2)&(cell[:,1]>=1)&(cell[:,1]<c['height']-2)
    ids,q,image,uv,cell = [a[inside] for a in (ids,q,image,uv,cell)]
    xx = cell[:,0,None,None]+np.arange(2)[None,None,:];yy = cell[:,1,None,None]+np.arange(2)[None,:,None]
    depth = layer['depth'][yy,xx];normals = layer['normals'][yy,xx]
    size = np.sqrt(np.einsum('nijk,nijk->nij',normals,normals))
    valid = np.all((depth>.3)&(depth<5),axis=(1,2))&np.all(np.isfinite(normals).all(axis=3)&(size>.5),axis=(1,2))
    valid &= np.ptp(depth,axis=(1,2))<=.05+.01*np.min(depth,axis=(1,2))
    ids,q,image,uv,cell,xx,yy,depth,normals,size = [a[valid] for a in (ids,q,image,uv,cell,xx,yy,depth,normals,size)]
    pixels = np.stack([np.broadcast_to(xx,depth.shape),np.broadcast_to(yy,depth.shape),np.ones_like(depth)],axis=3)
    rays = pixels@np.linalg.inv(k).T;vertices = rays*depth[:,:,:,None]
    plane = np.einsum('nijk,nijk->nij',normals,vertices)
    valid = np.all(np.isfinite(plane)&(abs(plane)/size>1e-6),axis=(1,2))
    ids,q,image,uv,cell,depth,normals,plane = [a[valid] for a in (ids,q,image,uv,cell,depth,normals,plane)]
    coefficients = (normals/plane[:,:,:,None])@np.linalg.inv(k)
    constraints = np.zeros((len(ids),4,4));constraints[:,:2,:2] = 1/depth
    constraints[:,:2,2:] = coefficients[:,:,:,0];constraints[:,2:,:2] = coefficients[:,:,:,1]
    # Solve endpoint value/derivative constraints for polynomial coefficients;
    # operational code uses explicit tensor Hermite bases instead.
    endpoint = np.array([[1,0,0,0],[1,1,1,1],[0,1,0,0],[0,1,2,3.]])
    inverse = np.linalg.inv(endpoint);a = inverse@constraints@inverse.T;xy = uv-cell
    px = np.column_stack([np.ones(len(ids)),xy[:,0],xy[:,0]**2,xy[:,0]**3])
    py = np.column_stack([np.ones(len(ids)),xy[:,1],xy[:,1]**2,xy[:,1]**3])
    dx = np.column_stack([np.zeros(len(ids)),np.ones(len(ids)),2*xy[:,0],3*xy[:,0]**2])
    dy = np.column_stack([np.zeros(len(ids)),np.ones(len(ids)),2*xy[:,1],3*xy[:,1]**2])
    rho = np.einsum('ni,nij,nj->n',py,a,px);ru = np.einsum('ni,nij,nj->n',py,a,dx);rv = np.einsum('ni,nij,nj->n',dy,a,px)
    valid = np.isfinite(rho)&(rho>0)&np.isfinite(ru)&np.isfinite(rv)
    ids,q,image,uv,rho,ru,rv = [a[valid] for a in (ids,q,image,uv,rho,ru,rv)]
    projective = np.repeat(k[None,:2,:],len(q),axis=0)/q[:,2,None,None]
    projective[:,:,2] -= image[:,:2]/q[:,2,None]**2
    gradient = np.einsum('ni,nij->nj',np.column_stack([ru,rv]),projective);gradient[:,2] += 1/q[:,2]**2
    target = np.column_stack([uv,np.ones(len(uv))])@np.linalg.inv(k).T/rho[:,None]
    return ids,rho-1/q[:,2],gradient,target


def values(nodes,poses,a,b,points,normals,scale):
    relative = np.linalg.solve(poses[a],poses[b]);q = np.column_stack([points,np.ones(len(points))])@relative.T
    ids,f,df,_ = independent_sample(nodes[a]['surface'][scale],q[:,:3])
    size = np.sqrt(np.einsum('ij,ij->i',df,df))
    require(np.isfinite(size).all() and np.all(size>1e-12),'finite continuous gradient')
    return ids,f,df,size


def population(nodes,poses,scale):
    result = []
    for item in selector.population(nodes,poses,scale):
        ids,_,_,_ = values(nodes,poses,item['target'],item['source'],item['points'],item['normals'],scale)
        if len(ids)>=200:
            result.append(dict(item,**{key:item[key][ids] for key in ('points','normals','source_ids','normalizer')}))
    return result


fixed.values = values
fixed.population = population
check_metadata,system,objective = fixed.check_metadata,fixed.system,fixed.objective
compare_search = fixed.compare_search


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


def verify(report,nodes,initial,prior,audit):
    require(type(report['cap']) is int and report['cap'] in (10,20,30) and report['scale'] in (4,2,1),'unchanged correction limit')
    poses = np.asarray(initial).copy();np.testing.assert_array_equal(report['initial_poses'],poses)
    scale = report['scale'];matches = population(nodes,poses,scale);check_metadata(report['frozen_population'],matches)
    np.testing.assert_allclose(report['initial_cost'],objective(matches,nodes,poses,prior,scale,audit),atol=1e-10,rtol=1e-8)
    require(report['applied_iterations'] == len(report['trace']) <= report['cap'],'honest bounded continuous corrections')
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
        require(reason is not None or converged or len(report['trace']) == report['cap'],'no unexplained early continuous stop')
    require(report['continuous_converged'] == converged,'no false continuous convergence')
    termination = 'guard' if report['guard_failure'] is not None else ('converged' if converged else 'budget')
    require(report['termination'] == termination,'honest continuous termination')
    reference.check_correction(report['final_correction'],final,len(nodes)-1,audit)
    reference.check_correction(report['final_discrete_correction'],discrete,len(nodes)-1,audit)
    np.testing.assert_array_equal(report['final_poses'],poses)
    np.testing.assert_allclose(report['final_cost'],objective(matches,nodes,poses,prior,scale,audit),atol=1e-10,rtol=1e-8)
    return checked+2


def verify_checkpoint(actual,report,nodes,initial,prior,audit,cap):
    require(type(cap) is int and cap in (10,20,30) and actual['requested_cap'] == cap,'fixed checkpoint limits')
    used = min(cap,len(report['trace']));require(actual['actual_applied_corrections'] == used,'honest checkpoint applied count')
    require(actual['terminal_carried_forward'] == (used < cap),'explicit early terminal carry-forward')
    poses = np.asarray(initial if used == 0 else report['trace'][used-1]['after'])
    np.testing.assert_array_equal(actual['final_poses'],poses)
    matches = population(nodes,np.asarray(initial),report['scale'])
    e = system(matches,nodes,poses,prior,report['scale'],audit);reason,d,_ = reference.failure(e,len(nodes)-1,audit)
    converged = reason is None and reference.small(d)
    if reason is not None:termination,failure = 'guard',reason
    elif converged:termination,failure = 'converged',None
    elif used == cap:termination,failure = 'budget',None
    else:
        require(used < cap and report['guard_failure'] == 'line search','terminal search precedes requested endpoint')
        termination,failure = 'guard','line search'
    require(actual['termination'] == termination and actual['guard_failure'] == failure and actual['continuous_converged'] == converged,'independent checkpoint event order')
    audit.compare_equation(actual['final_equation'],e);reference.check_correction(actual['final_correction'],e,len(nodes)-1,audit)
    fresh = audit.independent_equation(nodes,poses,report['scale'],prior)
    audit.compare_equation(actual['final_discrete_equation'],fresh)
    reference.check_correction(actual['final_discrete_correction'],fresh,len(nodes)-1,audit)
    np.testing.assert_allclose(actual['final_cost'],objective(matches,nodes,poses,prior,report['scale'],audit),atol=1e-10,rtol=1e-8)
    return 2


def source_proof():
    def body(file,name):
        text = (ROOT/'scripts'/file).read_text();node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    for name in ('values','evaluate','equation','energy'):
        require(body('normal_hermite_depth_refinement.py',name)==body('sampling_depth_refinement.py',name),'unchanged fixed objective body '+name)
    for name in ('search','refine'):
        require(body('normal_hermite_depth_refinement.py',name)==body('deep_search_depth_refinement.py',name),'unchanged solver body '+name)
    for name in ('snapshot','checkpoint'):
        require(body('normal_hermite_depth_refinement.py',name)==body('long_depth_refinement.py',name),'unchanged checkpoint body '+name)
    for name in ('verify','verify_checkpoint'):
        require(body('check-normal-hermite-depth-refinement.py',name)==body('check-long-depth-refinement.py',name),'unchanged independent audit body '+name)
    require(body('check-normal-hermite-depth-refinement.py','search')==body('check-deep-search-depth-refinement.py','search'),'unchanged independent search')
    require(kernel.discrete is kernel.previous.discrete and kernel.base is kernel.previous.base and kernel.metadata is kernel.previous.metadata,'unchanged priors, solve and metadata')
    require(selector.independent.values is not values and fixed.values is values,'new sampler isolated from old initial qualifier')
    return True


def interpolation_controls():
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);y,x = np.indices((32,32))
    def points(uv):
        return np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(len(uv),2.)])
    def compare(layer,p):
        a,b = kernel.sample(layer,p),independent_sample(layer,p)
        for actual,expected in zip(a,b,strict=True):np.testing.assert_allclose(actual,expected,atol=2e-13,rtol=0)
        return a
    # Plane inverse depth is affine in image coordinates, with exact measured
    # normals reconstructed from its covector; no renderer truth in fitting.
    rho = .5+.0002*x+.0001*y
    normal = np.array([.0002*c['fx'],.0001*c['fy'],.5+.0002*c['cx']+.0001*c['cy']]);normal /= np.linalg.norm(normal)
    layer = dict(camera=c,depth=1/rho,normals=np.tile(normal,(32,32,1)))
    uv = np.array([[10.2,11.4],[14.,10.],[8.7,20.8],[16.-1e-6,12.3],[16.+1e-6,12.3],[12.3,16.-1e-6],[12.3,16.+1e-6]])
    p = points(uv);actual = compare(layer,p);value = .5+.0002*uv[:,0]+.0001*uv[:,1]
    np.testing.assert_allclose(actual[1],value-.5,atol=2e-15,rtol=0)
    df = np.column_stack([np.full(len(p),.0002*c['fx']/2),np.full(len(p),.0001*c['fy']/2),.25-(.0002*c['fx']*p[:,0]+.0001*c['fy']*p[:,1])/4])
    np.testing.assert_allclose(actual[2],df,atol=2e-13,rtol=0)
    for axis in range(3):
        d = np.zeros(3);d[axis] = 1e-7
        numeric = (kernel.sample(layer,p+d)[1]-kernel.sample(layer,p-d)[1])/2e-7
        np.testing.assert_allclose(numeric,actual[2][:,axis],atol=5e-9,rtol=1e-6)
    # Deliberately inconsistent spatially varying slopes also share edge values
    # and BOTH first derivatives. Check vertical and horizontal grid edges.
    noisy = dict(layer,depth=1/(.5+.001*abs(x-16)+.0001*y),normals=np.stack([.02*np.sin(x),.015*np.cos(y),np.ones_like(x)],axis=2))
    interior = points(np.array([[10.2,11.4],[18.3,21.6]]));curved = compare(noisy,interior)
    for axis in range(3):
        d = np.zeros(3);d[axis] = 1e-7
        numeric = (kernel.sample(noisy,interior+d)[1]-kernel.sample(noisy,interior-d)[1])/2e-7
        np.testing.assert_allclose(numeric,curved[2][:,axis],atol=5e-9,rtol=1e-6)
    for axis in (0,1):
        edge = np.array([[16.,12.25],[12.25,16.]])[axis];grid = np.tile(edge,(3,1));grid[:,axis] += [-1e-7,0,1e-7]
        result = compare(noisy,points(grid))
        np.testing.assert_allclose(result[1],result[1][1],atol=2e-10,rtol=0)
        np.testing.assert_allclose(result[2],np.broadcast_to(result[2][1],(3,3)),atol=2e-8,rtol=0)
    integers = np.array([[10.,11.],[16.,16.],[20.,9.]])
    result = compare(noisy,points(integers))
    np.testing.assert_allclose(result[1],1/noisy['depth'][integers[:,1].astype(int),integers[:,0].astype(int)]-.5,atol=1e-15,rtol=0)
    for scale in (-1.,2.):
        result = compare(dict(noisy,normals=noisy['normals']*scale),p)
        for a,b in zip(result,kernel.sample(noisy,p),strict=True):np.testing.assert_allclose(a,b,atol=2e-13,rtol=0)
    # The 4x4-only outer corner does not belong to this sampler; each actual
    # corner, invalid normal and near-zero plane offset must still reject.
    altered = dict(layer,depth=layer['depth'].copy());altered['depth'][10,9] = 0.
    require(len(compare(altered,p[:1])[0])==1,'missing outer corner needs no larger stencil')
    for yy,xx in ((11,10),(11,11),(12,10),(12,11)):
        bad = dict(layer,depth=layer['depth'].copy());bad['depth'][yy,xx] = 0.
        require(len(compare(bad,p[:1])[0])==0,'each missing corner is guarded')
    for vector in ([0.,0.,0.],[1.,0.,-(10-c['cx'])/c['fx']],[float('nan'),0.,1.]):
        bad = dict(layer,normals=layer['normals'].copy());bad['normals'][11,10] = vector
        require(len(compare(bad,p[:1])[0])==0,'invalid or tangent plane offset guarded')
    overshoot = dict(layer,depth=np.full((32,32),2.),normals=layer['normals'].copy())
    for yy in (11,12):
        for xx,slope in ((10,-5.),(11,5.)):
            vector = np.array([slope*c['fx'],0.,.5-slope*(xx-c['cx'])]);vector /= np.linalg.norm(vector)
            overshoot['normals'][yy,xx] = vector
    require(len(compare(overshoot,points(np.array([[10.5,11.5]])))[0])==0,'negative cubic inverse depth rejects without clipping or fallback')
    return dict(plane_samples=len(p),xyz_finite_difference_axes=3,nonaffine_xyz_finite_difference_axes=3,interpolatory_integer_samples=3,c1_inconsistent_edge_axes=2,
        normal_sign_and_scale_checks=2,missing_outer_corner_allowed=True,missing_corners_rejected=4,invalid_normal_and_plane_guards=3,negative_overshoot_guard=True)


def controls():
    proof = source_proof();interpolation = interpolation_controls()
    audit = load('cubic_rendered_oracle','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA
    depth = audit.rays.render(identity);truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = [dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(nodes,(depth,source),strict=True)]
    report = kernel.refine(nodes,initial,None);checked = verify(report,other,initial,None,audit)
    endpoints = sum(verify_checkpoint(kernel.checkpoint(report,nodes,None,cap),report,other,initial,None,audit,cap) for cap in (10,20,30))
    # Nonzero incoming prior with both measured endpoints moving: all12 axes.
    third = dict(timestamp=1.06,root=initial[1].copy(),surface=nodes[1]['surface']);raw3 = nodes+[third]
    ind3 = other+[dict(third,surface=other[1]['surface'])];poses3 = np.array([identity,initial[1],initial[1]])
    poses3[1] = audit.increment(poses3[1],np.array([.00073,-.00031,.00021,.00012,-.00014,.00009]))
    poses3[2] = audit.increment(poses3[2],np.array([-.00041,.00023,-.00011,-.00011,.00008,.00017]))
    prior = kernel.base.Prior(1.,[1.03],[identity],np.eye(6)*3,np.array([.1,-.03,.04,.02,.01,-.02]),[],1)
    matches = kernel.population(raw3,poses3,1);expected = population(ind3,poses3,1);e = kernel.equation(matches,raw3,poses3,prior,1)
    audit.compare_equation(kernel.base.encode_equation(e),system(expected,ind3,poses3,prior.encode(),1,audit))
    numeric = []
    for node in (1,2):
        for axis in range(6):
            d = np.zeros(6);d[axis] = 1e-7;plus,minus = poses3.copy(),poses3.copy()
            plus[node] = audit.increment(poses3[node],d);minus[node] = audit.increment(poses3[node],-d)
            numeric.append((objective(expected,ind3,plus,prior.encode(),1,audit)-objective(expected,ind3,minus,prior.encode(),1,audit))/2e-7)
    np.testing.assert_allclose(numeric,e['g'],atol=3e-7,rtol=2e-6)
    rejected = []
    for name,change in (('false convergence',lambda r:r.update(continuous_converged=not r['continuous_converged'])),
        ('changed source identities',lambda r:r['frozen_population'][0].update(source_pixel_sha256='0'*64)),
        ('changed normalizer digest',lambda r:r['frozen_population'][0].update(normalizer_sha256='0'*64)),
        ('wrong final matrix',lambda r:r['final_equation'].update(g=(np.asarray(r['final_equation']['g'])+1).tolist()))):
        bad = copy.deepcopy(report);change(bad)
        try:verify(bad,other,initial,None,audit)
        except (ValueError,AssertionError):rejected.append(name)
        else:raise ValueError('undetected cubic corruption: '+name)
    guards = {}
    for name,z in (('missing',np.zeros_like(depth)),('plane',np.full_like(depth,2.))):
        raw = [dict(timestamp=t,root=identity.copy(),surface=kernel.base.pyramid(z,camera)) for t in (1.,1.03)]
        independent = [dict(n,surface=audit.pyramid(z,camera)) for n in raw]
        q = kernel.refine(raw,np.array([identity,identity]),None);checked += verify(q,independent,np.array([identity,identity]),None,audit)
        require(q['termination']=='guard' and not q['trace'],'degenerate cubic images cannot fabricate information');guards[name] = q['guard_failure']
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    old = sum(len(i['source_ids']) for i in kernel.previous.population(nodes,initial,1));new = sum(len(i['source_ids']) for i in kernel.population(nodes,initial,1))
    return dict(analytic_pass=True,source_proof=proof,interpolation=interpolation,rendered_continuous_converged=report['continuous_converged'],
        rendered_applied_corrections=report['applied_iterations'],rendered_guard=report['guard_failure'],reported_systems=checked,checkpoint_systems=endpoints,
        prior_system_checks=1,objective_gradient_axes=12,original_rendered_sources=old,hermite_rendered_sources=new,
        degenerate_guards=guards,rejected_corruptions=rejected,input_roots_unchanged=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('normal_hermite_depth_factors.py','normal_hermite_depth_refinement.py','check-normal-hermite-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
