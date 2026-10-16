#!/usr/bin/env python3
"""Independent harmonic slopes and bicubic polynomial objective audit."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import depth_slope_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


private = load('depth_slope_private_oracle','check-normal-hermite-depth-refinement.py')


def independent_field(layer):
    z = layer['depth'];h,w = z.shape;center = z[2:h-2,2:w-2]
    five = np.stack([center,z[2:h-2,1:w-3],z[2:h-2,3:w-1],z[1:h-3,2:w-2],z[3:h-1,2:w-2]],axis=2)
    good = np.all((five>.3)&(five<5)&(np.abs(five-center[:,:,None])<.05+.01*center[:,:,None]),axis=2)
    inverse = np.zeros_like(five);np.divide(1.,five,out=inverse,where=(five>.3)&(five<5))
    out = dict(du=np.zeros_like(z),dv=np.zeros_like(z),valid=np.zeros_like(z,dtype=bool));out['valid'][2:h-2,2:w-2] = good
    for key,lo,hi in (('du',1,2),('dv',3,4)):
        a,b = inverse[:,:,0]-inverse[:,:,lo],inverse[:,:,hi]-inverse[:,:,0]
        same = good&(((a>0)&(b>0))|((a<0)&(b<0)))
        values = np.zeros_like(center);values[same] = 1/(.5/a[same]+.5/b[same]);out[key][2:h-2,2:w-2] = values
    return out


def prepare(nodes):
    return [dict(node,surface={s:dict(layer,depth_slopes=independent_field(layer)) for s,layer in node['surface'].items()}) for node in nodes]


def independent_sample(layer,points):
    require('depth_slopes' in layer,'prepared depth-slope field required')
    slopes = layer['depth_slopes']
    c = layer['camera'];k = np.array([[c['fx'],0,c['cx']],[0,c['fy'],c['cy']],[0,0,1.]])
    ids = np.flatnonzero((points[:,2]>.3)&np.isfinite(points).all(axis=1));q = points[ids];image = q@k.T;uv = image[:,:2]/image[:,2,None]
    cell = np.floor(uv).astype(int)
    inside = (cell[:,0]>=1)&(cell[:,0]<c['width']-2)&(cell[:,1]>=1)&(cell[:,1]<c['height']-2)
    ids,q,image,uv,cell = [a[inside] for a in (ids,q,image,uv,cell)]
    xx = cell[:,0,None,None]+np.arange(2)[None,None,:];yy = cell[:,1,None,None]+np.arange(2)[None,:,None]
    depth = layer['depth'][yy,xx]
    valid = np.all((depth>.3)&(depth<5),axis=(1,2))&slopes['valid'][yy,xx].all(axis=(1,2))
    valid &= np.ptp(depth,axis=(1,2))<=.05+.01*np.min(depth,axis=(1,2))
    du,dv = slopes['du'][yy,xx],slopes['dv'][yy,xx]
    ids,q,image,uv,cell,depth,du,dv = [a[valid] for a in (ids,q,image,uv,cell,depth,du,dv)]
    constraints = np.zeros((len(ids),4,4));constraints[:,:2,:2] = 1/depth
    constraints[:,:2,2:] = du;constraints[:,2:,:2] = dv
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

private.independent_sample = independent_sample
population,values = private.population,private.values
check_metadata,system,objective = private.check_metadata,private.system,private.objective
verify,verify_checkpoint,reference = private.verify,private.verify_checkpoint,private.reference


def source_proof():
    def body(file,name):
        text = (ROOT/'scripts'/file).read_text();node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    for name in ('population','values','evaluate','equation','energy','search','refine','snapshot','checkpoint'):
        require(body('depth_slope_refinement.py',name)==body('normal_hermite_depth_refinement.py',name),'unchanged fixed-source solver '+name)
    require(verify.__globals__['independent_sample'] is independent_sample and private.fixed.values is values,'private interpolation audit binding')
    require(private.selector.independent.values is not values,'original measured-normal initial selection isolated')
    return True


def compare_prepared(nodes,other):
    receipt = []
    for node,oracle in zip(nodes,other,strict=True):
        for scale,layer in node['surface'].items():
            a,b = layer['depth_slopes'],oracle['surface'][scale]['depth_slopes']
            np.testing.assert_array_equal(a['valid'],b['valid'])
            for key in ('du','dv'):np.testing.assert_allclose(a[key],b[key],atol=2e-15,rtol=1e-12)
            receipt.append(dict(timestamp=node.get('timestamp'),scale=scale,valid_slope_vertices=int(a['valid'].sum()),independent_fields_agree=True))
    return receipt


def interpolation_controls():
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);yy,xx = np.indices((32,32))
    def make(rho):
        z = 1/rho;vertices = np.stack([(xx-c['cx'])*z/c['fx'],(yy-c['cy'])*z/c['fy'],z],axis=2)
        return dict(camera=c,depth=z,vertices=vertices,normals=np.zeros_like(vertices))
    def prepared(layer):
        a,b = kernel.prepare([dict(surface={1:layer})]),prepare([dict(surface={1:layer})]);compare_prepared(a,b)
        for key in ('depth','vertices','normals'):require(a[0]['surface'][1][key] is layer[key],'unchanged measured '+key)
        return a[0]['surface'][1],b[0]['surface'][1]
    def points(uv):
        return np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(len(uv),2.)])
    def compare(a,b,p):
        result,expected = kernel.sample(a,p),independent_sample(b,p)
        for x,y in zip(result,expected,strict=True):np.testing.assert_allclose(x,y,atol=2e-13,rtol=0)
        return result
    layer = make(.5+.0002*xx+.0001*yy);a,b = prepared(layer)
    uv = np.array([[10.2,11.4],[14.,10.],[8.7,20.8],[16.-1e-6,12.3],[16.+1e-6,12.3],[12.3,16.-1e-6],[12.3,16.+1e-6]])
    p = points(uv);result = compare(a,b,p)
    np.testing.assert_allclose(result[1],.0002*uv[:,0]+.0001*uv[:,1],atol=2e-15,rtol=0)
    df = np.column_stack([np.full(len(p),.0002*c['fx']/2),np.full(len(p),.0001*c['fy']/2),.25-(.0002*c['fx']*p[:,0]+.0001*c['fy']*p[:,1])/4])
    np.testing.assert_allclose(result[2],df,atol=2e-13,rtol=0)
    require(len(result[0])==len(p),'sampling needs measured depth differences without normal eligibility')
    from depth_slope_factors import limited
    np.testing.assert_allclose(limited(np.array([1.,2.,-1.,-2.,0.,1.]),np.array([1.,4.,-3.,1.,2.,0.])),[1.,8/3,-1.5,0.,0.,0.],atol=1e-15,rtol=0)
    curved = make(.5+.0001*(xx-15)**2+.00008*(yy-16)**2+.00001*xx*yy);ca,cb = prepared(curved)
    q = points(np.array([[10.2,11.4],[18.3,21.6]]));curved_result = compare(ca,cb,q)
    for axis in range(3):
        delta = np.zeros(3);delta[axis] = 1e-7
        numeric = (kernel.sample(ca,q+delta)[1]-kernel.sample(ca,q-delta)[1])/2e-7
        np.testing.assert_allclose(numeric,curved_result[2][:,axis],atol=5e-9,rtol=1e-6)
    for axis in (0,1):
        edge = np.array([[16.,12.25],[12.25,16.]])[axis];grid = np.tile(edge,(3,1));grid[:,axis] += [-1e-7,0,1e-7]
        edge_result = compare(ca,cb,points(grid))
        np.testing.assert_allclose(edge_result[1],edge_result[1][1],atol=3e-10,rtol=0)
        np.testing.assert_allclose(edge_result[2],np.broadcast_to(edge_result[2][1],(3,3)),atol=3e-8,rtol=0)
    integer = np.array([[10.,11.],[16.,16.],[20.,9.]])
    values = compare(ca,cb,points(integer))[1]
    np.testing.assert_allclose(values,1/curved['depth'][integer[:,1].astype(int),integer[:,0].astype(int)]-.5,atol=1e-15,rtol=0)
    monotone = make(.5+.004*np.clip(xx-14,0,4));ma,mb = prepared(monotone)
    grid = np.column_stack([np.linspace(13,19,241),np.full(241,11.25)]);value = compare(ma,mb,points(grid))
    rho = value[1]+.5;cell = np.floor(grid[:,0]).astype(int)
    lo = 1/monotone['depth'][11,cell];hi = 1/monotone['depth'][11,cell+1]
    require(np.all(rho>=lo-1e-14) and np.all(rho<=hi+1e-14) and np.all(value[2][:,0]>=-1e-13),'authored one-dimensional monotone interpolant')
    # Four corner depths and all five cardinal samples at each corner are
    # required. A diagonal outside that union is not sampling support.
    for y,x in ((11,10),(11,11),(12,10),(12,11),(11,9)):
        missing = dict(layer,depth=layer['depth'].copy());missing['depth'][y,x] = 0.
        la,lb = prepared(missing);require(len(compare(la,lb,p[:1])[0])==0,'missing corner/cardinal sample rejects')
    diagonal = dict(layer,depth=layer['depth'].copy());diagonal['depth'][10,9] = 0.
    la,lb = prepared(diagonal);require(len(compare(la,lb,p[:1])[0])==1,'unused diagonal is not required')
    jump = dict(layer,depth=layer['depth'].copy());jump['depth'][11,9] = 2.2
    la,lb = prepared(jump);require(len(compare(la,lb,p[:1])[0])==0,'depth discontinuity rejects cardinal support')
    require(not a['depth_slopes']['valid'][:2].any() and not a['depth_slopes']['valid'][-2:].any(),'two-pixel field border preserved')
    require(not a['depth_slopes']['valid'][:,:2].any() and not a['depth_slopes']['valid'][:,-2:].any(),'two-pixel horizontal border preserved')
    try:kernel.sample(layer,p)
    except ValueError as error:require(str(error)=='prepared depth-slope field required','explicit unprepared-field failure')
    else:raise ValueError('silently used unprepared slope field')
    rejected = []
    for key in ('valid','du'):
        bad = copy.deepcopy(a)
        if key=='valid':bad['depth_slopes'][key][11,10] = False
        else:bad['depth_slopes'][key][11,10] += .01
        try:compare_prepared([dict(surface={1:bad})],[dict(surface={1:b})])
        except (ValueError,AssertionError):rejected.append(key)
        else:raise ValueError('undetected slope-field corruption '+key)
    return dict(plane_samples=len(p),harmonic_authored_cases=6,nonaffine_xyz_finite_difference_axes=3,c1_edge_axes=2,
        integer_samples=3,one_dimensional_monotone_samples=241,zero_normal_sampling_supported=True,
        missing_corner_or_cardinal_rejections=5,unused_diagonal_allowed=True,depth_jump_rejected=True,
        unprepared_field_rejected=True,source_geometry_and_normals_unchanged=True,rejected_field_corruptions=rejected,
        limits='one-dimensional monotonicity does not certify two-dimensional range preservation or physical surface accuracy')


def controls():
    proof = source_proof();interpolation = interpolation_controls()
    audit = load('cubic_rendered_oracle','check-reciprocal-keyframes.py').auditor();identity = np.eye(4);camera = audit.rays.CAMERA
    depth = audit.rays.render(identity);truth = audit.increment(identity,np.array([.04,0,.002,0,.0015,0]));source = audit.rays.render(truth)
    tracker = kernel.base.Tracker(camera,'retain');tracker.step(1.,depth);warm = tracker.step(1.03,source)
    require(warm['accepted'],'measured rendered initialization');initial = np.asarray(warm['revised_roots'])
    nodes = [dict(timestamp=t,root=p.copy(),surface=kernel.base.pyramid(d,camera)) for t,p,d in zip((1.,1.03),initial,(depth,source),strict=True)]
    other = prepare([dict(n,surface=audit.pyramid(d,camera)) for n,d in zip(nodes,(depth,source),strict=True)])
    nodes = kernel.prepare(nodes);compare_prepared(nodes,other)
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
        raw = kernel.prepare(raw)
        independent = prepare([dict(n,surface=audit.pyramid(z,camera)) for n in raw])
        q = kernel.refine(raw,np.array([identity,identity]),None);checked += verify(q,independent,np.array([identity,identity]),None,audit)
        require(q['termination']=='guard' and not q['trace'],'degenerate cubic images cannot fabricate information');guards[name] = q['guard_failure']
    for node,p in zip(nodes,initial,strict=True):np.testing.assert_array_equal(node['root'],p)
    old = sum(len(i['source_ids']) for i in kernel.previous.population(nodes,initial,1));new = sum(len(i['source_ids']) for i in kernel.population(nodes,initial,1))
    return dict(analytic_pass=True,source_proof=proof,interpolation=interpolation,rendered_continuous_converged=report['continuous_converged'],
        rendered_applied_corrections=report['applied_iterations'],rendered_guard=report['guard_failure'],reported_systems=checked,checkpoint_systems=endpoints,
        prior_system_checks=1,objective_gradient_axes=12,original_rendered_sources=old,depth_slope_rendered_sources=new,
        degenerate_guards=guards,rejected_corruptions=rejected,input_roots_unchanged=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('depth_slope_factors.py','depth_slope_refinement.py','check-depth-slope-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
