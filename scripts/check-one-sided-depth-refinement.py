#!/usr/bin/env python3
"""Independent side eligibility, reciprocal-mean slopes and fixed objective audit."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import one_sided_depth_refinement as kernel
import depth_slope_refinement as previous
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


private = load('one_sided_private_verifier','check-depth-slope-refinement.py')
parent = load('one_sided_original_controls','check-depth-slope-refinement.py')


def independent_field(layer):
    z = layer['depth'];h,w = z.shape;center = z[2:h-2,2:w-2]
    five = np.stack([center,z[2:h-2,1:w-3],z[2:h-2,3:w-1],z[1:h-3,2:w-2],z[3:h-1,2:w-2]],axis=2)
    available = (five>.3)&(five<5)&(abs(five-center[:,:,None])<.05+.01*center[:,:,None])
    valid = available[:,:,0]&(available[:,:,1]|available[:,:,2])&(available[:,:,3]|available[:,:,4])
    inverse = np.zeros_like(five);np.divide(1.,five,out=inverse,where=(five>.3)&(five<5))
    result = dict(valid=np.zeros_like(z,dtype=bool),du=np.zeros_like(z),dv=np.zeros_like(z));result['valid'][2:h-2,2:w-2] = valid
    for key,left,right in (('du',1,2),('dv',3,4)):
        a,b = inverse[:,:,0]-inverse[:,:,left],inverse[:,:,right]-inverse[:,:,0]
        values = np.zeros_like(center);both = available[:,:,left]&available[:,:,right]
        same = valid&both&(((a>0)&(b>0))|((a<0)&(b<0)))
        values[same] = 1/(.5/a[same]+.5/b[same])
        only_left = valid&available[:,:,left]&~available[:,:,right];only_right = valid&~available[:,:,left]&available[:,:,right]
        values[only_left] = a[only_left];values[only_right] = b[only_right];result[key][2:h-2,2:w-2] = values
    return result


def prepare(nodes):
    return [dict(node,surface={s:dict(layer,depth_slopes=independent_field(layer)) for s,layer in node['surface'].items()}) for node in nodes]


independent_sample = private.independent_sample
population,values = private.population,private.values
check_metadata,system,objective = private.check_metadata,private.system,private.objective
verify,verify_checkpoint,reference = private.verify,private.verify_checkpoint,private.reference
compare_prepared = private.compare_prepared


def source_proof():
    def body(file,name):
        text = (ROOT/'scripts'/file).read_text();node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    for name in ('population','values','evaluate','equation','energy','search','refine','snapshot','checkpoint'):
        require(body('one_sided_depth_refinement.py',name)==body('depth_slope_refinement.py',name),'unchanged bilateral depth-slope body '+name)
    require(kernel.sample is previous.sample,'unchanged Hermite sampler reads the prepared field')
    require(verify is private.verify and verify_checkpoint is private.verify_checkpoint,'unchanged independent verification functions')
    require(kernel.previous is previous.previous and kernel.base is previous.base,'original initial source selection and priors')
    return True


def support_controls():
    inherited = parent.interpolation_controls()
    c = dict(width=32,height=32,fx=40.,fy=42.,cx=15.5,cy=15.5);yy,xx = np.indices((32,32))
    from depth_slope_factors import field as bilateral
    def layer(rho):
        z = 1/rho;xyz = np.stack([(xx-c['cx'])*z/c['fx'],(yy-c['cy'])*z/c['fy'],z],axis=2)
        return dict(camera=c,depth=z,vertices=xyz,normals=np.zeros_like(xyz))
    def points(uv):
        return np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(len(uv),2.)])
    base = layer(.5+.0002*xx+.0001*yy)
    fixtures = dict(both=(),left_missing=((12,11),),right_missing=((12,13),),up_missing=((11,12),),down_missing=((13,12),),
        neither_horizontal=((12,11),(12,13)),neither_vertical=((11,12),(13,12)),center_missing=((12,12),))
    checked = old_samples = new_samples = 0
    grid = np.array([(u+.25,v+.25) for v in range(2,29) for u in range(2,29)]);queries = points(grid)
    for name,missing in fixtures.items():
        raw = dict(base,depth=base['depth'].copy())
        for y,x in missing:raw['depth'][y,x] = 0.
        actual,expected = kernel.prepare([dict(surface={1:raw})]),prepare([dict(surface={1:raw})]);compare_prepared(actual,expected)
        a,b = actual[0]['surface'][1],expected[0]['surface'][1];new = a['depth_slopes'];old = bilateral(raw)
        for key in ('depth','vertices','normals'):require(a[key] is raw[key],'original geometry and normals preserved')
        require(np.all(~old['valid']|new['valid']),'bilateral field domain contained in new field')
        for key in ('du','dv'):np.testing.assert_array_equal(new[key][old['valid']],old[key][old['valid']])
        valid_center = name not in ('neither_horizontal','neither_vertical','center_missing')
        require(bool(new['valid'][12,12])==valid_center,'at least one measured same-side neighbor per axis')
        if valid_center:
            np.testing.assert_allclose([new['du'][12,12],new['dv'][12,12]],[.0002,.0001],atol=2e-15,rtol=0)
        result = kernel.sample(a,queries);oracle = independent_sample(b,queries)
        for x,y in zip(result,oracle,strict=True):np.testing.assert_allclose(x,y,atol=2e-13,rtol=0)
        old_result = previous.sample(dict(raw,depth_slopes=old),queries);index = np.searchsorted(result[0],old_result[0])
        np.testing.assert_array_equal(result[0][index],old_result[0])
        for x,y in zip(result[1:],old_result[1:],strict=True):np.testing.assert_array_equal(x[index],y)
        old_samples += len(old_result[0]);new_samples += len(result[0]);checked += 1
    require(new_samples>old_samples,'extra support uses measured one-sided differences')
    # A discontinuity outside the measured four-corner cell need not supply a
    # slope across that discontinuity; the opposite valid side is sufficient.
    raw = dict(base,depth=base['depth'].copy());raw['depth'][12,11] = 2.2
    a = kernel.prepare([dict(surface={1:raw})])[0]['surface'][1];b = prepare([dict(surface={1:raw})])[0]['surface'][1]
    p = points(np.array([[12.2,12.4]]))
    require(len(previous.sample(dict(raw,depth_slopes=bilateral(raw)),p)[0])==0 and len(kernel.sample(a,p)[0])==1,'opposite valid depth side replaces invalid cross-jump slope')
    for x,y in zip(kernel.sample(a,p),independent_sample(b,p),strict=True):np.testing.assert_allclose(x,y,atol=2e-13,rtol=0)
    # Newly admitted cells retain exact projective derivatives and shared C1
    # edge data, despite a missing sample outside the interpolated cell.
    curved = layer(.5+.0001*(xx-15)**2+.00008*(yy-16)**2+.00001*xx*yy);curved['depth'][11,9] = 0.
    ca = kernel.prepare([dict(surface={1:curved})])[0]['surface'][1];cb = prepare([dict(surface={1:curved})])[0]['surface'][1]
    q = points(np.array([[10.2,11.4],[10.7,11.6]]));r = kernel.sample(ca,q)
    require(len(r[0])==len(q) and len(previous.sample(dict(curved,depth_slopes=bilateral(curved)),q)[0])==0,'finite differences cover newly admitted support')
    for axis in range(3):
        delta = np.zeros(3);delta[axis] = 1e-7
        numeric = (kernel.sample(ca,q+delta)[1]-kernel.sample(ca,q-delta)[1])/2e-7
        np.testing.assert_allclose(numeric,r[2][:,axis],atol=5e-9,rtol=1e-6)
    for axis,edge in ((0,[11.,11.25]),(1,[10.25,11.])):
        uv = np.tile(edge,(3,1));uv[:,axis] += [-1e-7,0,1e-7]
        result = kernel.sample(ca,points(uv));oracle = independent_sample(cb,points(uv));require(len(result[0])==3,'joint support at one-sided cell edges')
        for x,y in zip(result,oracle,strict=True):np.testing.assert_allclose(x,y,atol=2e-13,rtol=0)
        np.testing.assert_allclose(result[1],result[1][1],atol=3e-10,rtol=0)
        np.testing.assert_allclose(result[2],np.broadcast_to(result[2][1],(3,3)),atol=3e-8,rtol=0)
    return dict(inherited_bilateral_controls=inherited,side_availability_fixture_cases=checked,
        bilateral_valid_slopes_and_samples_exactly_preserved=True,authored_old_supported_queries=old_samples,authored_new_supported_queries=new_samples,
        cardinal_jump_uses_valid_other_side=True,no_missing_corner_filling=True,new_support_xyz_finite_difference_axes=3,new_support_c1_edge_axes=2,
        original_geometry_and_normals_preserved=True)


def controls():
    proof = source_proof();interpolation = support_controls()
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
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('one_sided_depth_slopes.py','one_sided_depth_refinement.py','check-one-sided-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
