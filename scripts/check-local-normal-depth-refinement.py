#!/usr/bin/env python3
"""Independent full-nine-point SVD normal completion and private objective audit."""
import argparse
import ast
import copy
import importlib.util
from pathlib import Path

import numpy as np

import local_normal_depth_refinement as kernel
from projective_depth_common import ROOT,fresh,require,sha,write


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


parent = load('local_normal_original_controls','check-normal-hermite-depth-refinement.py')
private = load('local_normal_private_verifier','check-normal-hermite-depth-refinement.py')
original_sample = private.independent_sample


def independent_complete(layer):
    depth,xyz = layer['depth'],layer['vertices'];h,w = depth.shape;result = layer['normals'].copy()
    yy,xx = np.mgrid[2:h-2,2:w-2];yy,xx = yy.ravel(),xx.ravel()
    offsets = np.array([(y,x) for y in range(-1,2) for x in range(-1,2)])
    for begin in range(0,len(yy),8192):
        y,x = yy[begin:begin+8192],xx[begin:begin+8192]
        z = depth[y[:,None]+offsets[:,0],x[:,None]+offsets[:,1]];center = depth[y,x]
        valid = np.all((z>.3)&(z<5)&(abs(z-center[:,None])<.05+.01*center[:,None]),axis=1)
        valid &= (center>.3)&(center<5)&(np.sum(result[y,x]**2,axis=1)<=.25)
        y,x = y[valid],x[valid]
        points = xyz[y[:,None]+offsets[:,0],x[:,None]+offsets[:,1]]
        centered = points-points.mean(axis=1)[:,None,:]
        _,s,vh = np.linalg.svd(centered,full_matrices=False)
        good = (s[:,1]**2/9>1e-8)&(s[:,2]**2<=.1*s[:,1]**2)
        normal = vh[:,-1,:];normal *= np.where(np.einsum('ij,ij->i',normal,xyz[y,x])<0,-1.,1.)[:,None]
        result[y[good],x[good]] = normal[good]
    return result


def prepare(nodes):
    return [dict(node,surface={s:dict(layer,hermite_normals=independent_complete(layer)) for s,layer in node['surface'].items()}) for node in nodes]


def independent_sample(layer,points):
    require('hermite_normals' in layer,'prepared sampling-normal field required')
    return original_sample(dict(layer,normals=layer['hermite_normals']),points)


private.independent_sample = independent_sample
population,values = private.population,private.values
check_metadata,system,objective = private.check_metadata,private.system,private.objective
verify,verify_checkpoint,reference = private.verify,private.verify_checkpoint,private.reference


def source_proof():
    def body(file,name):
        text = (ROOT/'scripts'/file).read_text();node = next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef) and n.name==name)
        return ast.get_source_segment(text,node)
    for name in ('population','values','evaluate','equation','energy','search','refine','snapshot','checkpoint'):
        require(body('local_normal_depth_refinement.py',name)==body('normal_hermite_depth_refinement.py',name),'unchanged normal-Hermite solver '+name)
    require(verify.__globals__['independent_sample'] is independent_sample and private.fixed.values is values,'private oracle sampler binding')
    require(parent.independent_sample is not independent_sample,'old sampler remains isolated')
    require(kernel.base is parent.kernel.base and kernel.previous is parent.kernel.previous,'original geometry, priors and initial population')
    return True


def compare_prepared(nodes,other):
    counts = []
    for node,oracle in zip(nodes,other,strict=True):
        for scale,layer in node['surface'].items():
            old,new = layer['normals'],layer['hermite_normals'];independent = oracle['surface'][scale]['hermite_normals']
            original_valid = np.linalg.norm(old,axis=2)>.5
            np.testing.assert_array_equal(new[original_valid],old[original_valid])
            np.testing.assert_array_equal(np.linalg.norm(new,axis=2)>.5,np.linalg.norm(independent,axis=2)>.5)
            np.testing.assert_allclose(new,independent,atol=2e-10,rtol=0)
            added = (~original_valid)&(np.linalg.norm(new,axis=2)>.5)
            counts.append(dict(timestamp=node.get('timestamp'),scale=scale,original=int(original_valid.sum()),added=int(added.sum()),original_normals_preserved=True))
    return counts




def completion_controls():
    c = dict(width=32,height=32,fx=160.,fy=160.,cx=15.5,cy=15.5);yy,xx = np.indices((32,32))
    patch = np.full((32,32),2.);patch[12,18] = 2.06
    slope = 1/(.5+.0001*xx+.00015*yy)
    hole = np.full((32,32),2.);hole[12,16] = 0.
    rough = 2.+.04*((xx+yy)%2)
    fixtures = dict(plane=np.full((32,32),2.),sloped=slope,outer_outlier=patch,missing_center=hole,rough=rough)
    comparisons = 0
    for name,depth in fixtures.items():
        audit = load('local_normal_fixture_'+name,'check-reciprocal-keyframes.py').auditor()
        raw = kernel.base.pyramid(depth,c);oracle = audit.pyramid(depth,c)
        actual = kernel.prepare([dict(surface=raw)]);expected = prepare([dict(surface=oracle)])
        compare_prepared(actual,expected);comparisons += len(raw)
        for scale,layer in raw.items():
            completed = actual[0]['surface'][scale]
            for key in ('depth','vertices','normals'):
                require(completed[key] is layer[key],'completion does not mutate or replace original '+key)
            new = completed['hermite_normals']
            np.testing.assert_array_equal(new[:2],layer['normals'][:2]);np.testing.assert_array_equal(new[-2:],layer['normals'][-2:])
            np.testing.assert_array_equal(new[:,:2],layer['normals'][:,:2]);np.testing.assert_array_equal(new[:,-2:],layer['normals'][:,-2:])
        layer = actual[0]['surface'][1]
        if name=='outer_outlier':
            require(np.linalg.norm(layer['normals'][12,16])==0 and np.linalg.norm(layer['hermite_normals'][12,16])>.5,'fully measured local plane repairs wider nonplanarity')
            np.testing.assert_allclose(layer['hermite_normals'][12,16],[0,0,1.],atol=1e-12,rtol=0)
            uv = np.array([[15.75,12.25]]);p = np.column_stack([(uv[:,0]-c['cx'])*2/c['fx'],(uv[:,1]-c['cy'])*2/c['fy'],np.full(1,2.)])
            require(len(parent.kernel.sample(layer,p)[0])==0 and len(kernel.sample(layer,p)[0])==1,'sampler uses repaired normal without filling depth')
            for a,b in zip(kernel.sample(layer,p),independent_sample(expected[0]['surface'][1],p),strict=True):np.testing.assert_allclose(a,b,atol=2e-12,rtol=0)
            bad = copy.deepcopy(actual);bad[0]['surface'][1]['hermite_normals'][12,16] = 0.
            try:compare_prepared(bad,expected)
            except (ValueError,AssertionError):pass
            else:raise ValueError('undetected changed completed-normal mask')
        elif name=='missing_center':require(not layer['hermite_normals'][12,16].any(),'missing center cannot get a normal')
        elif name=='rough':require(not layer['hermite_normals'][12,16].any(),'small nonplanar patch still rejected')
    # Isolated missing-normal cache fixtures exercise the strict all-nine rule
    # and reject rank-one geometry independently of the original 5x5 cache.
    layer = kernel.base.pyramid(np.full((32,32),2.),c)[1];layer = dict(layer,normals=np.zeros_like(layer['normals']))
    import local_normal_support as frontend
    for count in (9,8):
        altered = dict(layer,depth=layer['depth'].copy())
        if count==8:altered['depth'][11,15] = 0.
        a,b = frontend.complete(altered),independent_complete(altered)
        np.testing.assert_allclose(a,b,atol=2e-12,rtol=0)
        require((np.linalg.norm(a[12,16])>.5)==(count==9),'all nine measured points required')
    xyz = layer['vertices'].copy();xyz[:,:,1] = 0.;line = dict(layer,vertices=xyz)
    require(not frontend.complete(line)[12,16].any() and not independent_complete(line)[12,16].any(),'rank-one patch rejected')
    try:kernel.sample(layer,np.array([[0.,0.,2.]]))
    except ValueError as error:require(str(error)=='prepared sampling-normal field required','missing cache rejects explicitly')
    else:raise ValueError('unprepared field silently accepted')
    return dict(normal_layers_compared=comparisons,original_normals_depth_vertices_and_border_preserved=True,
        local_measured_plane_repair=True,sampler_support_repaired=True,strict_nine_vs_eight_control=True,
        missing_center_roughness_and_rank_rejected=True,unprepared_field_rejected=True,corrupted_completed_mask_rejected=True)


def controls():
    proof = source_proof();interpolation = parent.interpolation_controls();normal_controls = completion_controls()
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
    return dict(analytic_pass=True,source_proof=proof,interpolation=interpolation,normal_completion=normal_controls,rendered_continuous_converged=report['continuous_converged'],
        rendered_applied_corrections=report['applied_iterations'],rendered_guard=report['guard_failure'],reported_systems=checked,checkpoint_systems=endpoints,
        prior_system_checks=1,objective_gradient_axes=12,original_rendered_sources=old,hermite_rendered_sources=new,
        degenerate_guards=guards,rejected_corruptions=rejected,input_roots_unchanged=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args();out = fresh(args.output);report = controls()
    report['source_sha256'] = {'scripts/'+n:sha((ROOT/'scripts'/n).read_bytes()) for n in ('local_normal_support.py','local_normal_depth_refinement.py','check-local-normal-depth-refinement.py')}
    write(out/'summary.json',report)


if __name__ == '__main__':main()
