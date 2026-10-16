"""Initial probes use the same depth field as the unchanged fixed-source fit."""
import importlib.util

import numpy as np

import continuous_depth_refinement as center
import extended_slope_refinement as previous
import sampling_depth_refinement as legacy
from projective_depth_common import ROOT,require

spec = importlib.util.spec_from_file_location('slope_admission_private_solver',ROOT/'scripts/extended_slope_refinement.py')
private = importlib.util.module_from_spec(spec);spec.loader.exec_module(private)
base,discrete,metadata,sample,prepare = private.base,private.discrete,private.metadata,private.sample,private.prepare


def sampling_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];t,s = poses[a],poses[b]
    root = item['points']@s[:3,:3].T+s[:3,3];local = (root-t[:3,3])@t[:3,:3]
    camera = nodes[a]['surface'][scale]['camera'];keep = np.ones(len(local),dtype=bool)
    for dv in (-1,0,1):
        for du in (-1,0,1):
            q = local.copy();q[:,0] += du*q[:,2]/camera['fx'];q[:,1] += dv*q[:,2]/camera['fy']
            ids,_,df,_ = sample(nodes[a]['surface'][scale],q)
            length = np.linalg.norm(df,axis=1)
            require(np.isfinite(length).all() and np.all(length > 1e-12),'finite sampling gradient')
            mask = np.zeros(len(local),dtype=bool);mask[ids] = True;keep &= mask
    return keep


def population(nodes,poses,scale):
    rows = []
    for item in center.population(nodes,poses,scale):
        keep = sampling_mask(item,nodes,poses,scale)
        if np.count_nonzero(keep) >= 200:
            rows.append(dict(item,**{key:item[key][keep] for key in ('points','normals','source_ids','normalizer')}))
    return rows


def admission_counts(nodes,poses,scale):
    result = []
    for item in discrete.population(nodes,poses,scale):
        a,b = item['target'],item['source'];points,normals = item['points'][2:]
        ids,_,_,size,_ = center.values(nodes[a],nodes[b],poses,a,b,points,normals,scale)
        row = dict(target=a,source=b,points=points[ids],normals=normals[ids],source_ids=item['pixel_ids'][ids,1],normalizer=size)
        old,new = legacy.sampling_mask(row,nodes,poses,scale),sampling_mask(row,nodes,poses,scale)
        result.append(dict(target=a,source=b,current_source=b==len(nodes)-1,raw_count=item['count'],
            bilinear_geometric_center=len(ids),old_bilinear_probe_count=int(old.sum()),new_model_probe_count=int(new.sum())))
    return result


private.population = population
values,evaluate,equation,energy,search,refine,snapshot,checkpoint = (
    private.values,private.evaluate,private.equation,private.energy,private.search,private.refine,private.snapshot,private.checkpoint)
