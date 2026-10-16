"""Nine initial cubic-domain probes bound to a private unchanged local solver."""
import importlib.util

import numpy as np

import cubic_depth_refinement as previous
from projective_depth_common import ROOT

spec = importlib.util.spec_from_file_location('cubic_margin_private_solver',ROOT/'scripts/cubic_depth_refinement.py')
private = importlib.util.module_from_spec(spec);spec.loader.exec_module(private)
center_population = private.population
base,discrete,metadata,sample = private.base,private.discrete,private.metadata,private.sample


def sampling_mask(item,nodes,poses,scale):
    a,b = item['target'],item['source'];t,s = poses[a],poses[b]
    root = item['points']@s[:3,:3].T+s[:3,3];local = (root-t[:3,3])@t[:3,:3]
    camera = nodes[a]['surface'][scale]['camera'];keep = np.ones(len(local),dtype=bool)
    for dv in (-1,0,1):
        for du in (-1,0,1):
            q = local.copy();q[:,0] += du*q[:,2]/camera['fx'];q[:,1] += dv*q[:,2]/camera['fy']
            ids,_,_,_ = sample(nodes[a]['surface'][scale],q)
            mask = np.zeros(len(local),dtype=bool);mask[ids] = True;keep &= mask
    return keep


def population(nodes,poses,scale):
    result = []
    for item in center_population(nodes,poses,scale):
        keep = sampling_mask(item,nodes,poses,scale)
        if np.count_nonzero(keep)>=200:
            result.append(dict(item,**{key:item[key][keep] for key in ('points','normals','source_ids','normalizer')}))
    return result


private.population = population
values,evaluate,equation,energy,search,refine,snapshot,checkpoint = (
    private.values,private.evaluate,private.equation,private.energy,private.search,private.refine,private.snapshot,private.checkpoint)
