"""Private unchanged fixed-source solver with conservative cell-boundary support."""
import importlib.util

import numpy as np

import slope_admission_refinement as previous
from boundary_depth_support import prepare,sample
from projective_depth_common import ROOT,require


def load(name,file):
    spec = importlib.util.spec_from_file_location(name,ROOT/'scripts'/file)
    module = importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module


local = load('boundary_private_objective','one_sided_depth_refinement.py')
driver = load('boundary_private_driver','extended_slope_refinement.py')
base,discrete,metadata = previous.base,previous.discrete,previous.metadata
center = previous.center


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


local.sample = sample
for name in ('values','evaluate','equation','energy','search'):setattr(driver,name,getattr(local,name))
driver.population = population
values,evaluate,equation,energy,search = local.values,local.evaluate,local.equation,local.energy,local.search
refine,snapshot,checkpoint = driver.refine,driver.snapshot,driver.checkpoint
