"""Initial nine-probe qualification for fixed-source continuous local refinement."""
import numpy as np

import continuous_depth_refinement as parent
from continuous_depth_refinement import base,discrete,equation,energy,evaluate,metadata,search
from projective_depth_common import require
from subpixel_depth_factors import sample


def margin_mask(item,nodes,poses,scale):
    """Test center and eight one-pixel offsets once, before freezing sources."""
    a,b = item['target'],item['source'];t,s = poses[a],poses[b]
    root = item['points']@s[:3,:3].T+s[:3,3]
    local = (root-t[:3,3])@t[:3,:3]
    normal = item['normals']@s[:3,:3].T
    camera = nodes[a]['surface'][scale]['camera'];keep = np.ones(len(local),dtype=bool)
    for dv in (-1,0,1):
        for du in (-1,0,1):
            query = local.copy();query[:,0] += du*query[:,2]/camera['fx'];query[:,1] += dv*query[:,2]/camera['fy']
            ids,_,df,point = sample(nodes[a]['surface'][scale],query)
            length = np.linalg.norm(df,axis=1)
            require(np.isfinite(length).all() and np.all(length > 1e-12),'finite margin gradient')
            gradient = df/length[:,None]@t[:3,:3].T
            qualified = ((np.linalg.norm(query[ids]-point,axis=1) < (.12 if scale == 4 else .08))
                &(np.sum(normal[ids]*gradient,axis=1) > .8))
            mask = np.zeros(len(local),dtype=bool);mask[ids[qualified]] = True;keep &= mask
    return keep


def population(nodes,poses,scale):
    result = []
    for item in parent.population(nodes,poses,scale):
        keep = margin_mask(item,nodes,poses,scale)
        if np.count_nonzero(keep) >= 200:
            result.append(dict(item,**{key:item[key][keep] for key in ('points','normals','source_ids','normalizer')}))
    return result


def refine(nodes,initial,prior,scale=1):
    require(2 <= len(nodes) <= 5 and scale in (4,2,1),'bounded continuous local window')
    poses = [np.asarray(p).copy() for p in initial];require(len(poses) == len(nodes),'one pose per node')
    for p in poses:base.valid_pose(p)
    require(np.array_equal(poses[0],np.eye(4)),'fixed optical origin')
    require(prior is None or nodes[-1]['timestamp'] not in prior.ids,'no prior on new acquisition')
    matches = population(nodes,poses,scale);initial_cost = energy(matches,nodes,poses,prior,scale)
    trace = [];failure = None;converged = False;failed_search = None
    for iteration in range(11):
        e = equation(matches,nodes,poses,prior,scale)
        try:d,condition = discrete.proposal(e,len(nodes)-1)
        except ValueError as error:failure = str(error);break
        converged = discrete.small(d)
        if converged or iteration == 10:break
        after,receipt = search(matches,nodes,poses,prior,scale,d,e['g'])
        if after is None:failure = 'line search';failed_search = receipt;break
        trace.append(dict(before=[p.tolist() for p in poses],equation=base.encode_equation(e),delta=d.tolist(),
            condition=condition,after=[p.tolist() for p in after],line_search=receipt))
        poses = after
    fresh = base.equation(nodes,poses,scale,prior)
    return dict(scale=scale,cap=10,initial_poses=np.asarray(initial).tolist(),frozen_population=metadata(matches),
        initial_cost=initial_cost,trace=trace,applied_iterations=len(trace),continuous_converged=converged,
        guard_failure=failure,failed_search=failed_search,
        termination='guard' if failure is not None else ('converged' if converged else 'budget'),
        final_poses=[p.tolist() for p in poses],final_equation=base.encode_equation(e),
        final_correction=discrete.correction(e,len(nodes)-1),final_cost=energy(matches,nodes,poses,prior,scale),
        final_discrete_equation=base.encode_equation(fresh),final_discrete_correction=discrete.correction(fresh,len(nodes)-1))
