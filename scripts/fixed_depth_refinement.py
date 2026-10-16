"""Bounded nonlinear refinement of one frozen reciprocal depth-pair population."""
import numpy as np

import reciprocal_keyframe_depth as base
from depth_correspondence_stability import select
from frozen_depth_objective import energy
from projective_depth_common import exp_rotation,require
from symmetric_depth_factors import residual


def population(nodes,poses,scale):
    result = []
    for source in range(1,len(nodes)):
        for target in range(source):
            item = select(nodes[target],nodes[source],poses[target],poses[source],scale)
            if item['count'] >= 200:result.append(dict(item,target=target,source=source))
    return result


def metadata(matches):
    return [{k:x[k] for k in ('target','source','count','pixel_sha256')} for x in matches]


def equation(matches,nodes,poses,prior):
    h,g,delta = base.prior_system(prior,nodes,poses)
    edges = [];count = 0;squared = 0.
    history = set() if prior is None else set(prior.consumed)
    for item in matches:
        a,b = item['target'],item['source'];t,s = poses[a],poses[b]
        require((nodes[a]['timestamp'],nodes[b]['timestamp']) not in history,'no already-consumed frozen raw factor')
        pt,nt,ps,ns = item['points']
        r,j = residual(pt@t[:3,:3].T+t[:3,3],nt@t[:3,:3].T,
            ps@s[:3,:3].T+s[:3,3],ns@s[:3,:3].T)
        require(np.isfinite(r).all() and np.isfinite(j).all(),'finite frozen differential')
        w = np.minimum(1.,.01/np.maximum(abs(r),1e-15))
        base.add_factor(h,g,b,a,j.T@(w[:,None]*j),j.T@(w*r))
        n = len(r);ss = float(r@r);count += n;squared += ss
        edges.append(dict(target=a,source=b,count=n,rms_m=float(np.sqrt(ss/n))))
    return dict(h=h,g=g,edges=edges,count=count,rms_m=float(np.sqrt(squared/count)) if count else 0.,prior_delta=delta)


def proposal(e,current):
    require(any(x['source'] == current for x in e['edges']),'new frame needs measured support')
    d,condition = base.solve(e['h'],e['g']);d = d.reshape(-1,6)
    require(all(np.linalg.norm(x[:3]) <= .1 and np.linalg.norm(x[3:]) <= .15 for x in d),'increment')
    return d,condition


def step_search(matches,nodes,poses,prior,d,g):
    before = energy(matches,nodes,poses,prior);directional = float(g@d.ravel())
    require(np.isfinite(directional) and directional <= 0,'nonascending linear proposal')
    trials = []
    for exponent in range(8):
        alpha = 2.**(-exponent);candidate = [p.copy() for p in poses]
        for node,delta in enumerate(d,1):
            inc = np.eye(4);inc[:3,:3],inc[:3,3] = exp_rotation(delta[3:]*alpha),delta[:3]*alpha
            candidate[node] = inc@poses[node];base.valid_pose(candidate[node])
        cost = energy(matches,nodes,candidate,prior)
        limit = before+.0001*alpha*directional+1e-12*max(1.,abs(before))
        trials.append(dict(alpha=alpha,cost=cost,limit=limit))
        if cost <= limit:
            return candidate,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=alpha)
    raise ValueError('line search')


def small(d):
    return all(np.linalg.norm(x[:3]) < 1e-5 and np.linalg.norm(x[3:]) < 1e-5 for x in d)


def correction(e,current):
    try:d,condition = proposal(e,current)
    except ValueError as error:return dict(guard_failure=str(error),converged=False,delta=None,condition=None)
    return dict(guard_failure=None,converged=small(d),delta=d.tolist(),condition=condition)


def refine(nodes,initial,prior,scale=1):
    """At most ten corrections, no re-association, and no tracker state commit."""
    require(2 <= len(nodes) <= 5 and scale in (4,2,1),'bounded frozen-refinement window/scale')
    poses = [np.asarray(p).copy() for p in initial]
    require(len(poses) == len(nodes),'one pose per frozen node')
    for pose in poses:base.valid_pose(pose)
    require(np.array_equal(poses[0],np.eye(4)),'fixed first optical origin')
    require(prior is None or nodes[-1]['timestamp'] not in prior.ids,'no prior on current acquisition')
    matches = population(nodes,poses,scale);trace = [];failure = None;converged = False
    initial_cost = energy(matches,nodes,poses,prior)
    for iteration in range(11):
        e = equation(matches,nodes,poses,prior)
        try:d,condition = proposal(e,len(nodes)-1)
        except ValueError as error:failure = str(error);break
        converged = small(d)
        if converged or iteration == 10:break
        before = [p.tolist() for p in poses]
        try:after,search = step_search(matches,nodes,poses,prior,d,e['g'])
        except ValueError as error:failure = str(error);break
        trace.append(dict(before=before,equation=base.encode_equation(e),delta=d.tolist(),condition=condition,
            after=[p.tolist() for p in after],line_search=search))
        poses = after
    fresh = base.equation(nodes,poses,scale,prior)
    return dict(scale=scale,cap=10,initial_poses=[np.asarray(p).tolist() for p in initial],
        frozen_population=metadata(matches),initial_fixed_cost=initial_cost,
        final_fixed_cost=energy(matches,nodes,poses,prior),final_fresh_cost=energy(population(nodes,poses,scale),nodes,poses,prior),
        trace=trace,applied_iterations=len(trace),fixed_converged=converged,guard_failure=failure,
        final_poses=[p.tolist() for p in poses],final_fixed_equation=base.encode_equation(e),
        final_fixed_correction=correction(e,len(nodes)-1),final_fresh_equation=base.encode_equation(fresh),
        final_fresh_correction=correction(fresh,len(nodes)-1))
