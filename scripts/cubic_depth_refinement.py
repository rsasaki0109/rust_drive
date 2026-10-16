"""Same bounded local solver on a C1 interpolatory inverse-depth objective."""
import numpy as np

import sampling_depth_refinement as previous
from sampling_depth_refinement import base,discrete,metadata
from long_depth_refinement import stop
from cubic_depth_factors import sample
from projective_depth_common import require


def population(nodes,poses,scale):
    result = []
    for item in previous.population(nodes,poses,scale):
        ids,_,_,_,_ = values(nodes[item['target']],nodes[item['source']],poses,item['target'],item['source'],item['points'],item['normals'],scale)
        if len(ids) >= 200:
            result.append(dict(item,**{key:item[key][ids] for key in ('points','normals','source_ids','normalizer')}))
    return result


def values(target,source,poses,a,b,points,normals,scale):
    t,s = poses[a],poses[b]
    root = points@s[:3,:3].T+s[:3,3]
    local = (root-t[:3,3])@t[:3,:3]
    ids,f,df,_ = sample(target['surface'][scale],local)
    length = np.linalg.norm(df,axis=1)
    require(np.isfinite(length).all() and np.all(length > 1e-12),'finite continuous gradient')
    return ids,f,df,length,root[ids]


def evaluate(item,nodes,poses,scale):
    a,b = item['target'],item['source']
    ids,f,df,_,root = values(nodes[a],nodes[b],poses,a,b,item['points'],item['normals'],scale)
    require(np.array_equal(ids,np.arange(len(item['points']))),'fixed source support')
    gradient = df/item['normalizer'][:,None]@poses[a][:3,:3].T
    r = f/item['normalizer']
    j = np.column_stack([gradient,np.cross(root,gradient)])
    require(np.isfinite(r).all() and np.isfinite(j).all(),'finite continuous residual')
    return r,j


def equation(matches,nodes,poses,prior,scale):
    h,g,delta = base.prior_system(prior,nodes,poses);edges = [];count = 0;squared = 0.
    consumed = set() if prior is None else set(prior.consumed)
    for item in matches:
        a,b = item['target'],item['source']
        require((nodes[a]['timestamp'],nodes[b]['timestamp']) not in consumed,'no consumed continuous raw factor')
        r,j = evaluate(item,nodes,poses,scale);w = np.minimum(1.,.01/np.maximum(abs(r),1e-15))
        base.add_factor(h,g,b,a,j.T@(w[:,None]*j),j.T@(w*r))
        n = len(r);ss = float(r@r);count += n;squared += ss
        edges.append(dict(target=a,source=b,count=n,rms_m=float(np.sqrt(ss/n))))
    return dict(h=h,g=g,edges=edges,count=count,rms_m=float(np.sqrt(squared/count)) if count else 0.,prior_delta=delta)


def energy(matches,nodes,poses,prior,scale):
    cost = discrete.energy([],nodes,poses,prior)
    for item in matches:
        r,_ = evaluate(item,nodes,poses,scale);v = abs(r)
        cost += float(np.where(v <= .01,.5*r*r,.01*(v-.005)).sum())
    require(np.isfinite(cost),'finite continuous objective')
    return cost


def search(matches,nodes,poses,prior,scale,d,g):
    before = energy(matches,nodes,poses,prior,scale);directional = float(g@d.ravel())
    require(np.isfinite(directional) and directional <= 0,'nonascending continuous proposal')
    trials = []
    for exponent in range(21):
        alpha = 2.**(-exponent);candidate = [p.copy() for p in poses]
        for node,delta in enumerate(d,1):
            inc = np.eye(4);inc[:3,:3] = discrete.exp_rotation(delta[3:]*alpha);inc[:3,3] = delta[:3]*alpha
            candidate[node] = inc@poses[node];base.valid_pose(candidate[node])
        limit = before+.0001*alpha*directional+1e-12*max(1.,abs(before))
        try:cost = energy(matches,nodes,candidate,prior,scale);failure = None
        except ValueError as error:cost = None;failure = str(error)
        trials.append(dict(alpha=alpha,cost=cost,limit=limit,guard_failure=failure))
        if failure is None and cost <= limit:
            return candidate,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=alpha)
    return None,dict(before_cost=before,directional_derivative=directional,trials=trials,accepted_alpha=None)


def refine(nodes,initial,prior,scale=1,cap=30):
    require(type(cap) is int and cap in (10,20,30),'fixed diagnostic correction cap')
    require(2 <= len(nodes) <= 5 and scale in (4,2,1),'bounded continuous local window')
    poses = [np.asarray(p).copy() for p in initial];require(len(poses) == len(nodes),'one pose per node')
    for p in poses:base.valid_pose(p)
    require(np.array_equal(poses[0],np.eye(4)),'fixed optical origin')
    require(prior is None or nodes[-1]['timestamp'] not in prior.ids,'no prior on new acquisition')
    matches = population(nodes,poses,scale);initial_cost = energy(matches,nodes,poses,prior,scale)
    trace = [];failure = None;converged = False;failed_search = None
    for iteration in range(cap+1):
        e = equation(matches,nodes,poses,prior,scale)
        try:d,condition = discrete.proposal(e,len(nodes)-1)
        except ValueError as error:failure = str(error);break
        converged = discrete.small(d)
        if converged or iteration == cap:break
        after,receipt = search(matches,nodes,poses,prior,scale,d,e['g'])
        if after is None:failure = 'line search';failed_search = receipt;break
        trace.append(dict(before=[p.tolist() for p in poses],equation=base.encode_equation(e),delta=d.tolist(),
            condition=condition,after=[p.tolist() for p in after],line_search=receipt))
        poses = after
    fresh = base.equation(nodes,poses,scale,prior)
    return dict(scale=scale,cap=cap,initial_poses=np.asarray(initial).tolist(),frozen_population=metadata(matches),
        initial_cost=initial_cost,trace=trace,applied_iterations=len(trace),continuous_converged=converged,
        guard_failure=failure,failed_search=failed_search,
        termination='guard' if failure is not None else ('converged' if converged else 'budget'),
        final_poses=[p.tolist() for p in poses],final_equation=base.encode_equation(e),
        final_correction=discrete.correction(e,len(nodes)-1),final_cost=energy(matches,nodes,poses,prior,scale),
        final_discrete_equation=base.encode_equation(fresh),final_discrete_correction=discrete.correction(fresh,len(nodes)-1))


def snapshot(report,nodes,prior,cap):
    """Reconstruct the exact shorter attempt, without changing initial support."""
    require(type(cap) is int and cap in (10,20,30) and cap <= report['cap'],'bounded checkpoint cap')
    used = min(cap,len(report['trace']))
    poses = np.asarray(report['initial_poses'] if used == 0 else report['trace'][used-1]['after'])
    matches = population(nodes,np.asarray(report['initial_poses']),report['scale'])
    require(metadata(matches) == report['frozen_population'],'checkpoint retains original observations and weights')
    e = equation(matches,nodes,poses,prior,report['scale']);correction = discrete.correction(e,len(nodes)-1)
    termination,failure = stop(correction,used,cap,report['guard_failure'])
    fresh = base.equation(nodes,poses,report['scale'],prior)
    return dict(report,cap=cap,trace=report['trace'][:used],applied_iterations=used,
        continuous_converged=correction['converged'],guard_failure=failure,termination=termination,
        failed_search=report['failed_search'] if failure == 'line search' else None,
        final_poses=poses.tolist(),final_equation=base.encode_equation(e),final_correction=correction,
        final_cost=energy(matches,nodes,poses,prior,report['scale']),
        final_discrete_equation=base.encode_equation(fresh),final_discrete_correction=discrete.correction(fresh,len(nodes)-1))


def checkpoint(report,nodes,prior,cap):
    value = snapshot(report,nodes,prior,cap)
    keys = ('continuous_converged','guard_failure','termination','final_poses','final_equation','final_correction',
        'final_discrete_equation','final_discrete_correction','final_cost')
    return dict(requested_cap=cap,actual_applied_corrections=value['applied_iterations'],
        terminal_carried_forward=value['applied_iterations'] < cap,**{k:value[k] for k in keys})
