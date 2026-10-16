"""Fixed-source, continuously sampled inverse-depth local refinement."""
import numpy as np

import fixed_depth_refinement as discrete
from projective_depth_common import require,sha
from subpixel_depth_factors import sample

base = discrete.base


def values(target,source,poses,a,b,points,normals,scale):
    t,s = poses[a],poses[b]
    root = points@s[:3,:3].T+s[:3,3]
    local = (root-t[:3,3])@t[:3,:3]
    ids,f,df,point = sample(target['surface'][scale],local)
    length = np.linalg.norm(df,axis=1)
    require(np.isfinite(length).all() and np.all(length > 1e-12),'finite continuous gradient')
    normal = df/length[:,None]@t[:3,:3].T
    source_normal = normals@s[:3,:3].T
    ok = ((np.linalg.norm(local[ids]-point,axis=1) < (.12 if scale == 4 else .08))
        &(np.sum(source_normal[ids]*normal,axis=1) > .8))
    return ids[ok],f[ok],df[ok],length[ok],root[ids[ok]]


def population(nodes,poses,scale):
    result = []
    for item in discrete.population(nodes,poses,scale):
        a,b = item['target'],item['source'];points,normals = item['points'][2:]
        ids,_,_,length,_ = values(nodes[a],nodes[b],poses,a,b,points,normals,scale)
        if len(ids) >= 200:
            result.append(dict(target=a,source=b,points=points[ids],normals=normals[ids],
                source_ids=item['pixel_ids'][ids,1],normalizer=length))
    return result


def metadata(matches):
    return [dict(target=x['target'],source=x['source'],count=len(x['source_ids']),
        source_pixel_sha256=sha(np.asarray(x['source_ids'],dtype='<i8').tobytes()),
        normalizer=x['normalizer'].tolist(),
        normalizer_sha256=sha(np.asarray(x['normalizer'],dtype='<f8').tobytes())) for x in matches]


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
    for exponent in range(8):
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
