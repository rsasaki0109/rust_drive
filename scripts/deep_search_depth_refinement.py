"""Same fixed-objective local refiner with at most twenty-one Armijo trials."""
import numpy as np

from sampling_depth_refinement import base,discrete,population,metadata,equation,energy
from long_depth_refinement import snapshot,checkpoint
from projective_depth_common import require


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
