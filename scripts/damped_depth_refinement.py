"""Conditional, bounded direction alternatives for a fixed measured objective."""
import numpy as np

import deep_search_depth_refinement as previous
from deep_search_depth_refinement import base,discrete,population,metadata,equation,energy,snapshot,checkpoint
from projective_depth_common import require

DAMPING = (.001,.01,.1,1.,10.,100.,1000.,10000.)


def search(matches,nodes,poses,prior,scale,d,e):
    candidate,gn = previous.search(matches,nodes,poses,prior,scale,d,e['g'])
    before = gn['before_cost'];slack = 1e-12*max(1.,abs(before))
    cost = None if candidate is None else gn['trials'][-1]['cost']
    selected = None if candidate is None else 'gauss_newton'
    applied = None if candidate is None else d.tolist()
    triggered = candidate is None or gn['accepted_alpha'] < 1/16
    trials = []
    if triggered:
        for damping in DAMPING:
            trial = dict(damping=damping,delta=None,condition=None,directional_derivative=None,cost=None,limit=None,guard_failure=None)
            try:
                damped = e['h']+damping*np.diag(np.diag(e['h']))
                direction,condition = base.solve(damped,e['g']);direction = direction.reshape(-1,6)
                trial.update(delta=direction.tolist(),condition=condition)
                require(all(np.linalg.norm(x[:3]) <= .1 and np.linalg.norm(x[3:]) <= .15 for x in direction),'increment')
                slope = float(e['g']@direction.ravel());require(np.isfinite(slope) and slope <= 0,'nonascending damped proposal')
                trial.update(directional_derivative=slope,limit=before+.0001*slope+slack)
                proposed = [p.copy() for p in poses]
                for node,delta in enumerate(direction,1):
                    inc = np.eye(4);inc[:3,:3] = discrete.exp_rotation(delta[3:]);inc[:3,3] = delta[:3]
                    proposed[node] = inc@poses[node];base.valid_pose(proposed[node])
                value = energy(matches,nodes,proposed,prior,scale);trial['cost'] = value
                if value <= trial['limit'] and value < before-slack and (cost is None or value < cost-slack):
                    candidate,cost,selected,applied = proposed,value,len(trials),direction.tolist()
            except ValueError as error:trial['guard_failure'] = str(error)
            trials.append(trial)
    return candidate,dict(gauss_newton=gn,triggered=triggered,damped_trials=trials,selected=selected,
        applied_delta=applied,accepted_cost=cost)


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
        after,receipt = search(matches,nodes,poses,prior,scale,d,e)
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
