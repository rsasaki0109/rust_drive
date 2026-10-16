"""Up to ninety corrections on the unchanged one-sided depth objective."""
import numpy as np

import one_sided_depth_refinement as previous
from one_sided_depth_refinement import (base,discrete,metadata,population,values,evaluate,equation,energy,search,prepare,sample)
from long_depth_refinement import stop
from projective_depth_common import require


def refine(nodes,initial,prior,scale=1,cap=90):
    require(type(cap) is int and cap in (10,20,30,60,90),'fixed diagnostic correction cap')
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
    require(type(cap) is int and cap in (10,20,30,60,90) and cap <= report['cap'],'bounded checkpoint cap')
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
