"""Bounded alternating reciprocal association and nonlinear fixed-pair updates."""
import numpy as np
from fixed_depth_refinement import (base,population,metadata,equation,proposal,step_search,small,correction)
from frozen_depth_objective import energy
from projective_depth_common import require


def batch(nodes,initial,prior,scale=1,cap=3):
    """One bounded frozen batch, no re-association or tracker state commit."""
    require(isinstance(cap,int) and not isinstance(cap,bool) and 1 <= cap <= 3,'one to three corrections per batch')
    require(2 <= len(nodes) <= 5 and scale in (4,2,1),'bounded frozen-refinement window/scale')
    poses = [np.asarray(p).copy() for p in initial]
    require(len(poses) == len(nodes),'one pose per frozen node')
    for pose in poses:base.valid_pose(pose)
    require(np.array_equal(poses[0],np.eye(4)),'fixed first optical origin')
    require(prior is None or nodes[-1]['timestamp'] not in prior.ids,'no prior on current acquisition')
    matches = population(nodes,poses,scale);trace = [];failure = None;converged = False
    initial_cost = energy(matches,nodes,poses,prior)
    for iteration in range(cap+1):
        e = equation(matches,nodes,poses,prior)
        try:d,condition = proposal(e,len(nodes)-1)
        except ValueError as error:failure = str(error);break
        converged = small(d)
        if converged or iteration == cap:break
        before = [p.tolist() for p in poses]
        try:after,search = step_search(matches,nodes,poses,prior,d,e['g'])
        except ValueError as error:failure = str(error);break
        trace.append(dict(before=before,equation=base.encode_equation(e),delta=d.tolist(),condition=condition,
            after=[p.tolist() for p in after],line_search=search))
        poses = after
    fresh = base.equation(nodes,poses,scale,prior)
    return dict(scale=scale,cap=cap,initial_poses=[np.asarray(p).tolist() for p in initial],
        frozen_population=metadata(matches),initial_fixed_cost=initial_cost,
        final_fixed_cost=energy(matches,nodes,poses,prior),final_fresh_cost=energy(population(nodes,poses,scale),nodes,poses,prior),
        trace=trace,applied_iterations=len(trace),fixed_converged=converged,guard_failure=failure,
        final_poses=[p.tolist() for p in poses],final_fixed_equation=base.encode_equation(e),
        final_fixed_correction=correction(e,len(nodes)-1),final_fresh_equation=base.encode_equation(fresh),
        final_fresh_correction=correction(fresh,len(nodes)-1))


def refine(nodes,initial,prior,scale=1):
    """At most ten applied corrections, refreshing after up to three at a time."""
    poses = [np.asarray(p).copy() for p in initial]
    batches = [];applied = 0;failure = None;converged = False
    while applied < 10:
        report = batch(nodes,poses,prior,scale,cap=min(3,10-applied))
        batches.append(report);applied += report['applied_iterations']
        poses = [np.asarray(p) for p in report['final_poses']]
        failure = report['guard_failure'] or report['final_fresh_correction']['guard_failure']
        if failure is not None:break
        converged = report['final_fresh_correction']['converged']
        if converged:break
        if report['applied_iterations'] == 0:
            failure = 'no progress';break
    require(bool(batches) and applied <= 10 and len(batches) <= 10,'bounded nonempty alternating schedule')
    final = batches[-1]
    return dict(scale=scale,total_cap=10,inner_cap=3,initial_poses=[np.asarray(p).tolist() for p in initial],
        batches=batches,applied_iterations=applied,association_rounds=len(batches),
        fresh_converged=converged,guard_failure=failure,
        termination='guard' if failure is not None else ('converged' if converged else 'budget'),
        final_poses=final['final_poses'],final_fresh_equation=final['final_fresh_equation'],
        final_fresh_correction=final['final_fresh_correction'],final_fresh_cost=final['final_fresh_cost'])
