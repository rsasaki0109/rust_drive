"""Diagnostic extension through three unchanged ten-correction local segments."""
import numpy as np

import alternating_depth_refinement as local
import depth_correspondence_stability as turnover
from frozen_depth_objective import energy


def endpoint(report,nodes,prior):
    """Last batch's pixel turnover and remaining fresh full proposal."""
    batch = report['batches'][-1]
    before,after = np.asarray(batch['initial_poses']),np.asarray(batch['final_poses'])
    edges = []
    for source in range(1,len(nodes)):
        for target in range(source):
            old = turnover.select(nodes[target],nodes[source],before[target],before[source],report['scale'])
            new = turnover.select(nodes[target],nodes[source],after[target],after[source],report['scale'])
            item = turnover.transition(old,new,before[target],before[source],after[target],after[source])
            edges.append(dict(item,target=target,source=source))
    totals = turnover.aggregate(edges,energy([],nodes,before,prior),energy([],nodes,after,prior))
    correction = report['final_fresh_correction']
    delta = None if correction['delta'] is None else np.asarray(correction['delta'])
    return dict(edges=edges,totals=totals,
        max_translation_proposal_m=None if delta is None else float(np.linalg.norm(delta[:,:3],axis=1).max()),
        max_rotation_proposal_rad=None if delta is None else float(np.linalg.norm(delta[:,3:],axis=1).max()))


def refine(nodes,initial,prior,scale=1):
    """At most thirty corrections; never continue after convergence or a guard."""
    poses = np.asarray(initial).copy();segments = [];endpoints = []
    for _ in range(3):
        report = local.refine(nodes,poses,prior,scale)
        segments.append(report);endpoints.append(endpoint(report,nodes,prior))
        poses = np.asarray(report['final_poses'])
        if report['termination'] != 'budget':break
    final = segments[-1]
    return dict(total_cap=30,segment_cap=10,scale=scale,initial_poses=np.asarray(initial).tolist(),
        segments=segments,endpoints=endpoints,applied_iterations=sum(s['applied_iterations'] for s in segments),
        association_rounds=sum(s['association_rounds'] for s in segments),
        fresh_converged=final['fresh_converged'],guard_failure=final['guard_failure'],termination=final['termination'],
        final_poses=final['final_poses'],final_fresh_equation=final['final_fresh_equation'],
        final_fresh_correction=final['final_fresh_correction'],final_fresh_cost=final['final_fresh_cost'])
