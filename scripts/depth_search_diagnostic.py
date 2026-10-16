"""Read-only directional probes of a fixed inverse-depth objective."""
import math

import numpy as np

import sampling_depth_refinement as kernel
from projective_depth_common import require

EXPONENTS = tuple(range(21))


def moved(poses,direction,alpha):
    result = np.asarray(poses).copy()
    for node,delta in enumerate(direction,1):
        inc = np.eye(4)
        inc[:3,:3] = kernel.discrete.exp_rotation(alpha*delta[3:])
        inc[:3,3] = alpha*delta[:3]
        result[node] = inc@poses[node]
        kernel.base.valid_pose(result[node])
    return result


def pixels(item,nodes,poses,scale):
    a,b = item['target'],item['source']
    root = item['points']@poses[b][:3,:3].T+poses[b][:3,3]
    q = (root-poses[a][:3,3])@poses[a][:3,:3]
    c = nodes[a]['surface'][scale]['camera']
    uv = np.column_stack([c['fx']*q[:,0]/q[:,2]+c['cx'],c['fy']*q[:,1]/q[:,2]+c['cy']])
    snapped = np.where(abs(uv-np.rint(uv)) <= 1e-10,np.rint(uv),uv)
    return uv,np.floor(snapped).astype(np.int64)


def huber_difference(before,after):
    """Subtract per-source losses before summing; avoid total-cost cancellation."""
    a,b = abs(before),abs(after)
    both_quadratic = (a <= .01)&(b <= .01)
    both_linear = (a > .01)&(b > .01)
    result = np.where(b <= .01,.5*after**2,.01*(b-.005))-np.where(a <= .01,.5*before**2,.01*(a-.005))
    result[both_quadratic] = .5*(after[both_quadratic]-before[both_quadratic])*(after[both_quadratic]+before[both_quadratic])
    result[both_linear] = .01*(b[both_linear]-a[both_linear])
    return result


def probe(matches,nodes,poses,prior,scale,direction,g,alpha):
    require(np.isfinite(alpha) and alpha != 0,'nonzero finite probe multiplier')
    candidate = moved(poses,direction,alpha)
    before = kernel.energy(matches,nodes,poses,prior,scale)
    slope = float(g@direction.ravel())
    limit_without_slack = before+.0001*alpha*slope
    slack = 1e-12*max(1.,abs(before))
    result = dict(alpha=alpha,cost=None,cost_delta=None,paired_cost_delta=None,
        secant=None,guard_failure=None,endpoint_cell_changes=None,max_pixel_displacement=None,
        changed_cell_residual_delta=None,same_cell_residual_delta=None,prior_delta=None,
        armijo_without_slack=False,armijo_with_slack=False)
    try:
        cost = kernel.energy(matches,nodes,candidate,prior,scale)
        changed_terms = [];same_terms = [];changed = 0;maximum = 0.
        for item in matches:
            r0,_ = kernel.evaluate(item,nodes,poses,scale)
            r1,_ = kernel.evaluate(item,nodes,candidate,scale)
            uv0,cell0 = pixels(item,nodes,poses,scale);uv1,cell1 = pixels(item,nodes,candidate,scale)
            different = np.any(cell0 != cell1,axis=1)
            terms = huber_difference(r0,r1)
            changed += int(different.sum());maximum = max(maximum,float(np.max(abs(uv1-uv0))))
            changed_terms.extend(terms[different]);same_terms.extend(terms[~different])
        prior_delta = kernel.energy([],nodes,candidate,prior,scale)-kernel.energy([],nodes,poses,prior,scale)
        changed_delta,same_delta = math.fsum(changed_terms),math.fsum(same_terms)
        paired = math.fsum((changed_delta,same_delta,prior_delta))
        result.update(cost=cost,cost_delta=cost-before,paired_cost_delta=paired,secant=paired/alpha,
            endpoint_cell_changes=changed,max_pixel_displacement=maximum,
            changed_cell_residual_delta=changed_delta,same_cell_residual_delta=same_delta,prior_delta=prior_delta,
            armijo_without_slack=bool(cost <= limit_without_slack),armijo_with_slack=bool(cost <= limit_without_slack+slack))
    except ValueError as error:
        result['guard_failure'] = str(error)
    return result


def sweep(matches,nodes,poses,prior,scale,direction,g):
    rows = []
    for exponent in EXPONENTS:
        alpha = 2.**(-exponent)
        plus = probe(matches,nodes,poses,prior,scale,direction,g,alpha)
        minus = probe(matches,nodes,poses,prior,scale,direction,g,-alpha)
        central = None if plus['cost'] is None or minus['cost'] is None else (plus['paired_cost_delta']-minus['paired_cost_delta'])/(2*alpha)
        rows.append(dict(exponent=exponent,positive=plus,negative=minus,central_secant=central))
    return rows
