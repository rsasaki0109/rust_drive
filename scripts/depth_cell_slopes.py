"""Read-only path derivatives and endpoint-cell partitions of fixed sources."""
import math

import numpy as np

import depth_search_diagnostic as path
from projective_depth_common import require,sha

kernel = path.kernel
POWERS = tuple(range(-4,5))


def multipliers(scale):
    require(np.isfinite(scale) and scale > 0,'positive archived probe scale')
    return sorted([0.]+[sign*scale*2.**power for sign in (-1,1) for power in POWERS])


def tangent(direction,alpha):
    result = np.asarray(direction).copy()
    result[:,:3] -= alpha*np.cross(direction[:,3:],direction[:,:3])
    return np.vstack([np.zeros(6),result])


def measure(matches,nodes,poses,prior,direction,alpha):
    candidate = path.moved(poses,direction,alpha);velocity = tangent(direction,alpha)
    pixels = [path.pixels(item,nodes,candidate,1) for item in matches]
    result = dict(alpha=alpha,cost=None,slope=None,prior_slope=None,prior_cost=None,
        guard_failure=None,uv=[p[0] for p in pixels],cells=[p[1] for p in pixels],terms=None,residuals=None)
    try:
        cost = kernel.energy(matches,nodes,candidate,prior,1)
        prior_gradient = kernel.base.prior_system(prior,nodes,candidate)[1]
        prior_slope = float(prior_gradient@velocity[1:].ravel())
        residuals = [];terms = []
        for item in matches:
            r,j = kernel.evaluate(item,nodes,candidate,1)
            dr = j@(velocity[item['source']]-velocity[item['target']])
            residuals.append(r);terms.append(np.clip(r,-.01,.01)*dr)
        slope = math.fsum([prior_slope]+[float(t) for row in terms for t in row])
        result.update(cost=cost,slope=slope,prior_slope=prior_slope,prior_cost=kernel.energy([],nodes,candidate,prior,1),
            terms=terms,residuals=residuals)
    except ValueError as error:result['guard_failure'] = str(error)
    return result


def paired_delta(before,after):
    if before['cost'] is None or after['cost'] is None:return None
    terms = [float(t) for r0,r1 in zip(before['residuals'],after['residuals'],strict=True) for t in path.huber_difference(r0,r1)]
    return math.fsum(terms+[after['prior_cost']-before['prior_cost']])


def partition(matches,before,after):
    changed = [];same_terms = [];changed_terms = []
    for i,item in enumerate(matches):
        different = np.any(before['cells'][i] != after['cells'][i],axis=1)
        ids = np.asarray(item['source_ids'][different],dtype='<i8')
        changed.append(dict(target=item['target'],source=item['source'],source_pixel_ids=ids.tolist(),source_pixel_sha256=sha(ids.tobytes())))
        if before['terms'] is not None and after['terms'] is not None:
            delta = after['terms'][i]-before['terms'][i]
            changed_terms.extend(delta[different]);same_terms.extend(delta[~different])
    result = dict(before_alpha=before['alpha'],after_alpha=after['alpha'],
        endpoint_changed_source_occurrences=sum(len(row['source_pixel_ids']) for row in changed),changed_sources=changed,
        changed_cell_slope_delta=None,same_cell_slope_delta=None,prior_slope_delta=None,total_slope_delta=None)
    if before['slope'] is not None and after['slope'] is not None:
        a,b = math.fsum(changed_terms),math.fsum(same_terms);p = after['prior_slope']-before['prior_slope']
        result.update(changed_cell_slope_delta=a,same_cell_slope_delta=b,prior_slope_delta=p,total_slope_delta=math.fsum((a,b,p)))
    return result


def encode(item,baseline):
    distances = [abs(uv-np.rint(uv)) for uv in item['uv']]
    return {k:item[k] for k in ('alpha','cost','slope','prior_slope','guard_failure')} | dict(
        cost_delta=None if item['cost'] is None else item['cost']-baseline['cost'],paired_cost_delta=paired_delta(baseline,item),
        snapped_coordinate_occurrences=sum(int((d <= 1e-10).sum()) for d in distances),
        minimum_raw_grid_distance_pixels=min(float(d.min()) for d in distances))
