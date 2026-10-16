"""Independent raw-view free-space observations and bounded evidence decay."""
import numpy as np

from projective_depth_common import require
from projective_depth_raw import vertices


def observations(model, weights, raw, normals, current_from_model, camera):
    ids = np.flatnonzero(((model > .3)&(model < 5)&(weights > 0)).ravel())
    query = vertices(model, camera).reshape(-1, 3)[ids]@current_from_model[:3, :3].T+current_from_model[:3, 3]
    valid = np.isfinite(query).all(axis=1)&(query[:, 2] > .3)&(query[:, 2] < 5)
    ids, query = ids[valid], query[valid]
    x = np.rint(camera['fx']*query[:, 0]/query[:, 2]+camera['cx'])
    y = np.rint(camera['fy']*query[:, 1]/query[:, 2]+camera['cy'])
    inside = (x >= 0)&(x < camera['width'])&(y >= 0)&(y < camera['height'])
    ids, query = ids[inside], query[inside]
    cells = y[inside].astype(np.int64)*camera['width']+x[inside].astype(np.int64)
    measured = raw.ravel()[cells]
    supported = (measured > .3)&(measured < 5)&(np.sum(normals.reshape(-1, 3)[cells]**2, axis=1) > .25)
    ids, query, measured = ids[supported], query[supported], measured[supported]
    gap = measured-query[:, 2]; band = .03+.01*query[:, 2]
    free = gap > band; occluded = gap < -band
    stats = dict(tested=len(ids), consistent=int((~free&~occluded).sum()),
                 occluded=int(occluded.sum()), free_space=int(free.sum()), removed=0)
    return ids[free], stats


def decay(model, weights, raw, normals, current_from_model, camera):
    ids, stats = observations(model, weights, raw, normals, current_from_model, camera)
    output, support = model.copy(), weights.copy()
    support.ravel()[ids] -= 1
    cleared = ids[support.ravel()[ids] == 0]
    output.ravel()[cleared] = 0.
    stats['removed'] = len(cleared)
    return output, support, stats


def controls():
    c = dict(width=32, height=32, fx=32., fy=32., cx=15.5, cy=15.5)
    model = np.full((32, 32), 2.); weights = np.full((32, 32), 7, dtype=np.uint8)
    raw = model.copy(); normals = np.zeros((32, 32, 3)); normals[:, :, 2] = 1.
    model[16, 12:16] = [1., 3., 1., 1.]; weights[16, 12:16] = [2, 4, 2, 2]
    raw[16, 14] = 0.; normals[16, 15] = 0.
    first, support, stats = decay(model, weights, raw, normals, np.eye(4), c)
    require(stats == dict(tested=1022, consistent=1020, occluded=1, free_space=1, removed=0),
            'unknown and unsupported excluded, foreground contradiction distinct from occlusion')
    np.testing.assert_array_equal(support[16, 12:16], [1, 4, 2, 2])
    second, support, stats = decay(first, support, raw, normals, np.eye(4), c)
    require(second[16, 12] == 0. and support[16, 12] == 0 and stats['removed'] == 1,
            'two free-space acquisitions exhaust exactly two evidence credits')
    require(second[16, 13] == 3. and support[16, 13] == 4, 'occluded background retains its evidence')
    return dict(unknown_skipped=True, unsupported_skipped=True, occluded_background_retained=True,
                per_acquisition_credit=True, zero_support_clears_depth=True)
