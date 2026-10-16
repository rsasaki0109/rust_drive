"""Independent visible-surface aggregation for prescribed-transform map probes."""
import numpy as np

from projective_depth_common import require
from projective_depth_raw import vertices


def raster(depth, weights, transform, camera):
    """Keep the nearest surface, average its samples, retain frame support cap.

    A raw frame supplies unit weights: multiple splats still count as one frame.
    Stored-map weights select the mean, but the output keeps only the largest
    contributing support. This is a bounded fusion heuristic, not confidence.
    """
    ids = np.flatnonzero(((depth > .3)&(depth < 5)).ravel())
    query = vertices(depth, camera).reshape(-1, 3)[ids]@transform[:3, :3].T+transform[:3, 3]
    support = weights.ravel()[ids]
    stats = dict(source_valid=len(ids), projected=0, selected=0)
    front = np.isfinite(query).all(axis=1)&(query[:, 2] > .3)&(query[:, 2] < 5)
    query, support = query[front], support[front]
    x = np.rint(camera['fx']*query[:, 0]/query[:, 2]+camera['cx'])
    y = np.rint(camera['fy']*query[:, 1]/query[:, 2]+camera['cy'])
    inside = (x >= 0)&(x < camera['width'])&(y >= 0)&(y < camera['height'])
    cells = y[inside].astype(np.int64)*camera['width']+x[inside].astype(np.int64)
    z, support = query[inside, 2], support[inside]
    require(np.all((support > 0)&(support <= 16)), 'bounded supported raster inputs')
    stats['projected'] = len(cells)
    nearest = np.full(depth.size, np.inf)
    np.minimum.at(nearest, cells, z)
    visible = z-nearest[cells] < .03+.01*nearest[cells]
    cells, z, support = cells[visible], z[visible], support[visible]
    total = np.bincount(cells, weights=support, minlength=depth.size)
    moment = np.bincount(cells, weights=z*support, minlength=depth.size)
    present = total > 0
    output = np.zeros(depth.size)
    np.divide(moment, total, out=output, where=present)
    output_support = np.zeros(depth.size, dtype=np.uint8)
    np.maximum.at(output_support, cells, support)
    stats['selected'] = int(present.sum())
    merged = len(cells)-stats['selected']
    if merged:
        stats['surface_merged'] = merged
    return output.reshape(depth.shape), output_support.reshape(depth.shape), stats


def fuse(model, weights, raw, transform, camera):
    projected, present, stats = raster(raw, np.ones(raw.shape, dtype=np.uint8), transform, camera)
    valid = present > 0
    filled = valid&(weights == 0)
    averaged = valid&(weights > 0)&(abs(projected-model) < .03+.01*model)
    replaced = valid&(weights > 0)&~averaged&(projected < model-.03-.01*model)
    ignored = valid&~filled&~averaged&~replaced
    output, support = model.copy(), weights.copy()
    prior = np.minimum(16, weights[averaged]).astype(float)
    output[averaged] = (prior*model[averaged]+projected[averaged])/(prior+1)
    support[averaged] = np.minimum(16, weights[averaged]+1)
    output[filled|replaced] = projected[filled|replaced]
    support[filled|replaced] = 1
    return output, support, dict(warp=None, measurement=stats, filled=int(filled.sum()),
        averaged=int(averaged.sum()), replaced=int(replaced.sum()), ignored=int(ignored.sum()))


def controls():
    c = dict(width=32, height=32, fx=32., fy=32., cx=15.5, cy=15.5)
    depth = np.zeros((32, 32)); weights = np.zeros((32, 32), dtype=np.uint8)
    depth[16, 16:18] = [1., 1.02]; weights[16, 16:18] = [2, 6]
    depth[17, 17] = 2.; weights[17, 17] = 16
    transform = np.eye(4); transform[2, 3] = 1.
    out, support, stats = raster(depth, weights, transform, c)
    require(stats == dict(source_valid=3, projected=3, selected=1, surface_merged=1),
            'only the two foreground splats merge')
    require(abs(out[16, 16]-2.015) < 1e-14 and support[16, 16] == 6,
            'weighted surface mean without background or inflated frame support')
    _, unit, _ = raster(depth, (weights > 0).astype(np.uint8), transform, c)
    require(unit[16, 16] == 1, 'one acquisition remains one frame')
    out, support, _ = raster(depth, weights, np.eye(4), c)
    np.testing.assert_array_equal(out, depth)
    np.testing.assert_array_equal(support, weights)
    return dict(weighted_mean=True, background_excluded=True, frame_support_not_summed=True,
                identity_exact=True)
