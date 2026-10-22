"""Independent centered 2x2 circle arithmetic on original acquired XY returns.

No physical actor, filtered track position or producer fit result seeds the fit.
The pose transform only changes the reconstructed sensor geometry's frame.
"""
import math


def clusters(points):
    points = [tuple(p[k] for k in ['x', 'y']) for p in points
              if all(math.isfinite(p[k]) for k in ['x', 'y']) and math.hypot(p['x'], p['y']) > .3]
    remaining = set(range(len(points)))
    result = []
    for first in range(len(points)):
        if first not in remaining: continue
        remaining.remove(first); component = [first]; cursor = 0
        while cursor < len(component):
            point = points[component[cursor]]; cursor += 1
            tolerance = min(.85, .28 + .015 * math.hypot(*point))
            neighbors = sorted(i for i in remaining if math.dist(point, points[i]) < tolerance)
            remaining.difference_update(neighbors); component.extend(neighbors)
        if len(component) >= 3: result.append([points[i] for i in component])
    return result


def circle(points, sparse):
    n = len(points)
    if n < (3 if sparse else 5): return None
    if n < 5:
        baseline = max(math.dist(a, b) for a in points for b in points)
        area = max(abs((b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0]))
                   for a in points for b in points for c in points)
        if baseline < .5 or area / baseline < .05: return None
    mean = tuple(math.fsum(p[k] for p in points)/n for k in [0, 1])
    centered = [(p[0]-mean[0], p[1]-mean[1]) for p in points]
    xx = math.fsum(x*x for x, y in centered); yy = math.fsum(y*y for x, y in centered)
    cross = math.fsum(x*y for x, y in centered)
    bx = math.fsum(x*(x*x+y*y) for x, y in centered)/2
    by = math.fsum(y*(x*x+y*y) for x, y in centered)/2
    determinant = xx*yy-cross*cross
    if determinant <= 1e-12: return None
    local = ((bx*yy-by*cross)/determinant, (by*xx-bx*cross)/determinant)
    center = tuple(mean[k]+local[k] for k in [0, 1])
    radius = math.sqrt((xx+yy)/n+sum(x*x for x in local))
    if not .4 <= radius <= 2.5 or any(abs(math.dist(p, center)-radius) > .12 for p in points): return None
    if n < 5 and any(sum(p[k]*(p[k]-center[k]) for k in [0, 1]) > .12*math.hypot(*p) for p in points):
        return None
    return center, radius+.12


def detections(points, sparse):
    result = []
    for component in clusters(points):
        fitted = circle(component, sparse)
        center = tuple(math.fsum(p[k] for p in component)/len(component) for k in [0, 1])
        radius = min(3., max(.85, max(math.dist(p, center) for p in component)+.65))
        result.append(dict(center=fitted[0] if fitted else center, radius=fitted[1] if fitted else radius,
                           returns=len(component), fitted=fitted is not None))
    return result


def reconstruct(config, ticks):
    enabled = config.get('fit_sparse_circles', False)
    if type(enabled) is not bool: raise ValueError('sparse-circle policy must be boolean')
    if not config.get('enclose_track_measurements') and not config.get('confirm_track_initialization'):
        raise ValueError('raw associated measurements required for geometry audit')
    if config.get('multi_height_lidar') or config.get('lidar3d'):
        raise ValueError('sparse-circle oracle requires planar lidar')
    counts = dict(acquisitions=0, detections=0, sparse_fits=0, sparse_fallbacks=0, dense_detections=0)
    maximum_center = maximum_radius = 0.; last_scan = None; poses = {}
    for tick in ticks:
        inp, out = tick['input'], tick['expected']; now = inp['time']
        transform = out['tracking_frame']; yaw = transform['yaw']; c, s = math.cos(yaw), math.sin(yaw)
        world = out['estimate']['pose']; dx = world['position']['x']-transform['position']['x']
        dy = world['position']['y']-transform['position']['y']
        poses[now] = ((c*dx+s*dy, -s*dx+c*dy), world['yaw']-yaw)
        scan = inp.get('lidar')
        accepted = (scan is not None and not inp.get('lidar_failed', False)
                    and not {'InvalidLidar', 'AcquisitionFailed'}.intersection(out['health'])
                    and out.get('motion_discontinuity') is None
                    and 0. <= scan['stamp'] <= now and now-scan['stamp'] <= .35+1e-9
                    and (last_scan is None or scan['stamp'] > last_scan))
        if not accepted: continue
        stamp = scan['stamp']; last_scan = stamp
        if stamp not in poses: raise ValueError('acquisition pose not on recorded control clock')
        position, yaw = poses[stamp]; c, s = math.cos(yaw), math.sin(yaw)
        expected = detections(scan['points'], enabled); actual = out.get('track_measurements')
        if not isinstance(actual, list) or len(actual) != len(expected):
            raise ValueError('measured cluster set differs from original returns')
        counts['acquisitions'] += 1
        for measured, fitted in zip(actual, expected):
            x, y = fitted['center']; center = (position[0]+c*x-s*y, position[1]+s*x+c*y)
            ce = math.dist(center, (measured['center']['x'], measured['center']['y']))
            re = abs(fitted['radius']-measured['radius'])
            maximum_center = max(maximum_center, ce); maximum_radius = max(maximum_radius, re)
            if measured['stamp'] != stamp or ce > 1e-7 or re > 1e-7:
                raise ValueError(f'acquired circle geometry differs at {now}: center={ce}, radius={re}')
            counts['detections'] += 1
            counts['dense_detections' if fitted['returns'] >= 5 else
                   'sparse_fits' if fitted['fitted'] else 'sparse_fallbacks'] += 1
    return dict(counts, passed=True, maximum_center_difference_m=maximum_center,
                maximum_radius_difference_m=maximum_radius, original_returns_used=True,
                independent_centered_2x2_fit=True, physical_shape_confidence_claimed=False)
