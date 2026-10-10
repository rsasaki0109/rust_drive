#!/usr/bin/env python3
"""Independent recorded RGB-D image correspondence and physical-motion audit.

Reconstructs pixel features, descriptor matches, depth points and accepted-fit
geometry; does not reuse the Rust detector, matcher or pose optimizer.
"""
import argparse
import copy
from functools import lru_cache
import importlib.util
import io
import json
import math
from pathlib import Path
import re
import struct
import zlib

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location(
    'visual_geometry_oracle', Path(__file__).with_name('check-recorded-rgbd.py'))
r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(r)
_kspec = importlib.util.spec_from_file_location(
    'visual_score_oracle', Path(__file__).with_name('check-recorded-keyframes.py'))
k = importlib.util.module_from_spec(_kspec)
_kspec.loader.exec_module(k)
IDENTITY = ([0., 0., 0.], [1., 0., 0., 0.])
CIRCLE = [(0, -3), (1, -3), (2, -2), (3, -1), (3, 0), (3, 1), (2, 2), (1, 3),
          (0, 3), (-1, 3), (-2, 2), (-3, 1), (-3, 0), (-3, -1), (-2, -2), (-1, -3)]
CONFIG = dict(max_matches=256, max_hypotheses=128, max_refits=4, max_point_checks=33792,
              inlier_distance_m=.04, min_inliers=12, min_inlier_ratio=.5,
              min_geometry_ratio=.005, max_translation_m=.5, max_rotation_rad=.35,
              ambiguity_support_ratio=.95, ambiguity_rms_ratio=1.05,
              ambiguity_translation_m=.04, ambiguity_rotation_rad=.07,
              ambiguity_noise_floor_m=.001)
JSON_METADATA_AUDIT = dict(
    field='pair_gap_seconds', absolute_tolerance_s=1e-15,
    scope='source-manifest versus parsed freeze/report derived gap only; source SHA, acquisition timestamps and indices remain exact; no sensor association or fit gate change')


def same_pair_gap(actual, expected):
    """Only derived gap display may differ by JSON parser rounding."""
    r.require(type(actual) in (int, float) and type(expected) in (int, float)
              and math.isfinite(actual) and math.isfinite(expected)
              and abs(actual-expected) <= 1e-15, 'invented derived sensor gap')


def same_acquisition(actual, expected):
    r.require(actual.keys() == expected.keys(), 'changed acquisition fields')
    for field in expected:
        if field == 'pair_gap_seconds':
            same_pair_gap(actual[field], expected[field])
        else:
            r.require(actual[field] == expected[field], 'changed acquisition '+field)


def clock_block_reason(lost, last_accepted, last_observed_rgb, stamp, rgb_stamp):
    if lost:
        return 'visual odometry lost; explicit new origin required'
    if last_accepted is not None and stamp-last_accepted > .20+1e-9:
        return 'visual accepted-pose age exceeded; localization lost'
    if last_observed_rgb is not None and rgb_stamp <= last_observed_rgb:
        return 'duplicate or stale RGB acquisition; no pose permission renewal'
    return None


def png_image(raw, depth):
    """Verify bounded static source PNG, including every chunk CRC."""
    r.require(len(raw) <= 4*1024*1024 and raw[:8] == b'\x89PNG\r\n\x1a\n',
              'invalid PNG byte bound/signature')
    at, kinds, layout = 8, [], None
    while at < len(raw):
        r.require(at+12 <= len(raw), 'truncated PNG chunk')
        size = struct.unpack_from('>I', raw, at)[0]
        r.require(at+12+size <= len(raw), 'PNG chunk bound')
        kind, content = raw[at+4:at+8], raw[at+8:at+8+size]
        r.require(zlib.crc32(kind+content) & 0xffffffff ==
                  struct.unpack_from('>I', raw, at+8+size)[0], 'PNG CRC mismatch')
        r.require(kind not in (b'acTL', b'fcTL', b'fdAT'), 'animated source PNG')
        if kind == b'IHDR':
            r.require(not kinds and size == 13, 'invalid PNG header')
            layout = struct.unpack('>IIBBBBB', content)
            valid = [(640, 480, 16, 0, 0, 0, 0)] if depth else [
                (640, 480, 8, 2, 0, 0, 0), (640, 480, 8, 6, 0, 0, 0)]
            r.require(layout in valid, 'unsupported pixel layout')
        if kind == b'IEND':
            r.require(size == 0 and at+12 == len(raw), 'incomplete/trailing source PNG')
        kinds.append(kind)
        at += size+12
    r.require(kinds and kinds[0] == b'IHDR' and kinds[-1] == b'IEND'
              and kinds.count(b'IHDR') == 1 and kinds.count(b'IEND') == 1
              and b'IDAT' in kinds, 'PNG structure incomplete')
    image = Image.open(io.BytesIO(raw))
    image.load()
    if depth:
        r.require(image.mode in ('I', 'I;16'), 'depth decoder mode')
        return np.asarray(image, dtype=np.uint16)
    r.require(image.mode in ('RGB', 'RGBA'), 'RGB decoder mode')
    pixels = np.asarray(image, dtype=np.uint8)
    if pixels.shape[2] == 4:
        r.require(np.all(pixels[:, :, 3] == 255), 'non-opaque RGB alpha')
    rgb = pixels[:, :, :3].astype(np.uint16)
    return ((77*rgb[:, :, 0]+150*rgb[:, :, 1]+29*rgb[:, :, 2]) >> 8).astype(np.uint8)


def brief_pattern():
    state = 0x9e3779b9
    def next_value():
        nonlocal state
        state = (state ^ (state << 13)) & 0xffffffff
        state ^= state >> 17
        state = (state ^ (state << 5)) & 0xffffffff
        return state
    def point():
        while True:
            x, y = next_value() % 27-13, next_value() % 27-13
            if x*x+y*y <= 169:
                return x, y
    pairs = []
    for _ in range(256):
        a, b = point(), point()
        while a == b:
            b = point()
        pairs.append((a, b))
    return pairs


def half_away(value):
    return math.floor(value+.5) if value >= 0 else math.ceil(value-.5)


def features(gray):
    """Integer-array FAST and blur; independent descriptor implementation."""
    h, w = gray.shape
    r.require((h, w) == (480, 640) and gray.dtype == np.uint8, 'invalid grayscale image')
    center = gray[16:h-16, 16:w-16].astype(np.int16)
    diffs = [gray[16+dy:h-16+dy, 16+dx:w-16+dx].astype(np.int16)-center
             for dx, dy in CIRCLE]
    strength = np.zeros(center.shape, dtype=np.int16)
    for start in range(16):
        arc = [diffs[(start+i) % 16] for i in range(9)]
        strength = np.maximum(strength, np.maximum(np.minimum.reduce(arc), -np.maximum.reduce(arc)))
    strength[strength <= 20] = 0
    scores = np.zeros((h, w), dtype=np.int16)
    scores[16:h-16, 16:w-16] = strength
    candidates = []
    inner = scores[17:h-17, 17:w-17]
    retain = inner > 0
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            other = scores[17+dy:h-17+dy, 17+dx:w-17+dx]
            earlier = dy < 0 or (dy == 0 and dx < 0)
            retain &= (other < inner) if earlier else (other <= inner)
    ys, xs = np.nonzero(retain)
    candidates = sorted((int(inner[y, x]), int(x)+17, int(y)+17)
                        for y, x in zip(ys, xs))
    candidates.sort(key=lambda item: (-item[0], item[2], item[1]))
    wide = gray.astype(np.uint32)
    horizontal = np.zeros((h, w), dtype=np.uint32)
    horizontal[:, 2:w-2] = sum(weight*wide[:, i:w-4+i]
                              for i, weight in enumerate((1, 4, 6, 4, 1)))
    blur = np.zeros((h, w), dtype=np.uint8)
    blur[2:h-2, 2:w-2] = ((sum(weight*horizontal[i:h-4+i, 2:w-2]
                                 for i, weight in enumerate((1, 4, 6, 4, 1)))+128)//256).astype(np.uint8)
    disk = [(dx, dy) for dy in range(-15, 16) for dx in range(-15, 16) if dx*dx+dy*dy <= 225]
    pairs, tile_counts, output = brief_pattern(), {}, []
    for score, x, y in candidates:
        tile = x//32, y//32
        if tile_counts.get(tile, 0) == 2:
            continue
        tile_counts[tile] = tile_counts.get(tile, 0)+1
        mx = sum(dx*int(gray[y+dy, x+dx]) for dx, dy in disk)
        my = sum(dy*int(gray[y+dy, x+dx]) for dx, dy in disk)
        angle = math.atan2(my, mx)
        sine, cosine = math.sin(angle), math.cos(angle)
        words = [0]*4
        def sample(point):
            dx, dy = point
            u, v = half_away(dx*cosine-dy*sine), half_away(dx*sine+dy*cosine)
            return int(blur[y+v, x+u])
        for bit, (a, b) in enumerate(pairs):
            if sample(a) < sample(b):
                words[bit//64] |= 1 << (bit % 64)
        output.append(dict(x=float(x), y=float(y), score=score, orientation=angle, descriptor=words))
        if len(output) == 400:
            break
    return output


def feature_check(actual, expected):
    r.require(len(actual) == len(expected) <= 400, 'invented/missing FAST features')
    for a, e in zip(actual, expected):
        r.require(a['x'] == e['x'] and a['y'] == e['y'] and a['score'] == e['score']
                  and a['descriptor'] == e['descriptor'], 'invented feature pixel/score/descriptor')
        r.close(a['orientation'], e['orientation'], 'invented feature orientation', 1e-10)


def descriptor_matches(previous, current):
    distances = [[sum((a^b).bit_count() for a, b in zip(p['descriptor'], c['descriptor']))
                  for c in current] for p in previous]
    def unique(row):
        if len(row) < 2:
            return None
        ranked = sorted((value, index) for index, value in enumerate(row))
        return ranked[0][1] if ranked[0][0] <= 64 and 5*ranked[0][0] < 4*ranked[1][0] else None
    forward = [unique(row) for row in distances]
    backward = [unique([row[j] for row in distances]) for j in range(len(current))]
    output = []
    for i, j in enumerate(forward):
        if j is not None and backward[j] == i:
            output.append(dict(previous_index=i, current_index=j, hamming_distance=distances[i][j]))
    return sorted(output, key=lambda item: (item['hamming_distance'], item['previous_index'],
                                          item['current_index']))[:256]


def depth_point(feature, depth, calibration):
    return depth_point_status(feature, depth, calibration)[0]


def depth_point_status(feature, depth, calibration):
    x, y = half_away(feature['x']), half_away(feature['y'])
    if not (1 <= x < 639 and 1 <= y < 479):
        return None, 'depth patch exceeds image'
    values = [float(depth[y+dy, x+dx])/5000. for dy in (-1, 0, 1) for dx in (-1, 0, 1)]
    if not all(.3 <= z <= 5. for z in values):
        return None, 'invalid or range-limited measured depth patch'
    if max(values)-min(values) > .05:
        return None, 'measured depth discontinuity exceeds patch gate'
    z = values[4]
    return ([(x-calibration['cx'])*z/calibration['fx'],
             (y-calibration['cy'])*z/calibration['fy'], z], None)


def scatter_ratio(points):
    """Known noncollinear correspondences need rank two, including a plane."""
    points = np.asarray(points, dtype=np.float64)
    if len(points) < 3:
        return 0.
    centered = points-points.mean(axis=0)
    eigenvalues = np.linalg.eigvalsh(centered.T@centered)
    return float(eigenvalues[1]/eigenvalues[2]) if eigenvalues[2] > 0 else 0.


def matrix_quaternion(matrix):
    trace = np.trace(matrix)
    if trace > 0:
        s = 2.*math.sqrt(trace+1.)
        q = [s/4., (matrix[2, 1]-matrix[1, 2])/s,
             (matrix[0, 2]-matrix[2, 0])/s, (matrix[1, 0]-matrix[0, 1])/s]
    else:
        i = int(np.argmax(np.diag(matrix)))
        j, k_index = (i+1) % 3, (i+2) % 3
        s = 2.*math.sqrt(1.+matrix[i, i]-matrix[j, j]-matrix[k_index, k_index])
        q = [(matrix[k_index, j]-matrix[j, k_index])/s, 0., 0., 0.]
        q[i+1] = s/4.
        q[j+1] = (matrix[j, i]+matrix[i, j])/s
        q[k_index+1] = (matrix[k_index, i]+matrix[i, k_index])/s
    return r.qnorm(q)


def rigid_svd(previous, current):
    """Kabsch SVD, separate from the Rust Horn quaternion/Jacobi solver."""
    previous, current = np.asarray(previous), np.asarray(current)
    cm, pm = current.mean(axis=0), previous.mean(axis=0)
    u, _, vt = np.linalg.svd((current-cm).T@(previous-pm))
    correction = np.diag([1., 1., np.linalg.det(vt.T@u.T)])
    rotation = vt.T@correction@u.T
    translation = pm-rotation@cm
    return translation, rotation


def allowed_motion(translation, rotation):
    return (np.linalg.norm(translation) <= CONFIG['max_translation_m'] and
            r.quaternion_error(matrix_quaternion(rotation), IDENTITY[1]) <= CONFIG['max_rotation_rad'])


def noncollinear(points):
    a, b = points[1]-points[0], points[2]-points[0]
    product = np.linalg.norm(a)*np.linalg.norm(b)
    return product > 1e-10 and np.linalg.norm(np.cross(a, b))/product >= .001


def score_pairs(previous, current, translation, rotation):
    residuals = np.linalg.norm(current@rotation.T+translation-previous, axis=1)
    indices = np.flatnonzero(residuals <= CONFIG['inlier_distance_m']).tolist()
    return indices, float(np.sum(residuals[indices]**2))


def robust_svd(pairs):
    # Cache only immutable measured inputs across deliberate report corruptions;
    # each distinct geometry still receives its independent complete replay.
    frozen = tuple(tuple(p['previous'])+tuple(p['current']) for p in pairs)
    return _robust_svd(frozen)


@lru_cache(maxsize=128)
def _robust_svd(frozen):
    """Replay fixed sampling and consensus with independently solved models."""
    n = len(frozen)
    if not 12 <= n <= 256:
        return None, 'invalid or oversized visual correspondences'
    previous = np.asarray([p[:3] for p in frozen], dtype=np.float64)
    current = np.asarray([p[3:] for p in frozen], dtype=np.float64)
    if not (np.isfinite(previous).all() and np.isfinite(current).all()
            and np.max(np.linalg.norm(previous, axis=1)) <= 1e6
            and np.max(np.linalg.norm(current, axis=1)) <= 1e6):
        return None, 'invalid or oversized visual correspondences'
    state, point_checks, candidates, best = 0x7275737464726976, 0, [], None
    def draw(count):
        nonlocal state
        state = (state+0x9e3779b97f4a7c15) & 0xffffffffffffffff
        z = ((state ^ (state >> 30))*0xbf58476d1ce4e5b9) & 0xffffffffffffffff
        z = ((z ^ (z >> 27))*0x94d049bb133111eb) & 0xffffffffffffffff
        return (z ^ (z >> 31)) % count
    for _ in range(128):
        a, b = draw(n), draw(n-1)
        if b >= a:
            b += 1
        c = draw(n-2)
        for excluded in sorted((a, b)):
            if c >= excluded:
                c += 1
        selected = [a, b, c]
        if not noncollinear(current[selected]) or not noncollinear(previous[selected]):
            continue
        t, rotation = rigid_svd(previous[selected], current[selected])
        if not allowed_motion(t, rotation):
            continue
        inliers, error = score_pairs(previous, current, t, rotation)
        point_checks += n
        if len(inliers) < 12 or len(inliers)/n < .5:
            continue
        if scatter_ratio(previous[inliers]) < .005 or scatter_ratio(current[inliers]) < .005:
            continue
        candidates.append((t, rotation, len(inliers), math.sqrt(error/len(inliers))))
        if best is None or len(inliers) > len(best[0]) or (len(inliers) == len(best[0]) and error < best[1]):
            best = inliers, error
    if best is None:
        return None, 'no bounded visual correspondence consensus'
    consensus = best[0]
    for refits in range(1, 5):
        t, rotation = rigid_svd(previous[consensus], current[consensus])
        if not allowed_motion(t, rotation):
            return None, 'visual consensus refit exceeds motion bound'
        indices, error = score_pairs(previous, current, t, rotation)
        point_checks += n
        if len(indices) < 12 or len(indices)/n < .5:
            return None, 'visual consensus refit has insufficient inliers'
        if indices == consensus:
            p_ratio, c_ratio = scatter_ratio(previous[indices]), scatter_ratio(current[indices])
            if min(p_ratio, c_ratio) < .005:
                return None, 'collinear or poorly conditioned visual consensus geometry'
            rms, q = math.sqrt(error/len(indices)), matrix_quaternion(rotation)
            competing = sum(count >= .95*len(indices) and residual <= 1.05*max(rms, .001)
                            and (np.linalg.norm(ct-t) > .04
                                 or r.quaternion_error(matrix_quaternion(cr), q) > .07)
                            for ct, cr, count, residual in candidates)
            if competing:
                return None, 'ambiguous visual correspondence registration: comparable distinct rigid model'
            return dict(estimate=(t.tolist(), q), rms_m=rms, inlier_indices=indices,
                        inlier_count=len(indices), inlier_ratio=len(indices)/n,
                        hypotheses_evaluated=128, point_checks=point_checks, refits=refits,
                        geometry_ratio_current=c_ratio, geometry_ratio_previous=p_ratio,
                        candidate_models=len(candidates), competing_models=0), None
        consensus = indices
    return None, 'visual consensus did not stabilize within refit bound'


def fit_check(actual, estimate, pairs, independently):
    k.same_pose(estimate, independently['estimate'], 'independent SVD rigid pose')
    previous = np.asarray([p['previous'] for p in pairs])
    current = np.asarray([p['current'] for p in pairs])
    # Reconstruct the declared final membership with the reported pose itself,
    # not merely the independently optimized SVD output.
    t, q = estimate
    residuals = [r.norm([a-b for a, b in zip([x+y for x, y in zip(r.rotate(q, point), t)], target)])
                 for point, target in zip(current.tolist(), previous.tolist())]
    indices = [i for i, residual in enumerate(residuals) if residual <= .04]
    r.require(actual['inlier_indices'] == indices, 'invented inlier membership')
    for field in ('inlier_indices', 'inlier_count', 'hypotheses_evaluated', 'point_checks',
                  'refits', 'candidate_models', 'competing_models'):
        r.require(actual[field] == independently[field], 'invented rigid-fit counter '+field)
    for field in ('rms_m', 'inlier_ratio', 'geometry_ratio_current', 'geometry_ratio_previous'):
        r.close(actual[field], independently[field], 'invented rigid-fit geometry '+field, 1e-7)
    r.close(actual['rms_m'], math.sqrt(sum(residuals[i]**2 for i in indices)/len(indices)),
            'invented final residual', 1e-7)
    r.require(12 <= len(indices) and len(indices)/len(pairs) >= .5
              and actual['rms_m'] <= .04 and actual['point_checks'] <= 33792,
              'unsafe visual pose acceptance')


def vector_check(actual, expected, message):
    r.vector(actual, 3, message)
    for a, e in zip(actual, expected):
        r.close(a, e, message, 1e-10)


def matched_geometry(row, expected_matches, previous_features, current_features,
                     previous_depth, current_depth, calibration):
    r.require(row['matches'] == expected_matches, 'invented descriptor association/distance')
    r.require(len(row['depth_matches']) == len(expected_matches), 'omitted invalid depth associations')
    pairs = []
    for matched, actual in zip(expected_matches, row['depth_matches']):
        for name in ('previous_index', 'current_index'):
            r.require(actual[name] == matched[name], 'changed measured depth feature association')
        previous, p_error = depth_point_status(previous_features[matched['previous_index']],
                                               previous_depth, calibration)
        current, c_error = depth_point_status(current_features[matched['current_index']],
                                             current_depth, calibration)
        good = previous is not None and current is not None
        r.require(actual['accepted'] is good, 'invented depth patch acceptance')
        if good:
            vector_check(actual['previous_xyz'], previous, 'invented previous feature depth')
            vector_check(actual['current_xyz'], current, 'invented current feature depth')
            r.require(actual['correspondence_index'] == len(pairs), 'invented correspondence order')
            pairs.append(dict(previous=previous, current=current))
        else:
            r.require(actual['previous_rejection'] == p_error and actual['current_rejection'] == c_error
                      and not any(name in actual for name in ('previous_xyz', 'current_xyz',
                                                            'correspondence_index')),
                      'concealed invalid depth correspondence')
    r.require(len(row['correspondences']) == len(pairs), 'invented correspondence count')
    for actual, expected in zip(row['correspondences'], pairs):
        for side in ('previous', 'current'):
            vector_check(actual[side], expected[side], 'invented matched '+side+' 3D point')
    return pairs


def initialized_geometry(row, current_features, depth, calibration):
    points, expected = [], []
    for index, feature in enumerate(current_features):
        if len(points) >= 256:
            break
        point, error = depth_point_status(feature, depth, calibration)
        if point is None:
            expected.append(dict(feature_index=index, accepted=False, rejection=error))
        else:
            expected.append(dict(feature_index=index, accepted=True, point=point))
            points.append(point)
    r.require(len(row['initialization_depth_features']) == len(expected),
              'omitted initialization patch rejection')
    for actual, e in zip(row['initialization_depth_features'], expected):
        for field in ('feature_index', 'accepted'):
            r.require(actual[field] == e[field], 'invented initialization feature selection')
        if e['accepted']:
            vector_check(actual['point'], e['point'], 'invented initialization point')
        else:
            r.require(actual.get('rejection') == e['rejection'] and 'point' not in actual,
                      'concealed initialization depth rejection')
    ratio = scatter_ratio(points)
    good = 12 <= len(points) <= 256 and ratio >= .005
    if good:
        r.close(row['initialization_geometry_ratio'], ratio, 'invented initialization geometry', 1e-7)
    else:
        r.require('initialization_geometry_ratio' not in row, 'invented initialized geometry')
    return good, ratio, len(points)


def audit(report, manifest, all_features, depths, gt):
    r.require(report['schema_version'] == 1 and report['ground_truth_operational'] is False
              and report['raw_redistributed'] is False, 'schema/truth/redistribution')
    for name in ('dataset', 'repository', 'revision'):
        r.require(report[name] == manifest[name], 'changed source identity')
    frames, rows = manifest['frames'], report['frames']
    r.require(len(rows) == len(frames) == 36, 'omitted observed/rejected acquisition')
    r.require(report['freeze']['registration_config'] == CONFIG, 'weakened robust registration gates')
    r.require(report['freeze']['tracking_policy']['max_unobserved_s'] == .20,
              'changed accepted-pose age')
    calibration = manifest['depth_calibration']
    origin = k.timed_pose(gt, frames[0]['depth_timestamp'])
    reference, root_reference, last_accepted, last_observed_rgb, lost = None, IDENTITY, None, None, False
    counters = dict(initialized_frames=0, accepted_updates=0, rejected_updates=0,
                    accurate_root_updates=0, reference_valid_updates=0)
    records = []
    for i, (row, frame, observed) in enumerate(zip(rows, frames, all_features)):
        for field in ('source_index', 'depth_file', 'depth_timestamp', 'rgb_file', 'rgb_timestamp',
                      'rgb_source_index', 'pair_gap_seconds', 'split'):
            if field == 'pair_gap_seconds':
                same_pair_gap(row[field], frame[field])
            else:
                r.require(row[field] == frame[field], 'changed paired observation/clock')
        r.require(type(row['accepted']) is bool and type(row['initialized']) is bool,
                  'invalid acceptance/initialization')
        r.require(isinstance(row['cpu_wall_seconds'], (int, float))
                  and math.isfinite(row['cpu_wall_seconds']) and row['cpu_wall_seconds'] >= 0,
                  'invalid CPU timing')
        r.require(row['last_accepted_stamp_before'] == last_accepted
                  and row['reference_frame_index_before'] == (frames[reference]['source_index']
                                                              if reference is not None else None)
                  and row['lost_before'] == lost
                  and row['last_observed_rgb_stamp_before'] == last_observed_rgb,
                  'invented before-acquisition pose/reference/clock state')
        stamp, rgb_stamp = frame['depth_timestamp'], frame['rgb_timestamp']
        blocked = clock_block_reason(lost, last_accepted, last_observed_rgb, stamp, rgb_stamp)
        computation_allowed = blocked is None
        r.require(row['features_computed'] is computation_allowed
                  and row['feature_count'] == (len(observed) if computation_allowed else 0),
                  'computed features before clock/freshness guard')
        feature_check(row['features'], observed if computation_allowed else [])
        truth = k.truth_relative(origin, k.timed_pose(gt, stamp))
        if i:
            counters['reference_valid_updates'] += int(truth is not None)
        rejected = None
        predicted = None
        evidence = dict(source_index=frame['source_index'], feature_count=row['feature_count'],
                        features_computed=computation_allowed)
        if blocked:
            rejected = blocked
            lost = lost or 'localization lost' in blocked
        else:
            last_observed_rgb = rgb_stamp
            if reference is None:
                good, ratio, point_count = initialized_geometry(row, observed, depths[i], calibration)
                evidence.update(initialization_geometry_ratio=ratio,
                                initialization_depth_points=point_count)
                if good:
                    predicted = IDENTITY
                    r.require(row['initialized'] is True, 'concealed initialization')
                    counters['initialized_frames'] += 1
                else:
                    lost = True
                    rejected = row.get('rejection')
                    r.require(bool(rejected), 'concealed initialization failure')
            else:
                r.require(row['reference_frame_index'] == frames[reference]['source_index']
                          and row['reference_stamp'] == frames[reference]['depth_timestamp'],
                          'invented reference acquisition')
                k.same_pose(r.pose(row['root_from_reference']), root_reference, 'invented reference root')
                matches = descriptor_matches(all_features[reference], observed)
                pairs = matched_geometry(row, matches, all_features[reference], observed,
                                         depths[reference], depths[i], calibration)
                independently, rejected = robust_svd(pairs)
                evidence.update(descriptor_matches=len(matches), valid_depth_matches=len(pairs))
                if independently is not None:
                    r.require(row['initialized'] is False, 'silently changed root origin')
                    relative = r.pose(row['relative_estimate'])
                    fit_check(row['fit'], relative, pairs, independently)
                    predicted = k.compose(root_reference, relative)
                    evidence['fit'] = {field: value for field, value in independently.items() if field != 'estimate'}
        accepted = predicted is not None
        r.require(row['accepted'] is accepted, 'changed independently reconstructed acceptance')
        evidence['accepted'] = accepted
        score = row['root_accuracy']
        r.require(score['valid'] == accepted, 'invented root validity')
        if accepted:
            k.same_pose(r.pose(row['root_estimate']), predicted, 'invented root pose/composition')
            evidence['root'] = k.scored_pose(score, predicted, truth, 'visual root')
            reference, root_reference, last_accepted = i, predicted, stamp
            if i:
                counters['accepted_updates'] += 1
                counters['accurate_root_updates'] += int(score['within_accuracy_gates'])
            r.require('rejection' not in row, 'accepted pose retains contradictory rejection')
        else:
            r.require(row.get('rejection') == rejected and row['initialized'] is False
                      and not any(field in row for field in ('root_estimate', 'relative_estimate', 'fit')),
                      'concealed failure/invented rejected pose')
            k.scored_pose(score, None, truth, 'rejected visual root')
            if i:
                counters['rejected_updates'] += 1
            evidence['rejection'] = rejected
        if rejected and ('lost' in rejected or 'duplicate or stale' in rejected):
            r.require(row['matches'] == [] and row['depth_matches'] == [] and row['correspondences'] == [],
                      'performed visual fit after expired/duplicate acquisition')
        r.require(row['last_accepted_stamp_after'] == last_accepted
                  and row['reference_frame_index_after'] == (frames[reference]['source_index']
                                                             if reference is not None else None)
                  and row['lost_after'] == lost
                  and row['last_observed_rgb_stamp_after'] == last_observed_rgb,
                  'invented after-acquisition pose/reference/clock state')
        records.append(evidence)
    updates = len(frames)-1
    passed = (counters['initialized_frames'] == 1 and counters['accepted_updates'] == updates
              and counters['accurate_root_updates'] == updates
              and counters['reference_valid_updates'] == updates)
    expected = dict(frames=len(frames), updates=updates, **counters, lost=lost,
                    all_updates_passed=passed)
    r.require(report['summary'] == expected, 'summary hides rejected/unscorable acquisitions')
    return records


def mutation_checks(report, manifest, all_features, depths, gt):
    tests = [
        ('omit_acquisition', lambda x: x['frames'].pop()),
        ('invent_summary', lambda x: x['summary'].__setitem__('accurate_root_updates', 999)),
        ('operational_truth', lambda x: x.__setitem__('ground_truth_operational', True)),
        ('source_revision', lambda x: x.__setitem__('revision', 'unverified')),
        ('RGB_clock', lambda x: x['frames'][1].__setitem__('rgb_timestamp', 0.)),
        ('depth_clock', lambda x: x['frames'][1].__setitem__('depth_timestamp', 0.)),
        ('invent_derived_pair_gap', lambda x: x['frames'][1].__setitem__(
            'pair_gap_seconds', manifest['frames'][1]['pair_gap_seconds']+1e-12)),
        ('accepted_clock', lambda x: x['frames'][0].__setitem__('last_accepted_stamp_after', 0.)),
        ('observed_RGB_clock', lambda x: x['frames'][0].__setitem__('last_observed_rgb_stamp_after', 0.)),
        ('reference_clock', lambda x: x['frames'][0].__setitem__('reference_frame_index_after', 999)),
        ('weaken_consensus', lambda x: x['freeze']['registration_config'].__setitem__('min_inliers', 1)),
        ('weaken_ambiguity', lambda x: x['freeze']['registration_config'].__setitem__('ambiguity_support_ratio', 1.1)),
    ]
    feature = next((i for i, row in enumerate(report['frames']) if row['features']), None)
    if feature is not None:
        tests.extend([
            ('wrong_feature_pixel', lambda x: x['frames'][feature]['features'][0].__setitem__('x', 0.)),
            ('wrong_FAST_score', lambda x: x['frames'][feature]['features'][0].__setitem__('score', 0)),
            ('wrong_BRIEF', lambda x: x['frames'][feature]['features'][0]['descriptor'].__setitem__(0, x['frames'][feature]['features'][0]['descriptor'][0] ^ 1)),
            ('wrong_orientation', lambda x: x['frames'][feature]['features'][0].__setitem__('orientation', 100.)),
        ])
    matched = next((i for i, row in enumerate(report['frames']) if row['matches']), None)
    if matched is not None:
        tests.extend([
            ('invent_descriptor_distance', lambda x: x['frames'][matched]['matches'][0].__setitem__('hamming_distance', 999)),
            ('wrong_feature_match_ID', lambda x: x['frames'][matched]['matches'][0].__setitem__('current_index', 999)),
            ('omit_depth_rejection', lambda x: x['frames'][matched]['depth_matches'].pop()),
        ])
    accepted = next((i for i, row in enumerate(report['frames']) if i and row['accepted']), None)
    if accepted is not None:
        tests.extend([
            ('invent_depth_point', lambda x: x['frames'][accepted]['correspondences'][0]['current'].__setitem__(0, 999.)),
            ('invent_relative_pose', lambda x: x['frames'][accepted]['relative_estimate']['translation_m'].__setitem__(0, 100.)),
            ('invent_root_pose', lambda x: x['frames'][accepted]['root_estimate']['translation_m'].__setitem__(0, 100.)),
            ('invent_reference_root', lambda x: x['frames'][accepted]['root_from_reference']['translation_m'].__setitem__(0, 100.)),
            ('invent_inlier_membership', lambda x: x['frames'][accepted]['fit']['inlier_indices'].pop()),
            ('invent_inlier_ratio', lambda x: x['frames'][accepted]['fit'].__setitem__('inlier_ratio', 0.)),
            ('invent_RMS', lambda x: x['frames'][accepted]['fit'].__setitem__('rms_m', 100.)),
            ('invent_hypothesis_work', lambda x: x['frames'][accepted]['fit'].__setitem__('point_checks', 0)),
            ('invent_consensus_refits', lambda x: x['frames'][accepted]['fit'].__setitem__('refits', 0)),
            ('invent_candidates', lambda x: x['frames'][accepted]['fit'].__setitem__('candidate_models', 999)),
            ('invent_competing_models', lambda x: x['frames'][accepted]['fit'].__setitem__('competing_models', 1)),
            ('invent_accuracy', lambda x: x['frames'][accepted]['root_accuracy'].__setitem__('translation_error_m', 999.)),
        ])
    rejected = next((i for i, row in enumerate(report['frames']) if not row['accepted']), None)
    if rejected is not None:
        tests.append(('invent_rejected_pose', lambda x: x['frames'][rejected].__setitem__(
            'root_estimate', dict(translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))))
    blocked = next((i for i, row in enumerate(report['frames']) if not row['features_computed']), None)
    if blocked is not None:
        tests.extend([
            ('compute_after_duplicate_guard', lambda x: x['frames'][blocked].__setitem__('features_computed', True)),
            ('renew_duplicate_accepted_clock', lambda x: x['frames'][blocked].__setitem__(
                'last_accepted_stamp_after', x['frames'][blocked]['depth_timestamp'])),
        ])
    failures = []
    for name, change in tests:
        altered = copy.deepcopy(report)
        change(altered)
        try:
            audit(altered, manifest, all_features, depths, gt)
        except (ValueError, KeyError, IndexError):
            failures.append(name)
        else:
            raise ValueError('corrupted visual report accepted: '+name)
    return failures


def mathematical_contract_checks():
    same_pair_gap(.01, math.nextafter(.01, math.inf))
    try:
        same_pair_gap(.01, .01+1e-12)
    except ValueError:
        pass
    else:
        raise ValueError('JSON derived-gap corruption outside 1e-15s accepted')
    r.require(clock_block_reason(False, 1., 1.1, 1.2, 1.15) is None,
              'boundary-age fresh acquisition blocked')
    r.require(clock_block_reason(False, 1., 1.1, 1.15, 1.1).startswith('duplicate'),
              'previously observed rejected image renewed pose permission')
    r.require(clock_block_reason(False, 1., 1.1, 1.21, 1.1).startswith('visual accepted-pose age'),
              'duplicate freshness concealed accepted-pose expiry')
    r.require(clock_block_reason(True, 1., 1.1, 1.22, 1.22).startswith('visual odometry lost'),
              'healthy new image resurrected latched lost localization')
    q = [math.cos(.07/2), 0., 0., math.sin(.07/2)]
    t = [.03, -.02, .01]
    plane = [[x/5., y/5., 1.] for y in range(4) for x in range(4)]
    pairs = [dict(current=p, previous=[a+b for a, b in zip(r.rotate(q, p), t)]) for p in plane]
    fitted, error = robust_svd(pairs)
    r.require(error is None, 'known noncollinear planar correspondences rejected')
    k.same_pose(fitted['estimate'], (t, q), 'hand-labelled planar rigid motion')
    fit_check(fitted, (t, q), pairs, fitted)
    collinear = [dict(current=[x/10., 0., 1.], previous=[x/10., 0., 1.]) for x in range(16)]
    r.require(robust_svd(collinear)[0] is None, 'collinear known-correspondence ambiguity accepted')
    altered = copy.deepcopy(fitted)
    altered['inlier_indices'].pop()
    caught = []
    for name, fit, pose in (
            ('hand_motion_wrong_pose', fitted, ([.3, 0., 0.], q)),
            ('hand_motion_wrong_membership', altered, (t, q))):
        try:
            fit_check(fit, pose, pairs, fitted)
        except (ValueError, KeyError, IndexError):
            caught.append(name)
        else:
            raise ValueError('hand-labelled motion corruption accepted '+name)
    ambiguous = []
    for repeat, offset in enumerate((-.08, .08)):
        for p in plane:
            ambiguous.append(dict(current=p, previous=[p[0]+offset, p[1], p[2]]))
    result, reason = robust_svd(ambiguous)
    r.require(result is None and reason.startswith('ambiguous visual'), 'distinct equal-support motions accepted')
    # Descriptor identities below are manually constructed: mutual unique best
    # and the strict ratio both matter; identical alternatives cannot authorize.
    descriptor = lambda word: dict(descriptor=[word, 0, 0, 0])
    r.require(descriptor_matches([descriptor(0), descriptor(15)],
                                 [descriptor(0), descriptor(15)]) == [
                                     dict(previous_index=0, current_index=0, hamming_distance=0),
                                     dict(previous_index=1, current_index=1, hamming_distance=0)],
              'hand descriptor identities do not match')
    r.require(not descriptor_matches([descriptor(0), descriptor(0)],
                                     [descriptor(0), descriptor(0)]), 'descriptor ties accepted')
    calibration = dict(fx=535.4, fy=539.2, cx=320.1, cy=247.6)
    depth = np.full((480, 640), 5000, dtype=np.uint16)
    sample = dict(x=320., y=248.)
    expected = [(320.-320.1)/535.4, (248.-247.6)/539.2, 1.]
    vector_check(depth_point(sample, depth, calibration), expected, 'hand depth projection')
    depth[247, 319] = 0
    r.require(depth_point(sample, depth, calibration) is None, 'invalid stencil depth accepted')
    depth[247, 319] = 6000
    r.require(depth_point(sample, depth, calibration) is None, 'depth discontinuity accepted')
    return dict(passed=['hand_planar_rigid_motion', 'collinear_rejection', 'competing_model_rejection',
                        'mutual_unique_descriptor_identity', 'descriptor_tie_rejection',
                        'calibrated_depth_projection', 'invalid_stencil_rejection',
                        'depth_discontinuity_rejection', 'JSON_gap_1ULP_only_rounding',
                        'accepted_age_boundary', 'rejected_image_duplicate_block',
                        'expiry_before_duplicate', 'lost_latched_on_fresh_image'],
                mutations_rejected=caught+['JSON_gap_outside_1e-15s'])


SOURCES = {
    'evaluator_source_sha256': 'integrations/rgbd/src/main.rs',
    'visual_source_sha256': 'integrations/rgbd/src/visual.rs',
    'feature_source_sha256': 'crates/perception/src/image_features.rs',
    'visual_pose_source_sha256': 'crates/localization/src/visual_odometry3d.rs',
    'pose_source_sha256': 'crates/localization/src/registration3d.rs',
    'localization_lib_source_sha256': 'crates/localization/src/lib.rs',
    'perception_lib_source_sha256': 'crates/perception/src/lib.rs',
    'core_lib_source_sha256': 'crates/core/src/lib.rs',
    'cargo_lock_sha256': 'integrations/rgbd/Cargo.lock',
    'cargo_manifest_sha256': 'integrations/rgbd/Cargo.toml',
    'rust_toolchain_sha256': 'rust-toolchain.toml',
    'independent_checker_sha256': 'scripts/check-recorded-visual.py',
    'geometry_checker_sha256': 'scripts/check-recorded-rgbd.py',
    'keyframe_checker_sha256': 'scripts/check-recorded-keyframes.py',
    'acquisition_source_sha256': 'scripts/fetch-visual-datasets.py',
    'checker_requirements_sha256': 'scripts/requirements-visual.txt',
}


def verify_sources(freeze, manifest_raw, source_root):
    r.require(freeze['schema_version'] == 1 and freeze['protocol_version'] == 1,
              'unsupported visual protocol')
    for field, path in SOURCES.items():
        r.require(freeze[field] == r.digest(r.bounded(source_root/path)), 'changed frozen source '+path)
    r.require(freeze['manifest_sha256'] == r.digest(manifest_raw), 'changed frozen manifest')
    manifest = json.loads(manifest_raw)
    for field in ('depth_calibration', 'calibration_source', 'preprocessing', 'feature_policy',
                  'registration_config', 'tracking_policy'):
        r.require(freeze[field+'_sha256'] == r.canonical(freeze[field]), 'changed frozen policy '+field)
    for field in ('dataset', 'depth_calibration', 'calibration_source'):
        r.require(freeze[field] == manifest[field], 'changed frozen acquisition '+field)
    r.require(len(freeze['frames']) == len(manifest['frames']), 'changed frozen acquisition count')
    for actual, expected in zip(freeze['frames'], manifest['frames']):
        same_acquisition(actual, expected)
    r.require(freeze['json_metadata_audit'] == JSON_METADATA_AUDIT,
              'changed narrow JSON-derived-gap rounding policy')
    r.require(type(freeze['regression_requested']) is bool, 'invalid reporting-role flag')
    fresh = manifest['dataset'] == 'tum-fr3-sitting-visual' and not freeze['regression_requested']
    r.require(freeze['kind'] == ('preregistered_sequence' if fresh else 'calibration_regression'),
              'misclassified viewed/fresh visual sequence')
    r.require(freeze['registration_config'] == CONFIG, 'weakened rigid fit/ambiguity acceptance')
    r.require(freeze['accuracy_gates'] == dict(translation_m=.1, rotation_rad=.1)
              and freeze['ground_truth_interpolation'] == dict(
                  method='linear translation and shortest-arc quaternion SLERP',
                  max_bracket_s=.02, extrapolation=False), 'changed physical evaluation gates')
    preprocessing = freeze['preprocessing']
    for key, value in dict(width=640, height=480, min_depth_m=.3, max_depth_m=5.,
                           depth_units_per_metre=5000, patch_radius_pixels=1,
                           patch_max_spread_m=.05, maximum_pair_gap_s=.02).items():
        r.require(preprocessing[key] == value, 'changed depth/RGB association policy '+key)
    r.require(freeze['feature_policy']['max_features'] == 400
              and freeze['feature_policy']['max_matches'] == 256
              and freeze['tracking_policy']['max_unobserved_s'] == .20,
              'changed feature/clock resource bound')
    c = manifest['depth_calibration']
    precise = dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989)
    if manifest['dataset'] != 'tum-fr1-desk-visual':
        r.require(manifest['dataset'] in ('tum-fr3-office-visual', 'tum-fr3-sitting-visual'),
                  'unknown sensor calibration')
        precise = dict(fx=535.4, fy=539.2, cx=320.1, cy=247.6)
    r.require(all(c[key] == value for key, value in precise.items())
              and c['width'] == 640 and c['height'] == 480 and c['units_per_metre'] == 5000,
              'unsupported projection of measured feature depth')


def calibration_check(report, manifest, raw):
    source = manifest['calibration_source']
    r.require(source['file'] == 'camera-calibration.yaml', 'unsafe calibration source path')
    content = r.bounded(raw/source['file'])
    r.require(len(content) == source['bytes'] and r.digest(content) == source['sha256'],
              'camera calibration byte mismatch')
    r.require(report['calibration_source'] == source and report['calibration_sha256_verified'] is True,
              'report omits calibrated camera provenance')
    values = {}
    for line in content.decode('utf8').splitlines():
        match = re.fullmatch(r'\s*(Camera\.(?:fx|fy|cx|cy|width|height|k[123]|p[12])|DepthMapFactor)\s*:\s*([-+0-9.eE]+)\s*(?:#.*)?', line)
        if match:
            r.require(match[1] not in values, 'duplicate camera parameter')
            values[match[1]] = float(match[2])
    c = manifest['depth_calibration']
    for key in ('fx', 'fy', 'cx', 'cy', 'width', 'height'):
        r.require(values['Camera.'+key] == c[key], 'invented published camera projection '+key)
    r.require(values['DepthMapFactor'] == c['units_per_metre'], 'invented published depth scale')
    for key, value in c['source_rgb_distortion'].items():
        r.require(values['Camera.'+key] == value, 'invented unapplied RGB distortion provenance')


def main():
    global r, k
    parser = argparse.ArgumentParser()
    for name in ('report', 'manifest', 'raw', 'freeze', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--source-snapshot', type=Path,
                        help='Exact archived source root; every original source hash remains mandatory')
    args = parser.parse_args()
    report_raw = r.bounded(args.report, 16*1024*1024)
    manifest_raw, freeze_raw = r.bounded(args.manifest, 256*1024), r.bounded(args.freeze, 256*1024)
    report, manifest, freeze = map(json.loads, (report_raw, manifest_raw, freeze_raw))
    r.require(report['freeze'] == freeze and report['freeze_sha256'] == r.digest(freeze_raw)
              and report['manifest_sha256'] == r.digest(manifest_raw), 'external freeze mismatch')
    source_root = args.source_snapshot or ROOT
    verify_sources(freeze, manifest_raw, source_root)
    if args.source_snapshot is not None:
        for name, path in (('archived_visual_geometry', 'scripts/check-recorded-rgbd.py'),
                           ('archived_visual_scores', 'scripts/check-recorded-keyframes.py')):
            spec = importlib.util.spec_from_file_location(name, source_root/path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            if name.endswith('geometry'):
                r = module
            else:
                k = module
                k.r = r
    provenance_mutations = []
    for field in ('feature_source_sha256', 'visual_pose_source_sha256', 'visual_source_sha256',
                  'independent_checker_sha256', 'keyframe_checker_sha256', 'manifest_sha256'):
        altered = copy.deepcopy(freeze)
        altered[field] = '0'*64
        try:
            verify_sources(altered, manifest_raw, source_root)
        except (ValueError, KeyError, IndexError):
            provenance_mutations.append(field)
        else:
            raise ValueError('corrupted frozen provenance accepted '+field)
    for name, field, key, value in (
            ('weaken_full_registration_policy', 'registration_config', 'min_geometry_ratio', 0.),
            ('weaken_clock_policy', 'tracking_policy', 'max_unobserved_s', 2.),
            ('weaken_depth_patch_policy', 'preprocessing', 'patch_max_spread_m', 1.),
            ('weaken_feature_cap', 'feature_policy', 'max_features', 4000),
            ('weaken_JSON_rounding_policy', 'json_metadata_audit', 'absolute_tolerance_s', 1e-6)):
        altered = copy.deepcopy(freeze)
        altered[field][key] = value
        if field+'_sha256' in altered:
            altered[field+'_sha256'] = r.canonical(altered[field])
        try:
            verify_sources(altered, manifest_raw, source_root)
        except (ValueError, KeyError, IndexError):
            provenance_mutations.append(name)
        else:
            raise ValueError('corrupted frozen policy accepted '+name)
    calibration_check(report, manifest, args.raw)
    raw, total = {}, 0
    for item in manifest['files']:
        filename = item['file']
        r.require(Path(filename).name == filename and '/' not in filename and '\\' not in filename,
                  'unsafe raw filename')
        content = r.bounded(args.raw/filename)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'measured RGB/depth/mocap byte mismatch')
        raw[filename] = content
        total += len(content)
    r.require(total+manifest['calibration_source']['bytes'] <= 32*1024*1024, 'raw acquisition total bound')
    r.require(report['files'] == [{key: item[key] for key in ('file', 'bytes', 'sha256', 'role')}
                                  for item in manifest['files']], 'changed input provenance')
    frames = manifest['frames']
    expected_names = {'depth.txt', 'rgb.txt', 'groundtruth.txt'} | {
        frame[key] for frame in frames for key in ('depth_file', 'rgb_file')}
    r.require(set(raw) == expected_names and len(manifest['files']) == len(expected_names),
              'unexpected/incomplete visual input inventory')
    indices = {kind: [line.split() for line in raw[kind+'.txt'].decode().splitlines()
                      if line.strip() and not line.startswith('#')] for kind in ('rgb', 'depth')}
    for i, frame in enumerate(frames):
        update_split = ('held_out' if manifest['dataset'] == 'tum-fr3-sitting-visual'
                        else 'viewed_development')
        r.require(frame['source_index'] == 100+i and frame['split'] == (
            'initialization' if i == 0 else update_split), 'changed preregistered window')
        for kind, ordinal in (('depth', frame['source_index']), ('rgb', frame['rgb_source_index'])):
            row = indices[kind][ordinal]
            file, stamp = frame[kind+'_file'], frame[kind+'_timestamp']
            r.require(len(row) == 2 and float(row[0]) == stamp and file.startswith(kind+'-')
                      and row[1] == kind+'/'+file[len(kind)+1:], 'changed original image acquisition index')
        gap = abs(frame['depth_timestamp']-frame['rgb_timestamp'])
        r.require(gap <= .02, 'RGB/depth association time limit exceeded')
        same_pair_gap(frame['pair_gap_seconds'], gap)
        if i:
            previous = frames[i-1]
            r.require(frame['depth_timestamp'] > previous['depth_timestamp']
                      and frame['rgb_timestamp'] >= previous['rgb_timestamp']
                      and frame['rgb_source_index'] >= previous['rgb_source_index'], 'nonchronological source images')
    reconstructed, depth_images, cache = [], [], {}
    # Independent reconstruction happens only after source-frozen fit reports
    # exist; the operational Rust evaluator parses mocap after all fits.
    for frame in frames:
        rgb = frame['rgb_file']
        if rgb not in cache:
            cache[rgb] = features(png_image(raw[rgb], False))
        reconstructed.append(cache[rgb])
        depth_images.append(png_image(raw[frame['depth_file']], True))
    gt = r.gt_rows(raw['groundtruth.txt'])
    records = audit(report, manifest, reconstructed, depth_images, gt)
    mutations = mutation_checks(report, manifest, reconstructed, depth_images, gt)
    mathematics = mathematical_contract_checks()
    missing = k.missing_reference_contract_checks()
    root_records = [record['root'] for record in records[1:] if record.get('root')]
    result = dict(schema_version=1, passed_integrity=True, summary=report['summary'], frames=records,
                  report_sha256=r.digest(report_raw), manifest_sha256=r.digest(manifest_raw),
                  freeze_sha256=r.digest(freeze_raw), mutations_rejected=mutations,
                  frozen_provenance_mutations_rejected=provenance_mutations,
                  mathematical_contract_checks=mathematics,
                  missing_reference_score_mutations_rejected=missing,
                  running_checker_sha256=r.digest(r.bounded(Path(__file__))),
                  frozen_checker_sha256=freeze['independent_checker_sha256'],
                  archived_source_snapshot=args.source_snapshot is not None,
                  raw_sha256_verified=True, source_freeze_verified=True,
                  pixel_features_descriptors_and_associations_independently_reconstructed=True,
                  rigid_fit_independently_replayed='Kabsch SVD vs operational Horn quaternion/Jacobi',
                  consensus_stationarity_and_competing_models_independently_checked=True,
                  reference_and_clock_state_independently_reconstructed=True,
                  physical_pose_independently_scored=True,
                  json_metadata_audit=JSON_METADATA_AUDIT,
                  oracle_dependencies=dict(numpy=np.__version__, pillow=Image.__version__),
                  calibrated_covariance_or_root_confidence_claim=False,
                  maximum_root_translation_error_m=max((row['translation_error_m'] for row in root_records), default=None),
                  maximum_root_rotation_error_rad=max((row['rotation_error_rad'] for row in root_records), default=None))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result['summary']))


if __name__ == '__main__':
    main()
