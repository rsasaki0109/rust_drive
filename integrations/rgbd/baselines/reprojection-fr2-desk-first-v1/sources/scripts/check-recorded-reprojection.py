#!/usr/bin/env python3
"""Independent measured-image reprojection refinement and physical-root audit.

The immutable visual oracle reconstructs measured pixels and the original coarse
3D consensus. This module independently solves calibrated pixel refinement using
numerical projection Jacobians and SVD, separately from Rust's analytic normal
matrix solve. The three legacy recordings are viewed regression evidence; a
separately pinned FR2 recording supports an explicitly preregistered first trial.
"""
import argparse
import copy
from functools import lru_cache
import importlib.util
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
OLD_VISUAL_SHA = 'c138992bd5fe63a42757ff2cf5b43c7eb69272b85ea6ce7030cd298306bc0d6a'
_spec = importlib.util.spec_from_file_location('immutable_visual_oracle',
    Path(__file__).with_name('check-recorded-visual.py'))
v = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(v)
r, k = v.r, v.k
r.require(r.digest(r.bounded(Path(v.__file__))) == OLD_VISUAL_SHA,
          'original visual checker must remain immutable')
v.SOURCES = dict(v.SOURCES,
    independent_checker_sha256='scripts/check-recorded-reprojection.py',
    visual_checker_sha256='scripts/check-recorded-visual.py',
    reprojection_source_sha256='integrations/rgbd/src/reprojection.rs',
    reprojection_pose_source_sha256='crates/localization/src/reprojection3d.rs',
    reprojection_acquisition_source_sha256='scripts/fetch-reprojection-dataset.py')

def audit(report, manifest, all_features, depths, gt):
    r.require(report['schema_version'] == 1 and report['ground_truth_operational'] is False
              and report['raw_redistributed'] is False
              and report['algorithm'] == 'bounded_visual_reprojection_refinement',
              'schema/truth/redistribution/algorithm')
    for name in ('dataset', 'repository', 'revision'):
        r.require(report[name] == manifest[name], 'changed source identity')
    frames, rows = manifest['frames'], report['frames']
    r.require(len(rows) == len(frames) == 36, 'omitted observed/rejected acquisition')
    r.require(report['freeze']['registration_config'] == v.CONFIG, 'weakened robust registration gates')
    r.require(report['freeze']['refinement_config'] == REFINEMENT_CONFIG, 'weakened pixel refinement gates')
    r.require(report['freeze']['tracking_policy']['max_unobserved_s'] == .20,
              'changed accepted-pose age')
    calibration = manifest['depth_calibration']
    origin = k.timed_pose(gt, frames[0]['depth_timestamp'])
    reference, root_reference, last_accepted, last_observed_rgb, lost = None, v.IDENTITY, None, None, False
    counters = dict(initialized_frames=0, accepted_updates=0, rejected_updates=0,
                    accurate_root_updates=0, reference_valid_updates=0)
    records = []
    for i, (row, frame, observed) in enumerate(zip(rows, frames, all_features)):
        for field in ('source_index', 'depth_file', 'depth_timestamp', 'rgb_file', 'rgb_timestamp',
                      'rgb_source_index', 'pair_gap_seconds', 'split'):
            if field == 'pair_gap_seconds':
                v.same_pair_gap(row[field], frame[field])
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
        blocked = v.clock_block_reason(lost, last_accepted, last_observed_rgb, stamp, rgb_stamp)
        computation_allowed = blocked is None
        r.require(row['features_computed'] is computation_allowed
                  and row['feature_count'] == (len(observed) if computation_allowed else 0),
                  'computed features before clock/freshness guard')
        v.feature_check(row['features'], observed if computation_allowed else [])
        truth = k.truth_relative(origin, k.timed_pose(gt, stamp))
        if i:
            counters['reference_valid_updates'] += int(truth is not None)
        rejected = None
        predicted = None
        refinement_attempted = False
        evidence = dict(source_index=frame['source_index'], feature_count=row['feature_count'],
                        features_computed=computation_allowed)
        if blocked:
            rejected = blocked
            lost = lost or 'localization lost' in blocked
        else:
            last_observed_rgb = rgb_stamp
            if reference is None:
                r.require(row['matches'] == [] and row['depth_matches'] == [] and row['correspondences'] == []
                          and not row.get('refinement_observations')
                          and not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement', 'relative_estimate')),
                          'invented initialization fit')
                good, ratio, point_count = v.initialized_geometry(row, observed, depths[i], calibration)
                evidence.update(initialization_geometry_ratio=ratio,
                                initialization_depth_points=point_count)
                if good:
                    predicted = v.IDENTITY
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
                matches = v.descriptor_matches(all_features[reference], observed)
                pairs = v.matched_geometry(row, matches, all_features[reference], observed,
                                         depths[reference], depths[i], calibration)
                independently, rejected = v.robust_svd(pairs)
                evidence.update(descriptor_matches=len(matches), valid_depth_matches=len(pairs))
                if independently is not None:
                    r.require(row['initialized'] is False, 'silently changed root origin')
                    coarse = r.pose(row['coarse_relative_estimate'])
                    v.fit_check(row['fit'], coarse, pairs, independently)
                    observations = refinement_observations(row, independently, matches, observed, pairs)
                    refinement_attempted = True
                    refined, rejected = refine_independent(observations, calibration, coarse)
                    if refined is not None:
                        refinement_check(row['refinement'], r.pose(row['relative_estimate']), observations, calibration, coarse, refined)
                        predicted = k.compose(root_reference, r.pose(row['relative_estimate']))
                        evidence['refinement'] = refined
                    else:
                        r.require('refinement' not in row and 'relative_estimate' not in row, 'concealed refinement failure')
                        r.require(row['refinement_rejection'] == rejected, 'concealed backend refinement rejection')
                    evidence['fit'] = {field: value for field, value in independently.items() if field != 'estimate'}
                else:
                    r.require(not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement', 'relative_estimate'))
                              and not row.get('refinement_observations'), 'invented refinement without coarse consensus')
        accepted = predicted is not None
        r.require(row['refinement_attempted'] is refinement_attempted,
                  'invented or concealed refinement invocation')
        if not refinement_attempted or accepted:
            r.require('refinement_rejection' not in row, 'invented refinement rejection')
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
                      and not any(field in row for field in ('root_estimate', 'relative_estimate', 'refinement')),
                      'concealed failure/invented rejected pose')
            k.scored_pose(score, None, truth, 'rejected visual root')
            if i:
                counters['rejected_updates'] += 1
            evidence['rejection'] = rejected
        if rejected and ('lost' in rejected or 'duplicate or stale' in rejected):
            r.require(row['matches'] == [] and row['depth_matches'] == [] and row['correspondences'] == []
                      and not any(field in row for field in ('fit', 'coarse_relative_estimate', 'refinement'))
                      and not row.get('refinement_observations'),
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



# Fixed common defaults; names are checked against the final Rust configuration.
REFINEMENT_CONFIG = dict(max_observations=256, max_iterations=8,
    max_line_search_steps=8, max_point_checks=18688, min_observations=12,
    huber_delta_px=3., min_depth_m=.1, max_depth_m=10.,
    max_condition_number=1e8, max_translation_m=.5,
    max_rotation_rad=.35, translation_tolerance_m=1e-6,
    rotation_tolerance_rad=1e-6)
REFINEMENT_POLICY = {
    'failure': 'reject without current root output or accepted reference/clock renewal; no coarse-pose fallback',
    'initial_pose': 'original previous_from_current robust3D fit, never motion-capture labels',
    'method': 'bounded Huber pixel reprojection, maximum8 iterations; convergence and work reported explicitly',
    'observations': 'only original robust3D inlier correspondence indices; previous measured depth point and matched current RGB feature pixel; fixed support throughout refinement',
    'pose': 'refined previous_from_current pose is composed into accepted root; coarse fit and pose are separate diagnostics',
}


def rotation_matrix(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def skew(p):
    x, y, z = p
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exponential(rotation):
    angle = np.linalg.norm(rotation)
    matrix = skew(rotation)
    if angle < 1e-12:
        return np.eye(3)
    return np.eye(3)+math.sin(angle)/angle*matrix+(1-math.cos(angle))/angle**2*matrix@matrix


def project(points, camera):
    z = points[:, 2]
    return np.column_stack((camera['fx']*points[:, 0]/z+camera['cx'],
                            camera['fy']*points[:, 1]/z+camera['cy']))


def pixel_score(points, pixels, camera, translation, rotation):
    transformed = points@rotation.T+translation
    checks = 0
    for point in transformed:
        checks += 1
        if not (np.isfinite(point).all() and .1 <= point[2] <= 10.):
            return None, checks
    residual = project(transformed, camera)-pixels
    lengths = np.linalg.norm(residual, axis=1)
    cost = float(np.sum(np.where(lengths <= 3., .5*lengths**2, 3.*(lengths-1.5))))
    return dict(huber_cost=cost, rms_px=float(np.sqrt(np.mean(lengths**2))),
                residual=residual, transformed=transformed), checks


def numerical_system(transformed, pixels, camera):
    """Central differences of image projection under left camera increments."""
    epsilon = 1e-6
    columns = []
    for axis in range(6):
        delta = np.zeros(6); delta[axis] = epsilon
        positive = transformed@exponential(delta[3:]).T+delta[:3]
        negative = transformed@exponential(-delta[3:]).T-delta[:3]
        columns.append(((project(positive, camera)-project(negative, camera))/(2*epsilon)).ravel())
    jacobian = np.column_stack(columns)
    residual = project(transformed, camera)-pixels
    lengths = np.linalg.norm(residual, axis=1)
    weights = np.minimum(1., 3./np.maximum(lengths, 1e-12))
    square_weights = np.sqrt(np.repeat(weights, 2))
    weighted = jacobian*square_weights[:, None]
    scale = np.sqrt(np.sum(weighted**2, axis=0))
    if not np.isfinite(scale).all() or np.any(scale <= 1e-12):
        return None, math.inf
    normalized = weighted/scale
    eigenvalues = np.linalg.eigvalsh(normalized.T@normalized)
    condition = float(eigenvalues[-1]/eigenvalues[0]) if eigenvalues[0] > 0 else math.inf
    solution = np.linalg.lstsq(normalized, -residual.ravel()*square_weights, rcond=1e-12)[0]/scale
    return solution, condition


def refinement_observations(row, coarse, matches, current_features, pairs):
    mapping = {item['correspondence_index']: item for item in row['depth_matches'] if item['accepted']}
    expected = []
    for index in coarse['inlier_indices']:
        matched = mapping[index]
        feature = current_features[matched['current_index']]
        expected.append(dict(correspondence_index=index, previous_index=matched['previous_index'],
            current_index=matched['current_index'], previous_xyz=pairs[index]['previous'],
            current_pixel_xy=[feature['x'], feature['y']]))
    actual = row['refinement_observations']
    r.require(len(actual) == len(expected), 'dropped original consensus refinement support')
    for a, e in zip(actual, expected):
        r.require(a.keys() == e.keys(), 'changed reprojection observation schema')
        for field in ('correspondence_index', 'previous_index', 'current_index', 'current_pixel_xy'):
            r.require(a[field] == e[field], 'invented pixel/inlier association '+field)
        v.vector_check(a['previous_xyz'], e['previous_xyz'], 'invented previous depth refinement point')
    return expected


def refine_independent(observations, camera, coarse):
    frozen = tuple(tuple(item['previous_xyz'])+tuple(item['current_pixel_xy']) for item in observations)
    return _refine_independent(frozen, tuple(camera[key] for key in ('fx', 'fy', 'cx', 'cy')),
                               tuple(coarse[0])+tuple(coarse[1]))


@lru_cache(maxsize=128)
def _refine_independent(frozen, calibration, coarse):
    n = len(frozen)
    if not 12 <= n <= 256:
        return None, 'insufficient or excessive reprojection observations'
    camera = dict(zip(('fx', 'fy', 'cx', 'cy'), calibration))
    points = np.array([p[:3] for p in frozen]); pixels = np.array([p[3:] for p in frozen])
    if not (np.isfinite(points).all() and np.isfinite(pixels).all()
            and np.all(np.linalg.norm(points, axis=1) <= 1e6)
            and np.all((points[:, 2] >= .1) & (points[:, 2] <= 10.))
            and np.max(np.abs(pixels)) <= 1e6):
        return None, 'invalid reprojection observation'
    if not (all(math.isfinite(value) for value in calibration)
            and 1 <= camera['fx'] <= 1e5 and 1 <= camera['fy'] <= 1e5
            and abs(camera['cx']) <= 1e6 and abs(camera['cy']) <= 1e6):
        return None, 'invalid reprojection camera intrinsics'
    if not np.isfinite(coarse).all() or abs(np.linalg.norm(coarse[3:])-1) > 1e-10:
        return None, 'invalid or excessive reprojection pose'
    coarse_rotation = rotation_matrix(r.qnorm(coarse[3:])); rotation = coarse_rotation.T
    if not v.allowed_motion(np.array(coarse[:3]), coarse_rotation):
        return None, 'invalid or excessive reprojection pose'
    translation = -rotation@np.array(coarse[:3])
    score, point_checks = pixel_score(points, pixels, camera, translation, rotation)
    if score is None:
        return None, 'invalid projected reprojection depth'
    initial = dict(huber_cost=score['huber_cost'], rms_px=score['rms_px'])
    trace, maximum_condition, converged, iterations = [], 0., False, 0
    for iterations in range(1, 9):
        point_checks += n
        delta, condition = numerical_system(score['transformed'], pixels, camera)
        maximum_condition = max(maximum_condition, condition)
        if delta is None or condition > 1e8:
            return None, 'ill-conditioned reprojection normal matrix'
        if np.linalg.norm(delta[:3]) <= 1e-6 and np.linalg.norm(delta[3:]) <= 1e-6:
            converged = True
            break
        accepted = None
        for trial in range(1, 9):
            scale = 2.**(1-trial)
            change = exponential(scale*delta[3:])
            candidate_translation = change@translation+scale*delta[:3]
            candidate_rotation = change@rotation
            if not v.allowed_motion(candidate_translation, candidate_rotation):
                continue
            candidate, checks = pixel_score(points, pixels, camera, candidate_translation, candidate_rotation)
            point_checks += checks
            if candidate is not None and candidate['huber_cost'] < score['huber_cost']:
                accepted = candidate, candidate_translation, candidate_rotation
                trace.append(dict(current_from_previous=(candidate_translation.tolist(), v.matrix_quaternion(candidate_rotation)),
                    huber_cost_before=score['huber_cost'], huber_cost_after=candidate['huber_cost'],
                    scale=scale, increment=delta.tolist(), normal_condition_number=condition,
                    line_search_trials=trial))
                break
        if accepted is None:
            return None, 'reprojection line search failed'
        score, translation, rotation = accepted
    final_pose = ((-rotation.T@translation).tolist(), v.matrix_quaternion(rotation.T))
    return dict(estimate=final_pose, initial_rms_px=initial['rms_px'], final_rms_px=score['rms_px'],
        initial_huber_cost=initial['huber_cost'], final_huber_cost=score['huber_cost'],
        iterations=iterations, accepted_steps=len(trace), point_checks=point_checks,
        valid_support=n, converged=converged, max_normal_condition_number=maximum_condition,
        trace=trace), None


def refinement_check(actual, pose, observations, camera, coarse, expected):
    r.require(type(actual['converged']) is bool, 'invalid convergence claim')
    k.same_pose(pose, expected['estimate'], 'independent numerical-Jacobian refined pose')
    for field in ('iterations', 'accepted_steps', 'point_checks', 'valid_support', 'converged'):
        r.require(actual[field] == expected[field], 'invented reprojection counter '+field)
    for field in ('initial_rms_px', 'final_rms_px', 'initial_huber_cost', 'final_huber_cost',
                  'max_normal_condition_number'):
        r.close(actual[field], expected[field], 'invented reprojection statistic '+field, 1e-6)
    r.require(actual['point_checks'] <= 18688 and actual['accepted_steps'] <= 8,
              'exceeded bounded reprojection work')
    r.require((actual['converged'] and actual['iterations'] == actual['accepted_steps']+1)
              or (not actual['converged'] and actual['iterations'] == actual['accepted_steps'] == 8),
              'invented terminal convergence accounting')
    r.require(len(actual['trace']) == len(expected['trace']), 'invented reprojection steps')
    for step, e in zip(actual['trace'], expected['trace']):
        r.require(step['huber_cost_after'] < step['huber_cost_before'], 'nondecreasing accepted pixel cost')
        k.same_pose(r.pose(step['current_from_previous']), e['current_from_previous'], 'invented reprojection step pose')
        for field in ('scale', 'line_search_trials'):
            r.require(step[field] == e[field], 'invented reprojection line search '+field)
        for field in ('huber_cost_before', 'huber_cost_after', 'normal_condition_number'):
            r.close(step[field], e[field], 'invented reprojection trace '+field, 1e-6)
        for a, b in zip(r.vector(step['increment'], 6, 'invalid pixel increment'), e['increment']):
            r.close(a, b, 'invented reprojection increment', 1e-6)
    # Verify declared final cost independently, with no optimizer reuse.
    t, q = pose; rotation = rotation_matrix(q).T; translation = -rotation@np.array(t)
    points = np.array([o['previous_xyz'] for o in observations]); pixels = np.array([o['current_pixel_xy'] for o in observations])
    final, _ = pixel_score(points, pixels, camera, translation, rotation)
    r.require(final is not None, 'accepted invalid final depth projection')
    r.close(actual['final_huber_cost'], final['huber_cost'], 'invented final declared-pose pixel cost', 1e-8)
    r.close(actual['final_rms_px'], final['rms_px'], 'invented final declared-pose pixel RMS', 1e-8)
    if actual['converged']:
        terminal_increment, condition = numerical_system(final['transformed'], pixels, camera)
        r.require(terminal_increment is not None and condition <= 1e8
                  and np.linalg.norm(terminal_increment[:3]) <= 1e-6
                  and np.linalg.norm(terminal_increment[3:]) <= 1e-6,
                  'declared converged pose violates terminal increment bounds')


def depth_point_status(feature, depth, calibration):
    """Common patch policy, using the explicit source profile's native depth units."""
    x, y = v.half_away(feature['x']), v.half_away(feature['y'])
    if not (1 <= x < 639 and 1 <= y < 479):
        return None, 'depth patch exceeds image'
    # Legacy hand fixtures omit units; actual manifests always declare and pin it.
    units = calibration.get('units_per_metre', 5000.)
    values = [float(depth[y+dy, x+dx])/units for dy in (-1, 0, 1) for dx in (-1, 0, 1)]
    if not all(.3 <= z <= 5. for z in values):
        return None, 'invalid or range-limited measured depth patch'
    if max(values)-min(values) > .05:
        return None, 'measured depth discontinuity exceeds patch gate'
    z = values[4]
    return ([(x-calibration['cx'])*z/calibration['fx'],
             (y-calibration['cy'])*z/calibration['fy'], z], None)


v.depth_point_status = depth_point_status


@lru_cache(maxsize=8)
def acquisition_metadata_checker(source_root, fresh_profile):
    """Load only the already hash-verified, metadata-only acquisition validator."""
    relative = ('scripts/fetch-reprojection-dataset.py' if fresh_profile
                else 'scripts/fetch-visual-datasets.py')
    spec = importlib.util.spec_from_file_location('pinned_acquisition_metadata', Path(source_root)/relative)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    try:
        sys.dont_write_bytecode = True
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = previous
    return module.verify_manifest


def refinement_config_digest(value):
    """Match serde_json's numeric exponent spelling for this numeric-only policy.

    Python prints the fixed 1e-6 tolerances as 1e-06. This changes only the
    serialized policy hash; all numeric acceptance values remain exact.
    """
    encoded = json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
    encoded = re.sub(r'e-0+(\d+)', r'e-\1', encoded)
    return r.digest(encoded.encode())


def verify_sources(freeze, manifest_raw, source_root):
    r.require(freeze['schema_version'] == 1 and freeze['protocol_version'] == 2
              and freeze['algorithm'] == 'bounded_visual_reprojection_refinement',
              'unsupported visual protocol')
    for field, path in v.SOURCES.items():
        r.require(freeze[field] == r.digest(r.bounded(source_root/path)), 'changed frozen source '+path)
    r.require(freeze['manifest_sha256'] == r.digest(manifest_raw), 'changed frozen manifest')
    manifest = json.loads(manifest_raw)
    acquisition_metadata_checker(str(source_root.resolve()),
        manifest['dataset'] == 'tum-fr2-desk-reprojection')(manifest)
    for frame in manifest['frames']:
        gap = abs(frame['depth_timestamp']-frame['rgb_timestamp'])
        r.require(gap <= .02, 'changed source sensor association gate')
        v.same_pair_gap(frame['pair_gap_seconds'], gap)
    for field in ('depth_calibration', 'calibration_source', 'preprocessing', 'feature_policy',
                  'registration_config', 'tracking_policy', 'refinement_config', 'refinement_policy'):
        expected = (refinement_config_digest(freeze[field]) if field == 'refinement_config'
                    else r.canonical(freeze[field]))
        r.require(freeze[field+'_sha256'] == expected, 'changed frozen policy '+field)
    for field in ('dataset', 'depth_calibration', 'calibration_source'):
        r.require(freeze[field] == manifest[field], 'changed frozen acquisition '+field)
    r.require(len(freeze['frames']) == len(manifest['frames']), 'changed frozen acquisition count')
    for actual, expected in zip(freeze['frames'], manifest['frames']):
        v.same_acquisition(actual, expected)
    r.require(freeze['json_metadata_audit'] == v.JSON_METADATA_AUDIT,
              'changed narrow JSON-derived-gap rounding policy')
    r.require(type(freeze['regression_requested']) is bool, 'invalid reporting-role flag')
    fresh = manifest['dataset'] == 'tum-fr2-desk-reprojection' and not freeze['regression_requested']
    r.require(fresh or freeze['regression_requested'], 'viewed refinement mislabeled as a new trial')
    r.require(freeze['visual_checker_sha256'] == OLD_VISUAL_SHA, 'changed original visual oracle')
    r.require(freeze['refinement_config'] == REFINEMENT_CONFIG, 'weakened pixel refinement gates')
    r.require(freeze['refinement_policy'] == REFINEMENT_POLICY, 'changed fixed-support refinement policy')
    r.require(freeze['kind'] == ('preregistered_sequence' if fresh else 'calibration_regression'),
              'misclassified viewed/fresh visual sequence')
    r.require(freeze['registration_config'] == v.CONFIG, 'weakened rigid fit/ambiguity acceptance')
    r.require(freeze['accuracy_gates'] == dict(translation_m=.1, rotation_rad=.1)
              and freeze['ground_truth_interpolation'] == dict(
                  method='linear translation and shortest-arc quaternion SLERP',
                  max_bracket_s=.02, extrapolation=False,
                  max_source_rows=30000, max_source_bytes=4194304), 'changed physical evaluation gates')
    units = 5208. if manifest['dataset'] == 'tum-fr2-desk-reprojection' else 5000.
    preprocessing = freeze['preprocessing']
    for key, value in dict(width=640, height=480, min_depth_m=.3, max_depth_m=5.,
                           depth_units_per_metre=units, patch_radius_pixels=1,
                           patch_max_spread_m=.05, maximum_pair_gap_s=.02).items():
        r.require(preprocessing[key] == value, 'changed depth/RGB association policy '+key)
    r.require(freeze['feature_policy']['max_features'] == 400
              and freeze['feature_policy']['max_matches'] == 256
              and freeze['tracking_policy']['max_unobserved_s'] == .20,
              'changed feature/clock resource bound')
    c = manifest['depth_calibration']
    precise = dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989)
    if manifest['dataset'] == 'tum-fr2-desk-reprojection':
        precise = dict(fx=520.90862, fy=521.007327, cx=325.141442, cy=249.701764)
    elif manifest['dataset'] != 'tum-fr1-desk-visual':
        r.require(manifest['dataset'] in ('tum-fr3-office-visual', 'tum-fr3-sitting-visual'),
                  'unknown sensor calibration')
        precise = dict(fx=535.4, fy=539.2, cx=320.1, cy=247.6)
    r.require(all(c[key] == value for key, value in precise.items())
              and c['width'] == 640 and c['height'] == 480 and c['units_per_metre'] == units,
              'unsupported projection of measured feature depth')



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
    coarse_failed = next((i for i, row in enumerate(report['frames'])
                         if i and row['features_computed'] and not row['accepted'] and 'fit' not in row), None)
    if coarse_failed is not None:
        tests.extend([
            ('forge_failed_coarse_witness', lambda x: x['frames'][coarse_failed].__setitem__('fit', {})),
            ('forge_failed_coarse_pose', lambda x: x['frames'][coarse_failed].__setitem__('coarse_relative_estimate',
                dict(translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))),
            ('forge_failed_refinement_observation', lambda x: x['frames'][coarse_failed].__setitem__('refinement_observations', [{}])),
        ])
    blocked = next((i for i, row in enumerate(report['frames']) if not row['features_computed']), None)
    if blocked is not None:
        tests.extend([
            ('forge_blocked_coarse_witness', lambda x: x['frames'][blocked].__setitem__('fit', {})),
            ('compute_after_duplicate_guard', lambda x: x['frames'][blocked].__setitem__('features_computed', True)),
            ('renew_duplicate_accepted_clock', lambda x: x['frames'][blocked].__setitem__(
                'last_accepted_stamp_after', x['frames'][blocked]['depth_timestamp'])),
        ])
    refined = next((i for i, row in enumerate(report['frames']) if 'refinement' in row), None)
    if refined is not None:
        tests.extend([
            ('conceal_refinement_invocation', lambda x: x['frames'][refined].__setitem__('refinement_attempted', False)),
            ('invent_refinement_rejection', lambda x: x['frames'][refined].__setitem__('refinement_rejection', 'invented')),
            ('invent_coarse_pose', lambda x: x['frames'][refined]['coarse_relative_estimate']['translation_m'].__setitem__(0, 100.)),
            ('drop_refinement_support', lambda x: x['frames'][refined]['refinement_observations'].pop()),
            ('wrong_refinement_inlier_ID', lambda x: x['frames'][refined]['refinement_observations'][0].__setitem__('correspondence_index', 999)),
            ('wrong_refinement_pixel', lambda x: x['frames'][refined]['refinement_observations'][0]['current_pixel_xy'].__setitem__(0, 0.)),
            ('wrong_refinement_depth_point', lambda x: x['frames'][refined]['refinement_observations'][0]['previous_xyz'].__setitem__(0, 999.)),
            ('invent_pixel_initial_cost', lambda x: x['frames'][refined]['refinement'].__setitem__('initial_huber_cost', 9999.)),
            ('invent_pixel_final_cost', lambda x: x['frames'][refined]['refinement'].__setitem__('final_huber_cost', 9999.)),
            ('invent_pixel_RMS', lambda x: x['frames'][refined]['refinement'].__setitem__('final_rms_px', 9999.)),
            ('invent_pixel_work', lambda x: x['frames'][refined]['refinement'].__setitem__('point_checks', 0)),
            ('invent_pixel_support', lambda x: x['frames'][refined]['refinement'].__setitem__('valid_support', 1)),
            ('invent_pixel_convergence', lambda x: x['frames'][refined]['refinement'].__setitem__('converged', not x['frames'][refined]['refinement']['converged'])),
            ('weaken_pixel_Huber', lambda x: x['freeze']['refinement_config'].__setitem__('huber_delta_px', 1000.)),
            ('weaken_pixel_workcap', lambda x: x['freeze']['refinement_config'].__setitem__('max_point_checks', 1000000)),
        ])
        if report['frames'][refined]['refinement']['trace']:
            tests.extend([
                ('nonmonotonic_pixel_step', lambda x: x['frames'][refined]['refinement']['trace'][0].__setitem__('huber_cost_after', 99999.)),
                ('invent_pixel_increment', lambda x: x['frames'][refined]['refinement']['trace'][0]['increment'].__setitem__(0, 10.)),
                ('invent_pixel_line_search', lambda x: x['frames'][refined]['refinement']['trace'][0].__setitem__('line_search_trials', 999)),
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
    base = v.mathematical_contract_checks()
    camera = dict(fx=535.4, fy=539.2, cx=320.1, cy=247.6)
    q = [math.cos(.04/2), 0., 0., math.sin(.04/2)]
    true = ([.03, -.02, .01], q)
    points = np.array([[x/5., y/5., 1.5] for y in range(-2, 3) for x in range(-2, 3)])
    inverse_rotation = rotation_matrix(q).T
    inverse_translation = -inverse_rotation@np.array(true[0])
    pixels = project(points@inverse_rotation.T+inverse_translation, camera)
    observations = [dict(previous_xyz=p.tolist(), current_pixel_xy=uv.tolist()) for p, uv in zip(points, pixels)]
    fitted, rejection = refine_independent(observations, camera, ([.035, -.018, .012], q))
    r.require(rejection is None, 'hand-labelled calibrated pixel motion rejected')
    k.same_pose(fitted['estimate'], true, 'hand-labelled calibrated pixel motion')
    r.require(fitted['converged'] and fitted['final_huber_cost'] < fitted['initial_huber_cost'],
              'known pixel motion neither converged nor improved')
    # Convert the independently reconstructed trace to the external pose schema.
    external = copy.deepcopy(fitted)
    for step in external['trace']:
        t, orientation = step['current_from_previous']
        step['current_from_previous'] = dict(translation_m=t, quaternion_wxyz=orientation)
    refinement_check(external, fitted['estimate'], observations, camera, ([.035, -.018, .012], q), fitted)
    stationary, rejection = refine_independent(observations, camera, true)
    r.require(rejection is None and stationary['converged'] and stationary['accepted_steps'] == 0
              and stationary['iterations'] == 1 and stationary['point_checks'] == 2*len(points),
              'known stationary pixel pose failed bounded terminal accounting')
    collinear = [dict(previous_xyz=[i/10., 0., 1.5], current_pixel_xy=[camera['cx']+camera['fx']*(i/10.)/1.5, camera['cy']]) for i in range(12)]
    r.require(refine_independent(collinear, camera, v.IDENTITY)[0] is None,
              'singular calibrated pixel geometry accepted')
    bad_depth = copy.deepcopy(observations); bad_depth[0]['previous_xyz'][2] = 0.
    r.require(refine_independent(bad_depth, camera, true)[0] is None,
              'invalid fixed landmark depth accepted')
    corruptions = []
    for name, field, value in [('hand_pixel_wrong_cost', 'final_huber_cost', 999.),
                               ('hand_pixel_wrong_work', 'point_checks', 0),
                               ('hand_pixel_wrong_support', 'valid_support', 1)]:
        altered = copy.deepcopy(external); altered[field] = value
        try:
            refinement_check(altered, fitted['estimate'], observations, camera, ([.035, -.018, .012], q), fitted)
        except (ValueError, KeyError, IndexError):
            corruptions.append(name)
        else:
            raise ValueError('hand pixel corruption accepted '+name)
    base['passed'] += ['hand_calibrated_planar_pixel_motion', 'stationary_zero_step_accounting',
                      'singular_pixel_geometry_rejection', 'invalid_fixed_landmark_depth']
    native = dict(fx=520.90862, fy=521.007327, cx=325.141442, cy=249.701764,
                  units_per_metre=5208.)
    synthetic_depth = np.full((480, 640), 5208, dtype=np.uint16)
    projected, error = depth_point_status(dict(x=320., y=248.), synthetic_depth, native)
    r.require(error is None and projected[2] == 1., 'published native depth scale replaced by legacy5000')
    v.vector_check(projected, [(320.-native['cx'])/native['fx'],
                              (248.-native['cy'])/native['fy'], 1.], 'hand native FR2 projection')
    base['passed'].append('hand_native_5208_depth_profile')
    maximum_rows = ''.join(f'{i}.0 0 0 0 0 0 0 1\n' for i in range(30000)).encode()
    r.require(len(ground_truth_rows(maximum_rows)) == 30000, 'declared evaluation-only GT row cap unsupported')
    small = b'0.0 0 0 0 0 0 0 1\n0.01 0 0 0 0 0 0 1\n'
    maximum_bytes = small+b'#'+b' '*(4194304-len(small)-1)
    r.require(len(ground_truth_rows(maximum_bytes)) == 2, 'declared evaluation-only GT byte cap unsupported')
    for name, raw in [('GT_excess_rows', maximum_rows+b'30000.0 0 0 0 0 0 0 1\n'),
                      ('GT_excess_bytes', maximum_bytes+b' ')]:
        try:
            ground_truth_rows(raw)
        except ValueError:
            base['mutations_rejected'].append(name)
        else:
            raise ValueError('evaluation-only GT bound corruption accepted '+name)
    base['passed'] += ['evaluation_only_GT_30000_row_boundary', 'evaluation_only_GT_4MiB_byte_boundary']
    base['mutations_rejected'] += corruptions
    return base

def ground_truth_rows(raw):
    """New-mode-only evaluation label reader, called after all recorded fits exist."""
    r.require(len(raw) <= 4194304, 'evaluation-only GT byte bound')
    rows = []
    for line in raw.decode('ascii').splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        values = list(map(float, line.split()))
        r.require(len(values) == 8 and all(math.isfinite(x) for x in values), 'invalid GT row')
        q = [values[7], values[4], values[5], values[6]]
        r.require(abs(r.norm(q)-1) <= .001, 'invalid raw GT quaternion')
        r.require(not rows or values[0] > rows[-1][0], 'nonmonotonic raw GT timestamps')
        r.require(len(rows) < 30000, 'evaluation-only GT row bound')
        rows.append((values[0], values[1:4], r.qnorm(q)))
    r.require(len(rows) >= 2, 'evaluation-only GT row bound')
    return rows


def main():
    global r, k
    parser = argparse.ArgumentParser()
    for name in ('manifest', 'freeze', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    for name in ('report', 'raw'):
        parser.add_argument('--'+name, type=Path)
    parser.add_argument('--preregister-only', action='store_true',
                        help='Verify exact metadata/source freeze without reading raw input bytes or fitting')
    parser.add_argument('--source-snapshot', type=Path,
                        help='Exact archived source root; every original source hash remains mandatory')
    args = parser.parse_args()
    if args.preregister_only:
        manifest_raw = r.bounded(args.manifest, 256*1024)
        freeze_raw = r.bounded(args.freeze, 256*1024)
        freeze = json.loads(freeze_raw)
        verify_sources(freeze, manifest_raw, args.source_snapshot or ROOT)
        result = dict(schema_version=1, passed_integrity=True, source_freeze_verified=True,
                      metadata_only=True, raw_input_bytes_read=False,
                      features_or_fits_run=False, frozen_source_count=len(v.SOURCES),
                      freeze_sha256=r.digest(freeze_raw), manifest_sha256=r.digest(manifest_raw),
                      checker_sha256=freeze['independent_checker_sha256'],
                      dataset=freeze['dataset'], kind=freeze['kind'])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        print(json.dumps(result))
        return
    r.require(args.report is not None and args.raw is not None,
              'fit audit requires --report and --raw')
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
                  'independent_checker_sha256', 'keyframe_checker_sha256', 'manifest_sha256',
                  'reprojection_source_sha256', 'reprojection_pose_source_sha256',
                  'reprojection_acquisition_source_sha256', 'visual_checker_sha256'):
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
            ('weaken_pixel_refinement', 'refinement_config', 'huber_delta_px', 1000.),
            ('weaken_pixel_iterations', 'refinement_config', 'max_iterations', 100),
            ('weaken_GT_label_rowcap', 'ground_truth_interpolation', 'max_source_rows', 300000),
            ('weaken_JSON_rounding_policy', 'json_metadata_audit', 'absolute_tolerance_s', 1e-6)):
        altered = copy.deepcopy(freeze)
        altered[field][key] = value
        if field+'_sha256' in altered:
            altered[field+'_sha256'] = (refinement_config_digest(altered[field])
                                       if field == 'refinement_config' else r.canonical(altered[field]))
        try:
            verify_sources(altered, manifest_raw, source_root)
        except (ValueError, KeyError, IndexError):
            provenance_mutations.append(name)
        else:
            raise ValueError('corrupted frozen policy accepted '+name)
    # Even a self-consistently rehashed external manifest cannot rename the
    # pinned acquisition commit or calibration-document repository.
    for name, field, value in (
            ('rehash_manifest_repository', 'repository', 'unverified/repository'),
            ('rehash_manifest_revision', 'revision', '0'*40),
            ('rehash_calibration_repository', 'calibration_source', 'unverified/repository')):
        changed_manifest = copy.deepcopy(manifest)
        changed_freeze = copy.deepcopy(freeze)
        if field == 'calibration_source':
            changed_manifest[field]['repository'] = value
            changed_freeze[field] = changed_manifest[field]
            changed_freeze[field+'_sha256'] = r.canonical(changed_freeze[field])
        else:
            changed_manifest[field] = value
        changed_raw = json.dumps(changed_manifest, sort_keys=True).encode()
        changed_freeze['manifest_sha256'] = r.digest(changed_raw)
        try:
            verify_sources(changed_freeze, changed_raw, source_root)
        except (ValueError, KeyError, IndexError):
            provenance_mutations.append(name)
        else:
            raise ValueError('renamed pinned source accepted '+name)
    v.calibration_check(report, manifest, args.raw)
    raw, total = {}, 0
    for item in manifest['files']:
        filename = item['file']
        r.require(Path(filename).name == filename and '/' not in filename and '\\' not in filename,
                  'unsafe raw filename')
        content = r.bounded(args.raw/filename)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'measured RGB/depth/mocap byte mismatch')
        if 'git_blob_sha1' in item:
            blob = b'blob '+str(len(content)).encode()+b'\0'+content
            r.require(hashlib.sha1(blob).hexdigest() == item['git_blob_sha1'], 'changed pinned Git source blob')
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
        update_split = ('held_out' if manifest['dataset'] in ('tum-fr3-sitting-visual', 'tum-fr2-desk-reprojection')
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
        v.same_pair_gap(frame['pair_gap_seconds'], gap)
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
            cache[rgb] = v.features(v.png_image(raw[rgb], False))
        reconstructed.append(cache[rgb])
        depth_images.append(v.png_image(raw[frame['depth_file']], True))
    gt = ground_truth_rows(raw['groundtruth.txt'])
    v.r, v.k = r, k
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
                  pixel_refinement_independently_replayed='central numerical Jacobian and SVD vs analytic Jacobian and Cholesky',
                  fixed_original_inlier_support_independently_checked=True,
                  monotonic_search_and_terminal_step_accounting_independently_checked=True,
                  evidence_role='viewed regression' if freeze['regression_requested'] else 'preregistered sequence',
                  consensus_stationarity_and_competing_models_independently_checked=True,
                  reference_and_clock_state_independently_reconstructed=True,
                  physical_pose_independently_scored=True,
                  json_metadata_audit=v.JSON_METADATA_AUDIT,
                  oracle_dependencies=dict(numpy=np.__version__, pillow=v.Image.__version__),
                  calibrated_covariance_or_root_confidence_claim=False,
                  maximum_root_translation_error_m=max((row['translation_error_m'] for row in root_records), default=None),
                  maximum_root_rotation_error_rad=max((row['rotation_error_rad'] for row in root_records), default=None))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result['summary']))


if __name__ == '__main__':
    main()
