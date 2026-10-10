#!/usr/bin/env python3
"""Independent measured-submap state, geometry, bounded-fusion and mocap audit.

Reconstructs all map generations from recorded measurements and reported fits.
Does not rerun ICP, certify association choices or calibrate root confidence.
"""
import argparse
import copy
import importlib.util
import json
import math
from pathlib import Path
import re
import struct

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location(
    'rgbd_oracle', Path(__file__).with_name('check-recorded-rgbd.py'))
r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(r)
_kspec = importlib.util.spec_from_file_location(
    'keyframe_score_oracle', Path(__file__).with_name('check-recorded-keyframes.py'))
k = importlib.util.module_from_spec(_kspec)
_kspec.loader.exec_module(k)
IDENTITY = ([0., 0., 0.], [1., 0., 0., 0.])
PRECISE_FR1 = dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989)
PRECISE_FR3 = dict(fx=535.4, fy=539.2, cx=320.1, cy=247.6)
NEW_CALIBRATIONS = {'tum-fr1-desk-submaps': PRECISE_FR1,
                    'tum-fr3-office-submaps': PRECISE_FR3}
REGISTRATION_CONFIG = dict(
    max_scan_points=5000, max_map_points=5000, max_iterations=30,
    max_neighbor_checks=20_000_000, max_correspondence_m=.15, trim_fraction=.2,
    min_pairs=30, min_overlap=.4, max_translation_jump_m=.5, max_rotation_jump_rad=.35,
    max_rms_m=.08, min_geometry_ratio=.005, max_condition_number=10000.,
    max_position_variance_m2=.25, max_rotation_variance_rad2=.01,
    translation_tolerance_m=.0001, rotation_tolerance_rad=.0001,
    ambiguity_translation_probe_m=.15, ambiguity_rotation_probe_rad=.07,
    ambiguity_rms_ratio=1.05)


def depth_geometry(raw, preprocessing):
    # The unchanged legacy helper verifies PNG structure, CRC, dimensions,
    # integer pixels and bounded decoding. Its legacy projected points are
    # discarded when the independently pinned camera projection differs.
    validated = r.depth_geometry(raw)
    if all(preprocessing[key] == value for key, value in dict(
            fx=525., fy=525., cx=319.5, cy=239.5).items()):
        return validated
    from PIL import Image
    import io
    image = Image.open(io.BytesIO(raw))
    image.load()
    pixels, voxels = image.load(), {}
    for y in range(0, 480, 8):
        for x in range(0, 640, 8):
            d = pixels[x, y]
            z = d/5000.
            if d == 0 or not .3 <= z <= 5.:
                continue
            point = [(x-preprocessing['cx'])*z/preprocessing['fx'],
                     (y-preprocessing['cy'])*z/preprocessing['fy'], z]
            voxels.setdefault(tuple(math.floor(v/.03) for v in point), point)
    return [voxels[key] for key in sorted(voxels)]


def exact_pose(value):
    """Validate without renormalizing serialized IEEE754 map-transform values."""
    r.pose(value)
    return value['translation_m'], value['quaternion_wxyz']


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def transform(estimate, point):
    # Preserve explicitly frozen floating operation order for byte-level map
    # fingerprints; this is geometry reconstruction, not an optimizer.
    translation, q = estimate
    t = [2.*x for x in cross(q[1:], point)]
    v = cross(q[1:], t)
    return [(p+(q[0]*x+y))+z for p, x, y, z in zip(point, t, v, translation)]


def fusion(original, scan, estimate, policy, cfg):
    proposed = copy.deepcopy(original)
    for raw in scan:
        point = transform(estimate, raw)
        if not all(math.isfinite(x) for x in point):
            return None, 'submap transformed coordinate exceeds voxel bound'
        cells = [math.floor(x/policy['voxel_m']) for x in point]
        if any(abs(x) > 1e14 for x in cells):
            return None, 'submap transformed coordinate exceeds voxel bound'
        key = tuple(cells)
        if key in proposed:
            mean, count = proposed[key]
            if count >= policy['max_points_per_voxel']:
                return None, 'submap voxel observation-count limit reached'
            updated = [m+(p-m)/(count+1) for m, p in zip(mean, point)]
            if not all(math.isfinite(x) for x in updated):
                return None, 'nonfinite submap voxel representative'
            proposed[key] = updated, count+1
        else:
            if len(proposed) >= cfg['max_map_points']:
                return None, 'submap point limit reached'
            proposed[key] = point, 1
    if len(proposed) < cfg['min_pairs']:
        return None, 'too few fused submap representatives'
    return proposed, None


def map_points(voxels):
    return [voxels[key][0] for key in sorted(voxels)]


def map_digest(voxels):
    return r.digest(b''.join(struct.pack('<ddd', *p) for p in map_points(voxels)))


def statistics_digest(voxels):
    return r.digest(b''.join(struct.pack('<qqqQddd', *key, voxels[key][1], *voxels[key][0])
                             for key in sorted(voxels)))


def state_check(row, when, voxels, generation, last_accepted, last_map_update, prior, lost, origin):
    r.require(row['map_generation_'+when] == generation, 'invented map generation '+when)
    r.require(row['map_points_'+when] == len(voxels), 'invented map size '+when)
    r.require(row['map_sha256_'+when] == map_digest(voxels), 'invented map geometry '+when)
    r.require(row['map_statistics_sha256_'+when] == statistics_digest(voxels),
              'invented voxel counts/keys '+when)
    snapshot = row['state_'+when]
    for field in ('map_generation', 'map_points', 'map_sha256', 'map_statistics_sha256'):
        r.require(snapshot[field] == row[field+'_'+when], 'snapshot/flattened map disagreement '+when)
    r.require(snapshot['last_accepted_stamp'] == last_accepted
              and snapshot['last_map_update_stamp'] == last_map_update
              and snapshot['lost'] == lost and snapshot['root_origin_frame_index'] == origin,
              'invented persistent localization state '+when)
    for field in ('last_accepted_stamp', 'last_map_update_stamp', 'lost'):
        r.require(row[field+'_'+when] == snapshot[field], 'snapshot/flattened validity disagreement '+when)
    if origin is None:
        r.require(snapshot['last_accepted_pose'] is None, 'invented uninitialized pose')
    else:
        k.same_pose(exact_pose(snapshot['last_accepted_pose']), prior, 'invented last accepted pose '+when)


def registration_check(fit, scan, target, estimate, prior, truth, cfg, uncertainty):
    r.require(fit['accepted'] is True, 'pose accepted rejected registration')
    r.require(r.norm([x-y for x, y in zip(estimate[0], prior[0])])
              <= cfg['max_translation_jump_m']
              and r.quaternion_error(estimate[1], prior[1]) <= cfg['max_rotation_jump_rad'],
              'unsafe registration jump')
    n, rms = r.correspondences(scan, target, estimate, cfg)
    r.close(fit['rms_m'], rms, 'invented final residual', 1e-7)
    r.close(fit['inlier_fraction'], n/len(scan), 'invented overlap')
    r.require(n >= cfg['min_pairs'] and rms <= cfg['max_rms_m']
              and n/len(scan) >= cfg['min_overlap'], 'unsafe accepted geometry')
    r.require(fit['ambiguity_probes'] == 12
              and 0 < fit['neighbor_checks'] <= cfg['max_neighbor_checks'],
              'unsafe ambiguity/budget shortcut')
    conditional = fit['conditional_covariance_xyz_rotation']
    provisional = fit['provisional_covariance_xyz_rotation']
    for i in range(6):
        for j in range(6):
            floor = uncertainty['position_std_floor_m'] if i < 3 else uncertainty['rotation_std_floor_rad']
            r.close(provisional[i][j], conditional[i][j]+(floor*floor if i == j else 0),
                    'changed conditional uncertainty allowance')
    r.covariance_nees(conditional, [0.]*6)
    r.covariance_nees(provisional, [0.]*6)
    return k.scored_pose(fit, estimate, truth, 'root registration')


def audit(report, manifest, clouds, gt):
    r.require(report['schema_version'] == 1 and report['ground_truth_operational'] is False
              and report['raw_redistributed'] is False, 'schema/truth/redistribution')
    for key in ('dataset', 'repository', 'revision'):
        r.require(report[key] == manifest[key], 'source identity')
    frames = manifest['frames']
    r.require(len(report['frames']) == len(frames), 'omitted rejected observations')
    cfg = report['freeze']['registration_config']
    policy = report['freeze']['submap_policy']
    uncertainty = report['freeze']['uncertainty']
    r.require(policy['voxel_m'] == .06 and policy['map_update_interval_s'] == .10
              and policy['max_unobserved_s'] == .20
              and policy['max_points_per_voxel'] == 1000 and cfg['max_map_points'] == 5000,
              'changed frozen fusion/resources/expiry')
    r.require(cfg == REGISTRATION_CONFIG, 'changed frozen registration acceptance')
    origin = k.timed_pose(gt, frames[0]['timestamp'])
    voxels, generation = {}, 0
    prior = IDENTITY
    last_accepted = last_observed = last_map_update = None
    lost = False
    counters = dict(initialized_frames=0, accepted_updates=0, rejected_updates=0,
                    accurate_root_updates=0, reference_valid_updates=0,
                    map_updates=0, map_update_rejections=0)
    records = []
    for i, row in enumerate(report['frames']):
        frame, scan = frames[i], clouds[i]
        for key in ('file', 'timestamp', 'source_index', 'split'):
            r.require(row[key] == frame[key], 'changed selected observation/order')
        r.require(type(row['accepted']) is bool and row['scan_points'] == len(scan),
                  'invalid acceptance/geometry count')
        r.require(isinstance(row['cpu_wall_seconds'], (int, float))
                  and math.isfinite(row['cpu_wall_seconds']) and row['cpu_wall_seconds'] >= 0,
                  'invalid CPU timing')
        state_check(row, 'before', voxels, generation, last_accepted, last_map_update, prior, lost,
                    frames[0]['source_index'] if voxels else None)
        stamp, index = row['timestamp'], row['source_index']
        chronological = (math.isfinite(stamp) and 0 <= stamp <= 1e12 and
                         (last_observed is None or
                          (stamp > last_observed[0] and index > last_observed[1])))
        if chronological and not lost:
            last_observed = (stamp, index)
            if last_accepted is not None and stamp-last_accepted > policy['max_unobserved_s']+1e-9:
                lost = True
        truth = k.truth_relative(origin, k.timed_pose(gt, stamp))
        score = row['root_accuracy']
        r.require(score['valid'] == row['accepted'], 'invented rejected root validity')
        if i:
            counters['reference_valid_updates'] += int(truth is not None)
        evidence = dict(source_index=index, accepted=row['accepted'])
        if not row['accepted']:
            r.require(row.get('rejection') and row['map_updated'] is False
                      and row['initialized'] is False
                      and not any(field in row for field in (
                          'root_estimate', 'initial_pose', 'registration', 'root_origin_frame_index',
                          'map_update_rejection')), 'invented rejected pose/fusion')
            k.scored_pose(score, None, truth, 'rejected root')
            if not voxels:
                lost = True
            if i:
                counters['rejected_updates'] += 1
            evidence['rejection'] = row['rejection']
        else:
            r.require(chronological and not lost, 'accepted expired/lost/nonchronological observation')
            initializing = not voxels
            r.require(row['initialized'] == initializing
                      and row['root_origin_frame_index'] == frames[0]['source_index'],
                      'invented initialization/new origin')
            k.same_pose(exact_pose(row['initial_pose']), prior, 'truth/invented initialization prior')
            fit = row['registration']
            fit_pose = exact_pose(fit['estimate'])
            evidence['registration'] = registration_check(
                fit, scan, scan if initializing else map_points(voxels),
                fit_pose, prior, truth, cfg, uncertainty)
            estimate = IDENTITY if initializing else fit_pose
            k.same_pose(exact_pose(row['root_estimate']), estimate, 'invented root pose')
            evidence['root'] = k.scored_pose(score, estimate, truth, 'root')
            due = initializing or stamp-last_map_update+1e-9 >= policy['map_update_interval_s']
            candidate, rejection = fusion(voxels, scan, estimate, policy, cfg) if due else (None, None)
            updated = due and rejection is None
            r.require(row['map_updated'] == updated, 'invented update/timing/transaction')
            r.require(row.get('map_update_rejection') == rejection, 'concealed/invented atomic fusion rejection')
            if updated:
                voxels, generation, last_map_update = candidate, generation+1, stamp
            if i:
                counters['accepted_updates'] += 1
                counters['accurate_root_updates'] += int(score['within_accuracy_gates'])
            counters['initialized_frames'] += int(initializing)
            counters['map_updates'] += int(updated)
            counters['map_update_rejections'] += int(rejection is not None)
            last_accepted, prior = stamp, estimate
            evidence.update(map_updated=updated, map_update_rejection=rejection)
        state_check(row, 'after', voxels, generation, last_accepted, last_map_update, prior, lost,
                    frames[0]['source_index'] if voxels else None)
        r.require(row['last_accepted_stamp_after'] == last_accepted and row['lost_after'] == lost,
                  'invented accepted-pose validity state')
        records.append(evidence)
    updates = len(frames)-1
    passed = (counters['initialized_frames'] == 1 and counters['accepted_updates'] == updates
              and counters['accurate_root_updates'] == updates
              and counters['reference_valid_updates'] == updates)
    expected = dict(frames=len(frames), updates=updates, **counters,
                    final_map_points=len(voxels), final_map_generation=generation,
                    lost=lost, all_updates_passed=passed)
    r.require(report['summary'] == expected, 'omitted failures/invented summary')
    return records


def mutation_checks(report, manifest, clouds, gt):
    tests = [
        ('omit_observation', lambda x: x['frames'].pop()),
        ('invent_summary', lambda x: x['summary'].__setitem__('accepted_updates', 999)),
        ('operational_truth', lambda x: x.__setitem__('ground_truth_operational', True)),
        ('source_identity', lambda x: x.__setitem__('revision', 'unverified')),
        ('timestamp', lambda x: x['frames'][1].__setitem__('timestamp', 0.)),
        ('duplicate_acquisition', lambda x: x['frames'].__setitem__(1, copy.deepcopy(x['frames'][0]))),
        ('invent_map_points', lambda x: x['frames'][0].__setitem__('map_points_after', 99999)),
        ('invent_map_geometry', lambda x: x['frames'][0].__setitem__('map_sha256_after', '0'*64)),
        ('invent_voxel_counts', lambda x: x['frames'][0].__setitem__('map_statistics_sha256_after', '0'*64)),
        ('invent_generation', lambda x: x['frames'][0].__setitem__('map_generation_after', 999)),
        ('invent_initial_fusion', lambda x: x['frames'][0].__setitem__('map_updated', False)),
        ('conceal_expiry', lambda x: x['frames'][0].__setitem__('lost_after', not x['frames'][0]['lost_after'])),
        ('invent_accepted_clock', lambda x: x['frames'][0].__setitem__('last_accepted_stamp_after', 0.)),
        ('weaken_minimum_pairs', lambda x: x['freeze']['registration_config'].__setitem__('min_pairs', 1)),
        ('weaken_ambiguity_ratio', lambda x: x['freeze']['registration_config'].__setitem__('ambiguity_rms_ratio', 0.)),
    ]
    accepted = next((i for i, row in enumerate(report['frames']) if i and row['accepted']), None)
    if accepted is not None:
        tests.extend([
            ('root_pose', lambda x: x['frames'][accepted]['root_estimate']['translation_m'].__setitem__(0, 100.)),
            ('new_root_origin', lambda x: x['frames'][accepted].__setitem__('root_origin_frame_index', 999)),
            ('truth_prior', lambda x: x['frames'][accepted]['initial_pose']['translation_m'].__setitem__(0, 100.)),
            ('invent_residual', lambda x: x['frames'][accepted]['registration'].__setitem__('rms_m', 100.)),
            ('omit_ambiguity', lambda x: x['frames'][accepted]['registration'].__setitem__('ambiguity_probes', 0)),
            ('exceed_budget', lambda x: x['frames'][accepted]['registration'].__setitem__('neighbor_checks', 20_000_001)),
            ('non_spd_covariance', lambda x: x['frames'][accepted]['registration']['provisional_covariance_xyz_rotation'][0].__setitem__(0, -1.)),
            ('invent_accuracy', lambda x: x['frames'][accepted]['root_accuracy'].__setitem__('translation_error_m', 100.)),
            ('invent_map_update', lambda x: x['frames'][accepted].__setitem__('map_updated', not x['frames'][accepted]['map_updated'])),
            ('invent_transaction_rejection', lambda x: x['frames'][accepted].__setitem__('map_update_rejection', 'invented failure')),
        ])
    rejected = next((i for i, row in enumerate(report['frames']) if not row['accepted']), None)
    if rejected is not None:
        tests.extend([
            ('invent_rejected_pose', lambda x: x['frames'][rejected].__setitem__('root_estimate', dict(
                translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))),
            ('fuse_rejected_fit', lambda x: x['frames'][rejected].__setitem__('map_updated', True)),
        ])
    rejected_names = []
    for name, change in tests:
        altered = copy.deepcopy(report)
        change(altered)
        try:
            audit(altered, manifest, clouds, gt)
        except (ValueError, KeyError, IndexError):
            rejected_names.append(name)
        else:
            raise ValueError('corrupted report accepted: '+name)
    if accepted is not None:
        altered, selected = copy.deepcopy(report), copy.deepcopy(manifest)
        expired = altered['frames'][accepted-1]['timestamp']+.201
        altered['frames'][accepted]['timestamp'] = expired
        selected['frames'][accepted]['timestamp'] = expired
        try:
            audit(altered, selected, clouds, gt)
        except (ValueError, KeyError, IndexError) as error:
            r.require('expired/lost/nonchronological' in str(error),
                      'expiry mutation did not exercise accepted-age gate: '+str(error))
            rejected_names.append('accepted_pose_after_expiry')
        else:
            raise ValueError('expired accepted pose was trusted')
    return rejected_names


def fusion_contract_checks():
    """Independent small arithmetic/transaction examples, unrelated to ICP."""
    policy = dict(voxel_m=.06, max_points_per_voxel=3)
    cfg = dict(max_map_points=2, min_pairs=1)
    initial, error = fusion({}, [[.001, .002, .003], [.003, .004, .005]], IDENTITY, policy, cfg)
    r.require(error is None and initial == {(0, 0, 0): ([.002, .003, .004], 2)},
              'online raw-observation weighting example')
    before = copy.deepcopy(initial)
    candidate, error = fusion(initial, [[.005, .006, .007], [.007, .008, .009]], IDENTITY, policy, cfg)
    r.require(candidate is None and error == 'submap voxel observation-count limit reached'
              and initial == before, 'count-cap rejection partially committed')
    candidate, error = fusion(initial, [[.12, 0., 0.], [.24, 0., 0.]], IDENTITY, policy, cfg)
    r.require(candidate is None and error == 'submap point limit reached' and initial == before,
              'point-cap rejection partially committed')
    shifted, error = fusion({}, [[.005, 0., 0.]], ([.12, 0., 0.], [1., 0., 0., 0.]), policy, cfg)
    r.require(error is None and shifted == {(2, 0, 0): ([.125, 0., 0.], 1)},
              'sensor-to-root measured point example')
    return ['raw_weighted_online_mean', 'count_cap_atomic_rejection', 'point_cap_atomic_rejection',
            'fixed_root_sensor_transform']


def verify_sources(freeze, manifest_raw, source_root):
    sources = {
        'matcher_source_sha256': 'crates/localization/src/registration3d.rs',
        'evaluator_source_sha256': 'integrations/rgbd/src/main.rs',
        'motion_source_sha256': 'integrations/rgbd/src/motion.rs',
        'submap_source_sha256': 'integrations/rgbd/src/submaps.rs',
        'submap_core_source_sha256': 'crates/localization/src/submap3d.rs',
        'cargo_lock_sha256': 'integrations/rgbd/Cargo.lock',
        'independent_checker_sha256': 'scripts/check-recorded-submaps.py',
        'geometry_checker_sha256': 'scripts/check-recorded-rgbd.py',
        'keyframe_checker_sha256': 'scripts/check-recorded-keyframes.py',
        'localization_lib_source_sha256': 'crates/localization/src/lib.rs',
        'core_lib_source_sha256': 'crates/core/src/lib.rs',
        'workspace_cargo_lock_sha256': 'Cargo.lock',
        'rust_toolchain_sha256': 'rust-toolchain.toml',
        'cargo_manifest_sha256': 'integrations/rgbd/Cargo.toml',
        'acquisition_source_sha256': 'scripts/fetch-submap-datasets.py',
    }
    for key, path in sources.items():
        r.require(freeze[key] == r.digest(r.bounded(source_root/path)), 'frozen source changed '+path)
    r.require(freeze['manifest_sha256'] == r.digest(manifest_raw), 'manifest changed')
    for name in ('registration_config', 'submap_policy', 'motion_preprocessing', 'preprocessing'):
        r.require(freeze[name+'_sha256'] == r.canonical(freeze[name]), 'configuration freeze mismatch '+name)
    r.require(freeze['motion_preprocessing']['additional_voxel_m'] == .06,
              'changed measured-cloud coarse preprocessing')
    r.require(freeze['uncertainty']['position_std_floor_m'] == .02
              and freeze['uncertainty']['rotation_std_floor_rad'] == .03,
              'changed conditional covariance allowance')
    expected_preprocessing = dict(width=640, height=480, fx=525., fy=525., cx=319.5,
                                  cy=239.5, units_per_metre=5000, pixel_step=8,
                                  sample_origin_pixel=[0, 0], min_depth_m=.3, max_depth_m=5.,
                                  voxel_m=.03, voxel_representative='first row-major sampled valid pixel',
                                  point_order='lexicographic voxel key', maximum_sampled_pixels=4800)
    manifest = json.loads(manifest_raw)
    if freeze['protocol_version'] >= 8:
        r.require(type(freeze['regression_requested']) is bool, 'invalid viewed-regression role flag')
    if manifest['dataset'] in NEW_CALIBRATIONS:
        parameters = NEW_CALIBRATIONS[manifest['dataset']]
        expected_preprocessing.update(parameters)
        expected_kind = ('calibration_regression'
                         if (manifest['dataset'] == 'tum-fr1-desk-submaps'
                             and freeze['protocol_version'] >= 8)
                         or freeze.get('regression_requested', False)
                         else 'preregistered_sequence')
        r.require(freeze['kind'] == expected_kind, 'misclassified viewed/unviewed sequence')
        r.require(freeze['depth_calibration'] == manifest['depth_calibration']
                  and freeze['calibration_source'] == manifest['calibration_source'],
                  'changed precise camera calibration provenance')
        for key in parameters:
            r.require(manifest['depth_calibration'][key] == parameters[key],
                      'camera projection differs from pinned published parameters')
    else:
        r.require(freeze['kind'] == 'calibration_regression', 'misclassified viewed sequence')
    r.require(freeze['preprocessing'] == expected_preprocessing,
              'unsupported measured-sensor calibration')


def freeze_mutation_checks(freeze, manifest_raw, source_root):
    names = []
    for field in ('submap_core_source_sha256', 'evaluator_source_sha256',
                  'geometry_checker_sha256', 'independent_checker_sha256',
                  'acquisition_source_sha256', 'manifest_sha256'):
        altered = copy.deepcopy(freeze)
        altered[field] = '0'*64
        try:
            verify_sources(altered, manifest_raw, source_root)
        except (ValueError, KeyError):
            names.append('changed_'+field)
        else:
            raise ValueError('altered frozen provenance accepted '+field)
    return names


def main():
    global r, k
    parser = argparse.ArgumentParser()
    for name in ('report', 'manifest', 'raw', 'freeze', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--source-snapshot', type=Path,
                        help='Byte-verified historical source root; original freeze is mandatory')
    args = parser.parse_args()
    report_raw, manifest_raw, freeze_raw = (r.bounded(path) for path in (
        args.report, args.manifest, args.freeze))
    report, manifest, freeze = map(json.loads, (report_raw, manifest_raw, freeze_raw))
    r.require(report['freeze'] == freeze and report['freeze_sha256'] == r.digest(freeze_raw),
              'external freeze mismatch')
    verify_sources(freeze, manifest_raw, args.source_snapshot or ROOT)
    provenance_rejected = freeze_mutation_checks(freeze, manifest_raw, args.source_snapshot or ROOT)
    if args.source_snapshot is not None:
        for name, path in (
                ('archived_submap_geometry', 'scripts/check-recorded-rgbd.py'),
                ('archived_submap_scores', 'scripts/check-recorded-keyframes.py')):
            spec = importlib.util.spec_from_file_location(name, args.source_snapshot/path)
            module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(module)
            if name.endswith('geometry'):
                r = module
            else:
                k = module
                k.r = r
    raw = {}
    for item in manifest['files']:
        name = item['file']
        r.require(Path(name).name == name and '/' not in name and '\\' not in name, 'unsafe raw path')
        content = r.bounded(args.raw/name)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'], 'raw SHA mismatch')
        raw[name] = content
    r.require(report['files'] == [{key: item[key] for key in ('file', 'bytes', 'sha256', 'role')}
                                  for item in manifest['files']], 'report source provenance mismatch')
    if manifest.get('calibration_source') is not None:
        calibration = manifest['calibration_source']
        r.require(calibration['file'] == 'camera-calibration.yaml', 'unsafe calibration raw path')
        content = r.bounded(args.raw/calibration['file'])
        r.require(len(content) == calibration['bytes'] and r.digest(content) == calibration['sha256'],
                  'camera calibration raw hash mismatch')
        r.require(report['calibration_source'] == calibration
                  and report['calibration_sha256_verified'] is True,
                  'report omits sensor calibration provenance')
        numbers = {}
        for line in content.decode('utf-8').splitlines():
            match = re.fullmatch(r'\s*(Camera\.(?:fx|fy|cx|cy|width|height|k[123]|p[12])|DepthMapFactor)\s*:\s*([-+0-9.eE]+)\s*(?:#.*)?', line)
            if match:
                r.require(match[1] not in numbers, 'duplicate published camera parameter')
                numbers[match[1]] = float(match[2])
        parameters = NEW_CALIBRATIONS[manifest['dataset']]
        for field, value in dict(**parameters, width=640, height=480).items():
            r.require(numbers['Camera.'+field] == value, 'published camera projection mismatch '+field)
        r.require(numbers['DepthMapFactor'] == 5000., 'published depth scale mismatch')
        for field, value in manifest['depth_calibration']['source_rgb_distortion'].items():
            r.require(numbers['Camera.'+field] == value,
                      'invented provenance-only RGB distortion '+field)
    index = [line.split() for line in raw['depth.txt'].decode().splitlines()
             if line and not line.startswith('#')]
    clouds = []
    for frame in manifest['frames']:
        source = index[frame['source_index']]
        r.require(len(source) == 2 and float(source[0]) == frame['timestamp']
                  and source[1] == 'depth/'+frame['file'],
                  'source-index selection mismatch')
        coarse = {}
        for point in depth_geometry(raw[frame['file']], freeze['preprocessing']):
            coarse.setdefault(tuple(math.floor(x/.06) for x in point), point)
        clouds.append([coarse[key] for key in sorted(coarse)])
    gt = r.gt_rows(raw['groundtruth.txt'])
    records = audit(report, manifest, clouds, gt)
    rejected = mutation_checks(report, manifest, clouds, gt)
    contracts = fusion_contract_checks()
    missing = k.missing_reference_contract_checks()
    root_records = [row['root'] for row in records[1:] if row.get('root')]
    result = dict(schema_version=1, passed_integrity=True, summary=report['summary'], frames=records,
                  report_sha256=r.digest(report_raw), manifest_sha256=r.digest(manifest_raw),
                  freeze_sha256=r.digest(freeze_raw), mutations_rejected=rejected,
                  frozen_provenance_mutations_rejected=provenance_rejected,
                  running_checker_sha256=r.digest(r.bounded(Path(__file__))),
                  frozen_checker_sha256=freeze['independent_checker_sha256'],
                  archived_source_snapshot=args.source_snapshot is not None,
                  fusion_contract_checks=contracts, missing_reference_score_mutations_rejected=missing,
                  raw_sha256_verified=True, source_freeze_verified=True, geometry_reconstructed=True,
                  every_map_generation_independently_reconstructed=True,
                  physical_pose_independently_scored=True, optimizer_independently_rerun=False,
                  root_confidence_or_calibrated_covariance_claim=False,
                  maximum_root_translation_error_m=max((row['translation_error_m'] for row in root_records), default=None),
                  maximum_root_rotation_error_rad=max((row['rotation_error_rad'] for row in root_records), default=None))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result['summary']))


if __name__ == '__main__':
    main()
