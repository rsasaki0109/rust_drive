#!/usr/bin/env python3
"""Independent measured-keyframe transition, geometry, expiry and mocap oracle.

Reconstructs recorded clouds and accepted-fit final residuals, not the optimizer.
Every rejection stays in the denominator; per-fit covariance is not root confidence.
"""
import argparse
import copy
import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
_spec = importlib.util.spec_from_file_location('rgbd_oracle',
                                              Path(__file__).with_name('check-recorded-rgbd.py'))
r = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(r)
IDENTITY = ([0., 0., 0.], [1., 0., 0., 0.])


def compose(a, b):
    return ([x+y for x, y in zip(a[0], r.rotate(a[1], b[0]))],
            r.qnorm(r.multiply(a[1], b[1])))


def same_pose(actual, expected, message):
    for x, y in zip(actual[0], expected[0]):
        r.close(x, y, message, 1e-8)
    r.close(r.quaternion_error(actual[1], expected[1]), 0., message, 1e-7)


def timed_pose(gt, stamp):
    try:
        return r.interpolate(gt, stamp)
    except ValueError:
        return None


def truth_relative(a, b):
    return r.relative(a, b) if a is not None and b is not None else None


def scored_pose(row, estimate, truth, message):
    r.require(row['reference_valid'] == (truth is not None), message+' reference availability')
    if truth is None:
        r.require(row.get('reference_rejection') and row['within_accuracy_gates'] is False,
                  message+' hidden missing reference')
        r.require(not any(k in row for k in ('evaluation_only_truth', 'translation_error_m',
                                            'rotation_error_rad')), message+' invented truth')
        return None
    same_pose(r.pose(row['evaluation_only_truth']), truth, message+' mocap pose')
    if estimate is None:
        r.require(row['within_accuracy_gates'] is False and not any(
            k in row for k in ('estimate', 'translation_error_m', 'rotation_error_rad')),
            message+' invented rejected score')
        return None
    same_pose(r.pose(row['estimate']), estimate, message+' score estimate')
    position = r.norm([x-y for x, y in zip(estimate[0], truth[0])])
    angle = r.quaternion_error(estimate[1], truth[1])
    r.close(row['translation_error_m'], position, message+' translation error')
    r.close(row['rotation_error_rad'], angle, message+' rotation error')
    within = position <= .1 and angle <= .1
    r.require(row['within_accuracy_gates'] == within, message+' lowered accuracy gates')
    return dict(translation_error_m=position, rotation_error_rad=angle,
                within_accuracy_gates=within)


def audit(report, manifest, clouds, gt):
    r.require(report['schema_version'] == 1 and report['ground_truth_operational'] is False
              and report['raw_redistributed'] is False, 'truth/redistribution/schema')
    for key in ('dataset', 'repository', 'revision'):
        r.require(report[key] == manifest[key], 'source identity')
    frames = manifest['frames']
    r.require(len(report['frames']) == len(frames), 'omitted rejected observations')
    cfg = report['freeze']['registration_config']
    policy = report['freeze']['keyframe_policy']
    uncertainty = report['freeze']['uncertainty']
    r.require(policy['keyframe_interval_s'] == .10 and policy['max_unobserved_s'] == .20,
              'changed frozen replacement/expiry')
    origin = timed_pose(gt, frames[0]['timestamp'])
    reference = None
    root_reference = IDENTITY
    prior = IDENTITY
    last_accepted = None
    last_observed = None
    lost = False
    records = []
    counters = dict(initialized_frames=0, accepted_updates=0, rejected_updates=0,
                    accurate_root_updates=0, reference_valid_updates=0,
                    keyframe_replacements=0)
    for i, row in enumerate(report['frames']):
        frame = frames[i]
        for key in ('file', 'timestamp', 'source_index', 'split'):
            r.require(row[key] == frame[key], 'changed selection or observation order')
        r.require(row['scan_points'] == len(clouds[i]), 'invented measured cloud count')
        r.require(type(row['accepted']) is bool, 'invalid acceptance')
        r.require(isinstance(row['cpu_wall_seconds'], (int, float))
                  and math.isfinite(row['cpu_wall_seconds']) and row['cpu_wall_seconds'] >= 0,
                  'invalid CPU timing')
        stamp = row['timestamp']
        index = row['source_index']
        chronological = (last_observed is None or
                         (stamp > last_observed[0] and index > last_observed[1]))
        # A chronological input is observed even if its registration rejects;
        # rejected timestamps never renew accepted-observation validity.
        if chronological:
            last_observed = (stamp, index)
        if last_accepted is not None and stamp-last_accepted > policy['max_unobserved_s']+1e-9:
            lost = True
        current_truth = timed_pose(gt, stamp)
        root_truth = truth_relative(origin, current_truth)
        root_score = row['root_accuracy']
        r.require(root_score['valid'] == row['accepted'], 'invented rejected root validity')
        evidence = dict(source_index=index, accepted=row['accepted'])
        if i:
            counters['reference_valid_updates'] += int(root_truth is not None)
        if not row['accepted']:
            r.require(row.get('rejection') and not any(k in row for k in (
                'root_estimate', 'root_from_reference', 'initial_pose', 'registration',
                'reference_frame_index', 'reference_points', 'initialized', 'keyframe_replaced')),
                'invented rejected pose/reference update')
            scored_pose(root_score, None, root_truth, 'rejected root')
            if reference is None:
                lost = True
            if i:
                counters['rejected_updates'] += 1
            evidence['rejection'] = row['rejection']
            records.append(evidence)
            continue
        r.require(chronological and not lost, 'accepted expired/lost/nonchronological observation')
        initializing = reference is None
        active = i if initializing else reference
        expected_root_reference = IDENTITY if initializing else root_reference
        expected_prior = IDENTITY if initializing else prior
        r.require(row['root_origin_frame_index'] == frames[0]['source_index'], 'silently changed root origin')
        r.require(row['reference_frame_index'] == frames[active]['source_index']
                  and row['reference_stamp'] == frames[active]['timestamp']
                  and row['reference_file'] == frames[active]['file'], 'invented/stale reference')
        r.require(row['reference_points'] == len(clouds[active]), 'invented reference geometry')
        same_pose(r.pose(row['root_from_reference']), expected_root_reference, 'root reference chain')
        same_pose(r.pose(row['initial_pose']), expected_prior, 'truth/invented registration prior')
        r.require(row['initialized'] == initializing, 'invented initialization')
        fit = row['registration']
        r.require(fit['accepted'] is True, 'root accepted rejected registration')
        estimate = r.pose(fit['estimate'])
        r.require(r.norm([x-y for x, y in zip(estimate[0], expected_prior[0])])
                  <= cfg['max_translation_jump_m']
                  and r.quaternion_error(estimate[1], expected_prior[1])
                  <= cfg['max_rotation_jump_rad'], 'unsafe local registration jump')
        n, rms = r.correspondences(clouds[i], clouds[active], estimate, cfg)
        r.close(fit['rms_m'], rms, 'invented final residual', 1e-7)
        r.close(fit['inlier_fraction'], n/len(clouds[i]), 'invented overlap')
        r.require(rms <= cfg['max_rms_m'] and n >= cfg['min_pairs']
                  and n/len(clouds[i]) >= cfg['min_overlap'], 'unsafe geometry acceptance')
        r.require(fit['ambiguity_probes'] == 12 and 0 < fit['neighbor_checks'] <= 20_000_000,
                  'unsafe ambiguity/budget shortcut')
        conditional = fit['conditional_covariance_xyz_rotation']
        provisional = fit['provisional_covariance_xyz_rotation']
        for j in range(6):
            for k in range(6):
                floor = uncertainty['position_std_floor_m'] if j < 3 else uncertainty['rotation_std_floor_rad']
                r.close(provisional[j][k], conditional[j][k]+(floor*floor if j == k else 0),
                        'changed per-fit uncertainty allowance')
        r.covariance_nees(conditional, [0.]*6)
        r.covariance_nees(provisional, [0.]*6)
        reference_truth = truth_relative(timed_pose(gt, frames[active]['timestamp']), current_truth)
        evidence['local_fit'] = scored_pose(fit, estimate, reference_truth, 'local fit')
        chained = compose(expected_root_reference, estimate)
        same_pose(r.pose(row['root_estimate']), chained, 'incorrect/invented root composition')
        evidence['root'] = scored_pose(root_score, chained, root_truth, 'root')
        if reference_truth is not None:
            delta = [x-y for x, y in zip(estimate[0], reference_truth[0])]
            angle = r.quaternion_error(estimate[1], reference_truth[1])
            dq = r.qnorm(r.multiply(estimate[1], r.conjugate(reference_truth[1])))
            length = r.norm(dq[1:])
            rotation = [x*angle/length for x in dq[1:]] if length > 1e-12 else [0.]*3
            evidence['local_conditional_nees'] = r.covariance_nees(conditional, delta+rotation)
            evidence['local_provisional_nees'] = r.covariance_nees(provisional, delta+rotation)
        replaced = not initializing and stamp-frames[active]['timestamp']+1e-9 >= policy['keyframe_interval_s']
        r.require(row['keyframe_replaced'] == replaced, 'incorrect replacement timing')
        counters['initialized_frames'] += int(initializing)
        counters['keyframe_replacements'] += int(replaced)
        if i:
            counters['accepted_updates'] += 1
            counters['accurate_root_updates'] += int(root_score['within_accuracy_gates'])
        last_accepted = stamp
        if initializing or replaced:
            reference, root_reference, prior = i, chained, IDENTITY
        else:
            prior = estimate
        evidence['reference_frame_index'] = frames[active]['source_index']
        evidence['keyframe_replaced'] = replaced
        records.append(evidence)
    updates = len(frames)-1
    passed = (counters['initialized_frames'] == 1
              and counters['accepted_updates'] == updates
              and counters['accurate_root_updates'] == updates
              and counters['reference_valid_updates'] == updates)
    expected = dict(frames=len(frames), updates=updates, **counters, all_updates_passed=passed)
    r.require(report['summary'] == expected, 'summary omitted failures or invented acceptance')
    return records


def mutation_checks(report, manifest, clouds, gt):
    tests = [('omit_observation', lambda x: x['frames'].pop()),
             ('invent_summary', lambda x: x['summary'].__setitem__('accepted_updates', 999)),
             ('truth_operational', lambda x: x.__setitem__('ground_truth_operational', True)),
             ('timestamp', lambda x: x['frames'][1].__setitem__('timestamp', 0.)),
             ('duplicate_observation', lambda x: x['frames'].__setitem__(1, copy.deepcopy(x['frames'][0]))),
             ('reversed_observations', lambda x: x['frames'].__setitem__(slice(1, 3), list(reversed(x['frames'][1:3]))))]
    accepted = next((i for i, x in enumerate(report['frames']) if i and x['accepted']), None)
    if accepted is not None:
        tests.extend([
            ('root_pose', lambda x: x['frames'][accepted]['root_estimate']['translation_m'].__setitem__(0, 100.)),
            ('root_origin', lambda x: x['frames'][accepted].__setitem__('root_origin_frame_index', 999)),
            ('root_chain', lambda x: x['frames'][accepted]['root_from_reference']['translation_m'].__setitem__(0, 100.)),
            ('truth_prior', lambda x: x['frames'][accepted]['initial_pose']['translation_m'].__setitem__(0, 100.)),
            ('reference', lambda x: x['frames'][accepted].__setitem__('reference_frame_index', 999)),
            ('residual', lambda x: x['frames'][accepted]['registration'].__setitem__('rms_m', 100.)),
            ('covariance', lambda x: x['frames'][accepted]['registration']['provisional_covariance_xyz_rotation'][0].__setitem__(0, -1.)),
            ('invent_accuracy', lambda x: x['frames'][accepted]['root_accuracy'].__setitem__('translation_error_m', 100.)),
            ('keyframe_timing', lambda x: x['frames'][accepted].__setitem__('keyframe_replaced', not x['frames'][accepted]['keyframe_replaced']))])
    rejected = next((i for i, x in enumerate(report['frames']) if not x['accepted']), None)
    if rejected is not None:
        tests.append(('invent_rejected_pose', lambda x: x['frames'][rejected].__setitem__(
            'root_estimate', dict(translation_m=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.]))))
    failed = []
    for name, change in tests:
        altered = copy.deepcopy(report)
        change(altered)
        try:
            audit(altered, manifest, clouds, gt)
        except (ValueError, KeyError, IndexError):
            failed.append(name)
        else:
            raise ValueError('corrupted report accepted: '+name)
    return failed


def main():
    parser = argparse.ArgumentParser()
    for name in ('report', 'manifest', 'raw', 'freeze', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    report_raw = r.bounded(args.report)
    report = json.loads(report_raw)
    manifest_raw = r.bounded(args.manifest)
    manifest = json.loads(manifest_raw)
    freeze_raw = r.bounded(args.freeze)
    freeze = json.loads(freeze_raw)
    r.require(report['freeze'] == freeze and report['freeze_sha256'] == r.digest(freeze_raw),
              'external freeze mismatch')
    sources = {'matcher_source_sha256': 'crates/localization/src/registration3d.rs',
               'evaluator_source_sha256': 'integrations/rgbd/src/main.rs',
               'motion_source_sha256': 'integrations/rgbd/src/motion.rs',
               'keyframe_source_sha256': 'integrations/rgbd/src/keyframes.rs',
               'keyframe_core_source_sha256': 'crates/localization/src/keyframes3d.rs',
               'cargo_lock_sha256': 'integrations/rgbd/Cargo.lock',
               'acquisition_source_sha256': 'scripts/fetch-keyframe-dataset.py',
               'independent_checker_sha256': 'scripts/check-recorded-keyframes.py',
               'geometry_checker_sha256': 'scripts/check-recorded-rgbd.py'}
    for key, path in sources.items():
        r.require(freeze[key] == r.digest(r.bounded(ROOT/path)), 'frozen source changed '+path)
    r.require(freeze['manifest_sha256'] == r.digest(manifest_raw), 'changed manifest')
    r.require(freeze['registration_config_sha256'] == r.canonical(freeze['registration_config']),
              'configuration freeze mismatch')
    for key in ('keyframe_policy', 'motion_preprocessing'):
        r.require(freeze[key+'_sha256'] == r.canonical(freeze[key]), 'policy/preprocessing freeze mismatch')
    r.require(freeze['motion_preprocessing']['additional_voxel_m'] == .06,
              'changed independently reconstructed preprocessing')
    r.require(freeze['uncertainty']['position_std_floor_m'] == .02
              and freeze['uncertainty']['rotation_std_floor_rad'] == .03,
              'changed previously frozen per-fit allowances')
    raw = {}
    for item in manifest['files']:
        name = item['file']
        r.require(Path(name).name == name and '/' not in name and '\\' not in name,
                  'unsafe source path')
        content = r.bounded(args.raw/name)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'], 'raw SHA mismatch')
        raw[name] = content
    r.require(report['files'] == [{key: item[key] for key in ('file', 'bytes', 'sha256', 'role')}
                                  for item in manifest['files']], 'report source provenance mismatch')
    index = [line.split() for line in raw['depth.txt'].decode().splitlines()
             if line and not line.startswith('#')]
    clouds = []
    for frame in manifest['frames']:
        source = index[frame['source_index']]
        r.require(float(source[0]) == frame['timestamp'] and Path(source[1]).name == frame['file'],
                  'source-index selection mismatch')
        coarse = {}
        for point in r.depth_geometry(raw[frame['file']]):
            coarse.setdefault(tuple(math.floor(x/.06) for x in point), point)
        clouds.append([coarse[key] for key in sorted(coarse)])
    gt = r.gt_rows(raw['groundtruth.txt'])
    records = audit(report, manifest, clouds, gt)
    rejected = mutation_checks(report, manifest, clouds, gt)
    root_records = [x['root'] for x in records[1:] if x.get('root')]
    result = dict(schema_version=1, passed_integrity=True, summary=report['summary'],
                  report_sha256=r.digest(report_raw), freeze_sha256=r.digest(freeze_raw),
                  manifest_sha256=r.digest(manifest_raw), frames=records,
                  mutations_rejected=rejected, raw_sha256_verified=True,
                  source_freeze_verified=True, geometry_reconstructed=True,
                  root_chain_independently_reconstructed=True,
                  physical_pose_independently_scored=True, optimizer_independently_rerun=False,
                  root_covariance_or_confidence_claim=False,
                  maximum_root_translation_error_m=max((x['translation_error_m'] for x in root_records), default=None),
                  maximum_root_rotation_error_rad=max((x['rotation_error_rad'] for x in root_records), default=None))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result['summary']))


if __name__ == '__main__':
    main()
