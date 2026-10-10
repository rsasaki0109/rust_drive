#!/usr/bin/env python3
"""Independent metadata qualification and measured-image motion oracle.

The pinned reprojection oracle supplies unchanged pixel reconstruction, Kabsch
coarse consensus, numerical-Jacobian/SVD refinement, clocks and physical scoring.
Only the new report algorithm name is normalized in a copy for that reuse.
Qualification is independently recomputed from timestamp columns and row arity;
no image pixels or ground-truth pose values enter metadata preregistration.
"""
import argparse
from bisect import bisect_left
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent.parent
OLD_REPROJECTION_SHA = 'fd1425f9c73c21b104d9453a7c2b298c21fa22effd6ecb5f0015bbc41f79cef7'
ALGORITHM = 'bounded_visual_reprojection_qualified'
DATASET = 'tum-fr1-room-qualified'
REPOSITORY = 'edrishakimi1/Indoor-SLAM-Floorplan-with-Gaussian-Splatting'
REVISION = '1d5b2c3e1ee186abc1709042b739e9cd8a3b41d1'
PREFIX = 'data/rgbd_dataset_freiburg1_room/'
OLD_PATH = Path(__file__).with_name('check-recorded-reprojection.py')
if hashlib.sha256(OLD_PATH.read_bytes()).hexdigest() != OLD_REPROJECTION_SHA:
    raise ValueError('original reprojection checker must remain immutable')
_spec = importlib.util.spec_from_file_location('immutable_reprojection_oracle', OLD_PATH)
g = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(g)
r, k, v = g.r, g.k, g.v

SOURCES = dict(g.v.SOURCES)
for _field in ('visual_source_sha256', 'reprojection_source_sha256',
               'reprojection_acquisition_source_sha256'):
    SOURCES.pop(_field)
SOURCES.update(
    evaluator_source_sha256='integrations/rgbd/src/bin/rustdriving-rgbd-qualified.rs',
    qualified_source_sha256='integrations/rgbd/src/qualified.rs',
    independent_checker_sha256='scripts/check-recorded-qualified.py',
    reprojection_checker_sha256='scripts/check-recorded-reprojection.py',
    qualification_source_sha256='scripts/qualify-rgbd-metadata.py',
    acquisition_source_sha256='scripts/fetch-qualified-dataset.py')


def timestamp_rows(content, columns, image_kind=None):
    """Never convert or interpret the seven ground-truth pose tokens."""
    r.require(len(content) <= 4194304, 'metadata file exceeds declared byte bound')
    rows = []
    for number, raw_line in enumerate(content.decode('utf8').splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith('#'):
            continue
        fields = line.split()
        r.require(len(fields) == columns, f'metadata row arity line {number}')
        try:
            stamp = float(fields[0])
        except ValueError:
            raise ValueError(f'invalid metadata timestamp line {number}') from None
        r.require(math.isfinite(stamp) and (not rows or stamp > rows[-1][0]),
                  f'nonfinite or nonmonotonic metadata timestamp line {number}')
        if image_kind is not None:
            name = fields[1]
            r.require(name.startswith(image_kind+'/') and name.endswith('.png')
                      and '\\' not in name and len(Path(name).parts) == 2
                      and '..' not in Path(name).parts, 'unsafe image index path')
            rows.append((stamp, name))
        else:
            rows.append((stamp,))
        r.require(len(rows) <= (20000 if image_kind is not None else 30000), 'metadata source row bound')
    r.require(len(rows) >= 2, 'insufficient metadata timestamps')
    return rows


def metadata_inputs(manifest, raw_path):
    files = {item['file']: item for item in manifest['files']}
    result = {}
    for name in ('depth.txt', 'rgb.txt', 'groundtruth.txt'):
        item = files[name]
        content = r.bounded(raw_path/name, 4194304)
        r.require(len(content) == item['bytes'] and r.digest(content) == item['sha256'],
                  'changed full-source qualification metadata bytes')
        if 'git_blob_sha1' in item:
            blob = b'blob '+str(len(content)).encode()+b'\0'+content
            r.require(hashlib.sha1(blob).hexdigest() == item['git_blob_sha1'],
                      'changed qualification metadata Git blob')
        result[name] = content
    return result


def independently_qualify(manifest, metadata):
    depth = timestamp_rows(metadata['depth.txt'], 2, 'depth')
    rgb = timestamp_rows(metadata['rgb.txt'], 2, 'rgb')
    truth = timestamp_rows(metadata['groundtruth.txt'], 8)
    truth_times = [row[0] for row in truth]
    rgb_times = [row[0] for row in rgb]
    frames = manifest['frames']
    r.require(len(frames) == 36 and [f['source_index'] for f in frames] == list(range(100, 136)),
              'changed fixed consecutive source window')
    maximum_bracket, maximum_gap = 0., 0.
    for ordinal, frame in enumerate(frames):
        r.require(frame['split'] == ('initialization' if ordinal == 0 else 'held_out'),
                  'changed preregistered source split')
        for kind, index, rows in [('depth', frame['source_index'], depth),
                                  ('rgb', frame['rgb_source_index'], rgb)]:
            r.require(type(index) is int and 0 <= index < len(rows), 'invalid source index')
            stamp, filename = rows[index]
            r.require(stamp == frame[kind+'_timestamp']
                      and frame[kind+'_file'] == kind+'-'+filename[len(kind)+1:],
                      'changed original source acquisition')
        stamp = frame['depth_timestamp']
        right = bisect_left(rgb_times, stamp)
        choices = [i for i in (right-1, right) if 0 <= i < len(rgb)]
        nearest = min(choices, key=lambda i: (abs(rgb_times[i]-stamp), i))
        r.require(frame['rgb_source_index'] == nearest, 'RGB is not nearest timestamp with earlier tie')
        gap = abs(frame['depth_timestamp']-frame['rgb_timestamp'])
        r.require(gap <= .02, 'qualification RGB/depth gap exceeds exact gate')
        v.same_pair_gap(frame['pair_gap_seconds'], gap)
        maximum_gap = max(maximum_gap, gap)
        right = bisect_left(truth_times, stamp)
        if right < len(truth_times) and truth_times[right] == stamp:
            bracket = 0.
        else:
            r.require(0 < right < len(truth_times), 'qualification requires GT bracket without extrapolation')
            bracket = truth_times[right]-truth_times[right-1]
            r.require(0 < bracket <= .02, 'qualification GT bracket exceeds exact gate')
        maximum_bracket = max(maximum_bracket, bracket)
    return dict(depth_rows=len(depth), rgb_rows=len(rgb), ground_truth_rows=len(truth),
                frame_count=36, reference_bracketed_frames=36,
                unique_rgb_frames=len({(f['rgb_source_index'], f['rgb_timestamp'], f['rgb_file']) for f in frames}),
                maximum_observed_ground_truth_bracket_us=math.floor(maximum_bracket*1e6+.5),
                maximum_pair_gap_us=math.floor(maximum_gap*1e6+.5),
                summary_time_unit='rounded microseconds from source timestamps; qualification gates use unrounded seconds')


def pinned_acquisition_check(manifest, source_root):
    """The hash-bound fetcher supplies source pins, never qualification results."""
    path = source_root/'scripts/fetch-qualified-dataset.py'
    spec = importlib.util.spec_from_file_location('qualified_acquisition_pins', path)
    module = importlib.util.module_from_spec(spec)
    before = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        spec.loader.exec_module(module)
    finally:
        sys.dont_write_bytecode = before
    module.verify_manifest(manifest)


def exact_json(actual, expected, message):
    # Treat JSON booleans separately from numbers (Python True equals 1).
    if isinstance(expected, bool):
        r.require(type(actual) is bool and actual == expected, message)
    elif isinstance(expected, dict):
        r.require(isinstance(actual, dict) and set(actual) == set(expected), message)
        for key in expected:
            exact_json(actual[key], expected[key], message+' '+key)
    elif isinstance(expected, list):
        r.require(isinstance(actual, list) and len(actual) == len(expected), message)
        for a, e in zip(actual, expected):
            exact_json(a, e, message)
    elif isinstance(expected, (int, float)):
        r.require(type(actual) in (int, float) and math.isfinite(actual) and actual == expected, message)
    else:
        r.require(actual == expected, message)


PREPROCESSING = dict(width=640, height=480,
    luma='(77*R+150*G+29*B)>>8; RGB/RGBA8; RGBA must be fully opaque',
    pixel_coordinates='nearest integer feature coordinate, ties round away from zero',
    min_depth_m=.3, max_depth_m=5., depth_units_per_metre=5000., patch_radius_pixels=1,
    patch_validity='all 9 depths valid and range-bounded', patch_max_spread_m=.05,
    point='centre depth at feature pixel; optical x-right y-down z-forward', maximum_pair_gap_s=.02)
FEATURE_POLICY = dict(max_features=400, max_matches=256,
    detector='fixed original FAST-9 threshold20 radius3, deterministic NMS,32pixel tile max2',
    descriptor='intensity-centroid oriented deterministic256bit BRIEF on5x5binomial blur',
    matching='both directional strict 5*best<4*second, maximumHamming64, mutual nearest; ties rejected')
TRACKING_POLICY = dict(max_unobserved_s=.20,
    reference='last accepted measured RGB features and depth image; initial root identity after measured geometry validation',
    pose='reference-root pose composed with measured refined previous_from_current pixel-reprojection pose',
    rejection='no root output or accepted clock/reference update; repeated/stale RGB timestamp versus last observed image rejects, including previously rejected fits; observed image clock advances without permission renewal; accepted-pose expiry checked before RGB freshness; evaluation never resets')
GT_POLICY = dict(method='linear translation and shortest-arc quaternion SLERP', max_bracket_s=.02,
                 extrapolation=False, max_source_rows=30000, max_source_bytes=4194304)
IMMUTABLE_ALGORITHMS = {
    'feature_source_sha256': '5b5b103d4e798929f753a850387609405cad1a303d49ef8e81433f064e901106',
    'visual_pose_source_sha256': '718192d69b02de0a668c54e8469812b7c5f1903ab74de660e1b221e2ec874d24',
    'reprojection_pose_source_sha256': '2895e1c3eafefcfacdd156fd51706598521f0344f427f24edf420d159e5da486',
    'visual_checker_sha256': g.OLD_VISUAL_SHA,
    'reprojection_checker_sha256': OLD_REPROJECTION_SHA,
}


def verify_sources(freeze, manifest_raw, qualification_raw, source_root):
    r.require(type(freeze['schema_version']) is int and freeze['schema_version'] == 1
              and type(freeze['protocol_version']) is int and freeze['protocol_version'] == 1
              and freeze['algorithm'] == ALGORITHM, 'unsupported qualified protocol')
    for field, path in SOURCES.items():
        r.require(freeze[field] == r.digest(r.bounded(source_root/path)), 'changed frozen source '+path)
    for field, expected in IMMUTABLE_ALGORITHMS.items():
        r.require(freeze[field] == expected, 'changed original measured-image mathematics '+field)
    r.require(freeze['manifest_sha256'] == r.digest(manifest_raw), 'changed frozen manifest')
    r.require(freeze['qualification_sha256'] == r.digest(qualification_raw), 'changed qualification proof bytes')
    qualification = json.loads(qualification_raw)
    exact_json(freeze['qualification'], qualification, 'changed frozen qualification object')
    manifest = json.loads(manifest_raw)
    pinned_acquisition_check(manifest, source_root)
    r.require(manifest['dataset'] == freeze['dataset'] == DATASET
              and manifest['repository'] == REPOSITORY and manifest['revision'] == REVISION
              and manifest['source_prefix'] == PREFIX, 'changed pinned source identity')
    r.require(type(freeze['regression_requested']) is bool, 'invalid qualified reporting role')
    r.require(freeze['kind'] == ('calibration_regression' if freeze['regression_requested'] else 'preregistered_sequence'),
              'misclassified qualified sequence')
    for field in ('depth_calibration', 'calibration_source'):
        exact_json(freeze[field], manifest[field], 'changed frozen acquisition '+field)
    r.require(len(freeze['frames']) == len(manifest['frames']), 'changed frozen frame count')
    for actual, expected in zip(freeze['frames'], manifest['frames']):
        v.same_acquisition(actual, expected)
    for field, expected in dict(preprocessing=PREPROCESSING, feature_policy=FEATURE_POLICY,
        tracking_policy=TRACKING_POLICY, registration_config=v.CONFIG, refinement_config=g.REFINEMENT_CONFIG,
        refinement_policy=g.REFINEMENT_POLICY, evaluation_label_policy=g.LABEL_POLICY,
        ground_truth_interpolation=GT_POLICY, accuracy_gates=dict(translation_m=.1, rotation_rad=.1),
        json_metadata_audit=v.JSON_METADATA_AUDIT).items():
        exact_json(freeze[field], expected, 'changed fixed policy '+field)
    for field in ('depth_calibration', 'calibration_source', 'preprocessing', 'feature_policy',
                  'tracking_policy', 'registration_config', 'refinement_config', 'refinement_policy',
                  'evaluation_label_policy'):
        expected = (g.refinement_config_digest(freeze[field]) if field == 'refinement_config'
                    else r.canonical(freeze[field]))
        r.require(freeze[field+'_sha256'] == expected, 'changed fixed policy hash '+field)
    calibration = manifest['depth_calibration']
    for key, value in dict(fx=517.306408, fy=516.469215, cx=318.643040, cy=255.313989,
                          width=640, height=480, units_per_metre=5000., invalid_depth=0).items():
        r.require(type(calibration[key]) in (int, float) and calibration[key] == value,
                  'changed pinned source camera profile '+key)
    return manifest, qualification


def qualification_check(freeze, manifest_raw, qualification, metadata):
    manifest = json.loads(manifest_raw)
    measured = independently_qualify(manifest, metadata)
    files = {item['file']: item for item in manifest['files']}
    expected = dict(schema_version=1, dataset=DATASET, manifest_sha256=r.digest(manifest_raw),
        source_commit=REVISION, window=dict(first_depth_index=100,last_depth_index=135,frames=36,updates=35),
        metadata_files={name:{key:files[name][key] for key in ('bytes','sha256','git_blob_sha1')}
                        for name in ('depth.txt','rgb.txt','groundtruth.txt')},
        timestamps=dict(depth_index_rows=measured['depth_rows'], rgb_index_rows=measured['rgb_rows'],
            ground_truth_rows=measured['ground_truth_rows'], strict_depth_order=True, strict_rgb_order=True,
            strict_ground_truth_order=True, all_frame_brackets_valid=True, max_ground_truth_bracket_s=.02,
            max_pair_gap_s=.02, maximum_observed_ground_truth_bracket_us=measured['maximum_observed_ground_truth_bracket_us'],
            maximum_pair_gap_us=measured['maximum_pair_gap_us'], unique_rgb_acquisitions=measured['unique_rgb_frames'],
            duplicate_rgb_associations=36-measured['unique_rgb_frames'], summary_time_unit=measured['summary_time_unit']),
        passed=True, pixels_read=False, image_headers_read=False, ground_truth_pose_values_parsed=False,
        features_or_fits_run=False, helper_sha256=freeze['qualification_source_sha256'],
        acquisition_helper_sha256=freeze['acquisition_source_sha256'])
    exact_json(qualification, expected, 'invented qualification proof')
    return measured


def normalized_report(report):
    r.require(report['algorithm'] == ALGORITHM, 'changed qualified report algorithm')
    normalized = dict(report)
    normalized['algorithm'] = 'bounded_visual_reprojection_refinement'
    return normalized


def audit(report, manifest, features, depths, gt, label_failure):
    r.require(report['qualification_verified'] is True, 'qualification gate not reported')
    exact_json(report['qualification'], report['freeze']['qualification'], 'report changed qualification proof')
    r.require(report['qualification_sha256'] == report['freeze']['qualification_sha256'],
              'report changed qualification proof hash')
    return g.audit(normalized_report(report), manifest, features, depths, gt, label_failure)


def write_result(output, result):
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')


def qualification_mutations(freeze, manifest_raw, proof_raw, metadata, source_root):
    rejected = []
    for field in SOURCES:
        changed = copy.deepcopy(freeze); changed[field] = '0'*64
        try:
            verify_sources(changed, manifest_raw, proof_raw, source_root)
        except (ValueError, KeyError):
            rejected.append('source_'+field)
        else:
            raise ValueError('changed source accepted '+field)
    policy_changes = [('refinement_config','max_iterations',100),
        ('registration_config','min_inliers',1), ('tracking_policy','max_unobserved_s',2.),
        ('preprocessing','patch_max_spread_m',1.), ('feature_policy','max_features',4000),
        ('ground_truth_interpolation','max_bracket_s',1.),
        ('evaluation_label_policy','invalid_source','deduplicate invalid reference poses')]
    for field, key, value in policy_changes:
        changed = copy.deepcopy(freeze); changed[field][key] = value
        if field+'_sha256' in changed:
            changed[field+'_sha256'] = (g.refinement_config_digest(changed[field]) if field == 'refinement_config'
                                        else r.canonical(changed[field]))
        try:
            verify_sources(changed, manifest_raw, proof_raw, source_root)
        except (ValueError, KeyError):
            rejected.append('policy_'+field)
        else:
            raise ValueError('weakened policy accepted '+field)
    proof = json.loads(proof_raw)
    changes = [ ('hide_metadata_failure',lambda x:x.__setitem__('passed',False)),
        ('invent_gt_row_order',lambda x:x['timestamps'].__setitem__('strict_ground_truth_order',False)),
        ('invent_gt_row_count',lambda x:x['timestamps'].__setitem__('ground_truth_rows',2)),
        ('invent_gt_bracket_stat',lambda x:x['timestamps'].__setitem__('maximum_observed_ground_truth_bracket_us',0)),
        ('invent_pair_gap_stat',lambda x:x['timestamps'].__setitem__('maximum_pair_gap_us',0)),
        ('weaken_pair_gap_gate',lambda x:x['timestamps'].__setitem__('max_pair_gap_s',2.)),
        ('invent_unique_rgb_count',lambda x:x['timestamps'].__setitem__('unique_rgb_acquisitions',999)),
        ('invent_qualification_window',lambda x:x['window'].__setitem__('first_depth_index',0)),
        ('invent_qualification_metadata_hash',lambda x:x['metadata_files']['groundtruth.txt'].__setitem__('sha256','0'*64)),
        ('invent_qualification_helper',lambda x:x.__setitem__('helper_sha256','0'*64)),
        ('qualification_parsed_pose_values',lambda x:x.__setitem__('ground_truth_pose_values_parsed',True)),
        ('qualification_decoded_pixels',lambda x:x.__setitem__('pixels_read',True)),
        ('qualification_opened_image_header',lambda x:x.__setitem__('image_headers_read',True)),
        ('qualification_ran_features',lambda x:x.__setitem__('features_or_fits_run',True)),
    ]
    for name, change in changes:
        altered_proof = copy.deepcopy(proof); change(altered_proof)
        altered_raw = json.dumps(altered_proof,sort_keys=True).encode()
        altered_freeze = copy.deepcopy(freeze)
        altered_freeze['qualification'] = altered_proof
        altered_freeze['qualification_sha256'] = r.digest(altered_raw)
        try:
            verify_sources(altered_freeze,manifest_raw,altered_raw,source_root)
            qualification_check(altered_freeze,manifest_raw,altered_proof,metadata)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('self-consistently rehashed qualification accepted '+name)
    manifest = json.loads(manifest_raw)
    for name, field, value in [('rename_repository','repository','unverified/repository'),
                               ('rename_source_commit','revision','0'*40)]:
        changed = copy.deepcopy(manifest); changed[field] = value
        changed_raw = json.dumps(changed,sort_keys=True).encode()
        changed_freeze = copy.deepcopy(freeze); changed_freeze['manifest_sha256'] = r.digest(changed_raw)
        try:
            verify_sources(changed_freeze,changed_raw,proof_raw,source_root)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('self-consistently rehashed source accepted '+name)
    return rejected


def metadata_contract_checks():
    frames = []
    depth_rows, rgb_rows = [], []
    for i in range(136):
        stamp, rgb_stamp = i/64.+1/128., i/64.
        dep = f'{stamp:.9f}.png'; rgb = f'{rgb_stamp:.9f}.png'
        depth_rows.append(f'{stamp:.9f} depth/{dep}\n')
        rgb_rows.append(f'{rgb_stamp:.9f} rgb/{rgb}\n')
        if i >= 100:
            frames.append(dict(source_index=i,depth_timestamp=stamp,depth_file='depth-'+dep,
                rgb_source_index=i,rgb_timestamp=rgb_stamp,rgb_file='rgb-'+rgb,
                pair_gap_seconds=1/128.,split='initialization' if i==100 else 'held_out'))
    # Pose tokens deliberately are not numeric: qualification must treat them
    # as opaque while still enforcing eight-column metadata syntax.
    labels = ''.join(f'{i/128.:.9f} opaque opaque opaque opaque opaque opaque opaque\n' for i in range(280)).encode()
    metadata = {'depth.txt':''.join(depth_rows).encode(),'rgb.txt':''.join(rgb_rows).encode(),
                'groundtruth.txt':labels}
    manifest = dict(frames=frames)
    result = independently_qualify(manifest,metadata)
    r.require(result['unique_rgb_frames']==36 and result['maximum_pair_gap_us']==7813
              and result['maximum_observed_ground_truth_bracket_us']==0,
              'known earlier-index RGB tie or opaque-pose qualification changed')
    positive = ['opaque_pose_tokens_not_converted','earlier_rgb_index_on_exact_tie',
                'exact_reference_timestamp_zero_bracket','all_36_rows_retained']
    rejected = []
    corrupted = [ ('duplicate_gt_timestamp',dict(metadata,**{'groundtruth.txt':labels+labels.splitlines()[-1]+b'\n'})),
        ('nonfinite_gt_timestamp',dict(metadata,**{'groundtruth.txt':b'nan opaque opaque opaque opaque opaque opaque opaque\n'+labels})),
        ('wrong_gt_row_arity',dict(metadata,**{'groundtruth.txt':b'0 opaque\n'+labels})),
        ('duplicate_depth_timestamp',dict(metadata,**{'depth.txt':metadata['depth.txt']+depth_rows[-1].encode()})),
        ('oversized_gt_metadata',dict(metadata,**{'groundtruth.txt':b' '*(4194304+1)})) ]
    for name, altered in corrupted:
        try:
            independently_qualify(manifest,altered)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('corrupted qualification metadata accepted '+name)
    for name, change in [('wrong_original_depth_index',lambda x:x['frames'][0].__setitem__('source_index',0)),
        ('wrong_nearest_rgb_index',lambda x:x['frames'][0].__setitem__('rgb_source_index',101)),
        ('invent_pair_gap',lambda x:x['frames'][0].__setitem__('pair_gap_seconds',.001)),
        ('omit_observed_depth_row',lambda x:x['frames'].pop())]:
        altered = copy.deepcopy(manifest); change(altered)
        try:
            independently_qualify(altered,metadata)
        except (ValueError,KeyError):
            rejected.append(name)
        else:
            raise ValueError('corrupted source association accepted '+name)
    return dict(passed=positive,mutations_rejected=rejected)


def qualified_report_mutations(report, manifest, features, depths, gt, label_failure):
    failures = []
    for name, change in [ ('conceal_qualification_gate',lambda x:x.__setitem__('qualification_verified',False)),
        ('conceal_report_qualification',lambda x:x.pop('qualification')),
        ('invent_report_qualification_hash',lambda x:x.__setitem__('qualification_sha256','0'*64)),
        ('invent_report_qualification_count',lambda x:x['qualification']['timestamps'].__setitem__('ground_truth_rows',2)),
        ('invent_report_algorithm',lambda x:x.__setitem__('algorithm','unverified_algorithm')) ]:
        changed = copy.deepcopy(report); change(changed)
        try:
            audit(changed,manifest,features,depths,gt,label_failure)
        except (ValueError,KeyError):
            failures.append(name)
        else:
            raise ValueError('qualified report corruption accepted '+name)
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('manifest', 'raw', 'freeze', 'qualification', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--source-snapshot', type=Path)
    parser.add_argument('--preregister-only', action='store_true')
    args = parser.parse_args()
    manifest_raw = r.bounded(args.manifest, 256*1024)
    freeze_raw = r.bounded(args.freeze, 256*1024)
    qualification_raw = r.bounded(args.qualification, 256*1024)
    freeze = json.loads(freeze_raw)
    source_root = args.source_snapshot or ROOT
    manifest, qualification = verify_sources(freeze, manifest_raw, qualification_raw, source_root)
    metadata = metadata_inputs(manifest, args.raw)
    measured = qualification_check(freeze, manifest_raw, qualification, metadata)
    common = dict(schema_version=1, passed_integrity=True, source_freeze_verified=True,
        frozen_source_count=len(SOURCES), dataset=DATASET, kind=freeze['kind'],
        manifest_sha256=r.digest(manifest_raw), freeze_sha256=r.digest(freeze_raw),
        qualification_sha256=r.digest(qualification_raw), qualification_independently_recomputed=True,
        qualification_method='whole-source row arity and timestamp columns only; nearest RGB and strict brackets',
        qualified_timestamp_summary=measured, running_checker_sha256=r.digest(r.bounded(Path(__file__))),
        frozen_checker_sha256=freeze['independent_checker_sha256'],
        archived_source_snapshot=args.source_snapshot is not None)
    source_mutations = qualification_mutations(freeze,manifest_raw,qualification_raw,metadata,source_root)
    metadata_contracts = metadata_contract_checks()
    common.update(frozen_provenance_and_qualification_mutations_rejected=source_mutations,
                  metadata_contract_checks=metadata_contracts)
    if args.preregister_only:
        r.require(args.report is None, 'metadata preregistration must not consume fit reports')
        common.update(metadata_only=True, metadata_files_read=['depth.txt','rgb.txt','groundtruth.txt'],
                      image_bytes_read=False, image_headers_read=False, pixels_read=False,
                      ground_truth_pose_values_parsed=False, features_or_fits_run=False)
        write_result(args.output, common)
        print(json.dumps(common))
        return
    r.require(args.report is not None, 'fit audit requires --report')
    report_raw = r.bounded(args.report, 16*1024*1024)
    report = json.loads(report_raw)
    exact_json(report['freeze'], freeze, 'external freeze mismatch')
    r.require(report['freeze_sha256'] == r.digest(freeze_raw)
              and report['manifest_sha256'] == r.digest(manifest_raw), 'external source/freeze mismatch')
    raw = g.verified_inputs(manifest, args.raw, report)
    features, depths, cache = [], [], {}
    for frame in manifest['frames']:
        filename = frame['rgb_file']
        if filename not in cache:
            cache[filename] = v.features(v.png_image(raw[filename], False))
        features.append(cache[filename])
        depths.append(v.png_image(raw[frame['depth_file']], True))
    # Reference pose columns are parsed only after the frozen operational report
    # and the independent measured-feature/refinement reconstruction exist.
    gt, label_failure = g.evaluation_labels(raw['groundtruth.txt'])
    records = audit(report, manifest, features, depths, gt, label_failure)
    old_mutations = g.mutation_checks(normalized_report(report), manifest, features, depths, gt, label_failure)
    qualified_mutations = qualified_report_mutations(report,manifest,features,depths,gt,label_failure)
    common.update(summary=report['summary'], frames=records, report_sha256=r.digest(report_raw),
        raw_sha256_verified=True, evaluation_label_failure=label_failure,
        physical_pose_independently_scored=label_failure is None,
        unscorable_updates=report['summary']['updates']-report['summary']['reference_valid_updates'],
        mutations_rejected=old_mutations+qualified_mutations, mathematical_contract_checks=g.mathematical_contract_checks(),
        missing_reference_score_mutations_rejected=k.missing_reference_contract_checks(),
        pixel_features_descriptors_and_associations_independently_reconstructed=True,
        rigid_fit_independently_replayed='Kabsch SVD vs operational Horn quaternion/Jacobi',
        pixel_refinement_independently_replayed='central numerical Jacobian and SVD vs analytic Jacobian and Cholesky',
        fixed_original_inlier_support_independently_checked=True,
        monotonic_search_and_terminal_step_accounting_independently_checked=True,
        reference_and_clock_state_independently_reconstructed=True,
        evidence_role='viewed regression' if freeze['regression_requested'] else 'preregistered sequence',
        calibrated_covariance_or_root_confidence_claim=False,
        oracle_dependencies=dict(numpy=g.np.__version__, pillow=v.Image.__version__),
        schema_normalization_only='report algorithm renamed in a copy for pinned original audit; no identity, profile, numerical gates, feature/fit/pose/state evidence changed',
        **g.maximum_scored_errors(records))
    write_result(args.output, common)
    print(json.dumps(report['summary']))


if __name__ == '__main__':
    main()
