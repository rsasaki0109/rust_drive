#!/usr/bin/env python3
"""Independent continuous multiscale RGB-D state, pixel and root-motion audit.

This additive oracle imports the checksum-verified pair mathematics unchanged.
Sensor fits and independently refined root composition precede numerical labels.
Every acquisition, rejection, repeated RGB and latched loss stays in the result.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PAIR_ORACLE_SHA = '78e9466da3c30605b1f25a8037a6c51fcfb2198c8caacbacde85a6eb9ccb3737'
MAX_REPORT_BYTES = 64*1024*1024
IDENTITY = ([0., 0., 0.], [1., 0., 0., 0.])
LOST = 'visual odometry lost; explicit new origin required'
EXPIRED = 'visual accepted-pose age exceeded; localization lost'
DUPLICATE = 'duplicate or stale RGB acquisition; no pose permission renewal'
GAP = 'sensor association gap exceeds .02s'
DESIGN_SHA = 'd0b6a8f21ded03ce8125e30e47440d223c77567d0f2e41b6d9031c32e1ea34df'
DESIGN_PATH = ROOT/'assets/multiscale-temporal-v1/design.json'
BASE_FREEZE_SHA = 'c9b9cc2be0d8168dfc1e9e3aa81eeb1b5b4c45a0c160d0139774c2ade8ee0a5c'
PAIR_FIELDS = {'accepted', 'matches', 'correspondences', 'refinement_observations',
               'coarse_fit', 'coarse_pose', 'refinement', 'relative_estimate', 'rejection'}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def bounded(path, limit=MAX_REPORT_BYTES):
    path = Path(path)
    require(path.is_file() and path.stat().st_size <= limit, 'missing/oversized input: '+str(path))
    with path.open('rb') as source:
        raw = source.read(limit+1)
    require(len(raw) <= limit, 'input grew beyond byte bound')
    return raw


def require_new_output(path):
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(),
            'output already exists or is a symlink: '+str(path))
    require(not any(parent.is_symlink() for parent in path.parents),
            'output parent is a symlink: '+str(path))


def write_new_output(path, value):
    require_new_output(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    require_new_output(path)
    with path.open('x', encoding='utf8') as destination:
        destination.write(json.dumps(value, indent=2, allow_nan=False)+'\n')


def load_pair_math():
    path = ROOT/'scripts/check-multiscale-features.py'
    require(digest(bounded(path)) == PAIR_ORACLE_SHA, 'immutable pair oracle changed')
    spec = importlib.util.spec_from_file_location('immutable_multiscale_pair_math', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Loaded only after output admission in main. No imported globals are overridden.
M = None
FIT_CACHE = {}
SENSOR_AUDIT_CACHE = {}


def pose_json(pose):
    return dict(translation_m=list(pose[0]), quaternion_wxyz=list(pose[1]))


def independent_fit(row, previous, current, previous_depth, current_depth, camera):
    """Audit full fit witnesses, then return the independently refined pose."""
    key = (digest(json.dumps([row, previous, current, camera], sort_keys=True,
                            separators=(',', ':'), allow_nan=False).encode()),
           digest(previous_depth.tobytes()), digest(current_depth.tobytes()))
    if key not in FIT_CACHE:
        evidence = M.check_pair(row, previous, current, previous_depth, current_depth, camera)
        pose = None
        if evidence['accepted']:
            # check_pair has already independently established all measured points
            # and consensus indices. Reuse its immutable solver, not Rust's pose.
            coarse, error = M.V.robust_svd(row['correspondences'])
            require(coarse is not None and error is None, 'accepted pair lacks independent consensus')
            observations = [dict(previous_xyz=o['previous'], current_pixel_xy=o['current_pixel'])
                            for o in row['refinement_observations']]
            refined, error = M.P.refine_independent(observations, camera, coarse['estimate'])
            require(refined is not None and error is None, 'accepted pair lacks independent refinement')
            pose = refined['estimate']
        require(len(FIT_CACHE) < 1024, 'independent fit cache bound')
        FIT_CACHE[key] = (evidence, pose)
    return copy.deepcopy(FIT_CACHE[key])


def block_reason(state, depth_stamp, rgb_stamp):
    # Accepted-pose expiry precedes RGB freshness and sensor decoding.
    if state['lost']:
        return LOST
    if state['accepted_stamp'] is not None and depth_stamp-state['accepted_stamp'] > .20+1e-9:
        return EXPIRED
    if state['observed_rgb_stamp'] is not None and rgb_stamp <= state['observed_rgb_stamp']:
        return DUPLICATE
    return None


def initial_state():
    return dict(reference=None, root=copy.deepcopy(IDENTITY), accepted_stamp=None,
                observed_rgb_stamp=None, lost=False)


def reject_mutant(baseline, mutate, validate):
    changed = copy.deepcopy(baseline)
    mutate(changed)
    # Keep the non-noop assertion OUTSIDE the expected rejection catch.
    require(changed != baseline, 'mutation did not change the baseline')
    try:
        validate(changed)
    except (ValueError, AssertionError, KeyError, TypeError, IndexError):
        return
    raise ValueError('non-noop corruption survived independent validation')


def exact(actual, expected, message):
    """JSON booleans and integers are distinct; clocks are exact source values."""
    if isinstance(expected, dict):
        require(isinstance(actual, dict) and actual.keys() == expected.keys(), message+' fields')
        for key in expected:
            exact(actual[key], expected[key], message+'/'+key)
    elif isinstance(expected, list):
        require(isinstance(actual, list) and len(actual) == len(expected), message+' length')
        for a, e in zip(actual, expected):
            exact(a, e, message)
    elif isinstance(expected, bool) or expected is None:
        require(type(actual) is type(expected) and actual == expected, message)
    elif isinstance(expected, (int, float)):
        require(type(actual) in (int, float) and math.isfinite(actual) and actual == expected, message)
    else:
        require(actual == expected, message)


def validate_design():
    raw = bounded(DESIGN_PATH, 512*1024)
    require(digest(raw) == DESIGN_SHA, 'fixed continuous comparison design changed')
    return json.loads(raw)


def verified_raw_loader(manifest, raw_root):
    inventory = {}
    for item in manifest['files']:
        name = item['file']
        require(name == Path(name).name and '/' not in name and '\\' not in name
                and name not in inventory, 'unsafe/duplicate raw inventory')
        require(type(item['bytes']) is int and 0 <= item['bytes'] <= 4*1024*1024,
                'invalid raw byte bound')
        inventory[name] = item
    total = 0
    for frame in manifest['frames']:
        for key, role in [('rgb_file', 'rgb_frame'), ('depth_file', 'depth_frame')]:
            entry = inventory[frame[key]]
            require(entry['role'] == role, 'sensor inventory role')
            total += entry['bytes']
    require(total <= 128*1024*1024, 'counted sensor bytes exceed design')
    def read(name):
        pin = inventory[name]
        data = bounded(raw_root/name, 4*1024*1024)
        require(len(data) == pin['bytes'] and digest(data) == pin['sha256'], 'raw source hash '+name)
        return data
    # Opaque startup verification includes unprocessed tails and reference files.
    # No PNG or numeric-label decoder is called during this inventory pass.
    for name in inventory:
        read(name)
    return inventory, read


def validate_acquisition(frames):
    require(2 <= len(frames) <= 180, 'frame capacity')
    for i, frame in enumerate(frames):
        require(type(frame['source_index']) is int and
                all(type(frame[k]) in (int, float) and math.isfinite(frame[k])
                    for k in ('depth_timestamp', 'rgb_timestamp')), 'invalid acquisition')
        if i:
            prior = frames[i-1]
            require(frame['source_index'] > prior['source_index'] and
                    frame['depth_timestamp'] > prior['depth_timestamp'] and
                    frame['rgb_timestamp'] >= prior['rgb_timestamp'], 'invalid acquisition chronology')


def initialization_geometry(features, depth, camera):
    points, witnesses = [], []
    for i, feature in enumerate(features):
        if len(points) == 256:
            break
        point, measured = M.depth_witness(feature, depth, camera)
        witnesses.append(dict(feature_index=i, **measured))
        if point is not None:
            points.append(point)
    ratio = M.V.scatter_ratio(points)
    good = 12 <= len(points) <= 256 and ratio >= .005
    return good, points, witnesses, ratio


def check_state(actual, state, frames, message):
    index = state['reference']
    expected = dict(reference_frame_index=index, last_accepted_stamp=state['accepted_stamp'],
                    last_observed_rgb_stamp=state['observed_rgb_stamp'], lost=state['lost'])
    require(set(actual) == set(expected)|{'root_at_reference'}, message+' schema')
    exact({key:actual[key] for key in expected}, expected, message)
    if state['reference'] is None:
        require(actual['root_at_reference'] is None, message+' invented uninitialized root')
    else:
        M.V.k.same_pose(M.V.r.pose(actual['root_at_reference']), state['root'], message+' root')


def empty_measurement(step):
    require(step['matches'] == step['correspondences'] == step['refinement_observations'] == [],
            'invented measured fit after clock/init block')
    require(not any(field in step for field in
                    ('coarse_fit', 'coarse_pose', 'refinement', 'relative_estimate')),
            'invented pose or refinement after clock/init block')


def score_root(evaluation, estimate, truth, rejection=None):
    if truth is not None:
        M.score_analytic(evaluation, estimate, truth)
        return (M.V.r.norm([a-b for a,b in zip(estimate[0], truth[0])]),
                M.V.r.quaternion_error(estimate[1], truth[1])) if estimate is not None else None
    exact(evaluation, dict(reference_valid=False, reference_rejection=rejection,
          estimate_present=estimate is not None, within_accuracy_gates=False), 'unavailable physical reference')
    return None


def check_initialization(step, features, depth, camera):
    good, points, witnesses, ratio = initialization_geometry(features, depth, camera)
    expected = []
    for item in witnesses:
        item = copy.deepcopy(item)
        feature_index = item.pop('feature_index')
        accepted = 'xyz' in item
        row = dict(feature_index=feature_index, accepted=accepted, depth=item)
        if accepted:
            row['point'] = item['xyz']
        else:
            row['rejection'] = item['rejection']
        expected.append(row)
    M.compare_value(step['initialization_depth_features'], expected, 'initial measured depth geometry')
    if good:
        M.V.r.close(step['initialization_geometry_ratio'], ratio, 'initial geometry ratio', 1e-7)
    else:
        require('initialization_geometry_ratio' not in step, 'invented initial geometry')
    return good, dict(points=len(points), geometry_ratio=ratio)


def check_frontend_subset(actual, gray, allowed):
    if all(allowed.values()):
        return M.check_frontend(actual, gray)
    require(set(actual) == {'levels', 'native_features', 'multiscale_features'}, 'partial frontend schema')
    levels, multi = M.pyramid(gray)
    if not allowed['multiscale']:
        levels = levels[:1]
    require(actual['levels'] == levels, 'partial frontend downsample hashes')
    a = M.native(gray) if allowed['native'] else []
    b = multi if allowed['multiscale'] else []
    M.compare_value(actual['native_features'], a, 'clock-gated native features')
    M.compare_value(actual['multiscale_features'], b, 'clock-gated multiscale features')
    return a, b


def checked_sensor(rgb, depth, frontend, allowed):
    # Re-read/hash the real inputs before this cache lookup. Only immutable bytes
    # plus the entire frontend witness and branch mask can reuse a full audit.
    key = (digest(rgb), digest(depth), digest(json.dumps([frontend,allowed],
        sort_keys=True,separators=(',',':'),allow_nan=False).encode()))
    if key not in SENSOR_AUDIT_CACHE:
        gray = M.V.png_image(rgb,False)
        decoded = M.V.png_image(depth,True)
        fronts = check_frontend_subset(frontend,gray,allowed)
        require(len(SENSOR_AUDIT_CACHE)<360,'sensor audit cache capacity')
        SENSOR_AUDIT_CACHE[key]=(decoded,fronts)
    return SENSOR_AUDIT_CACHE[key]


def check_sequence_sensor(report, manifest, inventory, raw, rendered=None):
    frames = manifest['frames']
    rows = report['frames']
    require(len(rows) == len(frames), 'omitted rejected acquisition')
    validate_acquisition(frames)
    camera = manifest['depth_calibration']
    states = {name:initial_state() for name in ('native', 'multiscale')}
    decoded, frontends, poses, evidence = {}, {}, [], []
    for i, (frame, row) in enumerate(zip(frames, rows)):
        require(row['index'] == i, 'changed acquisition ordinal')
        acquisition_keys = ('source_index', 'rgb_timestamp', 'depth_timestamp') if rendered is not None else (
            'source_index', 'rgb_file', 'depth_file', 'rgb_timestamp', 'depth_timestamp')
        for key in acquisition_keys:
            exact(row[key], frame[key], 'source acquisition '+key)
        if inventory:
            require(row['rgb_sha256'] == inventory[frame['rgb_file']]['sha256'] and
                    row['depth_sha256'] == inventory[frame['depth_file']]['sha256'], 'reported manifest sensor pins')
        reasons = {name:block_reason(state, frame['depth_timestamp'], frame['rgb_timestamp'])
                   for name,state in states.items()}
        allowed = {name:reason is None for name,reason in reasons.items()}
        require(type(row['sensor_decoded']) is bool, 'invalid decode flag')
        require(row['sensor_decoded'] == any(allowed.values()), 'decode before freshness/expiry guard')
        if any(allowed.values()):
            if rendered is None:
                rgb, depth = raw(frame['rgb_file']), raw(frame['depth_file'])
                require(row['rgb_sha256'] == digest(rgb) and row['depth_sha256'] == digest(depth),
                        'reported sensor source hash')
                decoded[i], fronts = checked_sensor(rgb,depth,row['frontend'],allowed)
            else:
                gray, decoded[i] = rendered[i]
                fronts = check_frontend_subset(row['frontend'], gray, allowed)
            frontends[i] = fronts
        else:
            exact(row['frontend'], dict(levels=[], native_features=[], multiscale_features=[]),
                  'features decoded after complete latched/duplicate block')
            # No sensor reads for an acquisition denied by both branches.
        roots, record = {}, dict(index=i, branches={})
        for ordinal, name in enumerate(('native', 'multiscale')):
            state, step = states[name], row[name]
            check_state(step['state_before'], state, frames, name+' before state')
            exact(step['clock_accepted'], allowed[name], 'reported clock decision')
            exact(step['sensor_attempted'], allowed[name], 'sensor attempted before clock')
            exact(step['features_computed'], allowed[name], 'feature computation before clock')
            require(step['reference_frame_index_before'] == state['reference'], 'wrong accepted reference before')
            estimate, rejected = None, reasons[name]
            item = dict(clock_accepted=allowed[name])
            if rejected:
                empty_measurement(step)
                require(step['initialization_depth_features'] == [], 'initialization after clock rejection')
                state['lost'] = state['lost'] or rejected in (LOST, EXPIRED)
            else:
                # Observe new RGB before any patch correspondence or optimizer.
                state['observed_rgb_stamp'] = frame['rgb_timestamp']
                features = frontends[i][ordinal]
                gap = abs(frame['rgb_timestamp']-frame['depth_timestamp'])
                if gap > .02:
                    rejected = GAP
                    empty_measurement(step)
                    state['lost'] = state['lost'] or i == 0
                elif i == 0:
                    empty_measurement(step)
                    good, item['initialization'] = check_initialization(step, features, decoded[i], camera)
                    if good:
                        estimate = copy.deepcopy(IDENTITY)
                    else:
                        state['lost'] = True
                        rejected = ('invalid or oversized visual initialization geometry' if
                            item['initialization']['points'] < 12 else
                            'collinear or poorly conditioned visual initialization geometry')
                else:
                    require(state['reference'] is not None, 'new origin after failed initialization')
                    require(step['initialization_depth_features'] == [] and
                            'initialization_geometry_ratio' not in step, 'silently restarted origin')
                    previous = state['reference']
                    require(step['reference_frame_index'] == previous and
                            step['reference_stamp'] == state['accepted_stamp'], 'wrong measured accepted reference')
                    M.V.k.same_pose(M.V.r.pose(step['root_from_reference']), state['root'], 'wrong reference root')
                    measured = {key:step[key] for key in PAIR_FIELDS if key in step}
                    # Relative estimate exists only after final refinement accepts.
                    item['fit'], relative = independent_fit(measured, frontends[previous][ordinal],
                            features, decoded[previous], decoded[i], camera)
                    if relative is not None:
                        estimate = M.V.k.compose(state['root'], relative)
                        item['independent_relative'] = pose_json(relative)
                    else:
                        rejected = item['fit']['rejection']
            accepted = estimate is not None
            exact(step['accepted'], accepted, 'independent acceptance')
            exact(step['initialized'], accepted and i == 0, 'changed single origin')
            if accepted:
                require('rejection' not in step, 'accepted fit has rejection')
                M.V.k.same_pose(M.V.r.pose(step['root_estimate']), estimate, 'independent continuous root composition')
                state['reference'], state['root'], state['accepted_stamp'] = i, estimate, frame['depth_timestamp']
            else:
                require(step.get('rejection') == rejected and
                        not any(key in step for key in ('root_estimate', 'relative_estimate', 'refinement')),
                        'concealed rejection or published rejected pose')
                item['rejection'] = rejected
            require(step['reference_frame_index_after'] == state['reference'], 'wrong accepted reference after')
            check_state(step['state_after'], state, frames, name+' after state')
            roots[name] = estimate
            item.update(accepted=accepted, root=pose_json(estimate) if estimate is not None else None)
            record['branches'][name] = item
        poses.append(roots)
        evidence.append(record)
    return poses, states, evidence


def physical_audit(report, manifest, poses, states, gt=None, label_error=None, analytic_roots=None):
    """Called only after BOTH branches' complete sensor/state reconstruction."""
    frames = manifest['frames']
    origin = None
    if gt is not None:
        try:
            origin = M.V.r.interpolate(gt, frames[0]['depth_timestamp'])
        except ValueError as error:
            label_error = str(error)
    counters = {name:dict(frames=len(frames), updates=len(frames)-1, initialized_frames=0,
                        accepted_updates=0, rejected_updates=0, accurate_root_updates=0,
                        reference_valid_updates=0, lost=states[name]['lost'])
                for name in ('native', 'multiscale')}
    maxima = {name:None for name in counters}
    for i, (frame, row, estimates) in enumerate(zip(frames, report['frames'], poses)):
        truth, rejection = None, label_error
        if analytic_roots is not None:
            truth = analytic_roots[i]
        elif origin is not None:
            try:
                truth = M.V.r.relative(origin, M.V.r.interpolate(gt, frame['depth_timestamp']))
            except ValueError as error:
                rejection = {'GT extrapolation forbidden':'truth cannot bracket; no extrapolation',
                    'GT bracket gap exceeds .02 seconds':'truth bracket exceeds .02s'}.get(str(error), str(error))
        for name, estimate in estimates.items():
            score = row[name]['evaluation']
            errors = score_root(score, estimate, truth, rejection)
            if errors is not None:
                if maxima[name] is None:
                    maxima[name] = dict(translation_m=errors[0],rotation_rad=errors[1])
                else:
                    maxima[name]['translation_m'] = max(maxima[name]['translation_m'], errors[0])
                    maxima[name]['rotation_rad'] = max(maxima[name]['rotation_rad'], errors[1])
            if i == 0:
                counters[name]['initialized_frames'] += int(estimate is not None)
            else:
                counters[name]['accepted_updates'] += int(estimate is not None)
                counters[name]['rejected_updates'] += int(estimate is None)
                counters[name]['reference_valid_updates'] += int(truth is not None)
                counters[name]['accurate_root_updates'] += int(score['within_accuracy_gates'])
    for name, count in counters.items():
        count['all_updates_passed'] = (count['initialized_frames'] == 1 and
            count['accepted_updates'] == count['accurate_root_updates'] ==
            count['reference_valid_updates'] == count['updates'])
    exact(report['summary'], counters, 'continuous denominator/availability summary')
    return counters, maxima


def temporal_mutations(report, validate):
    rows = report['frames']
    accepted = next(i for i,row in enumerate(rows) if i and row['multiscale']['accepted'])
    rejected = next(i for i,row in enumerate(rows) if i and not row['multiscale']['accepted'])
    duplicate = next((i for i,row in enumerate(rows)
                      if row['multiscale'].get('rejection') == DUPLICATE), None)
    latched = next((i for i,row in enumerate(rows)
                   if row['multiscale'].get('rejection') == LOST), None)
    operations = {}
    def change_root(data):
        data['frames'][accepted]['multiscale']['root_estimate']['translation_m'][0] += .1
    def reset(data):
        data['frames'][accepted]['multiscale']['initialized'] = True
    def omit_failure(data):
        data['frames'].pop(rejected)
    def wrong_reference(data):
        data['frames'][accepted]['multiscale']['reference_frame_index_before'] += 1
    def renew_failure(data):
        row = data['frames'][rejected]
        row['multiscale']['state_after']['last_accepted_stamp'] = row['depth_timestamp']
    def wrong_root_reference(data):
        data['frames'][accepted]['multiscale']['state_after']['root_at_reference']['translation_m'][0] += .1
    def root_as_relative(data):
        row = next(r for i,r in enumerate(data['frames']) if i > accepted and r['multiscale']['accepted'])
        branch = row['multiscale']
        reference = branch['reference_frame_index_before']
        a = M.V.r.pose(data['frames'][reference]['multiscale']['evaluation']['evaluation_only_truth'])
        b = M.V.r.pose(branch['evaluation']['evaluation_only_truth'])
        truth = M.V.r.relative(a, b)
        pose = M.V.r.pose(branch['relative_estimate'])
        translation = M.V.r.norm([x-y for x,y in zip(pose[0], truth[0])])
        rotation = M.V.r.quaternion_error(pose[1], truth[1])
        branch['evaluation'] = dict(reference_valid=True, estimate_present=True,
            evaluation_only_truth=pose_json(truth), translation_error_m=translation,
            rotation_error_rad=rotation, within_accuracy_gates=translation<=.1 and rotation<=.1)
    def operational_truth(data):
        data['ground_truth_operational'] = True
    def fake_correspondence(data):
        data['frames'][accepted]['multiscale']['correspondences'][0]['previous'][0] += .1
    def fake_descriptor(data):
        data['frames'][0]['frontend']['multiscale_features'][0]['descriptor'][0] ^= 1
    def fake_refined_pose(data):
        data['frames'][accepted]['multiscale']['relative_estimate']['translation_m'][0] += .1
    def hide_availability(data):
        data['summary']['multiscale']['accepted_updates'] += 1
    def unverified_inventory(data):
        data['inventory_integrity']['all_files_verified'] = False
    operations.update(invented_root_composition=change_root, restarted_origin=reset,
        omitted_failed_acquisition=omit_failure, wrong_accepted_reference=wrong_reference,
        renewed_clock_after_failure=renew_failure, changed_root_at_reference=wrong_root_reference,
        operational_truth=operational_truth, invented_measured_geometry=fake_correspondence,
        changed_descriptor=fake_descriptor, invented_refined_pose=fake_refined_pose,
        concealed_availability_failure=hide_availability,unverified_raw_inventory=unverified_inventory)
    if sum(r['multiscale']['accepted'] for r in rows[1:]) >= 2:
        operations['relative_instead_of_root_score'] = root_as_relative
    if duplicate is not None:
        def renew_duplicate(data):
            row = data['frames'][duplicate]
            row['multiscale']['state_after']['last_accepted_stamp'] = row['depth_timestamp']
        def decode_duplicate(data):
            data['frames'][duplicate]['multiscale']['sensor_attempted'] = True
        operations.update(duplicate_accepted_clock=renew_duplicate, decoded_duplicate=decode_duplicate)
    if latched is not None:
        def clear_loss(data):
            data['frames'][latched]['multiscale']['state_before']['lost'] = False
        def bypass_loss(data):
            data['frames'][latched]['multiscale']['clock_accepted'] = True
        operations.update(reset_latched_loss=clear_loss, bypass_latched_loss=bypass_loss)
    for name, mutate in operations.items():
        reject_mutant(report, mutate, validate)
    return list(operations)


def source_hashes():
    paths = dict(M.SOURCE_PATHS)
    paths.update(temporal_binary='integrations/rgbd/src/bin/rustdriving-rgbd-multiscale-temporal.rs',
        temporal_support='integrations/rgbd/src/multiscale_temporal_support.rs',
        design='assets/multiscale-temporal-v1/design.json', core_manifest='crates/core/Cargo.toml',
        perception_manifest='crates/perception/Cargo.toml', localization_manifest='crates/localization/Cargo.toml')
    actual = {key:digest(bounded(ROOT/path)) for key,path in paths.items()}
    base = bounded(ROOT/'assets/multiscale-pairs-v1/room-freeze.json', 512*1024)
    require(digest(base) == BASE_FREEZE_SHA, 'original pair freeze changed')
    for key, expected in json.loads(base)['sources'].items():
        require(actual[key] == expected, 'original frozen source changed '+key)
    M.verify_sources({key:actual[key] for key in M.SOURCE_PATHS})
    return actual


def protocol_check(report, manifest_bytes, freeze):
    design = validate_design()
    manifest = json.loads(manifest_bytes)
    name = manifest['dataset']
    require(name in design['datasets'] and
            digest(manifest_bytes) == design['datasets'][name]['manifest_sha256'], 'changed fixed viewed interval')
    exact(report['freeze'], freeze, 'report changed external freeze')
    expected = dict(schema_version=1, algorithm='bounded_multiscale_continuous_viewed_comparison',
        dataset=name, manifest_sha256=digest(manifest_bytes), design_sha256=DESIGN_SHA, design=design,
        sources=source_hashes(),
        source_reuse='Measurement functions feature_json/native/depth_point/pair/decode/verified/truth/interpolate/score/texture copied byte-for-byte from immutable pair prototype; private API prevented import. New sensor/state/evaluation/CLI functions are separately bound.',
        indices='ordinal0..179; original source_index100..279 separately retained',
        no_ground_truth_operational=True)
    exact(freeze, expected, 'continuous source/protocol freeze')
    require(report['schema_version'] == 1 and
            report['algorithm'] == expected['algorithm'] and report['dataset'] == name and
            report['manifest_sha256'] == digest(manifest_bytes), 'report identity')
    exact(report['ground_truth_operational'], False, 'operational truth prohibited')
    exact(report['raw_redistributed'], False, 'raw redistribution prohibited')
    require(report['input_failure'] is None, 'sensor-input failure requires separate source-invalid audit')
    require(len(manifest['frames']) == 180 and
            [f['source_index'] for f in manifest['frames']] == list(range(100,280)), 'fixed180frame source window')
    require(manifest['depth_calibration']['width'] == 640 and
            manifest['depth_calibration']['height'] == 480 and
            manifest['depth_calibration']['invalid_depth'] == 0, 'fixed acquisition profile')
    return manifest


def audit_recorded(report, manifest_bytes, raw_root, freeze):
    manifest = protocol_check(report, manifest_bytes, freeze)
    inventory, raw = verified_raw_loader(manifest, raw_root)
    exact(report['inventory_integrity'],dict(all_files_verified=True,
        files_verified=len(inventory),bytes_verified=sum(pin['bytes'] for pin in inventory.values()),
        no_pixels_decoded=True,no_numeric_labels_parsed=True),'opaque startup inventory verification')
    poses, states, evidence = check_sequence_sensor(report, manifest, inventory, raw)
    # No numeric truth is interpreted until every branch/frame fit is complete.
    labels = [f['file'] for f in manifest['files'] if f['role'] == 'evaluation_only_mocap_ground_truth']
    require(len(labels) == 1, 'physical label source count')
    gt, error = None, None
    try:
        gt = M.normalized_truth(raw(labels[0]))
    except ValueError as failure:
        error = str(failure)
    exact(report['evaluation_label_failure'], error, 'physical label failure')
    counts, maxima = physical_audit(report, manifest, poses, states, gt, error)
    return dict(kind='viewed_continuous_regression', summary=counts,
        maximum_accepted_root_errors=maxima, frames=evidence,
        numeric_truth_checked_after_all_sensor_fits=True, one_initial_origin=True,
        no_reset_or_loss_recovery=True, all_acquisitions_retained=True)


CONTROL_SPECS = {
    'healthy':[(0.,0.,[0,0],7500,False),(.04,.04,[8,4],7500,False),
               (.08,.08,[16,8],7500,False),(.12,.12,[24,12],7500,False)],
    'depth_failure_duplicate_expiry_latch':[(0.,0.,[0,0],7500,False),(.04,.04,[8,4],7500,False),
               (.06,.06,[16,8],7500,False),(.08,.08,[24,12],0,False),
               (.10,.08,[24,12],7500,False),(.261,.261,[24,12],7500,False),
               (.28,.28,[32,16],7500,False)],
    'initialization_failure_latch':[(0.,0.,[0,0],7500,True),(.04,.04,[8,4],7500,False)],
    'refinement_failure_no_fallback':[(0.,0.,[0,0],7500,False),(.04,.04,[81,0],7000,False),
               (.06,.06,[8,4],7500,False)],
}


def audit_control(report):
    require(set(report) == {'schema_version','kind','sources','design_sha256',
            'calibration','texture','cases'}, 'analytic control schema/truth flags')
    require(report['schema_version'] == 1 and report['kind'] == 'analytic_continuous_measured_controls',
            'analytic control identity')
    require(report['design_sha256'] == DESIGN_SHA, 'analytic design binding')
    validate_design()
    exact(report['sources'], source_hashes(), 'analytic sources')
    camera = dict(width=320,height=240,fx=240.,fy=240.,cx=159.5,cy=119.5,
                  units_per_metre=5000.,invalid_depth=0)
    exact(report['calibration'], camera, 'analytic calibration')
    require(report['texture'] == 'aperiodic checker: base40/170 plus (x*97+y*193+x*y*17)%61',
            'analytic render formula')
    require([case['kind'] for case in report['cases']] == list(CONTROL_SPECS), 'missing control case')
    results = []
    for case in report['cases']:
        specs = CONTROL_SPECS[case['kind']]
        require(len(case['frames']) == len(specs), 'omitted analytic acquisition')
        manifest = dict(depth_calibration=camera, frames=[])
        rendered, truths = [], []
        base = M.texture(320,240)
        for i, (stamp,rgb_stamp,shift,raw_depth,blank) in enumerate(specs):
            frame = dict(source_index=i,depth_timestamp=stamp,rgb_timestamp=rgb_stamp)
            manifest['frames'].append(frame)
            exact(case['frames'][i]['render'],dict(shift_pixels=shift,raw_depth=raw_depth,blank=blank),
                  'analytic measured render')
            gray = np.zeros_like(base)
            dx,dy = shift
            gray[dy:,dx:] = base[:240-dy,:320-dx]
            if blank:
                gray.fill(127)
            rendered.append((gray,np.full((240,320),raw_depth,dtype=np.uint16)))
            truths.append(([-dx*1.5/240.,-dy*1.5/240.,0.],[1.,0.,0.,0.]))
        poses,states,evidence = check_sequence_sensor(case,manifest,{},None,rendered)
        counts,maxima = physical_audit(case,manifest,poses,states,analytic_roots=truths)
        expected = ([True]*4 if case['kind']=='healthy' else [False]*2 if
            case['kind']=='initialization_failure_latch' else [True,False,True] if
            case['kind']=='refinement_failure_no_fallback' else [True,True,True,False,False,False,False])
        exact(case['expected_accepted'],expected,'analytic expected availability')
        passed = all(poses[i][name] is not None if expected[i] else poses[i][name] is None
            for i in range(len(specs)) for name in ('native','multiscale'))
        require(passed and case['expected_behavior_passed'] is True,'actual rendered state control failed')
        results.append(dict(kind=case['kind'],summary=counts,maximum_accepted_root_errors=maxima,
                            frames=evidence))
    return dict(kind='analytic_rendered_continuous_controls',cases=results,
        numeric_truth_checked_after_all_sensor_fits=True,one_initial_origin=True,
        no_reset_or_loss_recovery=True,all_acquisitions_retained=True)


def control_mutations(report, validate):
    def step(data,case,index):
        return data['cases'][case]['frames'][index]['multiscale']
    def root(data):
        step(data,0,2)['root_estimate']['translation_m'][0] += .1
    def reset(data):
        step(data,0,2)['initialized'] = True
    def omit(data):
        data['cases'][1]['frames'].pop(3)
    def ref(data):
        step(data,0,2)['reference_frame_index_before'] = 0
    def renew(data):
        step(data,1,3)['state_after']['last_accepted_stamp'] = .08
    def duplicate(data):
        step(data,1,4)['state_after']['last_accepted_stamp'] = .10
    def bypass(data):
        step(data,1,6)['clock_accepted'] = True
    def unlatch(data):
        step(data,1,6)['state_before']['lost'] = False
    def gt_flag(data):
        data['ground_truth_operational'] = True
    def relative_score(data):
        branch = step(data,0,2)
        branch['evaluation']['evaluation_only_truth'] = pose_json(([-.05,-.025,0.],[1.,0.,0.,0.]))
        branch['evaluation']['translation_error_m'] = 0.
    def geometry(data):
        step(data,0,1)['correspondences'][0]['current'][0] += .1
    def decode(data):
        step(data,1,5)['sensor_attempted'] = True
    def native_root(data):
        data['cases'][0]['frames'][2]['native']['root_estimate']['translation_m'][0] += .1
    def coarse_fallback(data):
        branch=step(data,3,1)
        branch['accepted']=True
        branch['relative_estimate']=copy.deepcopy(branch['coarse_pose'])
    def rejected_reference(data):
        step(data,3,1)['state_after']['reference_frame_index']=1
    operations = dict(root_composition=root,origin_reset=reset,omitted_failure=omit,
        wrong_accepted_reference=ref,rejected_clock_renewal=renew,duplicate_accepted_clock=duplicate,
        bypass_latched_loss=bypass,reset_latched_loss=unlatch,operational_gt_flag=gt_flag,
        relative_instead_of_root_score=relative_score,measured_geometry=geometry,
        decode_after_expiry=decode,native_root_composition=native_root,
        coarse_only_fallback=coarse_fallback,reference_after_failed_refinement=rejected_reference)
    for name, mutate in operations.items():
        reject_mutant(report,mutate,validate)
    return list(operations)


def self_tests():
    """Analytic boundary/algebra checks; no recorded images or reference values."""
    state = initial_state()
    state.update(reference=0, accepted_stamp=0., observed_rgb_stamp=.04)
    require(block_reason(state, .20+1e-9, .21) is None, 'inclusive age tolerance boundary')
    require(block_reason(state, math.nextafter(.20+1e-9, math.inf), .04) == EXPIRED,
            'expiry must precede duplicate rejection')
    require(block_reason(state, .1, .04) == DUPLICATE, 'duplicate cannot renew permission')
    require(block_reason(state, .1, .041) is None, 'new RGB can be observed')
    state['lost'] = True
    require(block_reason(state, .05, .041) == LOST, 'latched loss cannot recover')
    q = [math.sqrt(.5), 0., 0., math.sqrt(.5)]
    a, b = ([1., 2., 0.], q), ([.1, .2, 0.], [1.,0.,0.,0.])
    composed = M.V.k.compose(a,b)
    M.V.k.same_pose(composed, ([.8,2.1,0.],q), 'rotated root composition analytic truth')
    baseline = dict(root=pose_json(composed), initialized=False, accepted_stamp=0., lost=True)
    def validate(data):
        M.V.k.same_pose(M.V.r.pose(data['root']), composed, 'analytic root corruption')
        exact(data['initialized'], False, 'analytic root reset')
        exact(data['accepted_stamp'], 0., 'analytic rejected clock renewal')
        exact(data['lost'], True, 'analytic loss reset')
    operations = {
        'relative_as_root':lambda data:data.__setitem__('root',pose_json(b)),
        'reset_origin':lambda data:data.__setitem__('initialized',True),
        'rejected_clock_renewal':lambda data:data.__setitem__('accepted_stamp',.1),
        'reset_latched_loss':lambda data:data.__setitem__('lost',False)}
    validate(baseline)
    for name, mutate in operations.items():
        reject_mutant(baseline, mutate, validate)
    return dict(kind='analytic_state_and_composition', controls=[
        'inclusive_age_boundary', 'expiry_before_duplicate', 'duplicate_clock_rejection',
        'strictly_new_observed_rgb', 'latched_loss', 'rotated_root_composition'],
        non_noop_mutations_rejected=list(operations), recorded_data_read=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--raw', type=Path)
    parser.add_argument('--freeze', type=Path)
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    require_new_output(args.output)
    global M
    M = load_pair_math()
    if args.self_test:
        require(args.report is None, 'self-test takes no measured report')
        result = self_tests()
    else:
        require(args.report is not None, 'report required')
        report = json.loads(bounded(args.report))
        analytic = report.get('kind') == 'analytic_continuous_measured_controls'
        if analytic:
            validate = audit_control
        else:
            require(args.manifest is not None and args.raw is not None and args.freeze is not None,
                    'recorded audit requires --manifest, --raw and --freeze')
            manifest_bytes = bounded(args.manifest, 512*1024)
            freeze = json.loads(bounded(args.freeze, 512*1024))
            validate = lambda data:audit_recorded(data, manifest_bytes, args.raw, freeze)
        result = validate(report)
        result['non_noop_mutations_rejected'] = (control_mutations if analytic else temporal_mutations)(report, validate)
    result.update(schema='rustdriving-multiscale-continuous-oracle-v1',
        auditor_sha256=digest(bounded(Path(__file__))), design_sha256=DESIGN_SHA,
        imported_pair_oracle_sha256=PAIR_ORACLE_SHA,
        compiled_sources=source_hashes(),
        original_math_sha256=M.PINNED_MATH, imported_globals_overridden=False,
        limits='Viewed indoor integrity/availability/root-error audit; no real-time, automotive, safety, covariance or RNE integration claim')
    write_new_output(args.output, result)
    print(json.dumps({key:result[key] for key in ('kind','summary','maximum_accepted_root_errors',
        'controls','non_noop_mutations_rejected') if key in result}))


if __name__ == '__main__':
    main()
