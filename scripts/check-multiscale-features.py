#!/usr/bin/env python3
"""Independent bounded image-pyramid/descriptor/pair geometry audit.

The original mathematical oracles are checksum verified, never rewritten or
globally monkeypatched. This prototype audits pair measurements, not trajectories.
"""
import argparse
import ast
import copy
from functools import lru_cache
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import types

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
PINNED_MATH = {
    'check-recorded-visual.py': 'c138992bd5fe63a42757ff2cf5b43c7eb69272b85ea6ce7030cd298306bc0d6a',
    'check-recorded-rgbd.py': 'ab86ab67c0d46e08b89a0a47d8a59e9c95357cf41c40dbd40b1bcf5b49e6f9dc',
    'check-recorded-keyframes.py': '3c5ef7c2ad50b9ab015f0c6795aa955489343725d5a274ff1eaae730ecd95c01',
    'check-recorded-reprojection.py': 'fd1425f9c73c21b104d9453a7c2b298c21fa22effd6ecb5f0015bbc41f79cef7',
}
BUDGETS = [200, 120, 80]
MAX_REPORT_BYTES = 64*1024*1024
SOURCE_PATHS = dict(
    pair_binary='integrations/rgbd/src/bin/rustdriving-rgbd-multiscale-pairs.rs',
    multiscale_features='crates/perception/src/multiscale_features.rs',
    image_features='crates/perception/src/image_features.rs',
    registration3d='crates/localization/src/registration3d.rs',
    visual_odometry3d='crates/localization/src/visual_odometry3d.rs',
    reprojection3d='crates/localization/src/reprojection3d.rs',
    localization_lib='crates/localization/src/lib.rs',
    perception_lib='crates/perception/src/lib.rs', core_lib='crates/core/src/lib.rs',
    cargo_lock='integrations/rgbd/Cargo.lock', cargo_manifest='integrations/rgbd/Cargo.toml',
    rust_toolchain='rust-toolchain.toml')


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def bounded(path, limit=MAX_REPORT_BYTES):
    path = Path(path)
    require(path.is_file() and path.stat().st_size <= limit, 'missing/oversized file: '+str(path))
    return path.read_bytes()


def require_new_output(path):
    """Reject evidence replacement before input parsing or numerical verification."""
    path = Path(path).absolute()
    require(not path.exists() and not path.is_symlink(),
            'output already exists or is a symlink: '+str(path))
    require(not any(parent.is_symlink() for parent in path.parents),
            'output parent is a symlink: '+str(path))


def load_math():
    sources = {}
    for filename, expected in PINNED_MATH.items():
        raw = bounded(ROOT/'scripts'/filename)
        require(digest(raw) == expected, 'original math source changed: '+filename)
        sources[filename] = raw
    spec = importlib.util.spec_from_file_location('immutable_multiscale_visual_math',
                                                 ROOT/'scripts/check-recorded-visual.py')
    visual = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(visual)
    # The old reprojection CLI has historical global overrides. Execute ONLY
    # its checksum-verified pure mathematical definitions in a separate scope.
    names = {'rotation_matrix', 'skew', 'exponential', 'project', 'pixel_score',
             'numerical_system', 'refine_independent', '_refine_independent',
             'refinement_check'}
    tree = ast.parse(sources['check-recorded-reprojection.py'])
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name in names]
    require({node.name for node in functions} == names, 'missing pinned refinement mathematics')
    namespace = dict(np=np, math=math, lru_cache=lru_cache, v=visual, r=visual.r, k=visual.k)
    exec(compile(ast.Module(body=functions, type_ignores=[]),
                 'checksum-verified-pure-reprojection-math', 'exec'), namespace)
    return visual, types.SimpleNamespace(**namespace)


V, P = load_math()


@lru_cache(maxsize=600)
def features_generic(shape, pixels):
    """Independent integer FAST/blur/BRIEF with bounded generic dimensions.

    The historical Python detector requires exactly 640x480; this implementation
    supports smaller pyramid/control levels without changing that original code.
    """
    h, w = shape
    require(34 < h and 34 < w and h*w <= 307200 and len(pixels) == h*w,
            'invalid bounded grayscale dimensions')
    gray = np.frombuffer(pixels, dtype=np.uint8).reshape(h, w)
    center = gray[16:h-16, 16:w-16].astype(np.int16)
    diffs = [gray[16+dy:h-16+dy, 16+dx:w-16+dx].astype(np.int16)-center
             for dx, dy in V.CIRCLE]
    strength = np.zeros(center.shape, dtype=np.int16)
    for start in range(16):
        arc = [diffs[(start+i) % 16] for i in range(9)]
        strength = np.maximum(strength, np.maximum(np.minimum.reduce(arc), -np.maximum.reduce(arc)))
    strength[strength <= 20] = 0
    scores = np.zeros((h, w), dtype=np.int16)
    scores[16:h-16, 16:w-16] = strength
    inner = scores[17:h-17, 17:w-17]
    retain = inner > 0
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dx == 0 and dy == 0:
                continue
            other = scores[17+dy:h-17+dy, 17+dx:w-17+dx]
            retain &= (other < inner) if dy < 0 or (dy == 0 and dx < 0) else (other <= inner)
    ys, xs = np.nonzero(retain)
    candidates = sorted(((int(inner[y, x]), int(x)+17, int(y)+17) for y, x in zip(ys, xs)),
                        key=lambda item: (-item[0], item[2], item[1]))
    wide = gray.astype(np.uint32)
    horizontal = np.zeros((h, w), dtype=np.uint32)
    horizontal[:, 2:w-2] = sum(weight*wide[:, i:w-4+i]
                              for i, weight in enumerate((1, 4, 6, 4, 1)))
    blur = np.zeros((h, w), dtype=np.uint8)
    blur[2:h-2, 2:w-2] = ((sum(weight*horizontal[i:h-4+i, 2:w-2]
                                 for i, weight in enumerate((1, 4, 6, 4, 1)))+128)//256).astype(np.uint8)
    disk = [(dx, dy) for dy in range(-15, 16) for dx in range(-15, 16) if dx*dx+dy*dy <= 225]
    pattern, tile_counts, output = V.brief_pattern(), {}, []
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
            u = V.half_away(dx*cosine-dy*sine)
            vv = V.half_away(dx*sine+dy*cosine)
            return int(blur[y+vv, x+u])
        for bit, (a, b) in enumerate(pattern):
            if sample(a) < sample(b):
                words[bit//64] |= 1 << (bit % 64)
        output.append(dict(x=float(x), y=float(y), score=score, orientation=angle, descriptor=words))
        if len(output) == 400:
            break
    if (h, w) == (480, 640):
        # Keep an explicit cross-check against the byte-exact original detector.
        V.feature_check(output, V.features(gray))
    return output


def pyramid(gray):
    require(gray.dtype == np.uint8 and gray.ndim == 2 and gray.size <= 307200,
            'invalid image dtype/shape/bound')
    output, features = [], []
    for level, cap in enumerate(BUDGETS):
        h, w = gray.shape
        pixels = gray.tobytes()
        output.append(dict(level=level, width=w, height=h, gray_sha256=digest(pixels)))
        if h <= 34 or w <= 34:
            break
        scale = 2**level
        offset = (scale-1)/2
        for feature in features_generic((h, w), pixels)[:cap]:
            item = copy.deepcopy(feature)
            item.update(level=level, level_x=feature['x'], level_y=feature['y'],
                        x=scale*feature['x']+offset, y=scale*feature['y']+offset)
            features.append(item)
        # Use fresh uint16 storage and one integer rounding after the four samples.
        ww, hh = w//2, h//2
        wide = gray[:2*hh, :2*ww].astype(np.uint16)
        gray = ((wide[0::2, 0::2]+wide[0::2, 1::2]+wide[1::2, 0::2]+wide[1::2, 1::2]+2)//4).astype(np.uint8)
    return output, features


def check_features(actual, expected):
    require(len(actual) == len(expected) <= 400, 'global feature cap or feature count')
    for a, e in zip(actual, expected):
        require(a.keys() == e.keys(), 'feature schema')
        for key in ('level', 'level_x', 'level_y', 'x', 'y', 'score', 'descriptor'):
            require(a[key] == e[key], 'wrong multiscale feature '+key)
        V.r.close(a['orientation'], e['orientation'], 'wrong feature orientation', 1e-10)
    for level, cap in enumerate(BUDGETS):
        require(sum(f['level'] == level for f in actual) <= cap, 'octave feature cap')


def texture(width, height):
    require(34 < width and 34 < height and width*height <= 307200, 'control dimensions')
    y, x = np.indices((height, width), dtype=np.uint32)
    base = np.where((x//7+y//11) % 2 == 0, 40, 170)
    return (base+(x*97+y*193+x*y*17) % 61).astype(np.uint8)


def depth_point(feature, depth, calibration):
    x, y = V.half_away(feature['x']), V.half_away(feature['y'])
    if not (1 <= x < depth.shape[1]-1 and 1 <= y < depth.shape[0]-1):
        return None, 'depth patch exceeds image'
    units = calibration['units_per_metre']
    require(math.isfinite(units) and units > 0, 'invalid depth scale')
    patch = depth[y-1:y+2, x-1:x+2].astype(np.float64)/units
    if not np.all((patch >= .3) & (patch <= 5.)):
        return None, 'invalid or range-limited measured depth patch'
    if np.max(patch)-np.min(patch) > .05:
        return None, 'measured depth discontinuity exceeds patch gate'
    z = float(patch[1, 1])
    return [(feature['x']-calibration['cx'])*z/calibration['fx'],
            (feature['y']-calibration['cy'])*z/calibration['fy'], z], None


def verify_sources(actual):
    expected = {name: digest(bounded(ROOT/path)) for name, path in SOURCE_PATHS.items()}
    require(actual == expected, 'changed prototype source provenance')
    # None of the new code may replace the historical first-trial evidence.
    preserved = {'scripts/check-recorded-independent.py':
        '0a4696936103eda71824899607d6799c0f529deb036614bbab4053838d2e7af8',
        'assets/recorded-independent/desk2-v1/results.json':
        '2e621c78e2986539c386ff4a86be47b95774db929f6460373adc5b4549ac4e68'}
    for path, expected_hash in preserved.items():
        require(digest(bounded(ROOT/path)) == expected_hash, 'historical checker/first trial changed')


def native(gray):
    return [dict(f, level=0, level_x=f['x'], level_y=f['y'])
            for f in features_generic(gray.shape, gray.tobytes())]


def check_frontend(actual, gray):
    levels, multi = pyramid(gray)
    require(set(actual) == {'levels', 'native_features', 'multiscale_features'}, 'frontend schema')
    require(actual['levels'] == levels, 'wrong downsample dimensions/pixels or level SHA')
    a = native(gray)
    check_features(actual['multiscale_features'], multi)
    # Native owns the full original 400 quota, rather than multiscale's 200.
    require(len(actual['native_features']) == len(a) <= 400, 'native feature cap/count')
    for reported, expected in zip(actual['native_features'], a):
        require(reported.keys() == expected.keys(), 'native feature schema')
        for key in ('level', 'level_x', 'level_y', 'x', 'y', 'score', 'descriptor'):
            require(reported[key] == expected[key], 'wrong native feature '+key)
        V.r.close(reported['orientation'], expected['orientation'], 'native orientation', 1e-10)
    return a, multi


def depth_witness(feature, depth, calibration):
    x, y = V.half_away(feature['x']), V.half_away(feature['y'])
    point, reason = depth_point(feature, depth, calibration)
    stencil = [] if reason == 'depth patch exceeds image' else depth[y-1:y+2, x-1:x+2].ravel().tolist()
    witness = dict(sample_x=x, sample_y=y, raw_stencil=stencil)
    witness.update(rejection=reason) if reason else witness.update(xyz=point)
    return point, witness


def compare_value(actual, expected, message):
    if isinstance(expected, dict):
        require(isinstance(actual, dict) and actual.keys() == expected.keys(), message+' schema')
        for key in expected:
            compare_value(actual[key], expected[key], message+'/'+key)
    elif isinstance(expected, list):
        require(isinstance(actual, list) and len(actual) == len(expected), message+' length')
        for a, e in zip(actual, expected):
            compare_value(a, e, message)
    elif isinstance(expected, float):
        V.r.close(actual, expected, message, 1e-10)
    else:
        require(actual == expected, message)


PAIR_AUDIT_CACHE = {}


def check_pair(row, previous_features, current_features, previous_depth, current_depth, calibration,
               clock_reason=None):
    # Cache only fully audited, immutable row AND sensor geometry combinations.
    # A changed witness/pose/descriptor/depth input gets a distinct key and replay.
    metadata = [row, previous_features, current_features, calibration, clock_reason]
    key = (digest(json.dumps(metadata, sort_keys=True, separators=(',', ':'),
                             allow_nan=False).encode()), digest(previous_depth.tobytes()),
           digest(current_depth.tobytes()))
    if key not in PAIR_AUDIT_CACHE:
        result = check_pair_uncached(row, previous_features, current_features,
                                     previous_depth, current_depth, calibration, clock_reason)
        require(len(PAIR_AUDIT_CACHE) < 1024, 'pair audit cache bound')
        PAIR_AUDIT_CACHE[key] = result
    return copy.deepcopy(PAIR_AUDIT_CACHE[key])


def check_pair_uncached(row, previous_features, current_features, previous_depth, current_depth, calibration,
                        clock_reason=None):
    require(type(row['accepted']) is bool, 'invalid accepted flag')
    if clock_reason:
        require(not row['accepted'] and row['rejection'] == clock_reason
                and row['matches'] == row['correspondences'] == row['refinement_observations'] == [],
                'invalid clock-blocked pair')
        require(not any(key in row for key in ('coarse_pose', 'coarse_fit', 'refinement', 'relative_estimate')),
                'invented fit on rejected clock')
        return dict(accepted=False, rejection=clock_reason, matches=0, correspondences=0)
    matches = V.descriptor_matches(previous_features, current_features)
    require(len(row['matches']) == len(matches) <= 256, 'match count/cap')
    pairs, pixels = [], []
    for observed, matched in zip(row['matches'], matches):
        pa, wa = depth_witness(previous_features[matched['previous_index']], previous_depth, calibration)
        pb, wb = depth_witness(current_features[matched['current_index']], current_depth, calibration)
        expected = dict(matched, previous_depth=wa, current_depth=wb)
        if pa is not None and pb is not None:
            expected['correspondence_index'] = len(pairs)
            pairs.append(dict(previous=pa, current=pb))
            f = current_features[matched['current_index']]
            pixels.append([f['x'], f['y']])
        compare_value(observed, expected, 'match/depth association')
    compare_value(row['correspondences'], pairs, 'original measured depth correspondences')
    coarse, rejection = V.robust_svd(pairs)
    if coarse is None:
        require(row['accepted'] is False and row['rejection'] == rejection
                and row['refinement_observations'] == [], 'incorrect coarse rejection')
        require(not any(key in row for key in ('coarse_pose', 'coarse_fit', 'refinement', 'relative_estimate')),
                'invented coarse pose/refinement after coarse rejection')
        return dict(accepted=False, rejection=rejection, matches=len(matches), correspondences=len(pairs))
    V.fit_check(row['coarse_fit'], V.r.pose(row['coarse_pose']), pairs, coarse)
    observations = [dict(correspondence_index=i, previous=pairs[i]['previous'], current_pixel=pixels[i])
                    for i in coarse['inlier_indices']]
    compare_value(row['refinement_observations'], observations, 'original pixel refinement support')
    independent_observations = [dict(previous_xyz=o['previous'], current_pixel_xy=o['current_pixel'])
                                for o in observations]
    refined, rejection = P.refine_independent(independent_observations, calibration, coarse['estimate'])
    if refined is None:
        require(row['accepted'] is False and row['rejection'] == rejection
                and 'relative_estimate' not in row and 'refinement' not in row,
                'incorrect pixel refinement rejection')
        return dict(accepted=False, rejection=rejection, matches=len(matches), correspondences=len(pairs),
                    coarse_inliers=coarse['inlier_count'])
    require(row['accepted'] is True and 'rejection' not in row, 'concealed accepted fit')
    estimate = V.r.pose(row['relative_estimate'])
    P.refinement_check(row['refinement'], estimate, independent_observations, calibration,
                       coarse['estimate'], refined)
    return dict(accepted=True, matches=len(matches), correspondences=len(pairs),
                coarse_inliers=coarse['inlier_count'], refinement_support=len(observations))


def score_analytic(actual, pose, truth):
    require(actual['reference_valid'] is True and actual['estimate_present'] == (pose is not None),
            'analytic reference/estimate flags')
    V.k.same_pose(V.r.pose(actual['evaluation_only_truth']), truth, 'analytic expected physical pose')
    if pose is None:
        require(actual['within_accuracy_gates'] is False and 'translation_error_m' not in actual
                and 'rotation_error_rad' not in actual, 'invented analytic error after rejection')
    else:
        translation = V.r.norm([a-b for a, b in zip(pose[0], truth[0])])
        angle = V.r.quaternion_error(pose[1], truth[1])
        V.r.close(actual['translation_error_m'], translation, 'analytic translation error')
        V.r.close(actual['rotation_error_rad'], angle, 'analytic rotation error')
        require(actual['within_accuracy_gates'] == (translation <= .1 and angle <= .1), 'analytic gates')


def audit_control(report):
    verify_sources(report['sources'])
    require(report['schema_version'] == 1 and report['kind'] == 'analytic_rendered_measured_pair',
            'control report kind/schema')
    calibration = dict(width=320, height=240, fx=240., fy=240., cx=159.5, cy=119.5,
                       units_per_metre=5000., invalid_depth=0)
    require(report['calibration'] == calibration, 'analytic calibration')
    require(report['render'] == dict(depth_m=1.5, shift_pixels=[8, 4],
            texture='aperiodic checker: base40/170 plus (x*97+y*193+x*y*17)%61'), 'control render')
    gray = texture(320, 240)
    moved = np.zeros_like(gray)
    moved[4:, 8:] = gray[:-4, :-8]
    require(len(report['frames']) == 2, 'control frame count')
    fa = check_frontend(report['frames'][0], gray)
    fb = check_frontend(report['frames'][1], moved)
    depth = np.full((240, 320), 7500, dtype=np.uint16)
    truth = ([-.05, -.025, 0.], [1., 0., 0., 0.])
    result = {}
    for index, name in enumerate(('native', 'multiscale')):
        row = report[name]
        result[name] = check_pair(row, fa[index], fb[index], depth, depth, calibration)
        estimate = V.r.pose(row['relative_estimate']) if row['accepted'] else None
        score_analytic(report[name+'_evaluation'], estimate, truth)
    controls = report['frontend_controls']
    require(len(controls) == 2 and [c['kind'] for c in controls] ==
            ['repeat_twofold', 'clockwise_quarter_turn'], 'missing scale/rotation controls')
    geometry = []
    for case in controls:
        repeat = case['kind'] == 'repeat_twofold'
        original = texture(160, 120) if repeat else texture(320, 240)
        transformed = np.repeat(np.repeat(original, 2, axis=0), 2, axis=1) if repeat else np.rot90(original, -1).copy()
        require(case['previous_dimensions'] == list(original.shape[::-1]) and
                case['current_dimensions'] == list(transformed.shape[::-1]), 'control dimensions')
        require(case['geometric_claim'] == 'analytic pixel correspondence only; no 3D pose acceptance',
                'invented frontend control motion claim')
        pa = check_frontend(case['previous'], original)
        pb = check_frontend(case['current'], transformed)
        counts = {}
        for i, name in enumerate(('native', 'multiscale')):
            matches = V.descriptor_matches(pa[i], pb[i])
            require(case[name] == matches, 'wrong scale/rotation descriptor association')
            correct, cross_level = 0, 0
            for matched in matches:
                a, b = pa[i][matched['previous_index']], pb[i][matched['current_index']]
                correct_mapping = ((b['x'], b['y']) == (2*a['x']+.5, 2*a['y']+.5)) if repeat else (
                    (b['x'], b['y']) == (original.shape[0]-1-a['y'], a['x']))
                correct += int(correct_mapping)
                cross_level += int(correct_mapping and a['level'] != b['level'])
            counts[name] = dict(matches=len(matches), correct_geometry=correct,
                                correct_cross_level=cross_level)
            if name == 'multiscale':
                require(len(matches) >= (20 if repeat else 30) and correct*100 >= len(matches)*90,
                        'authored multiscale geometric control failed')
                if repeat:
                    require(cross_level >= 20, 'no actual scale-octave correspondence evidence')
        geometry.append(dict(kind=case['kind'], results=counts))
    return dict(kind='analytic', pair_results=result, frontend_control_geometry=geometry)


def expected_protocol(manifest_bytes):
    return dict(schema_version=1, algorithm='bounded_multiscale_pair_prototype',
        kind='viewed_pair_regression', manifest_sha256=digest(manifest_bytes),
        sources={name:digest(bounded(ROOT/path)) for name,path in SOURCE_PATHS.items()},
        level_budgets=BUDGETS, max_features=400, max_matches=256, max_level_candidates=1200,
        levels=[1, 2, 4], downsample='2x2 integer box: (sum+2)/4, floor dimensions; repeated at quarter',
        original_pixel='level_pixel*scale+(scale-1)/2',
        depth_ray='fractional original feature coordinate; nearest rounded original depth sample with all-nine stencil gate',
        depth_range_m=[.3, 5.], depth_patch_spread_m=.05, pair_gap_s=.02,
        hamming_max=64, ratio_strict=.8, mutual=True,
        registration='unchanged VisualOdometry3dConfig::default()',
        refinement='unchanged ReprojectionConfig3d::default()',
        evaluation_translation_gate_m=.1, evaluation_rotation_gate_rad=.1, gt_bracket_max_s=.02,
        max_frames=180, max_raw_png_bytes=4194304, max_report_bytes=67108864,
        max_total_sensor_bytes=134217728,
        no_sequence_pose_permission=True, no_truth_operational=True, no_raw_redistribution=True)


def normalized_truth(raw):
    """New pair export parser policy; no old label helper is overwritten."""
    rows = []
    for line in raw.decode('utf-8').splitlines():
        if not line.strip() or line.lstrip().startswith('#'):
            continue
        require(len(rows) < 30000, 'truth row bound')
        values = list(map(float, line.split()))
        require(len(values) == 8 and all(math.isfinite(v) for v in values)
                and (not rows or values[0] > rows[-1][0]), 'invalid chronological labels')
        q = V.r.qnorm([values[7], values[4], values[5], values[6]])
        rows.append((values[0], values[1:4], q))
    return rows


def audit_recorded(report, manifest, manifest_bytes, raw_root, freeze):
    protocol = expected_protocol(manifest_bytes)
    require(report['schema_version'] == 1 and report['ground_truth_operational'] is False
            and report['dataset'] == manifest['dataset'], 'recorded schema/truth/source')
    require(report['protocol'] == freeze == protocol, 'recorded source/protocol freeze disagreement')
    verify_sources(report['protocol']['sources'])
    frames = manifest['frames']
    require(2 <= len(frames) <= 180 and len(report['frames']) == len(frames)
            and len(report['pairs']) == len(frames)-1, 'omitted recorded pair/frame')
    camera = manifest['depth_calibration']
    require(camera['width'] == 640 and camera['height'] == 480, 'recorded pixel dimensions')
    inventory = {}
    for entry in manifest['files']:
        name = entry['file']
        require(name == Path(name).name and '/' not in name and '\\' not in name
                and name not in inventory, 'unsafe/duplicate input filename')
        inventory[name] = entry
    def raw(name):
        pin = inventory[name]
        data = bounded(raw_root/name, 4*1024*1024)
        require(len(data) == pin['bytes'] and digest(data) == pin['sha256'], 'measured source pin '+name)
        return data
    decoded, fronts = [], []
    for i, (frame, observed) in enumerate(zip(frames, report['frames'])):
        rgb, depth_bytes = raw(frame['rgb_file']), raw(frame['depth_file'])
        gray, depth = V.png_image(rgb, False), V.png_image(depth_bytes, True)
        expected = {field:frame[field] for field in
                    ('source_index', 'rgb_file', 'depth_file', 'rgb_timestamp', 'depth_timestamp')}
        expected.update(index=i, rgb_sha256=digest(rgb), depth_sha256=digest(depth_bytes))
        require(set(observed) == set(expected)|{'frontend'}, 'recorded frame schema')
        require({field:observed[field] for field in expected} == expected, 'recorded acquisition association')
        fronts.append(check_frontend(observed['frontend'], gray))
        decoded.append(depth)
    evidence = []
    for i, row in enumerate(report['pairs']):
        require(row['previous_frame'] == i and row['current_frame'] == i+1, 'recorded frame adjacency')
        previous, current = frames[i:i+2]
        require(current['source_index'] > previous['source_index'] and
                current['depth_timestamp'] > previous['depth_timestamp'] and
                current['rgb_timestamp'] >= previous['rgb_timestamp'], 'frame chronology')
        reason = None
        if current['rgb_timestamp'] <= previous['rgb_timestamp']:
            reason = 'duplicate or stale RGB acquisition'
        elif (abs(previous['rgb_timestamp']-previous['depth_timestamp']) > .02
              or abs(current['rgb_timestamp']-current['depth_timestamp']) > .02):
            reason = 'sensor association gap exceeds .02s'
        item = dict(pair=i)
        for which, name in enumerate(('native', 'multiscale')):
            item[name] = check_pair(row[name], fronts[i][which], fronts[i+1][which],
                                   decoded[i], decoded[i+1], camera, reason)
        evidence.append(item)
    # ALL measured geometry fits were checked above. Only now parse GT values.
    labels = [e['file'] for e in manifest['files'] if e['role'] == 'evaluation_only_mocap_ground_truth']
    require(len(labels) == 1, 'label source count')
    gt = None
    label_error = None
    try:
        gt = normalized_truth(raw(labels[0]))
    except ValueError as error:
        label_error = str(error)
    counts = {name:dict(fitted=0, accurate=0, scorable=0) for name in ('native', 'multiscale')}
    for i, row in enumerate(report['pairs']):
        truth = None
        reason = label_error
        if gt is not None:
            try:
                a = V.r.interpolate(gt, frames[i]['depth_timestamp'])
                b = V.r.interpolate(gt, frames[i+1]['depth_timestamp'])
                truth = V.r.relative(a, b)
            except ValueError as error:
                reason = {'GT extrapolation forbidden':'truth cannot bracket; no extrapolation',
                          'GT bracket gap exceeds .02 seconds':'truth bracket exceeds .02s'}.get(str(error), str(error))
        for name in ('native', 'multiscale'):
            branch = row[name]
            evaluation = branch['evaluation']
            accepted = branch['accepted']
            counts[name]['fitted'] += int(accepted)
            if truth is None:
                require(evaluation == dict(reference_valid=False, reference_rejection=reason,
                        estimate_present=accepted, within_accuracy_gates=False), 'concealed invalid source reference')
            else:
                estimate = V.r.pose(branch['relative_estimate']) if accepted else None
                score_analytic(evaluation, estimate, truth)
                counts[name]['scorable'] += 1
                counts[name]['accurate'] += int(evaluation['within_accuracy_gates'])
    require(report['summary'] == dict(total_pairs=len(frames)-1, **counts), 'concealed failed-pair denominator')
    return dict(kind='viewed_pair_regression', summary=report['summary'], pairs=evidence,
                physical_labels_checked_after_sensor_fits=True,
                no_trajectory_or_pose_permission_claim=True)


def require_changed_and_rejected(baseline, mutate, validate):
    changed = copy.deepcopy(baseline)
    mutate(changed)
    # Assert this outside the rejection catch: no no-op may count as a mutant.
    require(changed != baseline, 'mutation did not change the baseline')
    try:
        validate(changed)
    except (ValueError, AssertionError, KeyError, TypeError, IndexError):
        return
    raise ValueError('non-noop mutant survived validation')


def mutation_checks(report, validate):
    control = report.get('kind') == 'analytic_rendered_measured_pair'
    def frontend(data):
        return data['frames'][0] if control else data['frames'][0]['frontend']
    accepted_pair = None if control else next(i for i, p in enumerate(report['pairs'])
                                               if p['multiscale']['accepted'])
    def pair(data):
        return data['multiscale'] if control else data['pairs'][accepted_pair]['multiscale']
    def change_downsample(data):
        current = frontend(data)['levels'][1]['gray_sha256']
        frontend(data)['levels'][1]['gray_sha256'] = ('1' if current[0] == '0' else '0')+current[1:]
    def change_pixel(data):
        f = next(f for f in frontend(data)['multiscale_features'] if f['level'] > 0)
        f['x'] += .5
    def change_descriptor(data):
        frontend(data)['multiscale_features'][0]['descriptor'][0] ^= 1
    def change_octave(data):
        frontend(data)['multiscale_features'][0]['level'] = 3
    def change_match(data):
        pair(data)['matches'][0]['hamming_distance'] ^= 1
    def change_match_index(data):
        pair(data)['matches'][0]['current_index'] += 1
    def change_stencil(data):
        pair(data)['matches'][0]['previous_depth']['raw_stencil'][4] += 1
    def change_depth_point(data):
        measured = next(m for m in pair(data)['matches'] if 'xyz' in m['current_depth'])
        measured['current_depth']['xyz'][0] += .1
    def change_correspondence(data):
        pair(data)['correspondences'][0]['previous'][2] += .1
    def change_inlier(data):
        pair(data)['coarse_fit']['inlier_indices'].pop()
    def change_coarse_pose(data):
        pair(data)['coarse_pose']['translation_m'][0] += .05
    def change_refinement_pixel(data):
        pair(data)['refinement_observations'][0]['current_pixel'][0] += 1.
    def change_refinement_pose(data):
        pair(data)['relative_estimate']['translation_m'][0] += .05
    def change_refinement_cost(data):
        pair(data)['refinement']['final_huber_cost'] += 1.
    def exceed_features(data):
        rows = frontend(data)['multiscale_features']
        rows.extend([copy.deepcopy(rows[0]) for _ in range(401-len(rows))])
    def exceed_matches(data):
        rows = pair(data)['matches']
        rows.extend([copy.deepcopy(rows[0]) for _ in range(257-len(rows))])
    operations = dict(wrong_downsample=change_downsample, wrong_original_pixel=change_pixel,
        wrong_descriptor=change_descriptor, wrong_octave=change_octave,
        wrong_match_distance=change_match, wrong_match_index=change_match_index,
        invented_depth_stencil=change_stencil, invented_depth_point=change_depth_point,
        invented_correspondence=change_correspondence, dropped_inlier=change_inlier,
        invented_coarse_pose=change_coarse_pose, changed_refinement_pixel=change_refinement_pixel,
        invented_refined_pose=change_refinement_pose, invented_refinement_cost=change_refinement_cost,
        exceeded_global_feature_cap=exceed_features, exceeded_global_match_cap=exceed_matches)
    if control:
        def wrong_scale_association(data):
            data['frontend_controls'][0]['multiscale'][0]['current_index'] += 1
        operations['wrong_scale_association'] = wrong_scale_association
    else:
        def suppress_failure(data):
            index = next(i for i, row in enumerate(data['pairs']) if not row['multiscale']['accepted'])
            data['pairs'].pop(index)
        def invent_acquisition(data):
            data['frames'][0]['rgb_timestamp'] += .01
        def invent_summary(data):
            data['summary']['multiscale']['fitted'] += 1
        def invent_accuracy(data):
            pair(data)['evaluation']['translation_error_m'] += .1
        operations.update(omitted_rejected_pair=suppress_failure, changed_rgb_association= invent_acquisition,
                          concealed_failure_count=invent_summary, invented_physical_error=invent_accuracy)
    for name, mutate in operations.items():
        require_changed_and_rejected(report, mutate, validate)
    return list(operations)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--manifest', type=Path)
    parser.add_argument('--raw', type=Path)
    parser.add_argument('--freeze', type=Path)
    args = parser.parse_args()
    require_new_output(args.output)
    report = json.loads(bounded(args.report))
    if report.get('kind') == 'analytic_rendered_measured_pair':
        validate = audit_control
    else:
        require(args.manifest is not None and args.raw is not None and args.freeze is not None,
                'recorded audit requires --manifest, --raw, --freeze')
        manifest_bytes = bounded(args.manifest, 512*1024)
        manifest = json.loads(manifest_bytes)
        freeze = json.loads(bounded(args.freeze, 512*1024))
        validate = lambda data: audit_recorded(data, manifest, manifest_bytes, args.raw, freeze)
    result = validate(report)
    result['non_noop_mutations_rejected'] = mutation_checks(report, validate)
    result.update(schema='rustdriving-multiscale-pair-oracle-v1',
                  auditor_sha256=digest(bounded(Path(__file__))),
                  original_math_sha256=PINNED_MATH,
                  independence='Independent pixels/FAST/BRIEF/mutual association, measured depth, Kabsch consensus and numerical-Jacobian refinement; no Rust algorithm reuse or original global override')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    require_new_output(args.output)
    with args.output.open('x', encoding='utf-8') as destination:
        destination.write(json.dumps(result, indent=2)+'\n')
    print(json.dumps({key:result[key] for key in ('kind', 'summary', 'pair_results',
                     'frontend_control_geometry', 'non_noop_mutations_rejected') if key in result}))


if __name__ == '__main__':
    main()
