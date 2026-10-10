#!/usr/bin/env python3
"""Independent recorded-depth/mocap report integrity oracle.

This reconstructs geometry, physical relative truth and reported errors. It does
not rerun ICP, certify registration correctness, or establish covariance coverage.
Rejected pairs remain part of every summary and can produce a valid failed report.
Run with the existing pinned demo Pillow environment.
"""
import argparse
import bisect
import copy
import hashlib
import io
import itertools
import json
import math
from pathlib import Path
import struct
import sys
import zlib

ROOT = Path(__file__).resolve().parent.parent
MAX_BYTES = 4 * 1024 * 1024


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def canonical(value):
    return digest(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def bounded(path, limit=MAX_BYTES):
    require(path.stat().st_size <= limit, 'file exceeds byte bound')
    with path.open('rb') as source:
        raw = source.read(limit + 1)
    require(len(raw) <= limit, 'file grew beyond byte bound')
    return raw


def close(a, b, message, tolerance=1e-9):
    require(isinstance(a, (float, int)) and not isinstance(a, bool)
            and math.isfinite(a) and math.isfinite(b)
            and math.isclose(a, b, rel_tol=tolerance, abs_tol=tolerance), message)


def vector(v, size, message):
    require(isinstance(v, list) and len(v) == size
            and all(isinstance(x, (float, int)) and not isinstance(x, bool)
                    and math.isfinite(x) for x in v), message)
    return v


def qnorm(q):
    n = math.sqrt(sum(x*x for x in q))
    require(n > 1e-12 and math.isfinite(n), 'invalid quaternion norm')
    q = [x/n for x in q]
    return [-x for x in q] if q[0] < 0 else q


def multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return [w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v]


def conjugate(q):
    return [q[0], -q[1], -q[2], -q[3]]


def rotate(q, p):
    return multiply(multiply(q, [0]+p), conjugate(q))[1:]


def norm(v):
    return math.sqrt(sum(x*x for x in v))


def quaternion_error(a, b):
    d = qnorm(multiply(a, conjugate(b)))
    return 2 * math.atan2(norm(d[1:]), abs(d[0]))


def pose(value):
    t = vector(value['translation_m'], 3, 'invalid pose translation')
    q = vector(value['quaternion_wxyz'], 4, 'invalid pose quaternion')
    close(norm(q), 1, 'pose quaternion is not normalized', 1e-10)
    return t, qnorm(q)


def gt_rows(raw):
    rows = []
    for line in raw.decode('ascii').splitlines():
        line = line.strip()
        if not line or line.startswith('#'):
            continue
        v = list(map(float, line.split()))
        require(len(v) == 8 and all(math.isfinite(x) for x in v), 'invalid GT row')
        q = [v[7], v[4], v[5], v[6]]
        require(abs(norm(q)-1) <= .001, 'invalid raw GT quaternion')
        require(not rows or v[0] > rows[-1][0], 'nonmonotonic raw GT timestamps')
        rows.append((v[0], v[1:4], qnorm(q)))
    require(2 <= len(rows) <= 20000, 'GT row bound')
    return rows


def interpolate(rows, stamp):
    right = bisect.bisect_left([r[0] for r in rows], stamp)
    if right < len(rows) and rows[right][0] == stamp:
        return rows[right][1], rows[right][2], [stamp, stamp]
    require(0 < right < len(rows), 'GT extrapolation forbidden')
    a, b = rows[right-1:right+1]
    gap = b[0]-a[0]
    require(0 < gap <= .02, 'GT bracket gap exceeds .02 seconds')
    t = (stamp-a[0])/gap
    qa, qb = a[2], b[2]
    dot = sum(x*y for x, y in zip(qa, qb))
    if dot < 0:
        qb, dot = [-x for x in qb], -dot
    if dot > .9995:
        sa, sb = 1-t, t
    else:
        angle = math.acos(max(-1, min(1, dot)))
        sa, sb = math.sin((1-t)*angle)/math.sin(angle), math.sin(t*angle)/math.sin(angle)
    q = qnorm([sa*x+sb*y for x, y in zip(qa, qb)])
    p = [x+(y-x)*t for x, y in zip(a[1], b[1])]
    return p, q, [a[0], b[0]]


def relative(a, b):
    qa = conjugate(a[1])
    return rotate(qa, [y-x for x, y in zip(a[0], b[0])]), qnorm(multiply(qa, b[1]))


def depth_geometry(raw):
    from PIL import Image
    require(raw[:8] == b'\x89PNG\r\n\x1a\n', 'not PNG')
    at = 8
    types = []
    while at < len(raw):
        require(at+12 <= len(raw), 'truncated PNG chunk')
        length = struct.unpack_from('>I', raw, at)[0]
        require(at+12+length <= len(raw), 'PNG chunk length overflow')
        kind = raw[at+4:at+8]
        payload = raw[at+8:at+8+length]
        crc = struct.unpack_from('>I', raw, at+8+length)[0]
        require(zlib.crc32(kind+payload) & 0xffffffff == crc, 'PNG CRC mismatch')
        types.append(kind)
        if kind == b'IHDR':
            require(len(types) == 1 and payload == struct.pack('>IIBBBBB', 640, 480, 16, 0, 0, 0, 0),
                    'requires noninterlaced static 640x480 grayscale uint16 PNG')
        require(kind not in [b'acTL', b'fcTL', b'fdAT'], 'animated depth PNG forbidden')
        at += length+12
    require(types[0] == b'IHDR' and types[-1] == b'IEND' and types.count(b'IHDR') == 1
            and types.count(b'IEND') == 1 and b'IDAT' in types, 'PNG structure incomplete')
    image = Image.open(io.BytesIO(raw))
    require(image.size == (640, 480) and image.mode in ['I', 'I;16'], 'depth decoder mode')
    image.load()
    require(len(image.tobytes()) <= 640*480*4, 'depth decode allocation bound')
    pixels = image.load()
    voxels = {}
    for y in range(0, 480, 8):
        for x in range(0, 640, 8):
            d = pixels[x, y]
            require(isinstance(d, int) and 0 <= d <= 65535, 'depth not uint16')
            z = d/5000.
            if d == 0 or not .3 <= z <= 5:
                continue
            p = [(x-319.5)*z/525., (y-239.5)*z/525., z]
            key = tuple(math.floor(v/.03) for v in p)
            voxels.setdefault(key, p)
    points = [voxels[k] for k in sorted(voxels)]
    require(len(points) <= 4800, 'sampled pixel bound')
    return points


def covariance_nees(matrix, error):
    require(isinstance(matrix, list) and len(matrix) == 6, 'covariance dimension')
    for row in matrix:
        vector(row, 6, 'invalid covariance row')
    lower = [[0.]*6 for _ in range(6)]
    for i in range(6):
        for j in range(i+1):
            close(matrix[i][j], matrix[j][i], 'asymmetric covariance', 1e-10)
            s = matrix[i][j]-sum(lower[i][k]*lower[j][k] for k in range(j))
            if i == j:
                require(s > 0, 'covariance is not positive definite')
                lower[i][j] = math.sqrt(s)
            else:
                lower[i][j] = s/lower[j][j]
    y = []
    for i in range(6):
        y.append((error[i]-sum(lower[i][j]*y[j] for j in range(i)))/lower[i][i])
    return sum(v*v for v in y)


def correspondences(scan, map_points, estimate, config):
    # Independent final-pose NN/unique-target/trim reconstruction, no ICP fit.
    size = config['max_correspondence_m']
    cells = {}
    for i, p in enumerate(map_points):
        cells.setdefault(tuple(math.floor(v/size) for v in p), []).append(i)
    unique = {}
    t, q = estimate
    for source, p in enumerate(scan):
        p = [x+y for x, y in zip(rotate(q, p), t)]
        cell = tuple(math.floor(v/size) for v in p)
        best = None
        for delta in itertools.product([-1, 0, 1], repeat=3):
            key = tuple(a+b for a, b in zip(cell, delta))
            for target in cells.get(key, []):
                distance = sum((x-y)**2 for x, y in zip(p, map_points[target]))
                if distance <= size*size and (best is None or (distance, target) < best):
                    best = distance, target
        if best is not None:
            distance, target = best
            candidate = distance, source
            if target not in unique or candidate < unique[target]:
                unique[target] = candidate
    retained = sorted(unique.values())
    retained = retained[:math.floor(len(retained)*(1-config['trim_fraction']))]
    require(retained, 'accepted pose has no retained correspondences')
    return len(retained), math.sqrt(sum(v[0] for v in retained)/len(retained))


def check_report(report, manifest, manifest_raw, source, clouds, gt, freeze, source_paths):
    require(report.get('schema_version') == 1, 'report schema')
    for key in ['dataset', 'repository', 'revision']:
        require(report[key] == manifest[key], 'source identity mismatch')
    require(report['manifest_sha256'] == digest(manifest_raw), 'manifest digest mismatch')
    require(freeze.get('kind') in ['baseline_retrospective', 'preregistered_temporal', 'calibration_regression'], 'freeze must explicitly distinguish retrospective baseline and preregistered temporal evaluation')
    if freeze['kind'] == 'baseline_retrospective':
        require(manifest['dataset'] == 'tum-fr1-xyz', 'retrospective marker is only the original sparse baseline')
    if freeze['kind'] == 'calibration_regression':
        require(manifest['dataset'] == 'tum-fr1-xyz-fast', 'calibration_regression is only the previously viewed fast subset')
    if freeze['kind'] == 'calibration_regression' or manifest['dataset'] == 'tum-fr1-xyz-tight':
        require(report.get('evaluation_role') == freeze['kind'], 'evaluation role misrepresents viewed and temporal validation subsets')
    if freeze['kind'] in ['preregistered_temporal', 'calibration_regression']:
        require(report.get('freeze_json') == freeze, 'report does not retain exact preregistered freeze')
        require(report.get('freeze_sha256') == source['__freeze_sha256'], 'report freeze byte digest mismatch')
    for key, path in source_paths.items():
        require(report[key] == digest(bounded(path)) == freeze[key], 'source/lock freeze mismatch '+key)
    require(freeze['manifest_sha256'] == report['manifest_sha256'], 'manifest freeze mismatch')
    require(canonical(report['registration_config']) == freeze['registration_config_sha256'], 'configuration freeze mismatch')
    require(canonical(report['preprocessing']) == freeze['preprocessing_sha256'], 'preprocessing freeze mismatch')
    preprocessing = {'width':640, 'height':480, 'fx':525., 'fy':525., 'cx':319.5, 'cy':239.5,
                     'units_per_metre':5000, 'pixel_step':8, 'sample_origin_pixel':[0, 0],
                     'min_depth_m':.3, 'max_depth_m':5., 'voxel_m':.03,
                     'voxel_representative':'first row-major sampled valid pixel',
                     'point_order':'lexicographic voxel key', 'maximum_sampled_pixels':4800}
    require(report['preprocessing'] == preprocessing, 'preprocessing differs from frozen model')
    require(report['accuracy_gates'] == {'translation_m':.1, 'rotation_rad':.1}, 'accuracy gates changed')
    require(report['ground_truth_interpolation'] == {'method':'linear translation and shortest-arc quaternion SLERP', 'max_bracket_s':.02, 'extrapolation':False}, 'GT interpolation changed')
    require(report['files'] == [{k:e[k] for k in ['file', 'bytes', 'sha256', 'role']} for e in manifest['files']], 'source provenance entries changed')
    frames = manifest['frames']
    require(len(report['pairs']) == 11, 'omitted/added pair')
    accepted = passed = heldout_passed = heldout_pairs = 0
    checked = []
    c = report['registration_config']
    for i, row in enumerate(report['pairs']):
        a, b = frames[i:i+2]
        require(row['previous_file'] == a['file'] and row['current_file'] == b['file'], 'pair order/selection changed')
        for key, value in [('previous_timestamp', a['timestamp']), ('current_timestamp', b['timestamp']), ('interval_s', b['timestamp']-a['timestamp'])]:
            require(row[key] == value, 'pair sensor timestamp mismatch')
        require(row['split'] == b['split'], 'pair split changed')
        require(row['initial_pose'] == 'identity; no current motion-capture pose supplied', 'GT or nonidentity initialization')
        require(row['map_points'] == len(clouds[i]) and row['scan_points'] == len(clouds[i+1]), 'depth geometry count mismatch')
        require(isinstance(row['cpu_wall_seconds'], (int, float)) and math.isfinite(row['cpu_wall_seconds']) and 0 <= row['cpu_wall_seconds'] <= 300, 'invalid elapsed duration')
        p0 = interpolate(gt, a['timestamp']); p1 = interpolate(gt, b['timestamp'])
        truth = relative(p0, p1)
        actual_truth = pose(row['evaluation_only_relative_truth'])
        for x, y in zip(actual_truth[0], truth[0]):
            close(x, y, 'physical relative truth translation mismatch')
        close(quaternion_error(actual_truth[1], truth[1]), 0, 'physical relative truth rotation mismatch')
        require(type(row['accepted']) is bool and type(row['within_accuracy_gates']) is bool, 'status must be boolean')
        heldout = b['split'] == 'held_out'
        heldout_pairs += int(heldout)
        evidence = {'pair':i, 'previous_gt_bracket':p0[2], 'current_gt_bracket':p1[2], 'accepted':row['accepted']}
        if row['accepted']:
            accepted += 1
            require('rejection' not in row, 'accepted result carries rejection')
            estimate = pose(row['estimate'])
            error_xyz = [x-y for x, y in zip(estimate[0], truth[0])]
            translation_error = norm(error_xyz)
            rotation_error = quaternion_error(estimate[1], truth[1])
            close(row['translation_error_m'], translation_error, 'false translation error')
            close(row['rotation_error_rad'], rotation_error, 'false rotation error')
            ok = translation_error <= .1 and rotation_error <= .1
            require(row['within_accuracy_gates'] == ok, 'false accuracy pass')
            passed += int(ok); heldout_passed += int(ok and heldout)
            require(norm(estimate[0]) <= c['max_translation_jump_m'] and quaternion_error(estimate[1], [1,0,0,0]) <= c['max_rotation_jump_rad'], 'accepted pose jump exceeds bounds')
            count, rms = correspondences(clouds[i+1], clouds[i], estimate, c)
            require(row['inlier_count'] == count and count >= c['min_pairs'], 'false retained correspondence count')
            close(row['inlier_fraction'], count/len(clouds[i+1]), 'false overlap')
            close(row['rms_m'], rms, 'false RMS', 1e-7)
            require(rms <= c['max_rms_m'] and row['inlier_fraction'] >= c['min_overlap'], 'accepted residual/overlap out of bounds')
            require(type(row['iterations']) is int and 1 <= row['iterations'] <= c['max_iterations'] and row['converged'] is True, 'accepted iteration/convergence metadata')
            require(math.isfinite(row['geometry_ratio']) and row['geometry_ratio'] >= c['min_geometry_ratio'], 'geometry conditioning bound')
            require(math.isfinite(row['condition_number']) and 1 <= row['condition_number'] <= c['max_condition_number'], 'condition number bound')
            require(type(row['neighbor_checks']) is int and 0 <= row['neighbor_checks'] <= c['max_neighbor_checks'] and row['ambiguity_probes'] == 12, 'work/ambiguity metadata')
            covariance = row['conditional_covariance_xyz_rotation']
            dq = qnorm(multiply(estimate[1], conjugate(truth[1])))
            angle = quaternion_error(estimate[1], truth[1]); n = norm(dq[1:])
            rotation_vector = [x*angle/n for x in dq[1:]] if n > 1e-12 else [0.,0.,0.]
            nees = covariance_nees(covariance, error_xyz+rotation_vector)
            for k in range(6):
                require(covariance[k][k] <= (c['max_position_variance_m2'] if k < 3 else c['max_rotation_variance_rad2']), 'conditional variance bound')
            evidence.update({'translation_error_m':translation_error, 'rotation_error_rad':rotation_error, 'conditional_nees':nees})
        else:
            require(row['within_accuracy_gates'] is False and isinstance(row.get('rejection'), str) and row['rejection'], 'rejected pair concealed or reason absent')
            require(not any(key in row for key in ['estimate','translation_error_m','rotation_error_rad','conditional_covariance_xyz_rotation']), 'rejected pair invents an estimate')
            evidence['rejection'] = row['rejection']
        checked.append(evidence)
    expected = {'pairs':11, 'accepted':accepted, 'passed':passed, 'rejected':11-accepted,
                'heldout_pairs':heldout_pairs, 'heldout_passed':heldout_passed, 'all_pairs_passed':passed==11}
    require(report['summary'] == expected, 'summary suppresses failed pairs')
    return {'schema':'rustdrive-recorded-rgbd-oracle-v1', 'summary':expected, 'pairs':checked,
            'source_sha256_verified':True, 'depth_geometry_reconstructed':True,
            'scope':'Independent integrity/physical relative-pose accuracy; no ICP replay, covariance calibration, independent-scene, or driving claim'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--freeze', required=True, type=Path)
    parser.add_argument('--manifest', type=Path, default=ROOT/'data/tum-fr1-xyz/manifest.json')
    parser.add_argument('--raw', type=Path)
    parser.add_argument('--matcher-source', type=Path, default=ROOT/'crates/localization/src/registration3d.rs')
    parser.add_argument('--evaluator-source', type=Path, default=ROOT/'integrations/rgbd/src/main.rs')
    parser.add_argument('--cargo-lock', type=Path, default=ROOT/'integrations/rgbd/Cargo.lock')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    manifest_raw = bounded(args.manifest, 128*1024)
    manifest = json.loads(manifest_raw)
    report = json.loads(bounded(args.report))
    freeze_raw = bounded(args.freeze)
    freeze = json.loads(freeze_raw)
    source_paths = {'matcher_source_sha256':args.matcher_source, 'evaluator_source_sha256':args.evaluator_source, 'cargo_lock_sha256':args.cargo_lock}
    args.raw = args.raw or args.manifest.parent/'raw'
    calibration = manifest['depth_calibration']
    require(all(calibration[k] == v for k, v in {'width':640,'height':480,'fx':525.,'fy':525.,'cx':319.5,'cy':239.5,'units_per_metre':5000.,'invalid_depth':0}.items()), 'manifest calibration mismatch')
    frames = manifest['frames']
    require(manifest['dataset'] in ['tum-fr1-xyz', 'tum-fr1-xyz-fast', 'tum-fr1-xyz-tight'], 'unknown prerecorded dataset')
    require(len(frames) == 12 and all(f['source_index'] == (i*10 if manifest['dataset']=='tum-fr1-xyz' else (120+i if manifest['dataset']=='tum-fr1-xyz-fast' else 140+i)) and f['split'] == ('calibration' if i < 3 else 'held_out') for i,f in enumerate(frames)), 'manifest selection mismatch')
    source = {'__freeze_sha256':digest(freeze_raw)}
    for e in manifest['files']:
        require(Path(e['file']).name == e['file'] and '/' not in e['file'] and '\\' not in e['file'], 'unsafe raw filename')
        raw = bounded(args.raw/e['file'])
        require(len(raw) == e['bytes'] and digest(raw) == e['sha256'], 'raw source hash/size mismatch')
        source[e['file']] = raw
    index = [line.split() for line in source['depth.txt'].decode('ascii').splitlines() if line.strip() and not line.lstrip().startswith('#')]
    for f in frames:
        row = index[f['source_index']]
        require(len(row) == 2 and float(row[0]) == f['timestamp'] and row[1] == 'depth/'+f['file'], 'frame does not match original depth index')
        require(float(f['file'].removesuffix('.png')) == f['timestamp'], 'filename timestamp mismatch')
    clouds = [depth_geometry(source[f['file']]) for f in frames]
    gt = gt_rows(source['groundtruth.txt'])
    result = check_report(report, manifest, manifest_raw, source, clouds, gt, freeze, source_paths)
    operations = {
        'false_relative_truth':lambda r:r['pairs'][0]['evaluation_only_relative_truth']['translation_m'].__setitem__(0,123.),
        'false_sensor_stamp':lambda r:r['pairs'][0].__setitem__('current_timestamp',r['pairs'][0]['current_timestamp']+.0001),
        'ground_truth_initialization':lambda r:r['pairs'][0].__setitem__('initial_pose','current mocap pose'),
        'omitted_failed_pair':lambda r:r['pairs'].pop(),
        'changed_source_manifest':lambda r:r.__setitem__('manifest_sha256','0'*64),
        'changed_matcher_freeze':lambda r:r.__setitem__('matcher_source_sha256','0'*64),
        'changed_calibration':lambda r:r['preprocessing'].__setitem__('fx',500.),
        'invented_geometry_count':lambda r:r['pairs'][0].__setitem__('scan_points',1),
        'concealed_failure':lambda r:r['pairs'][0].__setitem__('within_accuracy_gates',not r['pairs'][0]['within_accuracy_gates']),
    }
    if freeze['kind'] == 'calibration_regression':
        require(manifest['dataset'] == 'tum-fr1-xyz-fast', 'calibration_regression is only the previously viewed fast subset')
    if freeze['kind'] in ['preregistered_temporal', 'calibration_regression']:
        operations['changed_embedded_freeze'] = lambda r:r['freeze_json'].__setitem__('matcher_source_sha256','0'*64)
        operations['changed_freeze_byte_digest'] = lambda r:r.__setitem__('freeze_sha256','0'*64)
    accepted_rows = [i for i,r in enumerate(report['pairs']) if r['accepted']]
    if accepted_rows:
        i = accepted_rows[0]
        operations['false_pose_error'] = lambda r:r['pairs'][i].__setitem__('translation_error_m',123.)
        operations['invented_estimate'] = lambda r:r['pairs'][i]['estimate']['translation_m'].__setitem__(0,123.)
        operations['invalid_covariance'] = lambda r:r['pairs'][i]['conditional_covariance_xyz_rotation'][0].__setitem__(0,-1.)
    rejected = []
    for name, mutate in operations.items():
        changed = copy.deepcopy(report); mutate(changed)
        try:
            check_report(changed, manifest, manifest_raw, source, clouds, gt, freeze, source_paths)
        except (ValueError, KeyError, TypeError):
            rejected.append(name)
        else:
            raise ValueError('oracle accepted mutation '+name)
    result['mutations_rejected'] = rejected
    result['freeze_kind'] = freeze['kind']
    result['accepted_branch_checked'] = bool(accepted_rows)
    result['conditional_nees_scope'] = 'Computed only for accepted estimates; conditional least-squares model excludes depth/map/association correlation and is not calibrated uncertainty.'
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, allow_nan=False))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, TypeError, IndexError, struct.error) as error:
        print('recorded RGB-D oracle: '+str(error), file=sys.stderr)
        sys.exit(2)
