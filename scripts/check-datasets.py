#!/usr/bin/env python3
"""Independently reconstruct measured-cloud scores and AABBs from pinned raw bytes.

This verifies report integrity, not a target accuracy or real-driving claim.
The Rust evaluator and this oracle do not share parsers or metric code.
"""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parent.parent
MAX_FILE_BYTES = 16_000_000
MAX_POINTS = 500_000


def require(condition, message):
    if not condition:
        raise ValueError(message)


def bounded_bytes(path):
    require(path.stat().st_size <= MAX_FILE_BYTES, 'input file exceeds oracle bound')
    with path.open('rb') as source:
        raw = source.read(MAX_FILE_BYTES+1)
    require(len(raw) <= MAX_FILE_BYTES, 'input grew beyond oracle bound')
    return raw


def pcd(path):
    raw = bounded_bytes(path)
    header = {}
    position = 0
    while True:
        end = raw.find(b'\n', position)
        require(end >= 0 and end < 4096, 'invalid bounded PCD header')
        line = raw[position:end].decode('ascii').strip()
        position = end + 1
        if not line or line.startswith('#'):
            continue
        key, *values = line.split()
        require(key not in header, 'repeated PCD field')
        header[key] = values
        if key == 'DATA':
            break
    require(header.get('FIELDS') == ['x', 'y', 'z']
            and header.get('SIZE') == ['4', '4', '4']
            and header.get('TYPE') == ['F', 'F', 'F']
            and header.get('COUNT') == ['1', '1', '1']
            and header.get('DATA') == ['binary_compressed'], 'unsupported PCD layout')
    count = int(header['POINTS'][0])
    require(0 < count <= MAX_POINTS and int(header['WIDTH'][0])*int(header['HEIGHT'][0]) == count,
            'invalid PCD point count')
    require(len(raw) >= position + 8, 'truncated PCD sizes')
    compressed, expanded = struct.unpack_from('<II', raw, position)
    position += 8
    end = position + compressed
    require(expanded == count*12 and end <= len(raw), 'PCD payload sizes differ')
    # These SHA-pinned historical PCL files carry zero padding after the block.
    require(len(raw)-end <= 4096 and not any(raw[end:]), 'unexpected PCD trailing bytes')
    payload = raw[position:end]
    output = bytearray()
    offset = 0
    while offset < len(payload):
        control = payload[offset]
        offset += 1
        if control < 32:
            length = control + 1
            require(offset + length <= len(payload), 'truncated LZF literal')
            require(len(output) + length <= expanded, 'LZF expansion exceeds bound')
            output.extend(payload[offset:offset+length])
            offset += length
        else:
            length = control >> 5
            back = (control & 31) << 8
            if length == 7:
                require(offset < len(payload), 'truncated LZF long match')
                length += payload[offset]
                offset += 1
            require(offset < len(payload), 'truncated LZF backreference')
            back += payload[offset] + 1
            offset += 1
            length += 2
            require(back <= len(output) and len(output)+length <= expanded, 'invalid LZF reference')
            for _ in range(length):
                output.append(output[-back])
    require(len(output) == expanded, 'incomplete LZF expansion')
    axes = [struct.unpack_from(f'<{count}f', output, axis*count*4) for axis in range(3)]
    points = list(zip(*axes))
    require(all(math.isfinite(x) for p in points for x in p), 'nonfinite measured coordinates')
    return points


def indices(values, count, name):
    require(isinstance(values, list) and all(type(x) is int and 0 <= x < count for x in values),
            f'{name}: invalid original index')
    require(len(set(values)) == len(values), f'{name}: repeated original index')
    return set(values)


def vtk(path):
    words = bounded_bytes(path).decode('ascii').split()
    require(words[:5] == ['#', 'vtk', 'DataFile', 'Version', '3.0']
            and 'ASCII' in words[:20], 'unsupported VTK source')
    start = words.index('POINTS')
    count = int(words[start+1])
    require(0 < count <= MAX_POINTS and words[start+2] in ('float', 'double'), 'invalid VTK points')
    values = list(map(float, words[start+3:start+3+3*count]))
    require(len(values) == 3*count and all(math.isfinite(x) for x in values), 'invalid VTK coordinates')
    return [values[i:i+3] for i in range(0, len(values), 3)]


def quadratic_form(error, covariance):
    # Cholesky whitening is independent of the Rust evaluator's Gaussian elimination.
    lower = [[0.]*3 for _ in range(3)]
    for i in range(3):
        for j in range(i+1):
            residual = covariance[i][j]-sum(lower[i][k]*lower[j][k] for k in range(j))
            if i == j:
                require(residual > 0., 'non-SPD registration covariance')
                lower[i][j] = math.sqrt(residual)
            else:
                lower[i][j] = residual/lower[j][j]
    white = [0.]*3
    for i in range(3):
        white[i] = (error[i]-sum(lower[i][j]*white[j] for j in range(i)))/lower[i][i]
    return sum(x*x for x in white)


def metrics(confusion):
    tp, fp, tn, fn = (confusion[k] for k in ('tp', 'fp', 'tn', 'fn'))
    ratio = lambda a, b: a/b if b else None
    return {'precision': ratio(tp, tp+fp), 'recall': ratio(tp, tp+fn),
            'f1': ratio(2*tp, 2*tp+fp+fn), 'accuracy': ratio(tp+tn, tp+fp+tn+fn)}


def close(actual, expected, name, tolerance=1e-10):
    if expected is None:
        require(actual is None, f'{name}: undefined score must remain null')
        return
    require(type(actual) in (int, float) and math.isfinite(actual)
            and abs(actual-expected) <= tolerance*max(1., abs(expected)), f'{name}: numeric mismatch')


def xyz(value):
    return [value[k] for k in ('x', 'y', 'z')] if isinstance(value, dict) else value


def check_report(report, data_root):
    require(report['schema_version'] == 1, 'unsupported report schema')
    require(report['evaluation_complete'] is True and report['metric_acceptance_claim'] is False,
            'baseline completion/accuracy scope differs')
    require(report['hash_verification']['raw_sha256_verified'] is True, 'raw verification absent')
    terrain = report['frozen_parameters']['terrain_config']
    require(all(terrain[k] == value for k, value in {
        'cell_size_m': 1., 'initial_height_m': .15, 'max_height_m': 2.5,
        'max_slope': .3, 'window_radii_cells': [1, 2, 4, 8, 16],
        'min_support_neighbors': 3, 'min_supported_cells': 12,
        'min_supported_fraction': .5}.items()), 'frozen terrain parameters changed')
    manifests = {}
    for name, key in [('isprs-terrain', 'isprs_terrain'), ('libpointmatcher', 'libpointmatcher')]:
        manifest = json.loads((ROOT/'data'/name/'manifest.json').read_text())
        require(report['manifests'][key] == manifest, 'report source manifest differs from pinned input')
        manifests[name] = manifest
        for entry in manifest['files']:
            raw = bounded_bytes(data_root/name/'raw'/entry['file'])
            require(len(raw) == entry['bytes'] and hashlib.sha256(raw).hexdigest() == entry['sha256'],
                    'measured raw data differs from pinned SHA/size')
    manifest = manifests['isprs-terrain']
    expected = manifest['calibration_samples'] + manifest['held_out_samples']
    rows = report['terrain_samples']
    require(len(rows) == len(expected) and {r['sample'] for r in rows} == set(expected),
            'missing/repeated measured terrain site')
    aggregates = {s: dict.fromkeys(('tp', 'fp', 'tn', 'fn'), 0) for s in ('calibration', 'held_out')}
    clusters_checked = 0
    for row in rows:
        sample = row['sample']
        split = 'calibration' if sample in manifest['calibration_samples'] else 'held_out'
        require(row['split'] == split, 'calibration/held-out split changed')
        cloud_entry = next(e for e in manifest['files'] if e['sample'] == sample and e['role'] == 'input_cloud')
        label_entry = next(e for e in manifest['files'] if e['sample'] == sample and e['role'] == 'ground_reference')
        points = pcd(data_root/'isprs-terrain'/'raw'/cloud_entry['file'])
        labels = pcd(data_root/'isprs-terrain'/'raw'/label_entry['file'])
        require(row['input_sha256'] == cloud_entry['sha256']
                and row['ground_reference_sha256'] == label_entry['sha256']
                and row['points'] == len(points) and row['reference_points'] == len(labels),
                'terrain provenance/count differs')
        reference = set(labels)
        require(reference <= set(points) and row['reference_unique_points'] == len(reference),
                'ground reference is not the declared measured subset')
        predicted = indices(row['ground_indices'], len(points), 'ground')
        nonground = indices(row['non_ground_indices'], len(points), 'non-ground')
        require(not predicted & nonground and len(predicted | nonground) == len(points),
                'ground/non-ground partition incomplete')
        confusion = dict.fromkeys(('tp', 'fp', 'tn', 'fn'), 0)
        for i, p in enumerate(points):
            truth = p in reference
            confusion['tp' if truth and i in predicted else 'fn' if truth else 'fp' if i in predicted else 'tn'] += 1
        require(confusion == row['confusion'], 'ground confusion matrix differs from measured reference')
        for key, value in metrics(confusion).items():
            close(row['metrics'][key], value, 'ground '+key)
        for key, value in confusion.items():
            aggregates[split][key] += value
        origin = xyz(row['origin_xyz_m'])
        require(len(origin) == 3 and all(math.isfinite(x) for x in origin), 'invalid recentering origin')
        objects = row['objects']
        if objects['status'] == 'evaluated':
            require(objects['cluster_count'] == len(objects['aabbs']), 'cluster count differs')
            used = set(predicted)
            noise = indices(objects['noise_indices'], len(points), 'noise')
            require(not used & noise, 'noise overlaps ground')
            used |= noise
            for obj in objects['aabbs']:
                original = indices(obj['original_point_indices'], len(points), 'object')
                require(original and not used & original and obj['point_count'] == len(original),
                        'object indices overlap or count differs')
                used |= original
                coords = [[points[i][a]-origin[a] for a in range(3)] for i in original]
                lower = [min(p[a] for p in coords) for a in range(3)]
                upper = [max(p[a] for p in coords) for a in range(3)]
                for a in range(3):
                    close(xyz(obj['min'])[a], lower[a], 'measured AABB min')
                    close(xyz(obj['max'])[a], upper[a], 'measured AABB max')
                    close(xyz(obj['center'])[a], (lower[a]+upper[a])/2, 'measured AABB center')
                clusters_checked += 1
            require(len(used) == len(points), 'XYZ point partition incomplete')
        else:
            require(objects['status'] == 'rejected' and objects.get('error_reason'), 'unreported object failure')
    for split, confusion in aggregates.items():
        require(report['terrain_aggregate'][split]['confusion'] == confusion, 'aggregate confusion differs')
        for key, value in metrics(confusion).items():
            close(report['terrain_aggregate'][split]['metrics'][key], value, 'aggregate '+key)
    natural = report['registration']['natural_pair']
    require(natural['physical_pose_ground_truth'] is None, 'natural measured pair has unsupported pose truth')
    cases = report['registration']['semi_synthetic']
    require(len(cases) == 4 and len({c['case_id'] for c in cases}) == 4, 'missing semi-synthetic cases')
    preprocessing = report['registration']['cloud_preprocessing']
    require(len(preprocessing) == 2, 'missing measured apartment view')
    for view in preprocessing:
        entry = next(e for e in manifests['libpointmatcher']['files'] if e['file'] == view['cloud'])
        points = vtk(data_root/'libpointmatcher'/'raw'/entry['file'])
        cells = {}
        for i, point in enumerate(points):
            if .5 <= point[2] <= 2.:
                cells.setdefault((math.floor(point[0]/.2), math.floor(point[1]/.2)), i)
        selected = [cells[key] for key in sorted(cells)]
        require(view['input_sha256'] == entry['sha256'] and view['raw_points'] == len(points)
                and view['selected_point_indices'] == selected and view['selected_points'] == len(selected),
                'measured apartment projection differs from declared frozen preprocessing')
    accepted = 0
    for case in cases:
        require(case['split'] in ('calibration', 'held_out'), 'invalid registration split')
        require(case['status'] in ('accepted', 'rejected'), 'missing registration outcome')
        if case['status'] == 'rejected':
            require(case.get('reason'), 'registration failure reason missing')
            continue
        known, recovered = case['known_pose'], case['recovered_pose']
        dx = recovered['position']['x'] - known['position']['x']
        dy = recovered['position']['y'] - known['position']['y']
        yaw = math.atan2(math.sin(recovered['yaw']-known['yaw']), math.cos(recovered['yaw']-known['yaw']))
        close(case['pose_error']['translation_m'], math.hypot(dx, dy), 'semi-synthetic translation error')
        close(case['pose_error']['yaw_rad'], abs(yaw), 'semi-synthetic yaw error')
        require(case['conditioning']['neighbor_checks'] <= 20_000_000
                and case['conditioning']['ambiguity_probes'] == 6, 'registration work/ambiguity evidence differs')
        covariance = case['covariance']
        for i in range(3):
            require(covariance[i][i] > 0, 'nonpositive registration variance')
            for j in range(3):
                close(covariance[i][j], covariance[j][i], 'covariance symmetry')
        statistic = quadratic_form([dx, dy, yaw], covariance)
        close(case['nees'], statistic, 'semi-synthetic NEES', tolerance=1e-8)
        require(case['within_nominal_95_percent_ellipsoid'] == (statistic <= 7.814727903251179),
                'nominal ellipsoid outcome differs')
        accepted += 1
    return {'terrain_sites': len(rows), 'independently_checked_aabbs': clusters_checked,
            'semi_synthetic_cases': len(cases), 'semi_synthetic_accepted': accepted,
            'scope': 'Integrity and recomputed accuracy; no target accuracy, natural-pose GT or safety claim'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--data-root', type=Path, default=ROOT/'data')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    report = json.loads(bounded_bytes(args.report))
    result = check_report(report, args.data_root)
    # Mutate independent evidence to establish that the oracle rejects false scores.
    mutations = []
    operations = {
        'false_ground_score': lambda x: x['terrain_samples'][0]['confusion'].__setitem__('tp', -1),
        'missing_held_out_site': lambda x: x['terrain_samples'].pop(),
        'changed_split': lambda x: x['terrain_samples'][0].__setitem__('split', 'held_out'),
        'changed_source': lambda x: x['manifests']['isprs_terrain'].__setitem__('revision', '0'*40),
        'invented_physical_gt': lambda x: x['registration']['natural_pair'].__setitem__('physical_pose_ground_truth', {}),
        'changed_frozen_parameters': lambda x: x['frozen_parameters']['terrain_config'].__setitem__('cell_size_m', 2.),
        'duplicate_ground_index': lambda x: x['terrain_samples'][0]['ground_indices'].append(x['terrain_samples'][0]['ground_indices'][0]),
        'invented_aabb_extent': lambda x: x['terrain_samples'][0]['objects']['aabbs'][0]['min'].__setitem__('x', -12345.),
        'invented_pose_error': lambda x: x['registration']['semi_synthetic'][0]['pose_error'].__setitem__('translation_m', 1.),
    }
    for name, mutate in operations.items():
        altered = copy.deepcopy(report)
        mutate(altered)
        try:
            check_report(altered, args.data_root)
        except ValueError:
            mutations.append(name)
        else:
            raise ValueError('oracle accepted mutation: '+name)
    result['mutations_rejected'] = mutations
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, StopIteration, IndexError, struct.error) as error:
        print('dataset oracle: '+str(error), file=sys.stderr)
        sys.exit(2)
