#!/usr/bin/env python3
"""Independently check adaptive terrain report integrity, not classifier accuracy.

Reads measured PCD/LAS coordinates and evaluator-only labels, never reimplements
the adaptive classifier. Fresh results must match the externally recorded freeze.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parent.parent
MAX_REPORT_BYTES = 32_000_000
MAX_RAW_BYTES = 16_000_000
MAX_POINTS = 500_000
spec = importlib.util.spec_from_file_location('adaptive_dataset_oracle', ROOT/'scripts/check-datasets.py')
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
require, close, indices, metrics, xyz = base.require, base.close, base.indices, base.metrics, base.xyz


def bounded(path, maximum=MAX_RAW_BYTES):
    require(path.stat().st_size <= maximum, 'file exceeds independent oracle bound')
    with path.open('rb') as stream:
        raw = stream.read(maximum+1)
    require(len(raw) <= maximum, 'file grew beyond independent oracle bound')
    return raw


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def canonical_digest(value):
    return digest(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode())


def las(raw):
    """LAS 1.0–1.3 legacy uncompressed point formats 0–3 (ASPRS layout)."""
    require(227 <= len(raw) <= MAX_RAW_BYTES and raw[:4] == b'LASF', 'invalid LAS signature/header')
    major, minor = raw[24:26]
    require(major == 1 and minor in (0, 1, 2, 3), 'unsupported LAS version')
    header_size, = struct.unpack_from('<H', raw, 94)
    point_offset, vlr_count = struct.unpack_from('<II', raw, 96)
    point_format = raw[104]
    record_size, count = struct.unpack_from('<HI', raw, 105)
    minimum = {0: 20, 1: 28, 2: 26, 3: 34}
    require(point_format in minimum, 'compressed/unsupported LAS point format')
    require((235 if minor == 3 else 227) <= header_size <= point_offset <= len(raw)
            and minimum[point_format] <= record_size <= 256
            and 0 < count <= MAX_POINTS and point_offset+count*record_size <= len(raw),
            'invalid LAS header/point dimensions')
    require(vlr_count <= 4096, 'LAS VLR count exceeds bound')
    cursor = header_size
    for _ in range(vlr_count):
        require(cursor+54 <= point_offset, 'LAS variable record header overlaps points')
        length, = struct.unpack_from('<H', raw, cursor+20)
        cursor += 54+length
        require(cursor <= point_offset, 'LAS variable record payload overlaps points')
    scales = struct.unpack_from('<3d', raw, 131)
    offsets = struct.unpack_from('<3d', raw, 155)
    require(all(math.isfinite(v) and 0 < v <= 1000 for v in scales)
            and all(math.isfinite(v) and abs(v) <= 10_000_000 for v in offsets),
            'invalid bounded LAS scales/offsets')
    points, labels, classes, withheld = [], [], [], []
    for i in range(count):
        start = point_offset+i*record_size
        integers = struct.unpack_from('<3i', raw, start)
        point = tuple(integers[a]*scales[a]+offsets[a] for a in range(3))
        require(all(math.isfinite(v) and abs(v) <= 10_000_000 for v in point), 'invalid LAS XYZ')
        classification = raw[start+15]
        category = classification & 31
        hidden = bool(classification & 128)
        points.append(point)
        classes.append(category)
        withheld.append(hidden)
        labels.append(None if hidden or category not in {2, 3, 4, 5, 6, 9, 10, 11} else category == 2)
    return points, labels, {'version_minor': minor, 'point_format': point_format,
                            'scales': scales, 'offsets': offsets}, classes, withheld


def parser_self_test():
    """Authored binary fixtures test layout/labels, never measured accuracy."""
    categories = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 31, 130, 34, 67]
    expected = [None, None, True, False, False, False, False, None, None,
                False, False, False, None, None, None, True, False]
    for minor in range(4):
        for point_format, record_size in {0: 20, 1: 28, 2: 26, 3: 34}.items():
            header_size = 235 if minor == 3 else 227
            raw = bytearray(header_size+len(categories)*record_size)
            raw[:4] = b'LASF'
            raw[24:26] = bytes((1, minor))
            struct.pack_into('<HII', raw, 94, header_size, header_size, 0)
            raw[104] = point_format
            struct.pack_into('<HI', raw, 105, record_size, len(categories))
            struct.pack_into('<3d', raw, 131, .01, .02, .03)
            struct.pack_into('<3d', raw, 155, 100., 200., 300.)
            for i, category in enumerate(categories):
                start = header_size+i*record_size
                struct.pack_into('<3i', raw, start, -2, 3, 4)
                raw[start+15] = category
            points, labels, _, _, _ = las(raw)
            require(labels == expected, 'LAS reference class/flag parser self-test failed')
            for actual, value in zip(points[0], (99.98, 200.06, 300.12)):
                close(actual, value, 'LAS scale/offset parser self-test', 1e-12)
            for malformed in ('compressed_format', 'oversized_count', 'point_overlap'):
                bad = bytearray(raw)
                if malformed == 'compressed_format':
                    bad[104] |= 128
                elif malformed == 'oversized_count':
                    struct.pack_into('<I', bad, 107, MAX_POINTS+1)
                else:
                    struct.pack_into('<I', bad, 96, header_size-1)
                try:
                    las(bad)
                except ValueError:
                    pass
                else:
                    raise ValueError('LAS parser accepted malformed authored fixture: '+malformed)
    return {'layout_cases': 16, 'malformed_cases_rejected': 48,
            'scope': 'Authored format/flag regression fixtures; not measured classification accuracy'}


def unit_factor(manifest, entry):
    units = entry.get('coordinate_units', manifest.get('coordinate_units'))
    factors = {'foot': .3048, 'feet': .3048, 'international_foot': .3048,
               'us_survey_foot': 1200/3937, 'meter': 1., 'metre': 1., 'm': 1.}
    require(units in factors, 'LAS manifest lacks supported coordinate units')
    expected = factors[units]
    actual = entry.get('unit_to_m', manifest.get('unit_to_m'))
    close(actual, expected, 'manifest SI conversion', 1e-12)
    return expected


def safe_file(entry):
    name = entry['file']
    require(isinstance(name, str) and name not in ('', '.', '..')
            and '/' not in name and '\\' not in name, 'manifest raw filename is not local')
    return name


class Inputs:
    """Load/hash independent evidence once; mutation checks reuse immutable inputs."""
    def __init__(self, report, repository, data_root, freeze_path):
        self.repository = repository
        dataset = report['dataset']
        require(isinstance(dataset, str) and dataset not in ('', '.', '..')
                and '/' not in dataset and '\\' not in dataset, 'invalid dataset directory')
        manifest_path = repository/'data'/dataset/'manifest.json'
        self.manifest_raw = bounded(manifest_path, 128_000)
        self.manifest = json.loads(self.manifest_raw)
        require(self.manifest['dataset'] == dataset, 'manifest dataset identity differs')
        self.source_sha = digest(bounded(repository/'crates/perception/src/terrain_adaptive.rs'))
        self.freeze = json.loads(bounded(freeze_path, 128_000)) if freeze_path else None
        self.entries = {}
        for entry in self.manifest['files']:
            identity = (entry['sample'], entry['role'])
            require(identity not in self.entries, 'duplicate dataset sample/role')
            self.entries[identity] = entry
        split = report['split']
        require((dataset == 'isprs-terrain' and split in ('calibration_original', 'regression'))
                or (dataset != 'isprs-terrain' and split == 'fresh_heldout'), 'unsupported dataset/split scope')
        self.expected = self.manifest['calibration_samples' if split == 'calibration_original' else 'held_out_samples']
        require(self.expected and len(set(self.expected)) == len(self.expected), 'empty/duplicate expected samples')
        self.samples = {}
        for sample in self.expected:
            entry = self.entries[(sample, 'input_cloud')]
            raw_path = data_root/dataset/'raw'/safe_file(entry)
            raw = bounded(raw_path)
            require(len(raw) == entry['bytes'] and digest(raw) == entry['sha256'], 'raw source SHA/size differs')
            is_las = 'las' in entry.get('encoding', '').lower()
            if is_las:
                points, labels, geometry, classes, withheld = las(raw)
                factor = unit_factor(self.manifest, entry)
                reference = None
            else:
                points = base.pcd(raw_path)
                label_entry = self.entries[(sample, 'ground_reference')]
                label_path = data_root/dataset/'raw'/safe_file(label_entry)
                reference_raw = bounded(label_path)
                require(len(reference_raw) == label_entry['bytes'] and digest(reference_raw) == label_entry['sha256'],
                        'ground-reference SHA/size differs')
                reference = base.pcd(label_path)
                require(len(reference) == label_entry['points'], 'reference point count differs')
                ground = set(reference)
                require(ground <= set(points), 'reference is not original float32 XYZ subset')
                labels = [point in ground for point in points]
                factor, geometry, classes, withheld = 1., None, None, None
            require(len(points) == entry['points'], 'manifest point count differs from measured source')
            origin = points[0]
            local = [tuple((p[a]-origin[a])*factor for a in range(3)) for p in points]
            self.samples[sample] = dict(entry=entry, points=points, local=local, labels=labels,
                reference=reference, factor=factor, geometry=geometry, classes=classes, withheld=withheld)


def check_counts(actual, expected, name):
    require(isinstance(actual, dict) and set(actual) == {'tp', 'fp', 'tn', 'fn'}
            and all(type(v) is int and v >= 0 for v in actual.values()) and actual == expected,
            name+': confusion differs from independently decoded labels')


def check_metrics(actual, confusion, name):
    require(set(actual) == {'precision', 'recall', 'f1', 'accuracy'}, name+': metric keys differ')
    for key, expected in metrics(confusion).items():
        close(actual[key], expected, name+' '+key)


def check_report(report, inputs):
    require(report['schema'] == 'rustdrive-adaptive-ground-v1'
            and report['evaluation_complete'] is True and report['metric_acceptance_claim'] is False,
            'report completion/integrity-versus-accuracy scope differs')
    require(report['manifest'] == inputs.manifest and report['manifest_sha256'] == digest(inputs.manifest_raw),
            'reported source manifest differs from pinned bytes')
    require(report['algorithm_source_sha256'] == inputs.source_sha, 'adaptive algorithm source SHA differs')
    parameters = report['frozen_parameters']
    parameter_sha = canonical_digest(parameters)
    require(report['frozen_parameters_sha256'] == parameter_sha
            and parameters['algorithm'] == 'rustdrive_perception::terrain_adaptive::classify_ground_adaptive',
            'frozen adaptive parameters/hash differ')
    tc, oc = parameters['terrain_config'], parameters['object_config']
    require(report['hash_verification'] == {'raw_sha256_verified': True, 'before_and_after': True},
            'source verification evidence absent')
    if report['split'] == 'fresh_heldout':
        require(inputs.freeze is not None and report['freeze'] == inputs.freeze,
                'fresh result lacks matching external pre-evaluation freeze')
        for key, expected in [('algorithm_source_sha256', inputs.source_sha),
                              ('frozen_parameters_sha256', parameter_sha),
                              ('manifest_sha256', digest(inputs.manifest_raw))]:
            require(inputs.freeze[key] == expected, 'external freeze '+key+' differs')
        require(inputs.freeze.get('fresh_source') == report['dataset'], 'external freeze names a different fresh source')
    else:
        require(report.get('freeze') is None, 'original calibration/regression must not claim a fresh freeze')
    rows = report['terrain_samples']
    require(len(rows) == len(inputs.expected) and [r['sample'] for r in rows] == inputs.expected,
            'measured split samples missing/repeated/reordered')
    total = dict.fromkeys(('tp', 'fp', 'tn', 'fn'), 0)
    sites, aabbs, scored, positives, negatives, exclusions = {}, 0, 0, 0, 0, 0
    accuracy_samples = []
    for row in rows:
        sample = row['sample']
        evidence = inputs.samples[sample]
        entry, points, labels, local = (evidence[k] for k in ('entry', 'points', 'labels', 'local'))
        count = len(points)
        require(row['split'] == report['split'] and row['points'] == count
                and row['input_sha256'] == entry['sha256'], 'sample scope/count/source SHA differs')
        close(row['unit_to_m'], evidence['factor'], 'reported SI factor', 1e-12)
        for a in range(3):
            close(xyz(row['raw_origin_xyz_source_units'])[a], points[0][a], 'original measured origin')
            close(xyz(row['origin_xyz_m'])[a], points[0][a]*evidence['factor'], 'SI origin')
        expected_excluded = {i for i, label in enumerate(labels) if label is None}
        require(indices(row['excluded_scoring_indices'], count, 'excluded scoring') == expected_excluded,
                'scoring exclusion differs from unknown/withheld source classes')
        if evidence['geometry']:
            geometry = row['input_geometry']
            require(geometry['format'] == 'uncompressed LAS', 'LAS source format differs')
            for key in ('version_minor', 'point_format'):
                require(geometry[key] == evidence['geometry'][key], 'LAS geometry metadata differs')
            for key in ('scales', 'offsets'):
                require(len(geometry[key]) == 3, 'LAS geometry axis dimensions differ')
                for actual, expected in zip(geometry[key], evidence['geometry'][key]):
                    close(actual, expected, 'LAS '+key, 1e-12)
            require(row['label_metadata']['source_sha256'] == entry['sha256'], 'LAS label source SHA differs')
        else:
            reference_entry = inputs.entries[(sample, 'ground_reference')]
            require(row['input_geometry']['format'] == 'PCD binary_compressed XYZ float32'
                    and row['label_metadata']['reference_sha256'] == reference_entry['sha256']
                    and row['label_metadata']['reference_points'] == len(evidence['reference'])
                    and row['label_metadata']['reference_unique_points'] == len(set(evidence['reference']))
                    and row['label_metadata']['raw_unique_points'] == len(set(points)), 'PCD reference metadata differs')
        ground = indices(row['ground_indices'], count, 'ground')
        other = indices(row['non_ground_indices'], count, 'non-ground')
        require(not ground & other and len(ground | other) == count, 'original XYZ index partition incomplete')
        confusion = dict.fromkeys(('tp', 'fp', 'tn', 'fn'), 0)
        for i, label in enumerate(labels):
            if label is not None:
                confusion['tp' if label and i in ground else 'fn' if label else 'fp' if i in ground else 'tn'] += 1
        check_counts(row['confusion'], confusion, 'sample '+sample)
        check_metrics(row['metrics'], confusion, 'sample '+sample)
        site = entry.get('site', 'site'+sample[4] if report['dataset'] == 'isprs-terrain' else sample)
        require(row['site'] == site, 'source site identity differs')
        sites.setdefault(site, dict.fromkeys(total, 0))
        for key, value in confusion.items():
            total[key] += value
            sites[site][key] += value
        positives += confusion['tp']+confusion['fn']
        negatives += confusion['tn']+confusion['fp']
        scored += sum(confusion.values())
        exclusions += len(expected_excluded)
        diagnostic = row['classification_diagnostics']
        occupied = len({(math.floor(p[0]/tc['cell_size_m']), math.floor(p[1]/tc['cell_size_m'])) for p in local})
        require(diagnostic['occupied_cells'] == occupied
                and type(diagnostic['supported_cells']) is int and 0 <= diagnostic['supported_cells'] <= occupied,
                'measured occupied/support cell census differs')
        fraction = diagnostic['supported_cells']/occupied if occupied else 0.
        close(diagnostic['supported_fraction'], fraction, 'support fraction')
        confident = diagnostic['supported_cells'] >= tc['min_supported_cells'] and fraction >= tc['min_supported_fraction']
        require(type(diagnostic['confident']) is bool and diagnostic['confident'] == confident,
                'reported terrain confidence differs from declared support rule')
        require(type(diagnostic['candidate_work']) is int and 0 <= diagnostic['candidate_work'] <= tc['max_candidate_work'],
                'terrain candidate work exceeds frozen bound')
        objects = row['objects']
        if objects['status'] == 'evaluated':
            require(objects['cluster_count'] == len(objects['aabbs']) <= oc['max_clusters'], 'XYZ object count differs')
            noise = indices(objects['noise_indices'], count, 'object noise')
            require(noise <= other, 'object noise contains classified ground')
            used = set(noise)
            for obj in objects['aabbs']:
                original = indices(obj['original_point_indices'], count, 'object')
                require(original and original <= other and not original & used
                        and obj['point_count'] == len(original)
                        and oc['min_points'] <= len(original) <= oc['max_cluster_points'], 'object partition/count differs')
                used |= original
                lower = [min(local[i][a] for i in original) for a in range(3)]
                upper = [max(local[i][a] for i in original) for a in range(3)]
                for a in range(3):
                    close(xyz(obj['min'])[a], lower[a], 'measured XYZ AABB min')
                    close(xyz(obj['max'])[a], upper[a], 'measured XYZ AABB max')
                    close(xyz(obj['center'])[a], (lower[a]+upper[a])/2, 'measured XYZ AABB center')
                aabbs += 1
            require(used == other, 'object/noise indices do not partition all non-ground XYZ')
            require(type(objects['candidate_work']) is int and 0 <= objects['candidate_work'] <= oc['max_candidate_work'],
                    'XYZ clustering candidate work exceeds frozen bound')
        else:
            require(objects['status'] == 'rejected' and isinstance(objects.get('reason'), str) and objects['reason'],
                    'XYZ object failure has no reason')
        accuracy_samples.append((row['metrics'], confident))
    check_counts(report['terrain_aggregate']['confusion'], total, 'micro aggregate')
    check_metrics(report['terrain_aggregate']['metrics'], total, 'micro aggregate')
    require(set(report['terrain_per_site']) == set(sites), 'per-site identities differ')
    for site, counts in sites.items():
        check_counts(report['terrain_per_site'][site]['confusion'], counts, 'site '+site)
        check_metrics(report['terrain_per_site'][site]['metrics'], counts, 'site '+site)
    gate = inputs.freeze.get('accuracy_gate') if inputs.freeze else None
    accuracy_passed = None
    if gate:
        accuracy_passed = all((not gate['confidence_required'] or confident)
            and all(values[key] is not None and values[key] >= gate[key+'_min'] for key in ('precision', 'recall', 'f1'))
            for values, confident in accuracy_samples)
    return {'integrity_passed': True, 'dataset': report['dataset'], 'split': report['split'],
        'samples': len(rows), 'sites': len(sites), 'scored_points': scored, 'scored_ground_points': positives,
        'scored_non_ground_points': negatives, 'excluded_scoring_points': exclusions,
        'independently_checked_aabbs': aabbs, 'micro_metrics': metrics(total),
        'accuracy_gate': gate, 'accuracy_gate_passed': accuracy_passed,
        'negative_reference_controls_absent': negatives == 0,
        'scope': 'Integrity only. Frozen accuracy gates remain separate; no scored negatives means no false-positive/specificity validation. Support confidence is geometric, not calibrated correctness.'}


def mutations(report, inputs):
    def remove_index(value):
        row = value['terrain_samples'][0]
        row['ground_indices' if row['ground_indices'] else 'non_ground_indices'].pop()
    def false_exclusion(value):
        row = value['terrain_samples'][0]
        if row['excluded_scoring_indices']:
            row['excluded_scoring_indices'].pop()
        else:
            row['excluded_scoring_indices'].append(0)
    operations = {'incomplete_partition': remove_index,
        'false_metrics': lambda r: r['terrain_samples'][0]['metrics'].__setitem__('f1', -1.),
        'wrong_unit': lambda r: r['terrain_samples'][0].__setitem__('unit_to_m', 1.234),
        'wrong_raw_hash': lambda r: r['terrain_samples'][0].__setitem__('input_sha256', '0'*64),
        'wrong_scoring_exclusion': false_exclusion,
        'altered_frozen_parameters': lambda r: r['frozen_parameters']['terrain_config'].__setitem__('max_slope', 9.)}
    if report['split'] == 'fresh_heldout':
        operations['altered_external_freeze'] = lambda r: r['freeze'].__setitem__('algorithm_source_sha256', '0'*64)
    rejected = []
    for name, change in operations.items():
        changed = copy.deepcopy(report)
        change(changed)
        try:
            check_report(changed, inputs)
        except ValueError:
            rejected.append(name)
        else:
            raise ValueError('oracle accepted mutation: '+name)
    return rejected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', type=Path, required=True)
    parser.add_argument('--repository', type=Path, default=ROOT)
    parser.add_argument('--data-root', type=Path, help='defaults to REPOSITORY/data')
    parser.add_argument('--freeze', type=Path, help='external freeze; required for fresh_heldout')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--require-accuracy', action='store_true', help='also fail if frozen accuracy gates do not pass')
    args = parser.parse_args()
    parser_checks = parser_self_test()
    report_raw = bounded(args.report, MAX_REPORT_BYTES)
    report = json.loads(report_raw)
    inputs = Inputs(report, args.repository, args.data_root or args.repository/'data', args.freeze)
    result = check_report(report, inputs)
    result['mutations_rejected'] = mutations(report, inputs)
    require(bounded(args.report, MAX_REPORT_BYTES) == report_raw, 'report changed during independent checks')
    result['report_sha256'] = digest(report_raw)
    result['las_parser_regressions'] = parser_checks
    result['checker_sha256'] = digest(bounded(Path(__file__)))
    result['parser_dependency_sha256'] = digest(bounded(ROOT/'scripts/check-datasets.py'))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(result, allow_nan=False))
    return 1 if args.require_accuracy and result['accuracy_gate_passed'] is not True else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, IndexError, TypeError, struct.error) as error:
        print('adaptive ground oracle: '+str(error), file=sys.stderr)
        sys.exit(2)
