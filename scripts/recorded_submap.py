"""Compare historical submaps after reconstructing their exact fingerprints."""
import hashlib
import math
from pathlib import Path

from recorded_numeric import compare_recorded


def measured_clouds(manifest, raw, preprocessing, oracle):
    clouds = []
    files = {item['file']: item for item in manifest['files']}
    for frame in manifest['frames']:
        name = frame['file']
        if Path(name).name != name or '\\' in name:
            raise ValueError('unsafe measured submap path')
        content = oracle.r.bounded(raw / name)
        item = files[name]
        if len(content) != item['bytes'] or oracle.r.digest(content) != item['sha256']:
            raise ValueError('measured submap source changed: ' + name)
        coarse = {}
        for point in oracle.depth_geometry(content, preprocessing):
            coarse.setdefault(tuple(math.floor(x / .06) for x in point), point)
        clouds.append([coarse[key] for key in sorted(coarse)])
    return clouds


def compare_submap_reports(recorded, reproduced, clouds, oracle):
    """Keep computed hashes exact within each report; compare their geometry.

    The caller also runs the unchanged full source, geometry and scoring audit.
    No input/source/configuration hash is excluded from report comparison.
    """
    def operational(value):
        if isinstance(value, dict):
            return {key: operational(item) for key, item in value.items()
                    if key not in ('freeze', 'freeze_sha256', 'cpu_wall_seconds')}
        if isinstance(value, list):
            return [operational(item) for item in value]
        return value

    search_work = []

    def without_computed_hashes(report):
        value = operational(report)
        work = []
        for row in value['frames']:
            registration = row.get('registration')
            if registration is not None:
                checks = registration.pop('neighbor_checks')
                maximum = report['freeze']['registration_config']['max_neighbor_checks']
                if type(checks) is not int or not 0 < checks <= maximum:
                    raise ValueError('historical submap search exceeded its frozen budget')
                work.append(checks)
            else:
                work.append(None)
            for when in ('before', 'after'):
                for field in ('map_sha256', 'map_statistics_sha256'):
                    row.pop(field + '_' + when)
                    row['state_' + when].pop(field)
        search_work.append(work)
        return value

    comparison = compare_recorded(without_computed_hashes(recorded),
                                  without_computed_hashes(reproduced))
    for field in ('registration_config', 'submap_policy'):
        if recorded['freeze'][field] != reproduced['freeze'][field]:
            raise ValueError('submap reconstruction configuration changed: ' + field)
    if len(clouds) != len(recorded['frames']):
        raise ValueError('submap reconstruction omitted measured clouds')
    maps = [{}, {}]
    differing_hashes = []
    geometry_rounding = 0
    maximum = 0.
    for index, scan in enumerate(clouds):
        hashes = []
        for side, report in enumerate((recorded, reproduced)):
            row = report['frames'][index]
            states = {}
            for when in ('before', 'after'):
                if when == 'after' and row['map_updated']:
                    voxels, rejection = oracle.fusion(
                        maps[side], scan, oracle.exact_pose(row['root_estimate']),
                        report['freeze']['submap_policy'], report['freeze']['registration_config'])
                    if rejection is not None:
                        raise ValueError('historical submap fusion rejected: ' + str(rejection))
                    maps[side] = voxels
                expected = {'map_sha256': oracle.map_digest(maps[side]),
                            'map_statistics_sha256': oracle.statistics_digest(maps[side])}
                for field, digest in expected.items():
                    if row[field + '_' + when] != digest or row['state_' + when][field] != digest:
                        raise ValueError(f'unverified computed submap fingerprint: frame {index}/{when}/{field}')
                states[when] = expected
            hashes.append(states)
        for when in ('before', 'after'):
            if hashes[0][when] != hashes[1][when]:
                differing_hashes.append(dict(frame=index, state=when,
                                              recorded=hashes[0][when], reproduced=hashes[1][when]))
        def geometry(voxels):
            return [dict(key=list(key), count=voxels[key][1], mean=voxels[key][0])
                    for key in sorted(voxels)]
        proof = compare_recorded(geometry(maps[0]), geometry(maps[1]))
        geometry_rounding += proof['rounded_values']
        maximum = max(maximum, proof['maximum_absolute_difference'])
    comparison['exact'] = comparison['exact'] and not differing_hashes
    comparison['rounded_values'] += geometry_rounding
    comparison['maximum_absolute_difference'] = max(comparison['maximum_absolute_difference'], maximum)
    comparison['map_evidence'] = dict(
        exact_map_states_verified=4 * len(clouds), voxel_keys_and_counts_exact=True,
        differing_fingerprints=differing_hashes,
        reconstruction_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        geometry_auditor_sha256=hashlib.sha256(Path(oracle.__file__).read_bytes()).hexdigest())
    work_differences = [dict(frame=i, recorded=a, reproduced=b)
                        for i, (a, b) in enumerate(zip(*search_work)) if a != b]
    comparison['exact'] = comparison['exact'] and not work_differences
    comparison['search_work'] = dict(
        frozen_budget=recorded['freeze']['registration_config']['max_neighbor_checks'],
        both_runs_within_frozen_budget=True, differing_counts=work_differences)
    return comparison
