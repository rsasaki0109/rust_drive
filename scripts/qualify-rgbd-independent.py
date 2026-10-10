#!/usr/bin/env python3
"""Qualify fixed official FR1 desk2 metadata without opening PNGs or pose values."""
import argparse
from bisect import bisect_left
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parent.parent
MAX_BYTES = 4 * 1024 * 1024
MAX_INDEX_ROWS = 20000
MAX_GT_ROWS = 30000
SUMMARY_TIME_UNIT = ('rounded microseconds from source timestamps; '
                     'qualification gates use unrounded seconds')
WINDOW = dict(first_depth_index=100, last_depth_index=279, frames=180, updates=179)


def digest(content):
    return hashlib.sha256(content).hexdigest()


def timestamp_rows(content, columns, name, maximum):
    """Only the timestamp is numeric; seven reference-pose tokens stay opaque."""
    if len(content) > MAX_BYTES:
        raise ValueError(name + ': metadata byte bound exceeded')
    result, paths = [], set()
    for physical_line, line in enumerate(content.decode('utf8').splitlines(), 1):
        fields = line.strip().split()
        if not fields or fields[0].startswith('#'):
            continue
        if len(fields) != columns:
            raise ValueError(f'{name}: invalid arity on line {physical_line}')
        try:
            stamp = float(fields[0])
        except ValueError:
            raise ValueError(f'{name}: invalid timestamp on line {physical_line}') from None
        if not math.isfinite(stamp) or (result and stamp <= result[-1][0]):
            raise ValueError(f'{name}: nonfinite or nonincreasing timestamp on line {physical_line}')
        source_path = None
        if columns == 2:
            source_path = fields[1]
            kind = name.removesuffix('.txt')
            if not re.fullmatch(kind + r'/[0-9]+\.[0-9]+\.png', source_path):
                raise ValueError(name + ': unsafe or unsupported original image path')
            if source_path in paths:
                raise ValueError(name + ': ambiguous repeated source path')
            paths.add(source_path)
        result.append((stamp, source_path))
        if len(result) > maximum:
            raise ValueError(name + ': metadata row bound exceeded')
    if len(result) < 2:
        raise ValueError(name + ': insufficient metadata')
    return result


def nearest_index(times, stamp):
    upper = bisect_left(times, stamp)
    candidates = [i for i in (upper - 1, upper) if 0 <= i < len(times)]
    return min(candidates, key=lambda i: (abs(times[i] - stamp), i))


def bracket_width(times, stamp):
    upper = bisect_left(times, stamp)
    if upper < len(times) and times[upper] == stamp:
        return 0.0
    if upper == 0 or upper == len(times):
        raise ValueError('reference timestamp would require extrapolation')
    width = times[upper] - times[upper - 1]
    if not 0 < width <= 0.02:
        raise ValueError('reference timestamp bracket exceeds 0.02 seconds')
    return width


def select_metadata(tables):
    """Derive the sole fixed window using three complete tables, never pixels."""
    if set(tables) != {'depth.txt', 'rgb.txt', 'groundtruth.txt'}:
        raise ValueError('complete source metadata inventory required')
    if sum(len(content) for content in tables.values()) > MAX_BYTES:
        raise ValueError('aggregate complete metadata exceeds 4 MiB')
    rows = {name: timestamp_rows(tables[name], columns, name, maximum)
            for name, columns, maximum in [('depth.txt', 2, MAX_INDEX_ROWS),
                                           ('rgb.txt', 2, MAX_INDEX_ROWS),
                                           ('groundtruth.txt', 8, MAX_GT_ROWS)]}
    if len(rows['depth.txt']) < 280:
        raise ValueError('fixed original depth window absent; no replacement window')
    rgb_times = [r[0] for r in rows['rgb.txt']]
    gt_times = [r[0] for r in rows['groundtruth.txt']]
    frames, gaps, widths, rgb_identities = [], [], [], set()
    for ordinal in range(100, 280):
        stamp, source_path = rows['depth.txt'][ordinal]
        rgb_ordinal = nearest_index(rgb_times, stamp)
        rgb_stamp, rgb_source_path = rows['rgb.txt'][rgb_ordinal]
        gap = abs(stamp - rgb_stamp)
        if gap > 0.02:
            raise ValueError(f'RGB/depth gap exceeds 0.02 seconds at depth index {ordinal}')
        width = bracket_width(gt_times, stamp)
        frames.append(dict(source_index=ordinal,
                           depth_file='depth-' + Path(source_path).name,
                           depth_timestamp=stamp,
                           rgb_file='rgb-' + Path(rgb_source_path).name,
                           rgb_timestamp=rgb_stamp, rgb_source_index=rgb_ordinal,
                           pair_gap_seconds=gap,
                           split='initialization' if ordinal == 100 else 'independent_recording'))
        gaps.append(gap)
        widths.append(width)
        rgb_identities.add((rgb_ordinal, rgb_stamp, rgb_source_path))
    rounded_us = lambda value: math.floor(value * 1000000.0 + 0.5)
    metadata_files = {name: dict(bytes=len(content), sha256=digest(content))
                      for name, content in tables.items()}
    timestamps = dict(depth_index_rows=len(rows['depth.txt']),
                      rgb_index_rows=len(rows['rgb.txt']),
                      ground_truth_rows=len(rows['groundtruth.txt']),
                      strict_depth_order=True, strict_rgb_order=True,
                      strict_ground_truth_order=True, all_frame_brackets_valid=True,
                      max_ground_truth_bracket_s=0.02, max_pair_gap_s=0.02,
                      maximum_observed_ground_truth_bracket_us=rounded_us(max(widths)),
                      maximum_pair_gap_us=rounded_us(max(gaps)),
                      unique_rgb_acquisitions=len(rgb_identities),
                      duplicate_rgb_associations=180 - len(rgb_identities),
                      summary_time_unit=SUMMARY_TIME_UNIT)
    return dict(frames=frames, window=dict(WINDOW),
                metadata_files=metadata_files, timestamps=timestamps)


def qualify_metadata(manifest, read_metadata):
    items = {item['file']: item for item in manifest['files']}
    tables = {}
    for name in ('depth.txt', 'rgb.txt', 'groundtruth.txt'):
        content = read_metadata(name)
        if len(content) != items[name]['bytes'] or digest(content) != items[name]['sha256']:
            raise ValueError(name + ': pinned metadata bytes changed')
        tables[name] = content
    selected = select_metadata(tables)
    if manifest['frames'] != selected.pop('frames'):
        raise ValueError('manifest differs from fixed original timestamp/window associations')
    return selected


def qualify(manifest_raw, raw):
    if len(manifest_raw) > 512 * 1024:
        raise ValueError('manifest byte bound exceeded')
    manifest = json.loads(manifest_raw)
    path = ROOT / 'scripts/fetch-independent-dataset.py'
    spec = importlib.util.spec_from_file_location('independent_acquisition', path)
    acquisition = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(acquisition)
    acquisition.verify_manifest(manifest)
    expected = {item['file'] for item in manifest['files']} | {'camera-calibration.yaml'}
    if raw.is_symlink() or {path.name for path in raw.iterdir()} != expected:
        raise ValueError('extra or missing actual raw inputs; pinned camera sidecar is allowed')
    if any(path.is_symlink() or not path.is_file() for path in raw.iterdir()):
        raise ValueError('unsafe actual raw input type')

    def reader(name):
        path = raw / name
        if raw.is_symlink() or path.is_symlink() or path.stat().st_size > MAX_BYTES:
            raise ValueError('unsafe or oversized metadata input')
        return path.read_bytes()

    evidence = qualify_metadata(manifest, reader)
    return dict(schema_version=1, dataset=manifest['dataset'],
                manifest_sha256=digest(manifest_raw), official_archive=manifest['official_archive'],
                preregistration_sha256=manifest['preregistration_sha256'],
                helper_sha256=digest(Path(__file__).read_bytes()),
                acquisition_helper_sha256=digest(path.read_bytes()),
                passed=True, pixels_read=False, image_headers_read=False,
                ground_truth_pose_values_parsed=False, features_or_fits_run=False, **evidence)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = qualify(args.manifest.read_bytes(), args.raw)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
