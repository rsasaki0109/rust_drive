#!/usr/bin/env python3
"""Qualify a fixed temporal extension using timestamps only, before its image reads."""
import argparse
from bisect import bisect_left
import hashlib
import importlib.util
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MAX_BYTES = 4 * 1024 * 1024
MAX_INDEX_ROWS = 20000
MAX_GT_ROWS = 30000
SUMMARY_TIME_UNIT = ('rounded microseconds from source timestamps; '
                     'qualification gates use unrounded seconds')


def digest(content):
    return hashlib.sha256(content).hexdigest()


def timestamp_rows(content, columns, name, maximum):
    """Inspect only column zero and column count; pose values stay opaque."""
    if len(content) > MAX_BYTES:
        raise ValueError(name + ': metadata byte bound exceeded')
    result = []
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
        result.append((stamp, fields[1] if columns == 2 else None))
        if len(result) > maximum:
            raise ValueError(name + ': metadata row bound exceeded')
    if len(result) < 2:
        raise ValueError(name + ': insufficient metadata')
    return result


def bracket_width(times, stamp):
    upper = bisect_left(times, stamp)
    if upper < len(times) and times[upper] == stamp:
        return 0.0
    if upper == 0 or upper == len(times):
        raise ValueError('reference timestamp would require extrapolation')
    width = times[upper] - times[upper - 1]
    if width > 0.02:
        raise ValueError('reference timestamp bracket exceeds 0.02 seconds')
    return width


def nearest_index(times, stamp):
    upper = bisect_left(times, stamp)
    candidates = [i for i in (upper - 1, upper) if 0 <= i < len(times)]
    return min(candidates, key=lambda i: (abs(times[i] - stamp), i))


def qualify_metadata(manifest, read_metadata):
    """Read exactly the three metadata files; never read a PNG or pose column."""
    items = {item['file']: item for item in manifest['files']}
    metadata, rows = {}, {}
    for name, columns, maximum in [('depth.txt', 2, MAX_INDEX_ROWS),
                                   ('rgb.txt', 2, MAX_INDEX_ROWS),
                                   ('groundtruth.txt', 8, MAX_GT_ROWS)]:
        item = items[name]
        content = read_metadata(name)
        blob = b'blob ' + str(len(content)).encode() + b'\0' + content
        if (len(content) != item['bytes'] or digest(content) != item['sha256']
                or hashlib.sha1(blob).hexdigest() != item['git_blob_sha1']):
            raise ValueError(name + ': pinned metadata bytes changed')
        rows[name] = timestamp_rows(content, columns, name, maximum)
        metadata[name] = {key: item[key] for key in ('bytes', 'sha256', 'git_blob_sha1')}
    frames = manifest['frames']
    if len(frames) != 180 or [frame['source_index'] for frame in frames] != list(range(100, 280)):
        raise ValueError('changed fixed original depth window')
    rgb_times = [row[0] for row in rows['rgb.txt']]
    gt_times = [row[0] for row in rows['groundtruth.txt']]
    widths, gaps, rgb_identities = [], [], set()
    for i, frame in enumerate(frames):
        for kind, ordinal in [('depth', frame['source_index']), ('rgb', frame['rgb_source_index'])]:
            if type(ordinal) is not int or not 0 <= ordinal < len(rows[kind + '.txt']):
                raise ValueError('invalid original acquisition ordinal')
            stamp, source_path = rows[kind + '.txt'][ordinal]
            filename = frame[kind + '_file']
            if (stamp != frame[kind + '_timestamp'] or not filename.startswith(kind + '-')
                    or source_path != kind + '/' + filename[len(kind) + 1:]):
                raise ValueError('frame differs from original timestamp/index association')
        if frame['rgb_source_index'] != nearest_index(rgb_times, frame['depth_timestamp']):
            raise ValueError('RGB association is not the original nearest timestamp')
        gap = abs(frame['depth_timestamp'] - frame['rgb_timestamp'])
        if gap > 0.02:
            raise ValueError('RGB/depth gap exceeds 0.02 seconds')
        if frame['split'] != ('initialization' if i == 0 else 'viewed_prefix' if i < 36 else 'unviewed_extension'):
            raise ValueError('changed first-trial frame role')
        gaps.append(gap)
        widths.append(bracket_width(gt_times, frame['depth_timestamp']))
        rgb_identities.add((frame['rgb_source_index'], frame['rgb_timestamp'], frame['rgb_file']))
    rounded_us = lambda value: math.floor(value * 1000000.0 + 0.5)
    return dict(window=dict(first_depth_index=100, last_depth_index=279, frames=180, updates=179),
                metadata_files=metadata,
                timestamps=dict(depth_index_rows=len(rows['depth.txt']),
                                rgb_index_rows=len(rows['rgb.txt']),
                                ground_truth_rows=len(rows['groundtruth.txt']),
                                strict_depth_order=True, strict_rgb_order=True,
                                strict_ground_truth_order=True, all_frame_brackets_valid=True,
                                max_ground_truth_bracket_s=0.02, max_pair_gap_s=0.02,
                                maximum_observed_ground_truth_bracket_us=rounded_us(max(widths)),
                                maximum_pair_gap_us=rounded_us(max(gaps)),
                                unique_rgb_acquisitions=len(rgb_identities),
                                duplicate_rgb_associations=180 - len(rgb_identities),
                                summary_time_unit=SUMMARY_TIME_UNIT))


def qualify(manifest_raw, raw):
    if len(manifest_raw) > 512 * 1024:
        raise ValueError('manifest byte bound exceeded')
    manifest = json.loads(manifest_raw)
    path = ROOT / 'scripts/fetch-temporal-dataset.py'
    spec = importlib.util.spec_from_file_location('pinned_qualified_acquisition', path)
    acquisition = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(acquisition)
    acquisition.verify_manifest(manifest)

    def reader(name):
        path = raw / name
        if path.is_symlink() or path.stat().st_size > MAX_BYTES:
            raise ValueError('unsafe or oversized metadata input')
        return path.read_bytes()

    evidence = qualify_metadata(manifest, reader)
    return dict(schema_version=1, dataset=manifest['dataset'],
                manifest_sha256=digest(manifest_raw), source_commit=manifest['revision'],
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
    if args.output.exists():
        raise ValueError('qualification evidence exists; use a new output path')
    result = qualify(args.manifest.read_bytes(), args.raw)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
