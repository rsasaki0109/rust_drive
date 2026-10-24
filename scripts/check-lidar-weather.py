#!/usr/bin/env python3
"""Independently audit frozen LiDAR weather rays and score unchanged detections."""
import argparse
import gzip
import hashlib
import itertools
import json
import math
import os
from pathlib import Path
import struct

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np

NAMES = ['clear', 'rain_light', 'rain_heavy', 'snow_light', 'snow_heavy']
MASK = (1 << 64) - 1
SOURCES = ['Cargo.lock', 'crates/sim/Cargo.toml', 'crates/sim/src/lib.rs',
           'crates/sim/src/lidar_weather.rs', 'crates/sim/src/lidar_weather/tests.rs',
           'crates/sim/examples/lidar_weather_study.rs', 'crates/perception/src/lib.rs',
           'crates/perception/src/clustering.rs', 'crates/perception/src/planar_sweep.rs',
           'crates/core/src/lib.rs', 'crates/core/src/planar_sweep.rs', 'Cargo.toml',
           'rust-toolchain.toml', 'scripts/check-lidar-weather.py']
ROOT = Path(__file__).resolve().parents[1]


def draw(seed, stamp, beam, channel):
    bits = struct.unpack('>Q', struct.pack('>d', stamp))[0] if stamp != 0 else 0
    rotated = ((bits << 17) | (bits >> 47)) & MASK
    z = seed ^ rotated ^ ((beam * 0x9e3779b97f4a7c15) & MASK) ^ ((channel * 0xd1b54a32d192ed03) & MASK)
    z = (z + 0x9e3779b97f4a7c15) & MASK
    z = ((z ^ (z >> 30)) * 0xbf58476d1ce4e5b9) & MASK
    z = ((z ^ (z >> 27)) * 0x94d049bb133111eb) & MASK
    z ^= z >> 31
    return (z >> 11) / (1 << 53)


def close(a, b, tolerance=1e-10):
    assert math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tolerance, (a, b)


def length(p):
    return math.hypot(p['x'], p['y'])


def return_index(ray):
    return ray['Return'] if isinstance(ray, dict) else None


def audit_baseline(trial):
    scan = trial['baseline']
    sweep = scan['sweep']
    c = sweep['calibration']
    assert c['ray_count'] == 720 and len(sweep['rays']) == 720
    assert scan['stamp'] == 1 and c['max_range_m'] == 45
    owned = []
    for beam, ray in enumerate(sweep['rays']):
        theta = c['first_angle_rad'] + c['angular_step_rad'] * beam
        dx, dy = math.cos(theta), math.sin(theta)
        # Cross-product distance to each circle is independent of the sensor's
        # projection/discriminant implementation.
        expected = 45.
        for obj in trial['objects']:
            x, y, radius = obj['position']['x'], obj['position']['y'], obj['radius']
            cross, along = x * dy - y * dx, x * dx + y * dy
            if abs(cross) <= radius:
                near = along - math.sqrt(max(0., radius * radius - cross * cross))
                if .2 < near < expected:
                    expected = near
        index = return_index(ray)
        if expected == 45:
            assert ray == 'Clear'
        else:
            assert index is not None
            point = scan['points'][index]
            owned.append(index)
            assert abs(length(point) - expected) <= .015000001
            close(point['x'] * dy - point['y'] * dx, 0)
    assert sorted(owned) == list(range(len(scan['points'])))


def audit_case(trial, case, cfg):
    base = trial['baseline']
    c = base['sweep']['calibration']
    w, scan = case['weather'], case['weather']['scan']
    assert scan['stamp'] == base['stamp']
    assert len(w['rays']) == len(w['effects']) == 720
    assert len(w['sources']) == len(scan['points'])
    if case['condition'] == 'clear':
        assert scan == base
    else:
        assert scan.get('sweep') is None, 'uncalibrated weather cannot provide clear-sweep evidence'
    seen, retained, errors = [], set(), []
    for beam, (ray, effect) in enumerate(zip(w['rays'], w['effects'])):
        source_index = return_index(base['sweep']['rays'][beam])
        distance = None if source_index is None else length(base['points'][source_index])
        if distance is None:
            assert effect['input_range_m'] is None
        else:
            close(effect['input_range_m'], distance)
        loss = draw(trial['weather_seed'], base['stamp'], beam, 0)
        jitter = 2 * draw(trial['weather_seed'], base['stamp'], beam, 2) - 1
        close(loss, effect['loss_uniform'])
        close(jitter, effect['jitter_unit'])
        particle = None
        if cfg['backscatter_rate_m_inv'] > 0:
            particle = c['min_range_m'] - math.log1p(-draw(trial['weather_seed'], base['stamp'], beam, 1)) / cfg['backscatter_rate_m_inv']
            if particle >= 45: particle = None
        if particle is None:
            assert effect['particle_range_m'] is None
        else:
            close(particle, effect['particle_range_m'])
        source = None
        if particle is not None and particle < (distance if distance is not None else 45):
            source = {'kind': 'precipitation', 'beam': beam}
            before = particle
        elif loss >= math.exp(-2 * cfg['extinction_m_inv'] * (distance if distance is not None else 45)):
            assert ray == 'Unavailable'
        elif distance is None:
            assert ray == 'Clear'
        else:
            source = {'kind': 'surface', 'input': source_index}
            before = distance
        if source is None:
            continue
        point_index = return_index(ray)
        assert point_index is not None
        assert w['sources'][point_index] == source
        seen.append(point_index)
        point = scan['points'][point_index]
        expected_range = min(max(before + jitter * cfg['range_jitter_m'], max(min(.2, before), 1e-9)), max(45., before))
        if source['kind'] == 'surface':
            original = base['points'][source_index]
            dx, dy = original['x'] / before, original['y'] / before
            retained.add(source_index)
            errors.append(length(point) - before)
        else:
            theta = c['first_angle_rad'] + c['angular_step_rad'] * beam
            dx, dy = math.cos(theta), math.sin(theta)
        close(point['x'], dx * expected_range)
        close(point['y'], dy * expected_range)
    assert sorted(seen) == list(range(len(scan['points'])))
    return retained, errors


def matches(objects, detections):
    # Exhaustive one-to-one assignment for the three-object frozen fixture.
    edges = [[i for i, d in enumerate(detections)
              if math.hypot(obj['position']['x'] - d['center']['x'], obj['position']['y'] - d['center']['y']) <= 1
              and abs(obj['radius'] - d['radius']) <= .75] for obj in objects]
    def search(index, used):
        if index == len(edges): return 0
        return max([search(index + 1, used)] +
                   [1 + search(index + 1, used | {candidate}) for candidate in edges[index] if candidate not in used])
    return search(0, set())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    raw = args.report.read_bytes()
    report = json.loads(raw)
    assert report['schema_version'] == 1
    conditions = {c['name']: c['config'] for c in report['conditions']}
    assert list(conditions) == NAMES
    assert len(report['trials']) == 32
    assert {(t['fixture'], t['seed']) for t in report['trials']} == set(itertools.product(['targets', 'empty'], range(1, 17)))
    rows = {name: {'condition': name, 'config': conditions[name], 'bins': [[0, 0] for _ in range(3)],
                   'errors': [], 'fixtures': {f: [] for f in ['targets', 'empty']}} for name in NAMES}
    for trial in report['trials']:
        audit_baseline(trial)
        assert [c['condition'] for c in trial['cases']] == NAMES
        sets = {}
        for case in trial['cases']:
            name, row = case['condition'], rows[case['condition']]
            retained, errors = audit_case(trial, case, conditions[name])
            sets[name] = retained
            row['errors'].extend(errors)
            for i, point in enumerate(trial['baseline']['points']):
                b = min(2, int(length(point) // 15))
                row['bins'][b][1] += 1
                row['bins'][b][0] += i in retained
            matched = matches(trial['objects'], case['detections'])
            assert all(math.isfinite(d['radius']) and d['radius'] > 0 and
                       all(math.isfinite(v) for v in d['center'].values()) for d in case['detections'])
            row['fixtures'][trial['fixture']].append({'matched': matched, 'objects': len(trial['objects']),
                'false': len(case['detections']) - matched, 'precipitation': sum(s['kind'] == 'precipitation' for s in case['weather']['sources'])})
        assert sets['rain_heavy'] <= sets['rain_light'] <= sets['clear']
        assert sets['snow_heavy'] <= sets['snow_light'] <= sets['clear']
    summary = {'schemaVersion': 1, 'sourceCommit': os.environ.get('RUSTDRIVING_SOURCE_SHA', 'uncommitted'),
               'scope': report['scope'], 'seeds': 16, 'fixtures': 2, 'cases': 160,
               'matchingGates': {'centerErrorM': 1, 'radiusErrorM': .75},
               'coordinateFrame': 'body x forward / y left metres; frozen ENU ego origin yaw=0',
               'rows': []}
    for name, row in rows.items():
        fixtures = {}
        for fixture, values in row['fixtures'].items():
            total = sum(v['objects'] for v in values)
            matched = sum(v['matched'] for v in values)
            fixtures[fixture] = {'matchedObjects': matched, 'objects': total,
                'recall': matched / total if total else None,
                'falseDetectionsPerScan': float(np.mean([v['false'] for v in values])),
                'falseDetectionsStd': float(np.std([v['false'] for v in values], ddof=1)),
                'precipitationReturnsPerScan': float(np.mean([v['precipitation'] for v in values]))}
        summary['rows'].append({'condition': name, 'config': row['config'],
            'surfaceRetentionByRange': [{'minM': b * 15, 'maxM': (b + 1) * 15, 'retained': kept, 'baseline': total,
                                         'fraction': kept / total if total else None} for b, (kept, total) in enumerate(row['bins'])],
            'addedSurfaceRangeRmseM': math.sqrt(sum(e * e for e in row['errors']) / len(row['errors'])) if row['errors'] else None,
            'fixtures': fixtures})
    assert all(b['fraction'] == 1 for b in summary['rows'][0]['surfaceRetentionByRange'])
    assert summary['rows'][2]['surfaceRetentionByRange'][-1]['fraction'] < 1
    assert summary['rows'][4]['fixtures']['empty']['precipitationReturnsPerScan'] > 0

    args.output.mkdir(parents=True, exist_ok=False)
    fig = plt.figure(figsize=(16, 9), layout='constrained')
    grid = fig.add_gridspec(2, 15, height_ratios=[1.1, 1])
    sample = next(t for t in report['trials'] if t['fixture'] == 'targets' and t['seed'] == 7)
    for i, case in enumerate(sample['cases']):
        ax = fig.add_subplot(grid[0, i * 3:(i + 1) * 3])
        cloud = case['weather']
        for kind, color, label in [('surface', '#1976a3', 'surface return'), ('precipitation', '#e57432', 'precipitation return')]:
            points = [p for p, s in zip(cloud['scan']['points'], cloud['sources']) if s['kind'] == kind]
            ax.scatter([p['x'] for p in points], [p['y'] for p in points], s=6, color=color, label=label)
        for obj in sample['objects']:
            ax.add_patch(Circle((obj['position']['x'], obj['position']['y']), obj['radius'], fill=False, color='#24343d', lw=1.1))
        for d in case['detections']:
            ax.add_patch(Circle((d['center']['x'], d['center']['y']), d['radius'], fill=False, color='#aa3d62', lw=.8, linestyle=':'))
        ax.scatter([0], [0], marker='>', color='black', s=25)
        ax.set(xlim=(-45,45), ylim=(-45,45), title=case['condition'].replace('_',' ').title(), xlabel='x forward (m)')
        ax.set_aspect('equal')
        if i == 0:
            ax.set_ylabel('y left (m)')
            ax.plot([], [], color='#24343d', label='physical target')
            ax.plot([], [], color='#aa3d62', linestyle=':', label='detector proxy')
            ax.legend(fontsize=6, loc='lower left')
        ax.grid(alpha=.16)
    colors = ['#70838b', '#59a6bf', '#1976a3', '#bd91c6', '#92529f']
    ax = fig.add_subplot(grid[1, :5])
    for i, row in enumerate(summary['rows']):
        ax.plot([7.5,22.5,37.5], [b['fraction'] for b in row['surfaceRetentionByRange']],
                'o-', color=colors[i], label=row['condition'].replace('_',' '))
    ax.set(xlabel='Baseline surface range bin (m)', ylabel='Retained surface-return fraction', ylim=(-.03,1.05), xticks=[7.5,22.5,37.5], xticklabels=['0–15','15–30','30–45'])
    ax.legend(fontsize=7); ax.grid(alpha=.2)
    ax = fig.add_subplot(grid[1, 5:10])
    ax.bar(range(5), [r['fixtures']['targets']['recall'] for r in summary['rows']], color=colors)
    ax.set(ylabel='Geometric object-match rate', ylim=(0,1.05), xticks=range(5), xticklabels=[n.replace('_','\n') for n in NAMES])
    ax.tick_params(axis='x', labelsize=8); ax.grid(axis='y',alpha=.2)
    ax = fig.add_subplot(grid[1, 10:])
    for offset, fixture, color in [(-.18,'targets','#1976a3'),(.18,'empty','#e57432')]:
        ax.bar(np.arange(5)+offset, [r['fixtures'][fixture]['falseDetectionsPerScan'] for r in summary['rows']], width=.34,
               yerr=[r['fixtures'][fixture]['falseDetectionsStd'] for r in summary['rows']], color=color, label=fixture, capsize=2)
    ax.set(ylabel='Unmatched detections / scan (mean ± SD)', xticks=range(5), xticklabels=[n.replace('_','\n') for n in NAMES])
    ax.tick_params(axis='x',labelsize=8); ax.legend(fontsize=7); ax.grid(axis='y',alpha=.2)
    fig.suptitle('Planar LiDAR: synthetic rain / snow stress study\n16 paired seeds · frozen scenes · unchanged detector · uncalibrated weather coefficients', fontsize=14)
    fig.savefig(args.output / 'overview.png', dpi=130)
    plt.close(fig)
    with (args.output / 'report.json.gz').open('wb') as file:
        with gzip.GzipFile(filename='', mode='wb', fileobj=file, mtime=0) as zipped: zipped.write(raw)
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    sha = lambda path: hashlib.sha256(Path(path).read_bytes()).hexdigest()
    provenance = {'sourceCommit': summary['sourceCommit'], 'sourceSha256': {p: sha(ROOT / p) for p in SOURCES},
                  'rawReportSha256': hashlib.sha256(raw).hexdigest(),
                  'outputsSha256': {p: sha(args.output / p) for p in ['report.json.gz','summary.json','overview.png']},
                  'matplotlib': matplotlib.__version__, 'numpy': np.__version__, 'cases': 160,
                  'model': 'synthetic two-way extinction + effective Poisson particle returns + bounded range jitter',
                  'qualification': 'first-return frozen 2D fixtures only; not calibrated rain/snow or real-weather validation'}
    (args.output / 'provenance.json').write_text(json.dumps(provenance, indent=2) + '\n')
    print(json.dumps(summary, indent=2))
    print('WEATHER AUDIT PASSED: 160 cases / 115200 rays; physical first returns, draws, ownership, noise, no false clear proof, paired inclusion and one-to-one geometric scoring')


if __name__ == '__main__':
    main()
