#!/usr/bin/env python3
"""Freeze antenna-mount trials and audit acquisition geometry and complete EKF history."""
import argparse
import copy
import hashlib
import json
import lzma
import os
from pathlib import Path
import shutil

from check_gnss_covariance import audit as physical, scenario
from check_gnss_latency import angle
from check_gnss_map_history import sha
from check_gnss_recovery import POLICY, verify_ticks
from check_speed_zones import execute

REPO = Path(__file__).resolve().parents[1]
MOUNTS = {'front-left': {'x': 1.2, 'y': .35}, 'rear-right': {'x': -.9, 'y': -.3}}
WORLDS = ['straight', 'curve', 'north-bias', 'diagonal-bias', 'map-early', 'map-late']


def specifications(plants, smoke=False):
    rows = []
    for plant in plants:
        for world in WORLDS:
            for mount in MOUNTS:
                for seed in ([7] if smoke else [1, 7, 42]):
                    rows.append(dict(plant=plant, world=world, mount=mount, seed=seed, mode='calibrated'))
                if world in ['straight', 'curve']:
                    rows.append(dict(plant=plant, world=world, mount=mount, seed=7, mode='control'))
        if not smoke:
            for world in ['curve', 'map-late']:
                for seed in [809, 2027]:
                    rows.append(dict(plant=plant, world=world, mount='front-left', seed=seed, mode='confirm'))
    return rows


def materialize(spec):
    source, value = scenario(spec['world'])
    value['gnss_antenna_mount_m'] = copy.deepcopy(MOUNTS[spec['mount']])
    if spec['mode'] != 'control':
        value['gnss_antenna_offset_m'] = copy.deepcopy(MOUNTS[spec['mount']])
    if spec['world'].endswith('bias'):
        value['gnss_recovery'] = copy.deepcopy(POLICY)
    return source, value


def freeze(output, specs):
    manifest = {'mounts': MOUNTS, 'episodes': []}
    for index, spec in enumerate(specs):
        source, value = materialize(spec)
        path = output/'frozen'/f'{index:03d}.json'
        path.parent.mkdir(parents=True, exist_ok=True)
        content = json.dumps(value, indent=2)+'\n'
        if path.exists():
            if path.read_text() != content:
                raise ValueError('frozen antenna scenario was modified')
        else:
            with path.open('x', newline='\n') as stream:
                stream.write(content)
        manifest['episodes'].append(dict(spec, scenario=str(path.relative_to(output)), sha256=sha(path),
                                        source=source, source_sha256=sha(REPO/source)))
    path = output/'frozen-manifest.json'
    if path.exists():
        if json.loads(path.read_text()) != manifest:
            raise ValueError('frozen antenna cohort changed')
    else:
        with path.open('x', newline='\n') as stream:
            stream.write(json.dumps(manifest, indent=2)+'\n')
    return manifest


def archive(folder):
    """Losslessly retain every raw outcome; XZ bounds storage for full histories."""
    storage = {}
    for path in sorted(folder.rglob('*')):
        if path.suffix not in ['.json', '.jsonl']:
            continue
        digest = sha(path)
        target = Path(str(path)+'.xz')
        with path.open('rb') as source, lzma.open(target, 'xb', preset=3) as output:
            shutil.copyfileobj(source, output)
        with lzma.open(target, 'rb') as stream:
            if hashlib.file_digest(stream, 'sha256').hexdigest() != digest:
                raise ValueError('lossless antenna archive verification failed')
        storage[str(path.relative_to(folder))] = dict(path=str(target.relative_to(folder)), sha256=digest,
                                                     stored_sha256=sha(target), encoding='xz')
        path.unlink()
    return storage


def run_case(output, spec, binary, replay_binary):
    folder = output/'episodes'/spec['plant']/f"{spec['world']}-{spec['mount']}-{spec['mode']}-seed-{spec['seed']}"
    folder.mkdir(parents=True, exist_ok=False)
    source = output/spec['scenario']
    if sha(source) != spec['sha256']:
        raise ValueError('frozen antenna scenario changed')
    frozen = json.loads(source.read_text())
    args = ['run', '--scenario', source] if spec['plant'] == 'reference-2d' else ['--scenario', source, '--plant', 'dynamic']
    row = dict(spec, expectation_met=False, recordings=str(folder.relative_to(output)))
    try:
        row['run_exit_code'] = execute(binary, args+['--seed', spec['seed'], '--output', folder/'run'], folder/'run-process')
        row['replay_exit_code'] = execute(replay_binary, ['replay', '--log', folder/'run/sensors.jsonl', '--output', folder/'replay'], folder/'replay-process')
        run = json.loads((folder/'run/run.json').read_text())
        row['summary'] = run['summary']
        row['replay'] = json.loads((folder/'replay/replay.json').read_text())
        with (folder/'run/sensors.jsonl').open() as stream:
            config = json.loads(next(stream))['header']['config']
            ticks = [record['tick'] for line in stream if 'tick' in (record := json.loads(line))]
        if config.get('gnss_antenna_offset_m') != frozen.get('gnss_antenna_offset_m'):
            raise ValueError('physical mount was inferred as operational calibration')
        if config.get('gnss_recovery') != frozen.get('gnss_recovery'):
            raise ValueError('external recovery calibration changed')
        oracle = verify_ticks(config, ticks)
        row.update(physical(run, config, ticks, dict(spec, calibrated=spec['mode'] != 'control'), frozen, reconstruction=oracle))
        row['maximum_yaw_error_rad'] = max(abs(angle(f['truth']['pose']['yaw']-f['estimate']['pose']['yaw'])) for f in run['frames'])
        if spec['mode'] != 'control' and row['maximum_yaw_error_rad'] > .1:
            row['failed_gates'].append('yaw')
        row['quality_passed'] = row['run_exit_code'] == 0 and not row['failed_gates']
        replayed = row['replay_exit_code'] == 0 and row['replay']['verified'] and row['replay']['ticks'] == len(ticks)
        # Comparison worlds retain every quality/physical failure; they qualify
        # only data reconstruction and replay, not positive driving behavior.
        row['expectation_met'] = replayed and (row['quality_passed'] or spec['mode'] == 'control')
    except Exception as error:
        row['audit_error'] = repr(error)
    finally:
        row['storage'] = archive(folder)
    return row


def comparisons(rows):
    pairs = []
    for control in rows:
        if control['mode'] != 'control':
            continue
        calibrated = next(r for r in rows if r['mode'] == 'calibrated' and all(r[k] == control[k] for k in ['plant', 'world', 'mount', 'seed']))
        reduction = (1.-calibrated['maximum_error_m']/control['maximum_error_m']
                     if 'maximum_error_m' in calibrated and control.get('maximum_error_m') else None)
        pairs.append(dict(plant=control['plant'], world=control['world'], mount=control['mount'], seed=control['seed'],
                          maximum_error_reduction=reduction,
                          passed=bool(calibrated.get('quality_passed') and control['expectation_met'] and reduction is not None and reduction >= .4)))
    return pairs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    output = args.output.resolve()
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:
        print('Frozen antenna episodes:', len(manifest['episodes']))
        return
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    provenance = {p: dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}
    rows = []
    for spec in manifest['episodes']:
        rows.append(run_case(output, spec, binaries[spec['plant']], args.binary.resolve()))
        pairs = comparisons(rows)
        report = dict(passed=all(r['expectation_met'] for r in rows) and all(p['passed'] for p in pairs),
                      complete=len(rows) == len(manifest['episodes']), binaries=provenance,
                      manifest_sha256=sha(output/'frozen-manifest.json'), episodes=rows, paired_controls=pairs)
        (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
        row = rows[-1]
        print(row['plant'], row['world'], row['mount'], row['mode'], row['seed'], row['expectation_met'],
              row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['complete'] and report['passed'] else 1)


if __name__ == '__main__':
    main()
