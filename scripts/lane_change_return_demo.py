#!/usr/bin/env python3
"""Retain three reference examples of optional early lane-change return."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def invoke(binary, arguments, stdout):
    with stdout.open('wb') as output, stdout.with_suffix('.stderr').open('wb') as errors:
        status = subprocess.run([str(binary), *map(str, arguments)], cwd=ROOT, stdout=output, stderr=errors).returncode
    if status:
        raise RuntimeError(f'command exited {status}; retained {stdout} and {stdout.with_suffix(".stderr")}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/lane-change-return-demo')
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--binary', type=Path, default=ROOT / 'target/release/rustdriving')
    args = parser.parse_args()
    if not 0 <= args.seed < 2**64:
        parser.error('seed must be in 0..2^64-1')
    binary = args.binary.resolve()
    if not binary.is_file():
        parser.error('build first: cargo build --release --locked --bin rustdriving')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for case, activation, source_blocked in [('early', 20., False), ('source-blocked', 20., True), ('late', 24., False)]:
        folder = output / case
        folder.mkdir()
        scenario = folder / 'scenario.json'
        invoke(binary, ['lane-route', '--map', ROOT / 'maps/lanes/two-lane-change.json', '--start', 'source',
                        '--goal', 'target-exit', '--change-side', 'left', '--change-start', '25',
                        '--change-length', '30', '--wait-for-gap', '--return-on-conflict',
                        '--scenario-output', scenario], folder / 'plan.json')
        authored = json.loads(scenario.read_text())
        actors = [{'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': activation,
                   'world_motion': {'position': {'x': 52., 'y': 3.5}, 'velocity': {'x': 0., 'y': 0.}}}]
        if source_blocked:
            actors.append({'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': activation,
                           'world_motion': {'position': {'x': 40., 'y': 0.}, 'velocity': {'x': 0., 'y': 0.}}})
        authored.update(name=f'Early lane-change return {case}', duration=70., expected='stop', min_clearance_m=1., objects=actors)
        scenario.write_text(json.dumps(authored, indent=2) + '\n')
        invoke(binary, ['run', '--scenario', scenario, '--seed', args.seed, '--output', folder / 'run'], folder / 'run.stdout')
        invoke(binary, ['replay', '--log', folder / 'run/sensors.jsonl', '--output', folder / 'replay'], folder / 'replay.stdout')
        summary = json.loads((folder / 'run/summary.json').read_text())
        run = json.loads((folder / 'run/run.json').read_text())
        final = run['frames'][-1]
        returned = any(frame['lane_change']['phase'] == 'Aborting' for frame in run['frames'])
        if returned != (case == 'early') or (case == 'early' and final['lane_change']['phase'] != 'Aborted'):
            raise RuntimeError(f'{case}: expected return outcome failed; retained complete run and replay in {folder}')
        rows.append({'case': case, 'seed': args.seed, 'passed': summary['passed'],
                     'min_clearance_m': summary['min_clearance'], 'final_phase': final['lane_change']['phase'],
                     'final_position': final['truth']['pose']['position']})
    (output / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
