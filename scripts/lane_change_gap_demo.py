#!/usr/bin/env python3
"""Run three authored observed-gap examples with the ordinary CLI and sensor replay."""
import argparse
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def invoke(binary, arguments, stdout):
    with stdout.open('wb') as output, stdout.with_suffix('.stderr').open('wb') as errors:
        result = subprocess.run([str(binary), *map(str, arguments)], stdout=output, stderr=errors, cwd=ROOT)
    if result.returncode:
        raise RuntimeError(f'command exited {result.returncode}; retained {stdout} and {stdout.with_suffix(".stderr")}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/lane-change-gap-demo')
    parser.add_argument('--seed', type=int, default=7)
    arguments = parser.parse_args()
    if not 0 <= arguments.seed < 2**64:
        parser.error('seed must be in 0..2^64-1')
    binary = ROOT / 'target/release/rustdriving'
    if not binary.is_file():
        parser.error('build first: cargo build --release --locked --bin rustdriving')
    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    rows = []
    for case, x, vx, moving_from in [('rear', -40., 3., 0.), ('release', 28., 4., 15.), ('blocked', 28., 0., 0.)]:
        folder = output / case
        folder.mkdir()
        scenario = folder / 'scenario.json'
        invoke(binary, ['lane-route', '--map', ROOT / 'maps/lanes/two-lane-change.json',
                        '--start', 'source', '--goal', 'target-exit', '--change-side', 'left',
                        '--change-start', '25', '--change-length', '30', '--wait-for-gap',
                        '--scenario-output', scenario], folder / 'plan.json')
        authored = json.loads(scenario.read_text())
        authored.update(name=f'Observed lane-change gap {case}', duration=35. if case == 'blocked' else 120.,
                        expected='stop' if case == 'blocked' else 'goal', min_clearance_m=1.,
                        objects=[{'s': 0., 'lateral': 0., 'radius': 1.1, 'moving_from': moving_from,
                                  'world_motion': {'position': {'x': x, 'y': 3.5}, 'velocity': {'x': vx, 'y': 0.}}}])
        scenario.write_text(json.dumps(authored, indent=2) + '\n')
        invoke(binary, ['run', '--scenario', scenario, '--seed', arguments.seed, '--output', folder / 'run'], folder / 'run.stdout')
        invoke(binary, ['replay', '--log', folder / 'run/sensors.jsonl', '--output', folder / 'replay'], folder / 'replay.stdout')
        summary = json.loads((folder / 'run/summary.json').read_text())
        replay = json.loads((folder / 'replay/replay.json').read_text())
        rows.append({'case': case, 'seed': arguments.seed, 'passed': summary['passed'],
                     'reached_goal': summary['reached_goal'], 'collisions': summary['collisions'],
                     'road_violations': summary['road_violations'], 'min_clearance_m': summary['min_clearance'],
                     'replay_verified': replay['verified'], 'replay_ticks': replay['ticks']})
    (output / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
