#!/usr/bin/env python3
"""Record early resume holds for rear traffic, persistent blockage and sensing recovery."""
import argparse
import json
from pathlib import Path
from lane_change_return_demo import ROOT, invoke

CASES = {'rear': (30., 6.), 'rear-slow': (35., 4.), 'launch-blocked': (54., 0.), 'launch-recovery': (30., 6.)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/lane-change-launch-demo')
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
    for case, (x, speed) in CASES.items():
        folder = output / case
        folder.mkdir()
        scenario = folder / 'scenario.json'
        invoke(binary, ['lane-route', '--map', ROOT / 'maps/lanes/two-lane-change.json',
                        '--start', 'source', '--goal', 'target-exit', '--change-side', 'left',
                        '--change-start', '25', '--change-length', '30', '--wait-for-gap',
                        '--return-on-conflict', '--resume-after-return', '--scenario-output', scenario], folder / 'plan.json')
        authored = json.loads(scenario.read_text())
        actors = [{'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': 20., 'moving_from': 40.,
                   'world_motion': {'position': {'x': 52., 'y': 3.5}, 'velocity': {'x': 4., 'y': 0.}}},
                  {'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': 49.,
                   'world_motion': {'position': {'x': x, 'y': 3.5}, 'velocity': {'x': speed, 'y': 0.}}}]
        authored.update(name=f'Early resume hold {case}', duration=120., min_clearance_m=1.,
                        expected='stop' if case == 'launch-blocked' else 'goal', objects=actors)
        if case == 'launch-recovery':
            authored['sensor_timing'] = {'lidar_period_ticks': 2, 'lidar_delay_ticks': 0,
                                         'lidar_failure_windows': [{'from': 50., 'until': 51.}]}
        scenario.write_text(json.dumps(authored, indent=2) + '\n')
        invoke(binary, ['run', '--scenario', scenario, '--seed', args.seed, '--output', folder / 'run'], folder / 'run.stdout')
        invoke(binary, ['replay', '--log', folder / 'run/sensors.jsonl', '--output', folder / 'replay'], folder / 'replay.stdout')
        run = json.loads((folder / 'run/run.json').read_text())
        holds = [f['lane_change']['return_state']['resume_state'].get('launch_hold') for f in run['frames']]
        holds = [h for h in holds if h]
        if not holds or not any(h['stopped_at'] is not None for h in holds):
            raise RuntimeError(f'{case}: required measured launch stop missing; evidence retained in {folder}')
        final = holds[-1]
        if (final['released_at'] is None) != (case == 'launch-blocked'):
            raise RuntimeError(f'{case}: wrong release behavior; evidence retained in {folder}')
        rows.append({'case': case, 'seed': args.seed, 'passed': run['summary']['passed'],
                     'min_clearance_m': run['summary']['min_clearance'], 'initiated_at': final['initiated_at'],
                     'stopped_at': final['stopped_at'], 'released_at': final['released_at'],
                     'reached_goal': run['summary']['reached_goal']})
    (output / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
