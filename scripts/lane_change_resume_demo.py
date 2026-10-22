#!/usr/bin/env python3
"""Retain resume, persistent-blockage and newly-blocked mission examples."""
import argparse
import json
from pathlib import Path
from lane_change_return_demo import ROOT, invoke


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts/lane-change-resume-demo')
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
    for case in ['release', 'blocked', 'reblocked']:
        folder = output / case
        folder.mkdir()
        scenario = folder / 'scenario.json'
        invoke(binary, ['lane-route', '--map', ROOT / 'maps/lanes/two-lane-change.json',
                        '--start', 'source', '--goal', 'target-exit', '--change-side', 'left',
                        '--change-start', '25', '--change-length', '30', '--wait-for-gap',
                        '--return-on-conflict', '--resume-after-return', '--scenario-output', scenario], folder / 'plan.json')
        authored = json.loads(scenario.read_text())
        actors = [{'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': 20.,
                   'moving_from': 40., 'world_motion': {'position': {'x': 52., 'y': 3.5},
                   'velocity': {'x': 0. if case == 'blocked' else 4., 'y': 0.}}}]
        if case == 'reblocked':
            actors.append({'s': 0., 'lateral': 0., 'radius': 1.1, 'active_from': 54.,
                           'world_motion': {'position': {'x': 66., 'y': 3.5}, 'velocity': {'x': 0., 'y': 0.}}})
        authored.update(name=f'Lane-change mission retry {case}', duration=120.,
                        expected='goal' if case == 'release' else 'stop', min_clearance_m=1., objects=actors)
        scenario.write_text(json.dumps(authored, indent=2) + '\n')
        invoke(binary, ['run', '--scenario', scenario, '--seed', args.seed, '--output', folder / 'run'], folder / 'run.stdout')
        invoke(binary, ['replay', '--log', folder / 'run/sensors.jsonl', '--output', folder / 'replay'], folder / 'replay.stdout')
        run = json.loads((folder / 'run/run.json').read_text())
        frames = run['frames']
        stopped = [f for f in frames if f['lane_change']['phase'] == 'Aborted']
        resumed = [f for f in frames if f['lane_change']['phase'] == 'Resuming']
        if not stopped or bool(resumed) != (case != 'blocked'):
            raise RuntimeError(f'{case}: required return/hold/retry missing; complete evidence retained in {folder}')
        if case == 'release' and (not run['summary']['reached_goal'] or frames[-1]['lane_change']['phase'] != 'Completed'):
            raise RuntimeError(f'{case}: did not complete the original mission')
        rows.append({'case': case, 'seed': args.seed, 'passed': run['summary']['passed'],
                     'reached_goal': run['summary']['reached_goal'], 'min_clearance_m': run['summary']['min_clearance'],
                     'resume_started_at': resumed[0]['time'] if resumed else None,
                     'final_phase': frames[-1]['lane_change']['phase'], 'final_position': frames[-1]['truth']['pose']['position']})
    (output / 'summary.json').write_text(json.dumps(rows, indent=2) + '\n')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
