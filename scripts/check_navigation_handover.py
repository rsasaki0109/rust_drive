#!/usr/bin/env python3
"""Qualify braked standstill confirmation of a pending navigation route."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path

from check_gnss_map_history import sha
from check_navigation_relocalization import (
    GATES as PREVIOUS_GATES, REPO, audit as previous_audit,
    materialize as previous_materialize, specifications,
)
from check_relocalization_map_only import freeze as freeze_inputs, run_case

GATES = dict(PREVIOUS_GATES, maximum_reset_to_switch_s=.5,
             maximum_confirmation_speed_m_s=.05,
             minimum_paired_goal_improvement_s=2.)


def materialize(spec):
    source, value = previous_materialize(spec)
    value['navigation']['hold_for_handover'] = True
    return source, value


def freeze(output, specs):
    return freeze_inputs(output, specs, factory=materialize, gates=GATES)


def audit(run, config, ticks, spec, frozen):
    result = previous_audit(run, config, ticks, spec, frozen)
    if config['navigation'].get('hold_for_handover') is not True:
        raise ValueError('braked handover policy missing from operational configuration')
    reset = result['oracle']['reset_stamp']
    events = result['navigation_oracle']['switch_events']
    delay = None
    confirming = []
    if spec['mode'] == 'late-closure' and reset is not None and events:
        switch = events[0]['stamp']
        delay = switch-reset
        if not 0. < delay <= GATES['maximum_reset_to_switch_s']+1e-9:
            result['failed_gates'].append('prompt_post_reset_handover')
        for frame, tick in zip(run['frames'], ticks):
            if reset < frame['time'] < switch and tick['expected']['navigation'].get('handover_hold'):
                confirming.append(frame)
        if len(confirming) < 2:
            result['failed_gates'].append('physical_confirmation_coverage')
        elif max(abs(f['truth']['speed']) for f in confirming) > GATES['maximum_confirmation_speed_m_s']+1e-9:
            result['failed_gates'].append('physical_confirmation_standstill')
    result.update(reset_to_switch_s=delay, physical_confirmation_ticks=len(confirming),
                  maximum_confirmation_speed_m_s=max((abs(f['truth']['speed']) for f in confirming), default=None))
    return result


def goal_improvement(before, after):
    if not before['reached_goal'] or not after['reached_goal']:
        raise ValueError('both paired runs must reach their original destination')
    gain = before['simulated_seconds']-after['simulated_seconds']
    return dict(goal_improvement_s=gain, passed=gain+1e-9 >= GATES['minimum_paired_goal_improvement_s'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plant', choices=['all', 'reference-2d', 'rne-dynamic'], default='all')
    parser.add_argument('--smoke', action='store_true')
    parser.add_argument('--freeze-only', action='store_true')
    parser.add_argument('--binary', type=Path, default=REPO/'target/release'/('rustdriving.exe' if os.name == 'nt' else 'rustdriving'))
    parser.add_argument('--native-binary', type=Path, default=REPO/'integrations/rne/target/release/rustdriving-rne')
    args = parser.parse_args()
    output = args.output.resolve()
    plants = ['reference-2d', 'rne-dynamic'] if args.plant == 'all' else [args.plant]
    manifest = freeze(output, specifications(plants, args.smoke))
    if args.freeze_only:
        print('Frozen braked-handover episodes:', len(manifest['episodes']))
        return
    binaries = {'reference-2d': args.binary.resolve(), 'rne-dynamic': args.native_binary.resolve()}
    provenance = {p: dict(path=str(binaries[p]), sha256=sha(binaries[p])) for p in plants}
    rows = []
    with ThreadPoolExecutor(max_workers=2) as pool:
        for row in pool.map(lambda spec: run_case(output, spec, binaries[spec['plant']], args.binary.resolve(), auditor=audit), manifest['episodes']):
            rows.append(row)
            report = dict(passed=all(x['passed'] for x in rows), complete=len(rows) == len(manifest['episodes']),
                          manifest_sha256=sha(output/'frozen-manifest.json'), binaries=provenance, gates=GATES, episodes=rows)
            (output/'report.json').write_text(json.dumps(report, indent=2)+'\n')
            print(row['plant'], row['world'], row['mode'], row['seed'], row['passed'], row.get('audit_error', row.get('failed_gates')), flush=True)
    raise SystemExit(0 if report['passed'] and report['complete'] else 1)


if __name__ == '__main__':
    main()
