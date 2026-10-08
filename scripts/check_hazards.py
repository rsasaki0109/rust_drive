#!/usr/bin/env python3
"""Run real hazard scenarios, verify sensor recomputation, and retain acceptance evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
CASES = {
    'reference': ['occluded-crossing', 'cut-in', 'multiple-blocked', 'opposing-crossings'],
    'rne-dynamic': ['occluded-crossing', 'cut-in', 'low-friction', 'low-friction-stop', 'multiple-blocked', 'opposing-crossings'],
}


def invoke(args):
    result = subprocess.run([str(a) for a in args], cwd=ROOT, capture_output=True, text=True)
    return result.returncode, result.stderr[-2000:]


def source_fingerprint():
    files = [ROOT/'Cargo.toml', ROOT/'Cargo.lock', ROOT/'rust-toolchain.toml']
    files += list((ROOT/'crates').glob('*/Cargo.toml')) + list((ROOT/'crates').glob('**/*.rs'))
    files += list((ROOT/'scenarios').glob('*.json'))
    files += [ROOT/'integrations/rne'/p for p in ['Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml', 'rne-revision.txt']]
    files += list((ROOT/'integrations/rne/src').glob('*.rs'))
    digest = hashlib.sha256()
    for file in sorted(set(files)):
        digest.update(str(file.relative_to(ROOT)).encode()+b'\0'+file.read_bytes()+b'\0')
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['all', *CASES], default='all')
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 7, 42])
    parser.add_argument('--output', type=Path, default=ROOT/'artifacts/hazards')
    args = parser.parse_args()
    if any(seed < 0 or seed > 2**64-1 for seed in args.seeds) or len(set(args.seeds)) != len(args.seeds):
        parser.error('seeds must be distinct unsigned 64-bit integers')
    args.output.mkdir(parents=True, exist_ok=True)
    report_file = args.output/'report.json'
    report_file.unlink(missing_ok=True)
    backends = list(CASES) if args.backend == 'all' else [args.backend]
    cli = ROOT/'target/release/rustdrive'
    rne = ROOT/'integrations/rne/target/release/rustdrive-rne'
    if not cli.is_file() or ('rne-dynamic' in backends and not rne.is_file()):
        raise SystemExit('Build release binaries first: bash scripts/check-hazards.sh')
    revision = (ROOT/'integrations/rne/rne-revision.txt').read_text().strip()
    if 'rne-dynamic' in backends:
        head = subprocess.run(['git', '-C', str(ROOT.parent/'RobotNativeEngine'), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
        if head != revision:
            raise SystemExit('RNE revision differs from the committed pin; existing checkout is preserved')
    report = {'schema_version': 1, 'source_fingerprint_sha256': source_fingerprint(),
              'rne_revision': revision if 'rne-dynamic' in backends else None,
              'seeds': args.seeds, 'runs': [], 'passed': True}
    for backend in backends:
        for case in CASES[backend]:
            for seed in args.seeds:
                output = args.output/backend/case/f'seed-{seed}'
                output.mkdir(parents=True, exist_ok=True)
                # Remove previous success evidence before invoking a fresh run.
                summary_file = output/'summary.json'
                summary_file.unlink(missing_ok=True)
                replay_file = output/'replay/replay.json'
                replay_file.unlink(missing_ok=True)
                command = [cli, 'run'] if backend == 'reference' else [rne, '--plant', 'dynamic']
                command += ['--scenario', ROOT/'scenarios'/f'{case}.json', '--seed', seed, '--output', output]
                code, error = invoke(command)
                row = {'backend': backend, 'scenario': case, 'seed': seed, 'exit_code': code,
                       'output': str(output), 'passed': False}
                if summary_file.exists():
                    summary = json.loads(summary_file.read_text())
                    row['summary'] = summary
                    code_replay, replay_error = invoke([cli, 'replay', '--log', output/'sensors.jsonl', '--output', output/'replay'])
                    row['replay_exit_code'] = code_replay
                    if replay_file.exists():
                        row['replay'] = json.loads(replay_file.read_text())
                    ok = (code == 0 and summary['passed'] and summary['collisions'] == 0
                          and summary['road_violations'] == 0 and code_replay == 0
                          and row.get('replay', {}).get('verified') is True
                          and row['replay']['ticks'] == summary['steps'])
                    run = json.loads((output/'run.json').read_text())
                    if run['scenario'].get('dynamics'):
                        frames = run['frames']
                        acceleration = max(abs((b['truth']['speed']-a['truth']['speed'])/(b['time']-a['time'])) for a,b in zip(frames, frames[1:]))
                        row['max_measured_abs_longitudinal_acceleration_m_s2'] = acceleration
                        ok &= acceleration <= run['scenario']['dynamics']['friction_coefficient']*9.81+1e-8
                    row['passed'] = bool(ok)
                    if replay_error:
                        row['replay_stderr'] = replay_error
                if error:
                    row['stderr'] = error
                report['runs'].append(row)
                report['passed'] &= row['passed']
                print(f"{backend:11s} {case:20s} seed {seed:3d}: {'PASS' if row['passed'] else 'FAIL'}", flush=True)
    report_file.write_text(json.dumps(report, indent=2)+'\n')
    print(f'{report_file}: {len(report["runs"])} runs; passed={report["passed"]}')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, subprocess.CalledProcessError) as error:
        print(f'hazard check: {error}', file=sys.stderr)
        sys.exit(2)
