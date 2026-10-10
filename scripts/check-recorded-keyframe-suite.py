#!/usr/bin/env python3
"""Reproduce keyframe integrity, including immutable and live failed protocols.

Exit 0 verifies known outcomes and independent audits; it is not an accuracy
pass. The original temporal trial remains failed and all later uses are viewed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / 'assets/recorded-keyframes'
CASES = [
    ('fast', 'tum-fr1-xyz-fast', (11, 11, 11, 0, 3)),
    ('tight', 'tum-fr1-xyz-tight', (11, 11, 11, 0, 3)),
    ('motion-v2', 'tum-fr1-xyz-motion-v2', (11, 11, 11, 0, 3)),
    ('gap-loss', 'tum-fr1-xyz', (0, 0, 11, 11, 0)),
    ('missing-reference', 'tum-fr1-xyz-motion', (10, 7, 8, 1, 3)),
    ('temporal-v1', 'tum-fr1-xyz-keyframes', (33, 13, 35, 2, 10)),
]


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def without_timings(value):
    if isinstance(value, dict):
        return {k: without_timings(v) for k, v in value.items()
                if k != 'cpu_wall_seconds'}
    if isinstance(value, list):
        return [without_timings(v) for v in value]
    return value


def audit(report, manifest, raw, freeze, output, snapshot=None):
    command = [sys.executable, str(ROOT / 'scripts/check-recorded-keyframes.py'),
               '--report', str(report), '--manifest', str(manifest),
               '--raw', str(raw), '--freeze', str(freeze), '--output', str(output)]
    if snapshot is not None:
        command += ['--source-snapshot', str(snapshot)]
    subprocess.run(command, check=True, cwd=ROOT)
    return json.loads(output.read_text())


def current_freeze(binary, manifest, baseline, target):
    """Only executable-mode additions may change the viewed protocol identity.

    Matcher, keyframe algorithm, decoder contract, selection and every numeric
    gate remain fixed. Non-timing operational evidence is compared below.
    """
    if not target.exists():
        subprocess.run([str(binary), '--keyframes', '--manifest', str(manifest),
                        '--prepare-freeze', str(target)], check=True, cwd=ROOT)
    before = json.loads(baseline.read_text())
    after = json.loads(target.read_text())
    allowed = {'evaluator_source_sha256'}
    if set(before) != set(after) or any(before[k] != after[k] for k in before if k not in allowed):
        raise ValueError('viewed keyframe algorithm or numerical freeze changed')
    if after['kind'] != 'calibration_regression':
        raise ValueError('viewed keyframes mislabeled fresh')
    if before['evaluator_source_sha256'] != after['evaluator_source_sha256']:
        archive = ROOT / 'integrations/rgbd/baselines/public-keyframes-v6/sources/integrations/rgbd/src/main.rs'
        if digest(archive) != before['evaluator_source_sha256']:
            raise ValueError('original public executable source not preserved')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    binary = args.binary.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    first = BASELINE / 'first-temporal-v1'
    fresh_manifest = ROOT / 'data/tum-fr1-xyz-keyframes/manifest.json'
    historical = audit(first / 'results.json', fresh_manifest,
                       fresh_manifest.parent / 'raw', first / 'freeze.json',
                       output / 'first-temporal-v1-strengthened-audit.json',
                       ROOT / 'integrations/rgbd/baselines/keyframes-first-v1/sources')
    if historical['summary']['all_updates_passed'] is not False:
        raise ValueError('original failed temporal protocol concealed')
    records = []
    for name, dataset, expected in CASES:
        baseline = BASELINE / 'current-regression' / name
        manifest = ROOT / 'data' / dataset / 'manifest.json'
        folder = output / name
        folder.mkdir(exist_ok=True)
        freeze_path = folder / 'freeze.json'
        current_freeze(binary, manifest, baseline / 'freeze.json', freeze_path)
        report_path = folder / 'results.json'
        with (folder / 'run.log').open('w') as stream:
            run = subprocess.run([str(binary), '--keyframes', '--manifest', str(manifest),
                                  '--raw', str(manifest.parent / 'raw'),
                                  '--freeze', str(freeze_path),
                                  '--output', str(report_path)], cwd=ROOT,
                                 stdout=stream, stderr=subprocess.STDOUT)
        expected_rc = 0 if name in ('fast', 'tight', 'motion-v2') else 1
        if run.returncode != expected_rc:
            raise ValueError(f'{name}: expected evaluator status {expected_rc}, got {run.returncode}')
        report = json.loads(report_path.read_text())
        if report['freeze']['kind'] != 'calibration_regression':
            raise ValueError('viewed observations misrepresented as fresh')
        summary = report['summary']
        actual = tuple(summary[key] for key in ('accepted_updates', 'accurate_root_updates',
                       'reference_valid_updates', 'rejected_updates', 'keyframe_replacements'))
        if summary['initialized_frames'] != 1 or actual != expected:
            raise ValueError(f'{name}: changed known outcome {actual}, expected {expected}')
        numerical = {k: v for k, v in report.items() if k not in ('freeze', 'freeze_sha256')}
        original = {k: v for k, v in json.loads((baseline / 'results.json').read_text()).items()
                    if k not in ('freeze', 'freeze_sha256')}
        if without_timings(numerical) != without_timings(original):
            raise ValueError(f'{name}: deterministic non-timing evidence changed')
        proof = audit(report_path, manifest, manifest.parent / 'raw',
                      freeze_path, folder / 'audit.json')
        records.append(dict(case=name, dataset=dataset, summary=summary,
                            evaluator_exit_status=run.returncode,
                            independent_integrity_passed=proof['passed_integrity'],
                            mutations_rejected=len(proof['mutations_rejected']),
                            report_sha256=digest(report_path), audit_sha256=digest(folder / 'audit.json'),
                            freeze_sha256=digest(freeze_path)))
    result = dict(schema_version=1, regression_integrity_passed=True,
                  all_physical_protocols_passed=False,
                  original_temporal_accuracy_protocol_passed=False,
                  original_report_sha256=digest(first / 'results.json'),
                  original_freeze_sha256=digest(first / 'freeze.json'),
                  strengthened_historical_audit_sha256=digest(output / 'first-temporal-v1-strengthened-audit.json'),
                  cases=records,
                  scope='Viewed same-room measured-depth regressions; no automotive, global SLAM or accumulated-confidence claim.')
    (output / 'suite.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, allow_nan=False))


if __name__ == '__main__':
    main()
