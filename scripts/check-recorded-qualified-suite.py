#!/usr/bin/env python3
"""Audit the immutable first trial and reproduce its viewed numerical evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
FIRST = ROOT / 'assets/recorded-qualified/first-room-v1'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numerical(value):
    if isinstance(value, dict):
        return {key: numerical(item) for key, item in value.items()
                if key not in ('freeze', 'freeze_sha256', 'cpu_wall_seconds')}
    if isinstance(value, list):
        return [numerical(item) for item in value]
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest = ROOT / 'data/tum-fr1-room-qualified/manifest.json'
    raw = manifest.parent / 'raw'
    qualification = output / 'qualification.json'
    subprocess.run([sys.executable, str(ROOT / 'scripts/qualify-rgbd-metadata.py'),
                    '--manifest', str(manifest), '--raw', str(raw),
                    '--output', str(qualification)], check=True, cwd=ROOT)
    if digest(qualification) != digest(FIRST / 'qualification.json'):
        raise ValueError('first-trial metadata qualification changed')
    checker = ROOT / 'scripts/check-recorded-qualified.py'
    common = [sys.executable, str(checker), '--manifest', str(manifest),
              '--raw', str(raw), '--qualification', str(qualification)]
    phase = json.loads((FIRST / 'phase1.json').read_text())
    if digest(FIRST / 'results.json') != phase['results_sha256']:
        raise ValueError('immutable first-trial report changed')
    first_audit = output / 'first-audit.json'
    subprocess.run(common + ['--freeze', str(FIRST / 'freeze.json'),
                   '--report', str(FIRST / 'results.json'), '--source-snapshot',
                   str(ROOT / 'integrations/rgbd/baselines/qualified-room-first-v1/sources'),
                   '--output', str(first_audit)], check=True, cwd=ROOT)
    freeze, report = output / 'freeze.json', output / 'results.json'
    binary = str(args.binary.resolve())
    subprocess.run([binary, '--regression', '--manifest', str(manifest),
                    '--qualification', str(qualification), '--prepare-freeze', str(freeze)],
                   check=True, cwd=ROOT)
    first_freeze = json.loads((FIRST / 'freeze.json').read_text())
    current_freeze = json.loads(freeze.read_text())
    expected_freeze = dict(first_freeze, kind='calibration_regression', regression_requested=True)
    if current_freeze != expected_freeze:
        raise ValueError('source/configuration changed since first trial')
    with (output / 'run.log').open('w') as stream:
        run = subprocess.run([binary, '--regression', '--manifest', str(manifest),
                              '--raw', str(raw), '--qualification', str(qualification),
                              '--freeze', str(freeze), '--output', str(report)], cwd=ROOT,
                             stdout=stream, stderr=subprocess.STDOUT)
    if run.returncode != phase['evaluator_exit_status']:
        raise ValueError('physical acceptance status changed')
    original = json.loads((FIRST / 'results.json').read_text())
    actual = json.loads(report.read_text())
    if numerical(actual) != numerical(original):
        raise ValueError('non-timing operational evidence changed')
    audit = output / 'regression-audit.json'
    subprocess.run(common + ['--freeze', str(freeze), '--report', str(report),
                             '--output', str(audit)], check=True, cwd=ROOT)
    proofs = [json.loads(path.read_text()) for path in (first_audit, audit)]
    if not all(proof['passed_integrity'] and proof['source_freeze_verified'] for proof in proofs):
        raise ValueError('independent integrity failed')
    result = dict(schema_version=1, regression_integrity_passed=True,
                  first_evaluator_exit_status=phase['evaluator_exit_status'],
                  regression_evaluator_exit_status=run.returncode,
                  summary=actual['summary'], non_timing_evidence_identical=True,
                  qualification_sha256=digest(qualification),
                  first_report_sha256=digest(FIRST / 'results.json'),
                  regression_report_sha256=digest(report),
                  first_audit_sha256=digest(first_audit), regression_audit_sha256=digest(audit),
                  scope='One short indoor sequence; repeats are viewed regression, not new generalization')
    (output / 'qualified-regression-suite.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
