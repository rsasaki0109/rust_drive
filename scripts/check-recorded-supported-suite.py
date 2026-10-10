#!/usr/bin/env python3
"""Reproduce a viewed depth-supported trial without converting failure to success."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / 'assets/recorded-supported/room-v1'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numerical(value):
    if isinstance(value, dict):
        return {key: numerical(item) for key, item in value.items()
                if key not in ('cpu_wall_seconds',)}
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
    manifest = ROOT / 'data/tum-fr1-room-temporal/manifest.json'
    raw = manifest.parent / 'raw'
    qualification = output / 'qualification.json'
    subprocess.run([sys.executable, str(ROOT / 'scripts/qualify-rgbd-temporal.py'),
                    '--manifest', str(manifest), '--raw', str(raw),
                    '--output', str(qualification)], check=True, cwd=ROOT)
    phase = json.loads((BASELINE / 'phase.json').read_text())
    for name in ('qualification', 'freeze', 'results'):
        if digest(BASELINE / (name + '.json')) != phase[name + '_sha256']:
            raise ValueError('preserved viewed trial changed: ' + name)
    if digest(qualification) != phase['qualification_sha256']:
        raise ValueError('metadata qualification changed')
    binary = str(args.binary.resolve())
    freeze, report = output / 'freeze.json', output / 'results.json'
    subprocess.run([binary, '--regression', '--manifest', str(manifest),
                    '--qualification', str(qualification), '--prepare-freeze', str(freeze)],
                   check=True, cwd=ROOT)
    if freeze.read_bytes() != (BASELINE / 'freeze.json').read_bytes():
        raise ValueError('sources or gates changed since viewed trial')
    checker = ROOT / 'scripts/check-recorded-supported.py'
    common = [sys.executable, str(checker), '--manifest', str(manifest),
              '--raw', str(raw), '--qualification', str(qualification), '--freeze', str(freeze)]
    subprocess.run(common + ['--preregister-only', '--output', str(output / 'metadata-audit.json')],
                   check=True, cwd=ROOT)
    with (output / 'run.log').open('w') as stream:
        run = subprocess.run([binary, '--regression', '--manifest', str(manifest),
                              '--raw', str(raw), '--qualification', str(qualification),
                              '--freeze', str(freeze), '--output', str(report)], cwd=ROOT,
                             stdout=stream, stderr=subprocess.STDOUT)
    if run.returncode != phase['evaluator_exit_status'] or run.returncode not in (0, 1):
        raise ValueError('physical outcome changed or inputs failed')
    preserved = json.loads((BASELINE / 'results.json').read_text())
    actual = json.loads(report.read_text())
    if numerical(actual) != numerical(preserved):
        raise ValueError('non-timing evidence changed')
    if len(actual['frames']) != 180 or actual['summary']['updates'] != 179:
        raise ValueError('selected observations omitted')
    audit = output / 'audit.json'
    subprocess.run(common + ['--report', str(report), '--output', str(audit)], check=True, cwd=ROOT)
    proof = json.loads(audit.read_text())
    if not proof['passed_integrity'] or not proof['source_freeze_verified']:
        raise ValueError('independent integrity failed')
    result = dict(schema_version=1, regression_integrity_passed=True,
                  evaluator_exit_status=run.returncode, physical_protocol_passed=run.returncode == 0,
                  summary=actual['summary'], non_timing_evidence_identical=True,
                  freeze_sha256=digest(freeze), qualification_sha256=digest(qualification),
                  preserved_report_sha256=digest(BASELINE / 'results.json'),
                  repeated_report_sha256=digest(report), independent_audit_sha256=digest(audit),
                  scope='Viewed same-room calibration comparison; no new environment or automotive validation')
    (output / 'supported-regression-suite.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
