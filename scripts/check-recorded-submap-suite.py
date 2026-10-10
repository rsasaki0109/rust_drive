#!/usr/bin/env python3
"""Reproduce bounded measured fusion, including immutable failed first trials.

Successful exit verifies integrity and deterministic known outcomes, not that
all physical accuracy protocols pass. Every current evaluation is viewed.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / 'assets/recorded-submaps'
DATASETS = ('tum-fr1-xyz', 'tum-fr1-xyz-fast', 'tum-fr1-xyz-tight',
            'tum-fr1-xyz-motion', 'tum-fr1-xyz-motion-v2', 'tum-fr1-xyz-keyframes',
            'tum-fr1-desk-submaps', 'tum-fr3-office-submaps')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numerical(value):
    if isinstance(value, dict):
        return {key: numerical(item) for key, item in value.items()
                if key not in ('freeze', 'freeze_sha256', 'cpu_wall_seconds')}
    if isinstance(value, list):
        return [numerical(item) for item in value]
    return value


def audit(report, manifest, freeze, output, snapshot=None):
    command = [sys.executable, str(ROOT / 'scripts/check-recorded-submaps.py'),
               '--report', str(report), '--manifest', str(manifest),
               '--raw', str(manifest.parent / 'raw'), '--freeze', str(freeze),
               '--output', str(output)]
    if snapshot is not None:
        command += ['--source-snapshot', str(snapshot)]
    subprocess.run(command, cwd=ROOT, check=True)
    result = json.loads(output.read_text())
    if not result['passed_integrity'] or not result['source_freeze_verified']:
        raise ValueError('independent source/geometry integrity failed')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    binary, output = args.binary.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    historical = []
    for name, dataset, snapshot in (
            ('first-desk-v7', 'tum-fr1-desk-submaps',
             ROOT / 'integrations/rgbd/baselines/submap-desk-first-v7/sources'),
            ('first-office-v8', 'tum-fr3-office-submaps',
             ROOT / 'integrations/rgbd/baselines/submap-office-first-v8/sources')):
        original = BASELINE / name
        manifest = ROOT / 'data' / dataset / 'manifest.json'
        proof_path = output / (name + '-independent-audit.json')
        proof = audit(original / 'results.json', manifest, original / 'freeze.json',
                      proof_path, snapshot)
        historical.append(dict(case=name, summary=proof['summary'],
                               role='immutable original protocol; no new holdout',
                               original_report_sha256=digest(original / 'results.json'),
                               original_freeze_sha256=digest(original / 'freeze.json'),
                               current_audit_sha256=digest(proof_path)))
    if historical[0]['summary']['all_updates_passed']:
        raise ValueError('original failed desk protocol concealed')
    records = []
    for dataset in DATASETS:
        baseline = BASELINE / 'current-regression' / dataset
        folder = output / dataset
        folder.mkdir(exist_ok=True)
        manifest = ROOT / 'data' / dataset / 'manifest.json'
        freeze = folder / 'freeze.json'
        if not freeze.exists():
            subprocess.run([str(binary), '--submaps', '--regression',
                            '--manifest', str(manifest), '--prepare-freeze', str(freeze)],
                           cwd=ROOT, check=True)
        current = json.loads(freeze.read_text())
        if current != json.loads((baseline / 'freeze.json').read_text()):
            raise ValueError(dataset + ': current source/configuration freeze differs')
        if current['kind'] != 'calibration_regression':
            raise ValueError('viewed data misrepresented as a fresh trial')
        report = folder / 'results.json'
        with (folder / 'run.log').open('w') as stream:
            run = subprocess.run([str(binary), '--submaps', '--regression',
                                  '--manifest', str(manifest),
                                  '--raw', str(manifest.parent / 'raw'),
                                  '--freeze', str(freeze), '--output', str(report)],
                                 cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        actual = json.loads(report.read_text())
        original = json.loads((baseline / 'results.json').read_text())
        expected_exit = 0 if original['summary']['all_updates_passed'] else 1
        if run.returncode != expected_exit or numerical(actual) != numerical(original):
            raise ValueError(dataset + ': deterministic measured outcome changed')
        proof_path = folder / 'audit.json'
        proof = audit(report, manifest, freeze, proof_path)
        records.append(dict(dataset=dataset, role='viewed regression',
                            summary=actual['summary'], evaluator_exit_status=run.returncode,
                            independent_integrity_passed=proof['passed_integrity'],
                            mutations_rejected=len(proof['mutations_rejected']),
                            source_mutations_rejected=len(proof['frozen_provenance_mutations_rejected']),
                            report_sha256=digest(report), freeze_sha256=digest(freeze),
                            audit_sha256=digest(proof_path)))
    result = dict(schema_version=1, regression_integrity_passed=True,
                  all_physical_protocols_passed=all(row['evaluator_exit_status'] == 0
                                                   for row in records),
                  original_trials=historical, cases=records,
                  scope='Offline measured indoor cameras; no vehicle fusion, calibrated map '
                        'confidence, automotive accuracy or new holdout on repeated inputs.')
    (output / 'suite.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, allow_nan=False))


if __name__ == '__main__':
    main()
