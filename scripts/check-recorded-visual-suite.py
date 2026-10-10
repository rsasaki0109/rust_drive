#!/usr/bin/env python3
"""Reproduce recorded visual motion without turning viewed repeats into holdouts.

Exit zero verifies source/pixel/state integrity and unchanged known outcomes.
Physical accuracy/availability acceptance remains explicit in every report.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / 'assets/recorded-visual'
DATASETS = ('tum-fr1-desk-visual', 'tum-fr3-office-visual', 'tum-fr3-sitting-visual')


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
    command = [sys.executable, str(ROOT / 'scripts/check-recorded-visual.py'),
               '--report', str(report), '--manifest', str(manifest),
               '--raw', str(manifest.parent / 'raw'), '--freeze', str(freeze),
               '--output', str(output)]
    if snapshot is not None:
        command += ['--source-snapshot', str(snapshot)]
    subprocess.run(command, cwd=ROOT, check=True)
    result = json.loads(output.read_text())
    if not result['passed_integrity'] or not result['source_freeze_verified']:
        raise ValueError('independent visual source/pixel/geometry integrity failed')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location(
        'rgbd_extension', ROOT / 'scripts/check-rgbd-extension-sources.py')
    extension = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(extension)
    preserved = extension.verify_extension()
    binary, output = args.binary.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    first = BASELINE / 'first-sitting-v1'
    historical = audit(first / 'results.json', ROOT / 'data/tum-fr3-sitting-visual/manifest.json',
                       first / 'freeze.json', output / 'first-sitting-v1-audit.json',
                       ROOT / 'integrations/rgbd/baselines/visual-sitting-first-v1/sources')
    records = []
    for dataset in DATASETS:
        baseline = BASELINE / 'current-regression' / dataset
        folder = output / dataset
        folder.mkdir(exist_ok=True)
        manifest = ROOT / 'data' / dataset / 'manifest.json'
        freeze = folder / 'freeze.json'
        if not freeze.exists():
            subprocess.run([str(binary), '--visual', '--regression', '--manifest', str(manifest),
                            '--prepare-freeze', str(freeze)], cwd=ROOT, check=True)
        original = json.loads((baseline / 'freeze.json').read_text())
        current = json.loads(freeze.read_text())
        if current != original or current['kind'] != 'calibration_regression':
            raise ValueError(dataset + ': viewed source/configuration freeze changed')
        original_report = json.loads((baseline / 'results.json').read_text())
        report_path = folder / 'results.json'
        expected = 0 if original_report['summary']['all_updates_passed'] else 1
        with (folder / 'run.log').open('w') as stream:
            run = subprocess.run([str(binary), '--visual', '--regression',
                                  '--manifest', str(manifest), '--raw', str(manifest.parent / 'raw'),
                                  '--freeze', str(freeze), '--output', str(report_path)],
                                 cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        if run.returncode != expected:
            raise ValueError(f'{dataset}: expected physical status {expected}, got {run.returncode}')
        report = json.loads(report_path.read_text())
        if numerical(report) != numerical(original_report):
            raise ValueError(dataset + ': non-timing operational evidence changed')
        proof_path = folder / 'audit.json'
        proof = audit(report_path, manifest, freeze, proof_path)
        records.append(dict(dataset=dataset, role='viewed numerical regression',
                            summary=report['summary'], evaluator_exit_status=run.returncode,
                            independent_integrity_passed=True,
                            report_sha256=digest(report_path), audit_sha256=digest(proof_path),
                            freeze_sha256=digest(freeze)))
    result = dict(schema_version=1, regression_integrity_passed=True,
                  all_physical_protocols_passed=all(
                      item['summary']['all_updates_passed'] for item in records),
                  first_trial_summary=historical['summary'],
                  original_report_sha256=digest(first / 'results.json'),
                  original_freeze_sha256=digest(first / 'freeze.json'),
                  historical_audit_sha256=digest(output / 'first-sitting-v1-audit.json'),
                  historical_algorithms=preserved, cases=records,
                  scope='Offline indoor RGB-D measurements; no new holdout on repeat, '
                        'no vehicle calibration, SLAM or driving controls')
    (output / 'visual-regression-suite.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
