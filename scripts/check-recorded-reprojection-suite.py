#!/usr/bin/env python3
"""Reproduce viewed RGB-D reprojection baselines, retaining every rejected update.

Exit zero means independent integrity and numerical reproduction. Failed
physical protocols keep exit 1 and invalid labels keep exit 2; repeats never
become fresh holdout evidence.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent
BASELINE = ROOT / 'assets/recorded-reprojection/current-regression'
DATASETS = ('tum-fr1-desk-visual', 'tum-fr3-office-visual', 'tum-fr3-sitting-visual',
            'tum-fr2-desk-reprojection')


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def numerical(value):
    if isinstance(value, dict):
        return {key: numerical(item) for key, item in value.items()
                if key not in ('freeze', 'freeze_sha256', 'cpu_wall_seconds')}
    if isinstance(value, list):
        return [numerical(item) for item in value]
    return value


def replay_original_input_failure(first, output):
    """Rebuild archived v2 sources without editing the active checkout."""
    spec = importlib.util.spec_from_file_location(
        'original_source_guard', ROOT / 'scripts/check-rgbd-extension-sources.py')
    guard = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(guard)
    guard.verify_extension()
    archive = ROOT / 'integrations/rgbd/baselines/reprojection-fr2-desk-first-v1'
    identity = json.loads((archive / 'SOURCE.json').read_text())
    original_freeze = json.loads((first / 'freeze.json').read_text())
    with tempfile.TemporaryDirectory(prefix='first-v2-source-', dir=output) as directory:
        source = Path(directory)
        for name in ('Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml'):
            shutil.copyfile(ROOT / name, source / name)
        for name in ('core', 'localization', 'perception'):
            shutil.copytree(ROOT / 'crates' / name, source / 'crates' / name)
        shutil.copytree(ROOT / 'scripts', source / 'scripts',
                        ignore=shutil.ignore_patterns('__pycache__'))
        integration = source / 'integrations/rgbd'
        integration.mkdir(parents=True)
        shutil.copytree(ROOT / 'integrations/rgbd/src', integration / 'src')
        for name in ('Cargo.toml', 'Cargo.lock'):
            shutil.copyfile(ROOT / 'integrations/rgbd' / name, integration / name)
        for item in identity['source_records']:
            archived = archive / item['archived_path']
            if digest(archived) != item['sha256'] or original_freeze[item['freeze_field']] != item['sha256']:
                raise ValueError('changed original archived source: ' + item['original_path'])
            destination = source / item['original_path']
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(archived, destination)
        target = output / 'first-v2-build-target'
        with (output / 'first-v2-build.log').open('w') as stream:
            subprocess.run(['cargo', 'build', '--release', '--locked', '--offline',
                            '--manifest-path', str(integration / 'Cargo.toml'),
                            '--target-dir', str(target)], cwd=ROOT, check=True,
                           stdout=stream, stderr=subprocess.STDOUT)
        binary = target / 'release/rustdriving-rgbd-evaluate'
        frozen = output / 'first-v2-replayed-freeze.json'
        manifest = ROOT / 'data/tum-fr2-desk-reprojection/manifest.json'
        subprocess.run([str(binary), '--visual-reprojection', '--manifest', str(manifest),
                        '--prepare-freeze', str(frozen)], cwd=ROOT, check=True)
        if digest(frozen) != digest(first / 'freeze.json'):
            raise ValueError('recompiled original source/configuration freeze changed')
        log, report = output / 'first-v2-replayed-error.log', output / 'first-v2-replayed-results.json'
        if report.exists():
            raise ValueError('original source replay must not reuse an existing results file')
        with log.open('w') as stream:
            run = subprocess.run([str(binary), '--visual-reprojection', '--manifest', str(manifest),
                                  '--raw', str(manifest.parent / 'raw'), '--freeze', str(frozen),
                                  '--output', str(report)], cwd=ROOT,
                                 stdout=stream, stderr=subprocess.STDOUT)
        if run.returncode != 2 or report.exists() or log.read_bytes() != (first / 'run.log').read_bytes():
            raise ValueError('original invalid-label exit/log/no-report outcome changed')
        proof = dict(source_rebuilt_replay=True, exact_original_freeze_reproduced=True,
                     evaluator_exit_status=2, original_error_log_identical=True,
                     original_no_report_outcome_reproduced=True, binary_sha256=digest(binary),
                     first_binary_sha256=identity['binary_sha256'],
                     binary_identical_to_first_executable=digest(binary) == identity['binary_sha256'],
                     accuracy_claim=False)
        (output / 'first-v2-source-replay.json').write_text(json.dumps(proof, indent=2)+'\n')
        return proof


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    binary, output = args.binary.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    first = ROOT / 'assets/recorded-reprojection/first-fr2-desk-v1'
    first_audit = output / 'first-fr2-desk-v1-audit.json'
    first_command = [sys.executable, str(ROOT / 'scripts/check-recorded-reprojection.py'),
                     '--manifest', str(ROOT / 'data/tum-fr2-desk-reprojection/manifest.json'),
                     '--raw', str(ROOT / 'data/tum-fr2-desk-reprojection/raw'),
                     '--freeze', str(first / 'freeze.json'),
                     '--source-snapshot', str(ROOT / 'integrations/rgbd/baselines/reprojection-fr2-desk-first-v1/sources'),
                     '--output', str(first_audit)]
    phase1 = json.loads((first / 'phase1.json').read_text())
    if phase1['evaluator_exit_status'] == 2:
        if (first / 'results.json').exists():
            raise ValueError('original failed-input trial had no operational report')
        if digest(first / 'run.log') != phase1['run.log_sha256']:
            raise ValueError('original failed-input log changed')
        first_command += ['--audit-input-failure', '--expected-error-log', str(first / 'run.log')]
    else:
        first_command += ['--report', str(first / 'results.json')]
    subprocess.run(first_command, cwd=ROOT, check=True)
    historical = json.loads(first_audit.read_text())
    if not historical['passed_integrity'] or not historical['source_freeze_verified']:
        raise ValueError('immutable first FR2 trial independent audit failed')
    source_replay = (replay_original_input_failure(first, output)
                     if phase1['evaluator_exit_status'] == 2 else None)
    records = []
    for dataset in DATASETS:
        baseline, folder = BASELINE / dataset, output / dataset
        folder.mkdir(exist_ok=True)
        manifest = ROOT / 'data' / dataset / 'manifest.json'
        freeze, report = folder / 'freeze.json', folder / 'results.json'
        if not freeze.exists():
            subprocess.run([str(binary), '--visual-reprojection', '--regression',
                            '--manifest', str(manifest), '--prepare-freeze', str(freeze)],
                           cwd=ROOT, check=True)
        original = json.loads((baseline / 'freeze.json').read_text())
        current = json.loads(freeze.read_text())
        if current != original or current['kind'] != 'calibration_regression':
            raise ValueError(dataset + ': source/configuration freeze changed')
        original_report = json.loads((baseline / 'results.json').read_text())
        with (folder / 'run.log').open('w') as stream:
            run = subprocess.run([str(binary), '--visual-reprojection', '--regression',
                                  '--manifest', str(manifest), '--raw', str(manifest.parent / 'raw'),
                                  '--freeze', str(freeze), '--output', str(report)],
                                 cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
        expected = (2 if original_report.get('evaluation_label_failure') else
                    0 if original_report['summary']['all_updates_passed'] else 1)
        if run.returncode != expected:
            raise ValueError(f'{dataset}: physical exit {run.returncode}, expected {expected}')
        actual = json.loads(report.read_text())
        if numerical(actual) != numerical(original_report):
            raise ValueError(dataset + ': non-timing operational evidence changed')
        proof_path = folder / 'audit.json'
        subprocess.run([sys.executable, str(ROOT / 'scripts/check-recorded-reprojection.py'),
                        '--report', str(report), '--manifest', str(manifest),
                        '--raw', str(manifest.parent / 'raw'), '--freeze', str(freeze),
                        '--output', str(proof_path)], cwd=ROOT, check=True)
        proof = json.loads(proof_path.read_text())
        if not proof['passed_integrity'] or not proof['source_freeze_verified']:
            raise ValueError(dataset + ': independent integrity failed')
        records.append(dict(dataset=dataset, role='viewed numerical regression',
                            summary=actual['summary'], evaluator_exit_status=run.returncode,
                            evaluation_label_failure=actual.get('evaluation_label_failure'),
                            independent_integrity_passed=True, report_sha256=digest(report),
                            freeze_sha256=digest(freeze), audit_sha256=digest(proof_path)))
    result = dict(schema_version=1, regression_integrity_passed=True,
                  all_physical_protocols_passed=all(item['summary']['all_updates_passed']
                                                   for item in records),
                  cases=records, first_trial_summary=historical.get('summary'),
                  first_trial_evaluator_exit_status=phase1['evaluator_exit_status'],
                  first_trial_input_failure=historical.get('input_failure'),
                  first_trial_evaluation_label_failure=historical.get('evaluation_label_failure'),
                  original_source_replay=source_replay,
                  original_report_sha256=(digest(first / 'results.json')
                                          if (first / 'results.json').exists() else None),
                  original_error_log_sha256=digest(first / 'run.log'),
                  original_freeze_sha256=digest(first / 'freeze.json'),
                  historical_audit_sha256=digest(first_audit),
                  scope='Viewed short indoor camera odometry only; '
                  'no fresh generalization, calibrated uncertainty or driving fusion')
    (output / 'reprojection-regression-suite.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))


if __name__ == '__main__':
    main()
