#!/usr/bin/env python3
"""Reproduce the already-viewed official desk2 failure and independently audit all179 updates.

Exit zero means source/input integrity and unchanged numerical evidence, never a
passing physical motion protocol or fresh independent-scene validation. Each
attempt requires a new output directory and preserves every command's status.
"""
import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
FIRST = ROOT / 'assets/recorded-independent/desk2-v1'
PINS = {
    'manifest.json': 'f9de2bcb1037c46c70242f2c1bdbc8b09bbd06c95716d15b256b3c523ba24327',
    'freeze.json': 'd1d813c0d953a4ae4cd27abbc00dee4c817ee39f4e1ecd854c8457477074f9bf',
    'qualification.json': '783eb5ea1c54c642bea63b01430694bd55ca658c91c4b92e45526040d133f2d9',
    'results.json': '2e621c78e2986539c386ff4a86be47b95774db929f6460373adc5b4549ac4e68',
    'pre-pixel-preservation.json': 'e8a9371e55dadae3d763880015a402d9387c757488e005bf9d37dbf9a216d21d',
    'design.json': '6fa44837d27f6f1f4f69285780ccd5e4883d7b59ace15500699914aca323797a',
    'source-snapshot.tar.gz': '4564b37cd4d02fa603fb10854ffb952cf0602585913ecf90ab81aa1e46e1c3c4',
}
V2_SHA = '8c4e96b63cdd4f2c59907d8a71dbb43be972d4290547ae2495e10214fc34ce70'
ARCHIVE_BYTES = 349445005
ARCHIVE_SHA = 'a569e4cb453a3cd9285bc985fcb109e65f055c75b33a4b155acd9a68d96b77d2'
ARCHIVE_MD5 = '9250a26b897f770a6f9b5f4380020784'
EXPECTED_SUMMARY = dict(accepted_updates=16, accurate_root_updates=16,
    all_updates_passed=False, frames=180, initialized_frames=1, lost=True,
    reference_valid_updates=179, rejected_updates=163, updates=179)


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    require(path.is_file() and not path.is_symlink(), 'unsafe or missing file: '+str(path))
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def bounded_json(path, limit):
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= limit,
            'unsafe or oversized JSON: '+str(path))
    return json.loads(path.read_bytes())


def numerical(value):
    if isinstance(value, dict):
        return {key: numerical(item) for key, item in value.items()
                if key not in ('freeze', 'freeze_sha256', 'cpu_wall_seconds')}
    if isinstance(value, list):
        return [numerical(item) for item in value]
    return value


def run_suite(args):
    require(not args.output.exists() and not args.output.is_symlink(),
            'output already exists or is a symlink; preserve previous attempt and choose a new directory')
    output = args.output.resolve()
    require(not output.exists(), 'output already exists; preserve previous attempt and choose a new directory')
    output.mkdir(parents=True)
    status = dict(schema_version=1, regression_integrity_passed=False,
        evidence_role='viewed regression of the preserved first independent-recording failure',
        fresh_holdout_evidence=False, physical_protocol_passed=False, commands=[])
    result_path = output / 'independent-regression-suite.json'

    def save():
        result_path.write_text(json.dumps(status, indent=2, allow_nan=False)+'\n')

    def invoke(name, command, expected):
        log = output / (name+'.log')
        with log.open('x') as stream:
            run = subprocess.run([str(x) for x in command], cwd=ROOT,
                                 stdout=stream, stderr=subprocess.STDOUT)
        status['commands'].append(dict(stage=name, command=[str(x) for x in command],
            exit_code=run.returncode, expected_exit_code=expected,
            log=str(log.relative_to(output)), log_sha256=digest(log)))
        save()
        require(run.returncode == expected,
                f'{name}: exit {run.returncode}, expected {expected}; retained log {log}')

    save()
    try:
        require(all(digest(FIRST/name) == expected for name, expected in PINS.items()),
                'preserved first-trial assets changed')
        require(digest(args.manifest) == PINS['manifest.json'],
                'current manifest bytes differ from the preserved first official selection')
        first_freeze = bounded_json(FIRST/'freeze.json', 512*1024)
        first_report = bounded_json(FIRST/'results.json', 64*1024*1024)
        preservation = bounded_json(FIRST/'pre-pixel-preservation.json', 512*1024)
        manifest = bounded_json(args.manifest, 512*1024)
        require(first_report['freeze'] == first_freeze
                and first_report['summary'] == EXPECTED_SUMMARY,
                'first failure/configuration no longer matches the retained protocol')
        require(first_freeze['kind'] == 'preregistered_independent_recording'
                and first_freeze['regression_requested'] is False,
                'first-trial role changed')
        require(args.archive.is_file() and not args.archive.is_symlink()
                and args.archive.stat().st_size == ARCHIVE_BYTES,
                'official archive byte count differs from the first receipt')
        with args.archive.open('rb') as stream:
            sha = hashlib.sha256()
            md5 = hashlib.md5(usedforsecurity=False)
            while chunk := stream.read(1024*1024):
                sha.update(chunk)
                md5.update(chunk)
        receipt = dict(bytes=ARCHIVE_BYTES, sha256=sha.hexdigest(), md5=md5.hexdigest())
        require(receipt == dict(bytes=ARCHIVE_BYTES, sha256=ARCHIVE_SHA, md5=ARCHIVE_MD5)
                and all(manifest['official_archive'][key] == value for key, value in receipt.items()),
                'official archive hashes differ from the first exact receipt')
        auditor = ROOT/'scripts/check-recorded-independent-v2.py'
        require(digest(auditor) == V2_SHA, 'additive formal auditor changed')
        spec = importlib.util.spec_from_file_location('independent_suite_auditor', auditor)
        audit_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(audit_module)
        bindings = preservation['code_bindings']
        require(len(bindings) == 21 and set(bindings) == set(audit_module.i.SOURCES),
                'preserved source inventory must contain all21 original bindings')
        snapshot = output/'source-snapshot'
        total = 0
        for field, binding in bindings.items():
            relative = Path(binding['path'])
            require(not relative.is_absolute() and '..' not in relative.parts
                    and binding['path'] == audit_module.i.SOURCES[field], 'unsafe source snapshot path')
            source = ROOT/relative
            require(source.stat().st_size <= 2*1024*1024
                    and digest(source) == binding['sha256'] == first_freeze[field],
                    'current source differs from original frozen bytes: '+str(relative))
            total += source.stat().st_size
            require(total <= 4*1024*1024, 'source snapshot exceeds4MiB resource bound')
            destination = snapshot/relative
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes(source.read_bytes())
        design_path = snapshot/'assets/recorded-independent/desk2-v1/design.json'
        design_path.parent.mkdir(parents=True, exist_ok=True)
        design_path.write_bytes((FIRST/'design.json').read_bytes())
        require(digest(design_path) == first_freeze['preregistration_source_sha256'],
                'preregistration snapshot differs')
        status.update(first_asset_sha256=PINS, archive_receipt=receipt,
            current_manifest_identical_to_first=True, original_code_bindings=bindings,
            frozen_source_count=21, source_snapshot_bytes=total,
            running_checker_sha256=digest(Path(__file__)), formal_auditor_sha256=V2_SHA,
            binary_sha256=digest(args.binary))
        qualification = output/'qualification.json'
        invoke('qualification', [sys.executable, ROOT/'scripts/qualify-rgbd-independent.py',
            '--manifest', args.manifest, '--raw', args.raw, '--output', qualification], 0)
        require(digest(qualification) == PINS['qualification.json'],
                'fresh original metadata qualification differs from first trial')
        freeze, report = output/'freeze.json', output/'results.json'
        invoke('freeze', [args.binary, '--regression', '--manifest', args.manifest,
            '--qualification', qualification, '--prepare-freeze', freeze], 0)
        expected_freeze = copy.deepcopy(first_freeze)
        expected_freeze.update(kind='calibration_regression', regression_requested=True)
        require(bounded_json(freeze, 512*1024) == expected_freeze,
                'viewed freeze differs from original gates/source beyond explicit regression role')
        invoke('evaluation', [args.binary, '--regression', '--manifest', args.manifest,
            '--raw', args.raw, '--qualification', qualification, '--freeze', freeze,
            '--output', report], 1)
        actual = bounded_json(report, 64*1024*1024)
        require(actual['summary'] == EXPECTED_SUMMARY
                and numerical(actual) == numerical(first_report),
                'non-timing continuous evidence changed; preserve and inspect this outcome')
        audit_path = output/'audit-v2.json'
        invoke('audit', [sys.executable, auditor, '--manifest', args.manifest,
            '--raw', args.raw, '--freeze', freeze, '--qualification', qualification,
            '--archive', args.archive, '--source-snapshot', snapshot,
            '--report', report, '--output', audit_path], 0)
        audit = bounded_json(audit_path, 64*1024*1024)
        require(audit['passed_integrity'] and audit['source_freeze_verified']
                and audit['frozen_source_count'] == 21
                and audit['all_179_updates_audited_continuously']
                and len(audit['frames']) == 180 and audit['summary'] == EXPECTED_SUMMARY
                and audit['kind'] == 'calibration_regression'
                and audit['evidence_role'] == 'viewed regression', 'formal all179 continuous audit incomplete')
        require(all(digest(ROOT/binding['path']) == binding['sha256'] for binding in bindings.values())
                and all(digest(FIRST/name) == expected for name, expected in PINS.items()),
                'original code or first evidence changed during reproduction')
        status.update(regression_integrity_passed=True, summary=actual['summary'],
            non_timing_continuous_evidence_identical=True, original_gates_unchanged=True,
            all_179_updates_audited_continuously=True,
            output_sha256={name: digest(output/name) for name in
                ('qualification.json','freeze.json','results.json','audit-v2.json')})
        save()
        print(json.dumps(dict(report=str(result_path), regression_integrity_passed=True,
                              physical_protocol_passed=False, summary=actual['summary'])))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as error:
        status['failure'] = str(error)
        save()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('binary', 'archive', 'raw', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    parser.add_argument('--manifest', type=Path,
                        default=ROOT/'data/tum-fr1-desk2-independent/manifest.json')
    args = parser.parse_args()
    for name in ('binary', 'archive', 'raw', 'manifest'):
        setattr(args, name, getattr(args, name).resolve())
    return run_suite(args)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError) as error:
        print('independent viewed regression: '+str(error), file=sys.stderr)
        sys.exit(2)
