#!/usr/bin/env python3
"""Verify that visual-mode wiring preserves every pre-existing RGB-D algorithm.

The only lockfile change permitted is adding the existing local perception
crate and its evaluator dependency. Registry versions/checksums stay exact.
"""
import copy
import hashlib
from pathlib import Path
import tomllib

ROOT = Path(__file__).resolve().parent.parent
ARCHIVE = ROOT / 'integrations/rgbd/baselines/submap-office-first-v8/sources'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_extension():
    for name in ('crates/localization/src/registration3d.rs',
                 'crates/localization/src/submap3d.rs',
                 'integrations/rgbd/src/submaps.rs',
                 'integrations/rgbd/src/motion.rs',
                 'crates/core/src/lib.rs', 'Cargo.lock', 'rust-toolchain.toml'):
        if (ROOT / name).read_bytes() != (ARCHIVE / name).read_bytes():
            raise ValueError('visual extension changed historical implementation: ' + name)
    name = 'crates/localization/src/lib.rs'
    current = (ROOT / name).read_text()
    if current.count('pub mod visual_odometry3d;\n') != 1:
        raise ValueError('unsupported visual localization module wiring')
    if current.replace('pub mod visual_odometry3d;\n', '') != (ARCHIVE / name).read_text():
        raise ValueError('historical localization library changed')
    current = (ROOT / 'crates/perception/src/lib.rs').read_bytes()
    export = b'pub mod image_features;\n'
    original_sha = '508da6c305c1c551235c774ed797258e1b00bb2d51b8cfcee6ace8231ab3e329'
    if current.count(export) != 1 or hashlib.sha256(current.replace(export, b'')).hexdigest() != original_sha:
        raise ValueError('historical perception library changed')
    name = 'integrations/rgbd/src/main.rs'
    current = (ROOT / name).read_text()
    entry = 'if std::env::args().any(|a| a == "--visual") {\n        visual::run()\n    } else '
    if current.count('mod visual;\n') != 1 or current.count(entry) != 1:
        raise ValueError('unsupported visual evaluator wiring')
    normalized = current.replace('mod visual;\n', '').replace(entry, '')
    if normalized != (ARCHIVE / name).read_text():
        raise ValueError('historical evaluator implementation changed')
    name = 'integrations/rgbd/Cargo.toml'
    original = tomllib.loads((ARCHIVE / name).read_text())
    wanted = copy.deepcopy(original)
    wanted['dependencies']['rustdriving-perception'] = {'path': '../../crates/perception'}
    if tomllib.loads((ROOT / name).read_text()) != wanted:
        raise ValueError('unsupported RGB-D Cargo manifest change')
    name = 'integrations/rgbd/Cargo.lock'
    original = tomllib.loads((ARCHIVE / name).read_text())
    wanted = copy.deepcopy(original)
    evaluator = next(p for p in wanted['package'] if p['name'] == 'rustdriving-rgbd-evaluate')
    evaluator['dependencies'] = sorted(evaluator['dependencies'] + ['rustdriving-perception'])
    wanted['package'].append({'name': 'rustdriving-perception', 'version': '0.1.0',
                              'dependencies': ['rustdriving-core']})
    wanted['package'].sort(key=lambda p: (p['name'], p['version']))
    current = tomllib.loads((ROOT / name).read_text())
    current['package'].sort(key=lambda p: (p['name'], p['version']))
    if current != wanted:
        raise ValueError('registry pin, checksum or unrelated lock entry changed')
    return {'historical_algorithm_bytes_unchanged': True,
            'registry_versions_and_checksums_unchanged': True,
            'allowed_local_dependency': 'rustdriving-perception 0.1.0',
            'original_optional_lock_sha256': digest(ARCHIVE / name),
            'current_optional_lock_sha256': digest(ROOT / name)}


if __name__ == '__main__':
    import json
    print(json.dumps(verify_extension()))
