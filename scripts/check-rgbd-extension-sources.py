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
    if current.count('pub mod reprojection3d;\n') != 1:
        raise ValueError('unsupported reprojection localization module wiring')
    normalized = current.replace('pub mod visual_odometry3d;\n', '').replace('pub mod reprojection3d;\n', '')
    if normalized != (ARCHIVE / name).read_text():
        raise ValueError('historical localization library changed')
    current = (ROOT / 'crates/perception/src/lib.rs').read_bytes()
    export = b'pub mod image_features;\n'
    original_sha = '508da6c305c1c551235c774ed797258e1b00bb2d51b8cfcee6ace8231ab3e329'
    if current.count(export) != 1 or hashlib.sha256(current.replace(export, b'')).hexdigest() != original_sha:
        raise ValueError('historical perception library changed')
    name = 'integrations/rgbd/src/main.rs'
    current = (ROOT / name).read_text()
    entry = 'if std::env::args().any(|a| a == "--visual") {\n        visual::run()\n    } else '
    refinement_entry = 'if std::env::args().any(|a| a == "--visual-reprojection") {\n        reprojection::run()\n    } else '
    if current.count('mod reprojection;\n') != 1 or current.count(refinement_entry) != 1:
        raise ValueError('unsupported reprojection evaluator wiring')
    current = current.replace('mod reprojection;\n', '').replace(refinement_entry, '')
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


def compare_visual_freeze(original, current):
    """Only two additive wiring identities may change from visual v1.

    All actual v1 feature/fit/evaluator/checker bytes and policies stay exact.
    The archived first trial retains its original library and entry-point bytes.
    """
    archive = ROOT / 'integrations/rgbd/baselines/visual-sitting-first-v1/sources'
    verify_extension()
    wiring = {
        'localization_lib_source_sha256': ('crates/localization/src/lib.rs',
                                          'pub mod reprojection3d;\n'),
        'evaluator_source_sha256': ('integrations/rgbd/src/main.rs', None),
    }
    expected = copy.deepcopy(original)
    for field, (name, export) in wiring.items():
        old = (archive / name).read_bytes()
        now = (ROOT / name).read_bytes()
        if hashlib.sha256(old).hexdigest() != original[field]:
            raise ValueError('original visual wiring archive changed: ' + name)
        if export:
            if now.count(export.encode()) != 1:
                raise ValueError('duplicate or absent reprojection export')
            normalized = now.replace(export.encode(), b'')
        else:
            entry = b'if std::env::args().any(|a| a == "--visual-reprojection") {\n        reprojection::run()\n    } else '
            if now.count(b'mod reprojection;\n') != 1 or now.count(entry) != 1:
                raise ValueError('duplicate or absent reprojection entry')
            normalized = now.replace(b'mod reprojection;\n', b'').replace(entry, b'')
        if normalized != old:
            raise ValueError('visual v1 operative wiring changed beyond additive refinement: ' + name)
        expected[field] = hashlib.sha256(now).hexdigest()
    if current != expected:
        raise ValueError('visual v1 source, acquisition or numerical policy changed')
    return {'only_additive_module_wiring_changed': True,
            'all_original_visual_sources_and_policies_preserved': True}


if __name__ == '__main__':
    import json
    print(json.dumps(verify_extension()))
