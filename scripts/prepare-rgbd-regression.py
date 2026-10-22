#!/usr/bin/env python3
"""Stage frozen RGB-D reproductions without changing the active shared crates.

The active RGB-D workspace is tested first in CI. Historical protocols then use
their original shared libraries, with byte-identical active 3D kernels and Vec3.
"""
import argparse
import hashlib
import io
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parent.parent
REVISION = 'a2776053d88bcdcae4e39134c73d95412ff49125'
LIBRARIES = ('core', 'localization', 'perception')
KERNELS = (
    'crates/localization/src/registration3d.rs',
    'crates/localization/src/submap3d.rs',
    'crates/localization/src/keyframes3d.rs',
    'crates/localization/src/visual_odometry3d.rs',
    'crates/localization/src/reprojection3d.rs',
    'crates/perception/src/image_features.rs',
)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def vec3(data):
    start = b'/// Measured XYZ: body forward/left and up above the calibrated road datum.\n'
    end = b'/// One first return from a calibrated column/ring ordinal.\n'
    if data.count(start) != 1 or data.count(end) != 1:
        raise ValueError('missing or ambiguous shared Vec3 source')
    return data[data.index(start):data.index(end)]


def prepare(output):
    output = output.absolute()
    if output.exists() or output.is_symlink():
        raise ValueError('regression workspace must be a new directory')
    archive = subprocess.check_output(
        ['git', '-C', str(ROOT), 'archive', REVISION,
         *(f'crates/{name}' for name in LIBRARIES)])
    with tarfile.open(fileobj=io.BytesIO(archive)) as source:
        kernels = {}
        for name in KERNELS:
            original = source.extractfile(name).read()
            if (ROOT / name).read_bytes() != original:
                raise ValueError('active 3D kernel changed; qualify a new protocol: ' + name)
            kernels[name] = sha(original)
        original_core = source.extractfile('crates/core/src/lib.rs').read()
        common = vec3(original_core)
        if vec3((ROOT / 'crates/core/src/lib.rs').read_bytes()) != common:
            raise ValueError('active shared Vec3 changed; qualify a new protocol')
        output.mkdir(parents=True)
        for name in ('Cargo.toml', 'Cargo.lock', 'rust-toolchain.toml'):
            shutil.copyfile(ROOT / name, output / name)
        ignore = shutil.ignore_patterns('target', '__pycache__')
        for name in ('crates', 'scripts', 'integrations/rgbd'):
            shutil.copytree(ROOT / name, output / name, ignore=ignore)
        source.extractall(output, filter='data')
    for name in ('assets', 'data', 'artifacts', 'maps', 'scenarios'):
        (output / name).symlink_to(ROOT / name, target_is_directory=True)
    bindings = {}
    for name in LIBRARIES:
        path = f'crates/{name}/src/lib.rs'
        bindings[path] = {'active_sha256': sha((ROOT / path).read_bytes()),
                          'historical_sha256': sha((output / path).read_bytes())}
    receipt = dict(historical_revision=REVISION, source_archive_sha256=sha(archive),
                   active_3d_kernels_identical=kernels, shared_vec3_sha256=sha(common),
                   shared_libraries=bindings,
                   scope='Historical offline RGB-D reproduction; active shared crates '
                         'and the active RGB-D workspace are tested separately.')
    (output / 'regression-sources.json').write_text(json.dumps(receipt, indent=2) + '\n')
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    print(prepare(args.output))
