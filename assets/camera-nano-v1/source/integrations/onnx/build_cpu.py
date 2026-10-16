#!/usr/bin/env python3
"""Build the unchanged parallel detector for this Linux x86-64 CPU, separately.

Uses cached locked dependencies offline. Keep the normal release executable:
this opt-in build has different compiler settings and its own binary receipt.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / 'integrations/onnx/Cargo.toml'
BINARY = 'rustdriving-camera-parallel'
REFERENCE = ROOT / 'assets/camera-parallel-v1/verification/executable.json'
OVERRIDES = {
    'RUSTFLAGS': '-C target-cpu=native',
    'CARGO_PROFILE_RELEASE_OPT_LEVEL': '3',
    'CARGO_PROFILE_RELEASE_LTO': 'thin',
    'CARGO_PROFILE_RELEASE_CODEGEN_UNITS': '1',
    'CARGO_PROFILE_RELEASE_DEBUG': 'false',
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def digest(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def cpu_flags():
    require(platform.system() == 'Linux' and platform.machine() == 'x86_64',
            'this optional build is qualified only for Linux x86-64')
    flags = [set(line.split(':', 1)[1].split())
             for line in Path('/proc/cpuinfo').read_text().splitlines()
             if line.startswith('flags')]
    require(flags and all(flags), 'CPU flags unavailable')
    return sorted(set.intersection(*flags))


def write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def verify(directory):
    receipt = json.loads((directory / 'build.json').read_text())
    require(receipt['schema_version'] == 1 and receipt['compiler_overrides'] == OVERRIDES,
            'expected native build recipe')
    require(isinstance(receipt['linux_cpu_flags'], list) and receipt['linux_cpu_flags']
            and all(isinstance(flag, str) for flag in receipt['linux_cpu_flags']),
            'nonempty recorded CPU features required')
    require(set(receipt['linux_cpu_flags']).issubset(cpu_flags()),
            'current CPU lacks features recorded for the native build')
    require(receipt['executable_sha256'] == digest(directory / 'bin' / BINARY),
            'native executable differs from build receipt')
    require(receipt['build_script_sha256'] == digest(__file__), 'build script differs')
    require(receipt['compiled_sources'] == json.loads(REFERENCE.read_text())['compiled_sources'],
            'complete unchanged parallel-detector source receipt required')
    for name, pin in receipt['compiled_sources'].items():
        require(digest(ROOT / name) == pin, 'compiled source differs: ' + name)
    return receipt


def build(args):
    flags = cpu_flags()
    require(shutil.which('cargo') and shutil.which('rustc'), 'source scripts/env.sh first')
    version = subprocess.check_output(['rustc', '--version', '--verbose'], text=True)
    require(version.startswith('rustc 1.90.0 '), 'use the pinned Rust 1.90.0 toolchain')
    require(not args.output.exists() and not args.output.is_symlink(), 'fresh build output required')
    require(not args.target_dir.is_symlink(), 'target directory must not be a symlink')
    args.output.mkdir(parents=True)
    env = dict(os.environ)
    # Encoded flags override RUSTFLAGS. Remove inherited profile/package
    # overrides and wrappers so the recorded recipe describes this build.
    for key in list(env):
        if key.startswith('CARGO_PROFILE_') or key in (
                'CARGO_ENCODED_RUSTFLAGS', 'RUSTC', 'RUSTC_WRAPPER', 'RUSTC_WORKSPACE_WRAPPER'):
            del env[key]
    env.update(OVERRIDES, CARGO_TARGET_DIR=str(args.target_dir), CARGO_BUILD_JOBS='3')
    command = ['cargo', 'build', '--release', '--locked', '--offline',
               '--manifest-path', str(MANIFEST), '--bin', BINARY]
    write(args.output / 'before-build.json', dict(
        command=command, compiler_overrides=OVERRIDES, rustc=version,
        target_dir=str(args.target_dir), linux_cpu_flags=flags,
        build_script_sha256=digest(__file__),
        cargo_manifest_sha256=digest(MANIFEST), cargo_lock_sha256=digest(MANIFEST.with_name('Cargo.lock'))))
    with (args.output / 'build.log').open('xb') as log:
        subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT, check=True)
    binary_dir = args.output / 'bin'
    binary_dir.mkdir()
    executable = binary_dir / BINARY
    shutil.copy2(args.target_dir / 'release' / BINARY, executable)
    # Never run a moved native binary without checking the recorded CPU flags.
    require(set(flags).issubset(cpu_flags()), 'CPU features changed during build')
    subprocess.run([str(executable), '--executable-receipt', '--output',
                    str(args.output / 'executable.json')], check=True)
    compiled = json.loads((args.output / 'executable.json').read_text())
    require(compiled['compiled_sources'] == json.loads(REFERENCE.read_text())['compiled_sources'],
            'complete unchanged parallel-detector source receipt required')
    for name, pin in compiled['compiled_sources'].items():
        require(digest(ROOT / name) == pin, 'unexpected compiled source: ' + name)
    write(args.output / 'build.json', dict(
        schema_version=1, compiler_overrides=OVERRIDES, command=command, rustc=version,
        linux_cpu_flags=flags, target_dir=str(args.target_dir),
        compiled_sources=compiled['compiled_sources'], executable_sha256=digest(executable),
        build_script_sha256=digest(__file__),
        model_geometry_thresholds_and_rust_sources_changed=False,
        portable_binary=False, numerical_parity_verified_by_build=False))
    verify(args.output)
    print(json.dumps(dict(build_complete=True, executable=str(executable),
                          numerical_parity_verified=False)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['build', 'verify'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--target-dir', type=Path)
    args = parser.parse_args()
    args.output = args.output.absolute()
    args.target_dir = (args.target_dir or args.output / 'target').absolute()
    if args.mode == 'build':
        build(args)
    else:
        result = verify(args.output)
        print(json.dumps(dict(verified=True, executable_sha256=result['executable_sha256'])))
