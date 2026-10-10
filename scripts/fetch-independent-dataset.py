#!/usr/bin/env python3
"""Bounded official FR1 desk2 acquisition; opaque tar payloads, never image decoding."""
import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import subprocess
import tarfile
import tempfile
from datetime import datetime, timezone

ROOT = Path(__file__).resolve().parent.parent
DATASET = 'tum-fr1-desk2-independent'
DESIGN_PATH = ROOT / 'assets/recorded-independent/desk2-v1/design.json'
DESIGN_SHA256 = '6fa44837d27f6f1f4f69285780ccd5e4883d7b59ace15500699914aca323797a'
OFFICIAL_URL = 'https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz'
FINAL_URL = 'https://webshare.cvg.cit.tum.de/g/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_desk2.tgz'
ARCHIVE_ROOT = 'rgbd_dataset_freiburg1_desk2/'
DEFAULT_LIMITS = dict(compressed=512 * 1024 * 1024, decompressed=2 * 1024 * 1024 * 1024,
                      members=10000, member=4 * 1024 * 1024, metadata=4 * 1024 * 1024,
                      selected=128 * 1024 * 1024)
PROFILE = dict(width=640, height=480, fx=517.306408, fy=516.469215,
               cx=318.643040, cy=255.313989, units_per_metre=5000, invalid_depth=0)
CALIBRATION = dict(repository='luigifreda/pyslam',
                   revision='96019cfafcfc099ac9866884d7143a9ed1451a0d',
                   source_path='settings/TUM1.yaml', file='camera-calibration.yaml', bytes=1615,
                   sha256='5bd0ec559a251ac402756be7db0bd367bb364fe6a9d85e0770cd0681003602cf',
                   role='source_calibration_documentation_only')
METADATA_ROLES = {'depth.txt': 'depth_index', 'rgb.txt': 'rgb_index',
                  'groundtruth.txt': 'evaluation_only_mocap_ground_truth'}
MANIFEST_KEYS = {'schema_version', 'dataset', 'official_archive', 'preregistration_sha256',
                 'depth_calibration', 'calibration_source', 'frames', 'files'}
ARCHIVE_KEYS = {'url', 'final_url', 'bytes', 'sha256', 'md5', 'root', 'published_checksum'}
FRAME_KEYS = {'source_index', 'depth_file', 'depth_timestamp', 'rgb_file', 'rgb_timestamp',
              'rgb_source_index', 'pair_gap_seconds', 'split'}
FILE_KEYS = {'file', 'source_path', 'bytes', 'sha256', 'role'}


def digest(data):
    return hashlib.sha256(data).hexdigest()


def load_qualifier():
    path = ROOT / 'scripts/qualify-rgbd-independent.py'
    spec = importlib.util.spec_from_file_location('independent_metadata', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def verify_design(path):
    if path.is_symlink() or path.stat().st_size > 64 * 1024:
        raise ValueError('unsafe preregistration source')
    data = path.read_bytes()
    if digest(data) != DESIGN_SHA256:
        raise ValueError('changed preregistered source/window/design')
    return json.loads(data)


def archive_identity(path, limits=DEFAULT_LIMITS):
    if path.is_symlink() or not path.is_file():
        raise ValueError('archive must be a regular non-symlink file')
    size = path.stat().st_size
    if not 0 < size <= limits['compressed']:
        raise ValueError('compressed archive byte bound exceeded')
    sha, md5, count = hashlib.sha256(), hashlib.md5(), 0
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(65536), b''):
            count += len(chunk)
            if count > limits['compressed']:
                raise ValueError('compressed archive changed or exceeds bound')
            sha.update(chunk)
            md5.update(chunk)
    if count != size:
        raise ValueError('archive size changed while hashing')
    return dict(bytes=count, sha256=sha.hexdigest(), md5=md5.hexdigest())


class BoundedGzip:
    """Count the complete decompressed tar stream, including headers and padding."""
    def __init__(self, stream, maximum):
        self.stream, self.maximum, self.count = stream, maximum, 0

    def read(self, size):
        data = self.stream.read(min(size, max(1, self.maximum - self.count + 1)))
        self.count += len(data)
        if self.count > self.maximum:
            raise ValueError('full decompressed archive stream exceeds bound')
        return data

    def exact(self, size):
        result = bytearray()
        while len(result) < size:
            part = self.read(min(65536, size - len(result)))
            if not part:
                raise ValueError('truncated tar stream')
            result.extend(part)
        return bytes(result)


def member_relative(name):
    if not name or name.startswith('/') or '\\' in name:
        raise ValueError('unsafe archive path')
    if any(part in ('', '.', '..') for part in name.split('/')):
        raise ValueError('archive path traversal or noncanonical path')
    if name == ARCHIVE_ROOT.rstrip('/'):
        return ''
    if not name.startswith(ARCHIVE_ROOT):
        raise ValueError('foreign archive root')
    return name[len(ARCHIVE_ROOT):]


def local_filename(source_path):
    if source_path in METADATA_ROLES:
        return source_path
    if not re.fullmatch(r'(depth|rgb)/[0-9]+\.[0-9]+\.png', source_path):
        raise ValueError('unsupported selected original image path')
    kind, filename = source_path.split('/')
    return kind + '-' + filename


def scan_archive(path, selected_paths=(), output_directory=None, limits=DEFAULT_LIMITS):
    """Inventory safe headers; consume opaque payloads without interpreting PNGs.

    The first pass requests no selected paths. The second may hash/write only
    qualified source paths. Extended tar headers are rejected before allocating
    their payloads. No tarfile extraction method is used.
    """
    archive_identity(path, limits)
    selected = set(selected_paths)
    for source_path in selected:
        local_filename(source_path)
    if output_directory is not None:
        if output_directory.is_symlink():
            raise ValueError('unsafe selected-output symlink')
        output_directory.mkdir(parents=True, exist_ok=True)
    inventory, metadata, hashes, seen = {}, {}, {}, set()
    count, metadata_bytes, selected_bytes = 0, 0, 0
    with path.open('rb') as compressed, gzip.GzipFile(fileobj=compressed, mode='rb') as zipped:
        stream = BoundedGzip(zipped, limits['decompressed'])
        while True:
            header = stream.exact(512)
            if header == bytes(512):
                if stream.exact(512) != bytes(512):
                    raise ValueError('tar end marker missing second zero block')
                while True:
                    trailing = stream.read(65536)
                    if not trailing:
                        break
                    if any(trailing):
                        raise ValueError('nonzero payload after tar end markers')
                break
            try:
                member = tarfile.TarInfo.frombuf(header, 'utf8', 'strict')
            except (tarfile.HeaderError, UnicodeError, ValueError) as error:
                raise ValueError('invalid tar header: ' + str(error)) from None
            count += 1
            if count > limits['members']:
                raise ValueError('archive member count exceeds bound')
            if member.type not in (tarfile.REGTYPE, tarfile.AREGTYPE, tarfile.DIRTYPE):
                raise ValueError('unsupported archive member type; links/extensions/devices refused')
            if type(member.size) is not int or not 0 <= member.size <= limits['member']:
                raise ValueError('archive member size exceeds bound or is negative')
            if member.linkname:
                raise ValueError('archive link target forbidden')
            relative = member_relative(member.name)
            if member.name in seen:
                raise ValueError('duplicate archive member path')
            seen.add(member.name)
            if member.isdir():
                if member.size:
                    raise ValueError('directory payload forbidden')
                continue
            if not relative:
                raise ValueError('archive root cannot be a regular file')
            inventory[relative] = dict(source_path=relative, bytes=member.size)
            collect = relative in METADATA_ROLES
            if collect:
                metadata_bytes += member.size
                if metadata_bytes > limits['metadata']:
                    raise ValueError('aggregate complete metadata byte bound exceeded')
            if relative in selected:
                selected_bytes += member.size
                if selected_bytes > limits['selected']:
                    raise ValueError('selected archive byte bound exceeded')
            data, hasher = bytearray(), hashlib.sha256()
            target_stream = None
            if relative in selected and output_directory is not None:
                target = output_directory / local_filename(relative)
                if target.is_symlink():
                    raise ValueError('unsafe selected output file symlink')
                target_stream = target.open('xb')
            try:
                remaining = member.size
                while remaining:
                    chunk = stream.exact(min(65536, remaining))
                    remaining -= len(chunk)
                    if collect:
                        data.extend(chunk)
                    if relative in selected:
                        hasher.update(chunk)
                        if target_stream is not None:
                            target_stream.write(chunk)
                if collect:
                    metadata[relative] = bytes(data)
                if relative in selected:
                    hashes[relative] = dict(source_path=relative, bytes=member.size,
                                            sha256=hasher.hexdigest())
                if member.size % 512:
                    padding = stream.exact(512 - member.size % 512)
                    if any(padding):
                        raise ValueError('nonzero archive member padding')
            finally:
                if target_stream is not None:
                    target_stream.close()
        decompressed = stream.count
    if set(metadata) != set(METADATA_ROLES):
        raise ValueError('missing complete original metadata tables')
    if selected - set(hashes):
        raise ValueError('selected original image or metadata member absent')
    return dict(inventory=inventory, metadata=metadata, selected_files=hashes,
                members=count, decompressed_bytes=decompressed,
                archive_payload_decompressed=True)


def verify_manifest(manifest):
    """Strict public design schema; no Git-mirror fallback or source reselection."""
    if set(manifest) != MANIFEST_KEYS or type(manifest['schema_version']) is not int or manifest['schema_version'] != 1:
        raise ValueError('changed exact independent manifest schema')
    if manifest['dataset'] != DATASET or manifest['preregistration_sha256'] != DESIGN_SHA256:
        raise ValueError('changed independent source/window preregistration')
    archive = manifest['official_archive']
    if set(archive) != ARCHIVE_KEYS or archive['url'] != OFFICIAL_URL or archive['final_url'] != FINAL_URL or archive['root'] != ARCHIVE_ROOT:
        raise ValueError('changed official archive identity/URLs/root')
    if type(archive['bytes']) is not int or not 0 < archive['bytes'] <= DEFAULT_LIMITS['compressed']:
        raise ValueError('archive byte bound')
    if not re.fullmatch(r'[0-9a-f]{64}', archive['sha256']) or not re.fullmatch(r'[0-9a-f]{32}', archive['md5']):
        raise ValueError('archive hash format')
    if archive['published_checksum'] is not None:
        raise ValueError('publisher checksum requires a new preregistered design before data use')
    if (manifest['depth_calibration'] != PROFILE
            or any(type(value) not in (int, float) for value in manifest['depth_calibration'].values())
            or manifest['calibration_source'] != CALIBRATION):
        raise ValueError('changed exact published camera descriptor/profile')
    frames = manifest['frames']
    if len(frames) != 180 or [f['source_index'] for f in frames] != list(range(100, 280)):
        raise ValueError('changed fixed consecutive original window')
    expected = set(METADATA_ROLES)
    last_depth, last_rgb, last_rgb_index, recorded = None, None, None, {}
    for i, frame in enumerate(frames):
        if set(frame) != FRAME_KEYS or type(frame['source_index']) is not int or type(frame['rgb_source_index']) is not int or frame['rgb_source_index'] < 0:
            raise ValueError('changed exact frame schema or source ordinal')
        if frame['split'] != ('initialization' if i == 0 else 'independent_recording'):
            raise ValueError('changed frame evidence role')
        depth, rgb, gap = frame['depth_timestamp'], frame['rgb_timestamp'], frame['pair_gap_seconds']
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in (depth, rgb, gap)):
            raise ValueError('nonfinite association timestamp/gap')
        if gap < 0 or abs(depth - rgb) > .02 or abs(gap - abs(depth - rgb)) > 1e-15:
            raise ValueError('changed exact RGB/depth association gate')
        if (last_depth is not None and depth <= last_depth) or (last_rgb is not None and rgb < last_rgb) or (last_rgb_index is not None and frame['rgb_source_index'] < last_rgb_index):
            raise ValueError('nonincreasing original association order')
        for kind in ('depth', 'rgb'):
            filename = frame[kind + '_file']
            if not re.fullmatch(kind + r'-[0-9]+\.[0-9]+\.png', filename):
                raise ValueError('unsafe selected filename')
            expected.add(filename)
        identity = (rgb, frame['rgb_file'])
        if frame['rgb_source_index'] in recorded and recorded[frame['rgb_source_index']] != identity:
            raise ValueError('ambiguous repeated RGB acquisition')
        recorded[frame['rgb_source_index']] = identity
        last_depth, last_rgb, last_rgb_index = depth, rgb, frame['rgb_source_index']
    files = manifest['files']
    if len(files) != len(expected) or {f['file'] for f in files} != expected:
        raise ValueError('changed exact selected raw inventory')
    total, metadata_total = CALIBRATION['bytes'], 0
    for item in files:
        if set(item) != FILE_KEYS or type(item['bytes']) is not int or not 0 < item['bytes'] <= DEFAULT_LIMITS['member'] or not re.fullmatch(r'[0-9a-f]{64}', item['sha256']):
            raise ValueError('unsafe source file descriptor')
        if item['file'] != local_filename(item['source_path']):
            raise ValueError('source path/local filename mismatch')
        if item['file'] in METADATA_ROLES:
            metadata_total += item['bytes']
            expected_role = METADATA_ROLES[item['file']]
        else:
            expected_role = item['source_path'].split('/')[0] + '_frame'
        if item['role'] != expected_role:
            raise ValueError('source role mismatch')
        total += item['bytes']
    if metadata_total > DEFAULT_LIMITS['metadata'] or total > DEFAULT_LIMITS['selected']:
        raise ValueError('aggregate metadata or selected raw byte bound exceeded')
    return manifest


def response_headers(path):
    """Read HTTP status/Location only, never authentication/cookie header values."""
    status, location = None, None
    for line in path.read_text().splitlines():
        if line.startswith('HTTP/'):
            status, location = int(line.split()[1]), None
        elif line.lower().startswith('location:'):
            location = line.split(':', 1)[1].strip()
    return status, location


def observed_regular_bytes(path):
    """Observe only a retained regular file size; never follow a symlink."""
    try:
        if not path.is_symlink() and path.is_file():
            return path.stat().st_size
    except OSError:
        pass
    return None


def download_archive(path, exposure=None):
    """Follow only the single explicitly preregistered official redirect."""
    path.parent.mkdir(parents=True, exist_ok=True)
    partial, headers = path.with_suffix('.download'), path.with_suffix('.http-headers')
    if partial.exists() or partial.is_symlink() or headers.exists() or headers.is_symlink():
        raise ValueError('archive transfer scratch path exists; preserve prior attempt')
    common = ['curl', '--silent', '--show-error', '--fail', '--connect-timeout', '20',
              '--max-time', '300', '--proto', '=https', '--proto-redir', '=https']
    subprocess.run(common + ['--head', '--dump-header', str(headers), OFFICIAL_URL,
                             '--output', '/dev/null'], check=True)
    status, location = response_headers(headers)
    if status != 302 or location != FINAL_URL:
        raise ValueError('official archive redirect differs from preregistered final URL')
    headers.unlink()
    try:
        subprocess.run(common + ['--max-filesize', str(DEFAULT_LIMITS['compressed']),
                                 '--dump-header', str(headers), FINAL_URL,
                                 '--output', str(partial)], check=True)
        status, location = response_headers(headers)
        if status != 200 or location is not None:
            raise ValueError('unexpected final archive HTTP status or redirect')
        archive_identity(partial)
        partial.replace(path)
    finally:
        # Preserve a partial-transfer observation before removing scratch bytes.
        # A failed transfer cannot establish whether those bytes contain images.
        if exposure is not None:
            observed = observed_regular_bytes(partial)
            if observed is not None:
                exposure['compressed_archive_bytes_observed'] = observed
        partial.unlink(missing_ok=True)


def camera_bytes(path=None):
    if path is not None:
        if path.is_symlink() or path.stat().st_size != CALIBRATION['bytes']:
            raise ValueError('unsafe camera documentation source')
        data = path.read_bytes()
    else:
        url = f'https://raw.githubusercontent.com/{CALIBRATION["repository"]}/{CALIBRATION["revision"]}/{CALIBRATION["source_path"]}'
        result = subprocess.run(['curl', '--fail', '--silent', '--show-error', '--connect-timeout',
                                 '20', '--max-time', '60', '--max-filesize', str(CALIBRATION['bytes']),
                                 '--proto', '=https', url], check=True, stdout=subprocess.PIPE)
        data = result.stdout
    if len(data) != CALIBRATION['bytes'] or digest(data) != CALIBRATION['sha256']:
        raise ValueError('pinned camera documentation SHA256/size mismatch')
    return data


def write_bytes_new(path, value):
    """Publish a complete file atomically without replacing any prior evidence."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + '.prepare-', dir=path.parent)
    try:
        with os.fdopen(descriptor, 'wb') as stream:
            stream.write(value)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        Path(temporary).unlink(missing_ok=True)


def write_json_new(path, value):
    write_bytes_new(path, (json.dumps(value, indent=2, allow_nan=False) + '\n').encode())


def verify_inputs(manifest, raw, archive):
    verify_manifest(manifest)
    if raw.is_symlink():
        raise ValueError('unsafe raw directory symlink')
    receipt = archive_identity(archive)
    if any(receipt[key] != manifest['official_archive'][key] for key in receipt):
        raise ValueError('retained archive differs from frozen identity')
    expected_names = {i['file'] for i in manifest['files']} | {CALIBRATION['file']}
    if {p.name for p in raw.iterdir()} != expected_names:
        raise ValueError('extra or missing selected raw files (camera documentation sidecar allowed)')
    scanned = scan_archive(archive, [i['source_path'] for i in manifest['files']])
    for item in manifest['files']:
        path = raw / item['file']
        if path.is_symlink() or not path.is_file() or path.stat().st_size != item['bytes']:
            raise ValueError('unsafe or changed raw file')
        actual = digest(path.read_bytes())
        member = scanned['selected_files'][item['source_path']]
        if actual != item['sha256'] or member['bytes'] != item['bytes'] or member['sha256'] != actual:
            raise ValueError('raw file differs from original archive member')
    camera_bytes(raw / CALIBRATION['file'])
    load_qualifier().qualify_metadata(manifest, lambda name: (raw / name).read_bytes())
    return receipt


def acquire(archive, preregistration, output, acquisition_log, calibration=None):
    verify_design(preregistration)
    if acquisition_log.exists() or acquisition_log.is_symlink() or (output / 'manifest.json').exists():
        raise ValueError('existing acquisition success/evidence; use a new evidence path')
    started = acquisition_log.with_name(acquisition_log.name + '.started.json')
    write_json_new(started, dict(schema_version=1, dataset=DATASET, preregistration_sha256=DESIGN_SHA256,
                                fixed_window=dict(first_depth_index=100, last_depth_index=279,
                                                  frames=180, updates=179),
                                acquisition_helper_sha256=digest(Path(__file__).read_bytes()),
                                started_utc=datetime.now(timezone.utc).isoformat()))
    stage = 'archive_acquisition'
    exposure = dict(compressed_acquisition_attempted=False, decompression_attempted=False,
                    compressed_archive_bytes_observed=None, compressed_images_acquired=False,
                    archive_payload_decompressed=False)
    manifest_was_written, final_log_attempted = False, False
    try:
        exposure['compressed_acquisition_attempted'] = True
        exposure['compressed_images_acquired'] = None
        exposure['compressed_archive_bytes_observed'] = observed_regular_bytes(archive)
        if not archive.exists():
            download_archive(archive, exposure)
        receipt = archive_identity(archive)
        exposure['compressed_archive_bytes_observed'] = receipt['bytes']
        stage = 'bounded_inventory_and_metadata'
        exposure['decompression_attempted'] = True
        exposure['archive_payload_decompressed'] = None
        scanned = scan_archive(archive)
        exposure['compressed_images_acquired'] = True
        exposure['archive_payload_decompressed'] = True
        official_archive = dict(url=OFFICIAL_URL, final_url=FINAL_URL, **receipt,
                                root=ARCHIVE_ROOT, published_checksum=None)
        write_json_new(acquisition_log.with_name(acquisition_log.name + '.inventory.json'),
                       dict(official_archive=official_archive, members=scanned['members'],
                            decompressed_bytes=scanned['decompressed_bytes'], inventory=scanned['inventory']))
        raw = output / 'raw'
        if raw.is_symlink():
            raise ValueError('unsafe raw output symlink')
        raw.mkdir(parents=True, exist_ok=True)
        for name, data in scanned['metadata'].items():
            with (raw / name).open('xb') as stream:
                stream.write(data)
        stage = 'metadata_qualification'
        selection = load_qualifier().select_metadata(scanned['metadata'])
        images = {kind + '/' + f[kind + '_file'].removeprefix(kind + '-')
                  for f in selection['frames'] for kind in ('depth', 'rgb')}
        wanted = images | set(METADATA_ROLES)
        if wanted - set(scanned['inventory']):
            raise ValueError('fixed selection image member absent; no replacement window')
        if sum(scanned['inventory'][name]['bytes'] for name in wanted) + CALIBRATION['bytes'] > DEFAULT_LIMITS['selected']:
            raise ValueError('selected raw inventory exceeds 128 MiB')
        stage = 'qualified_opaque_selection'
        copied = scan_archive(archive, images, raw)
        if copied['metadata'] != scanned['metadata'] or archive_identity(archive) != receipt:
            raise ValueError('archive changed between qualification and selected copy')
        files = []
        for source_path in sorted(wanted):
            if source_path in METADATA_ROLES:
                data = scanned['metadata'][source_path]
                item = dict(bytes=len(data), sha256=digest(data))
                role = METADATA_ROLES[source_path]
            else:
                item = copied['selected_files'][source_path]
                role = source_path.split('/')[0] + '_frame'
            files.append(dict(file=local_filename(source_path), source_path=source_path,
                              bytes=item['bytes'], sha256=item['sha256'], role=role))
        documentation = camera_bytes(calibration)
        with (raw / CALIBRATION['file']).open('xb') as stream:
            stream.write(documentation)
        manifest = dict(schema_version=1, dataset=DATASET, official_archive=official_archive,
                        preregistration_sha256=DESIGN_SHA256, depth_calibration=dict(PROFILE),
                        calibration_source=dict(CALIBRATION), frames=selection['frames'], files=files)
        verify_manifest(manifest)
        manifest_bytes = (json.dumps(manifest, indent=2, allow_nan=False) + '\n').encode()
        if len(manifest_bytes) > 512 * 1024:
            raise ValueError('generated manifest exceeds 512 KiB')
        stage = 'final_input_verification'
        verify_inputs(manifest, raw, archive)
        write_bytes_new(output / 'manifest.json', manifest_bytes)
        manifest_was_written = True
        stage = 'final_evidence_publication'
        log = dict(schema_version=1, dataset=DATASET, passed=True, official_archive=official_archive,
                   preregistration_sha256=DESIGN_SHA256, manifest_sha256=digest(manifest_bytes),
                   acquisition_helper_sha256=digest(Path(__file__).read_bytes()),
                   qualification_helper_sha256=digest((ROOT / 'scripts/qualify-rgbd-independent.py').read_bytes()),
                   **exposure,
                   image_headers_read=False, pixels_read=False,
                   ground_truth_pose_values_parsed=False, features_or_fits_run=False,
                   metadata_qualified_before_selected_image_copy=True,
                   archive_members=scanned['members'], decompressed_bytes=scanned['decompressed_bytes'],
                   selected_raw_bytes=sum(i['bytes'] for i in files) + CALIBRATION['bytes'],
                   files=len(files), calibration_documentation_sidecar=True,
                   manifest_was_written=True, success_manifest_retained=True,
                   qualification={k: v for k, v in selection.items() if k != 'frames'})
        final_log_attempted = True
        write_json_new(acquisition_log, log)
        return log
    except Exception as error:
        if manifest_was_written:
            (output / 'manifest.json').unlink(missing_ok=True)
        if final_log_attempted:
            acquisition_log.unlink(missing_ok=True)
        observed = observed_regular_bytes(archive)
        if observed is not None:
            exposure['compressed_archive_bytes_observed'] = observed
        write_json_new(acquisition_log.with_name(acquisition_log.name + '.failure.json'),
                       dict(schema_version=1, dataset=DATASET, passed=False, stage=stage,
                            error=str(error), preregistration_sha256=DESIGN_SHA256,
                            fixed_window=dict(first_depth_index=100, last_depth_index=279),
                            **exposure,
                            image_headers_read=False, pixels_read=False,
                            ground_truth_pose_values_parsed=False, features_or_fits_run=False,
                            source_or_window_reselected=False,
                            manifest_was_written=manifest_was_written,
                            success_manifest_retained=False, success_log_retained=False))
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('dataset', choices=[DATASET])
    parser.add_argument('--archive', type=Path, required=True)
    parser.add_argument('--preregistration', type=Path, required=True)
    parser.add_argument('--acquisition-log', type=Path)
    parser.add_argument('--output', type=Path, default=ROOT / 'data' / DATASET)
    parser.add_argument('--calibration', type=Path)
    parser.add_argument('--verify-only', action='store_true')
    args = parser.parse_args()
    verify_design(args.preregistration)
    if args.verify_only:
        manifest_path = args.output / 'manifest.json'
        if manifest_path.is_symlink() or manifest_path.stat().st_size > 512 * 1024:
            raise ValueError('unsafe or oversized manifest')
        manifest = json.loads(manifest_path.read_bytes())
        verify_inputs(manifest, args.output / 'raw', args.archive)
        print(DATASET + ': retained archive and exact selected opaque bytes verified')
    else:
        if args.acquisition_log is None:
            parser.error('--acquisition-log required for acquisition')
        result = acquire(args.archive, args.preregistration, args.output,
                         args.acquisition_log, args.calibration)
        print(json.dumps(result))


if __name__ == '__main__':
    main()
