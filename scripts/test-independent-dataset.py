#!/usr/bin/env python3
"""Synthetic archive/metadata security tests; no network, images or numeric GT poses."""
import copy
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent


def module(name, filename):
    spec = importlib.util.spec_from_file_location(name, ROOT / 'scripts' / filename)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


A = module('independent_test_acquisition', 'fetch-independent-dataset.py')
Q = module('independent_test_qualification', 'qualify-rgbd-independent.py')


def tables(depth_count=300):
    """Binary-exact clocks, repeated nearest RGB, intentionally nonnumeric poses."""
    depth = ''.join(f'{1000 + i / 64:.6f} depth/{1000 + i / 64:.6f}.png\n'
                    for i in range(depth_count)).encode()
    rgb = ''.join(f'{1000 + i / 32:.6f} rgb/{1000 + i / 32:.6f}.png\n'
                  for i in range(160)).encode()
    gt = ''.join(f'{1000 + i / 128:.7f} opaque opaque opaque opaque opaque opaque opaque\n'
                 for i in range(650)).encode()
    return {'depth.txt': depth, 'rgb.txt': rgb, 'groundtruth.txt': gt}


def fixture_entries(metadata=None):
    metadata = tables() if metadata is None else metadata
    entries = [(A.ARCHIVE_ROOT + name, data, tarfile.REGTYPE)
               for name, data in metadata.items()]
    for name in ('depth.txt', 'rgb.txt'):
        for line in metadata[name].splitlines():
            source_path = line.split()[1].decode()
            entries.append((A.ARCHIVE_ROOT + source_path,
                            b'OPAQUE-NOT-A-DECODED-PNG:' + source_path.encode(), tarfile.REGTYPE))
    return entries


def tar_stream(entries):
    data = bytearray()
    for name, payload, kind in entries:
        member = tarfile.TarInfo(name)
        member.type = kind
        member.size = len(payload)
        if kind in (tarfile.SYMTYPE, tarfile.LNKTYPE):
            member.linkname = '../outside'
        data.extend(member.tobuf(format=tarfile.USTAR_FORMAT))
        data.extend(payload)
        data.extend(bytes((-len(payload)) % 512))
    data.extend(bytes(1024))
    return bytes(data)


def archive(path, entries):
    raw = tar_stream(entries)
    path.write_bytes(gzip.compress(raw, mtime=0))
    return raw


def manifest(path, metadata=None):
    metadata = tables() if metadata is None else metadata
    selected = Q.select_metadata(metadata)
    scanned = A.scan_archive(path)
    wanted = {kind + '/' + frame[kind + '_file'].removeprefix(kind + '-')
              for frame in selected['frames'] for kind in ('depth', 'rgb')} | set(A.METADATA_ROLES)
    measured = A.scan_archive(path, wanted)
    files = [dict(file=A.local_filename(source), source_path=source,
                  bytes=measured['selected_files'][source]['bytes'],
                  sha256=measured['selected_files'][source]['sha256'],
                  role=A.METADATA_ROLES[source] if source in A.METADATA_ROLES else source.split('/')[0] + '_frame')
             for source in sorted(wanted)]
    result = dict(schema_version=1, dataset=A.DATASET,
                  official_archive=dict(url=A.OFFICIAL_URL, final_url=A.FINAL_URL,
                                        **A.archive_identity(path), root=A.ARCHIVE_ROOT,
                                        published_checksum=None),
                  preregistration_sha256=A.DESIGN_SHA256, depth_calibration=dict(A.PROFILE),
                  calibration_source=dict(A.CALIBRATION), frames=selected['frames'], files=files)
    A.verify_manifest(result)
    return result, scanned, measured


class IndependentDatasetTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.source = self.directory / 'synthetic.tgz'
        self.raw_stream = archive(self.source, fixture_entries())
        self.network_guard = mock.patch.object(A.subprocess, 'run', side_effect=AssertionError('network forbidden in synthetic tests'))
        self.network_guard.start()

    def tearDown(self):
        self.network_guard.stop()
        self.temporary.cleanup()

    def rejected_archive(self, entries, limits=None):
        candidate = self.directory / 'changed.tgz'
        archive(candidate, entries)
        self.assertNotEqual(A.archive_identity(candidate)['sha256'], A.archive_identity(self.source)['sha256'])
        with self.assertRaises((ValueError, EOFError, gzip.BadGzipFile)):
            A.scan_archive(candidate, limits=A.DEFAULT_LIMITS if limits is None else limits)

    def test_complete_stream_inventory_and_no_full_extraction(self):
        initial = A.scan_archive(self.source)
        self.assertEqual(initial['decompressed_bytes'], len(self.raw_stream))
        self.assertEqual(initial['selected_files'], {})
        self.assertEqual(initial['metadata'], tables())
        self.assertFalse((self.directory / A.ARCHIVE_ROOT).exists())
        selection = Q.select_metadata(initial['metadata'])
        wanted = {kind + '/' + f[kind + '_file'].removeprefix(kind + '-')
                  for f in selection['frames'] for kind in ('depth', 'rgb')}
        output = self.directory / 'selected'
        copied = A.scan_archive(self.source, wanted, output)
        self.assertEqual({p.name for p in output.iterdir()}, {A.local_filename(p) for p in wanted})
        self.assertNotIn('depth-1000.000000.png', {p.name for p in output.iterdir()})
        for source, info in copied['selected_files'].items():
            self.assertEqual(A.digest((output / A.local_filename(source)).read_bytes()), info['sha256'])
        self.assertEqual(A.archive_identity(self.source)['md5'], hashlib.md5(self.source.read_bytes()).hexdigest())

    def test_malicious_paths_links_types_and_duplicate_names(self):
        cases = [(A.ARCHIVE_ROOT + '../outside', tarfile.REGTYPE),
                 ('/absolute/outside', tarfile.REGTYPE),
                 (A.ARCHIVE_ROOT + 'depth\\outside', tarfile.REGTYPE),
                 ('foreign_root/file', tarfile.REGTYPE),
                 (A.ARCHIVE_ROOT + 'a//b', tarfile.REGTYPE),
                 (A.ARCHIVE_ROOT + 'a/./b', tarfile.REGTYPE)]
        cases += [(A.ARCHIVE_ROOT + 'hazard', kind) for kind in
                  (tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.FIFOTYPE, tarfile.CHRTYPE,
                   tarfile.BLKTYPE, tarfile.GNUTYPE_LONGNAME, tarfile.XHDTYPE)]
        for name, kind in cases:
            with self.subTest(name=name, kind=kind):
                self.rejected_archive([(name, b'', kind)] + fixture_entries())
        self.rejected_archive(fixture_entries() + [fixture_entries()[0]])
        self.rejected_archive([(A.ARCHIVE_ROOT, b'not-empty', tarfile.DIRTYPE)] + fixture_entries())
        self.rejected_archive([(A.ARCHIVE_ROOT, b'', tarfile.DIRTYPE),
                               (A.ARCHIVE_ROOT.rstrip('/'), b'', tarfile.DIRTYPE)] + fixture_entries())

    def test_header_size_and_payload_corruption_rejected(self):
        for size_field in (b'-0000000001\0', b'00000000nan\0', b'00020000001\0'):
            with self.subTest(size_field=size_field):
                info = tarfile.TarInfo(A.ARCHIVE_ROOT + 'oversized')
                header = bytearray(info.tobuf(format=tarfile.USTAR_FORMAT))
                header[124:136] = size_field
                header[148:156] = b'        '
                header[148:156] = ('%06o\0 ' % sum(header)).encode()
                candidate = self.directory / 'invalid-size.tgz'
                candidate.write_bytes(gzip.compress(bytes(header) + bytes(1024), mtime=0))
                with self.assertRaises(ValueError):
                    A.scan_archive(candidate)
        candidate = self.directory / 'trailing.tgz'
        candidate.write_bytes(gzip.compress(self.raw_stream + b'HIDDEN SECOND ARCHIVE', mtime=0))
        with self.assertRaisesRegex(ValueError, 'after tar end'):
            A.scan_archive(candidate)
        damaged = bytearray(self.source.read_bytes())
        damaged[-8] ^= 1
        candidate.write_bytes(damaged)
        with self.assertRaises((ValueError, EOFError, gzip.BadGzipFile)):
            A.scan_archive(candidate)

    def test_caps_include_headers_padding_and_aggregate_metadata(self):
        for key, value in [('compressed', self.source.stat().st_size - 1),
                           ('decompressed', len(self.raw_stream) - 1), ('members', 3),
                           ('member', 20), ('metadata', 20), ('selected', 20)]:
            with self.subTest(cap=key):
                bounds = dict(A.DEFAULT_LIMITS, **{key: value})
                with self.assertRaises(ValueError):
                    A.scan_archive(self.source, ['depth.txt'], limits=bounds)
        larger = dict(tables())
        for name in larger:
            larger[name] += b'# padding\n' * 150000
            self.assertLessEqual(len(larger[name]), 4 * 1024 * 1024)
        self.assertGreater(sum(map(len, larger.values())), 4 * 1024 * 1024)
        with self.assertRaisesRegex(ValueError, 'aggregate'):
            Q.select_metadata(larger)
        self.rejected_archive(fixture_entries(larger))

    def test_source_file_and_selected_output_symlinks(self):
        link = self.directory / 'link.tgz'
        link.symlink_to(self.source)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            A.scan_archive(link)
        output = self.directory / 'output-link'
        output.symlink_to(self.directory, target_is_directory=True)
        with self.assertRaisesRegex(ValueError, 'symlink'):
            A.scan_archive(self.source, ['depth.txt'], output)

    def test_metadata_only_selection_retains_repeats_and_opaque_pose_tokens(self):
        selected = Q.select_metadata(tables())
        self.assertEqual(selected['window'], Q.WINDOW)
        self.assertEqual(len(selected['frames']), 180)
        self.assertEqual(selected['frames'][0]['split'], 'initialization')
        self.assertTrue(all(f['split'] == 'independent_recording' for f in selected['frames'][1:]))
        self.assertGreater(selected['timestamps']['duplicate_rgb_associations'], 0)
        self.assertEqual(selected['timestamps']['unique_rgb_acquisitions'] + selected['timestamps']['duplicate_rgb_associations'], 180)
        self.assertEqual(Q.nearest_index([1., 1.03125], 1.015625), 0)
        m, _, _ = manifest(self.source)
        reads = []
        def metadata_reader(name):
            self.assertIn(name, A.METADATA_ROLES)
            reads.append(name)
            return tables()[name]
        proof = Q.qualify_metadata(m, metadata_reader)
        self.assertEqual(set(reads), set(A.METADATA_ROLES))
        self.assertEqual(proof['metadata_files']['groundtruth.txt']['sha256'], A.digest(tables()['groundtruth.txt']))

    def test_entire_source_ordering_arity_and_finite_times(self):
        for name in tables():
            for fault in ('duplicate', 'decrease', 'nan', 'inf', 'arity'):
                with self.subTest(source=name, fault=fault):
                    original = tables()
                    lines = original[name].decode().splitlines()
                    index = len(lines) - 2  # after the selected window
                    fields = lines[index].split()
                    if fault == 'duplicate':
                        fields[0] = lines[index - 1].split()[0]
                    elif fault == 'decrease':
                        fields[0] = '0.0'
                    elif fault in ('nan', 'inf'):
                        fields[0] = fault
                    else:
                        fields.append('extra')
                    lines[index] = ' '.join(fields)
                    changed = dict(original, **{name: ('\n'.join(lines) + '\n').encode()})
                    self.assertNotEqual(changed[name], original[name])
                    with self.assertRaises(ValueError):
                        Q.select_metadata(changed)
        with self.assertRaisesRegex(ValueError, 'fixed original depth window'):
            Q.select_metadata(tables(depth_count=200))
        bad_paths = tables()
        bad_paths['rgb.txt'] = bad_paths['rgb.txt'].replace(b'rgb/1000.000000.png', b'../1000.000000.png', 1)
        with self.assertRaises(ValueError):
            Q.select_metadata(bad_paths)

    def test_pair_and_reference_gates_without_window_search(self):
        bad = tables()
        bad['rgb.txt'] = b'2000.000000 rgb/2000.000000.png\n2001.000000 rgb/2001.000000.png\n'
        with self.assertRaisesRegex(ValueError, 'RGB/depth gap'):
            Q.select_metadata(bad)
        bad = tables()
        bad['groundtruth.txt'] = ''.join(f'{1000+i/16:.6f} x x x x x x x\n' for i in range(100)).encode()
        with self.assertRaisesRegex(ValueError, 'bracket exceeds'):
            Q.select_metadata(bad)
        bad['groundtruth.txt'] = b'0.000000 x x x x x x x\n0.010000 x x x x x x x\n'
        with self.assertRaisesRegex(ValueError, 'extrapolation'):
            Q.select_metadata(bad)

    def test_exact_manifest_schemas_source_window_roles_and_hash_formats(self):
        original, _, _ = manifest(self.source)
        mutations = [lambda x:x.__setitem__('repository', 'mirror/repo'),
                     lambda x:x.__setitem__('preregistration_sha256', '0'*64),
                     lambda x:x['official_archive'].__setitem__('url', 'https://example.com/archive'),
                     lambda x:x['official_archive'].__setitem__('published_checksum', {'md5': '0'*32}),
                     lambda x:x['official_archive'].__setitem__('bytes', A.DEFAULT_LIMITS['compressed']+1),
                     lambda x:x['official_archive'].__setitem__('md5', 'X'*32),
                     lambda x:x['depth_calibration'].__setitem__('units_per_metre', 5208),
                     lambda x:x['depth_calibration'].__setitem__('invalid_depth', False),
                     lambda x:x['depth_calibration'].__setitem__('extra', 'unknown'),
                     lambda x:x['frames'][-1].__setitem__('source_index', 280),
                     lambda x:x['frames'][1].__setitem__('split', 'viewed_prefix'),
                     lambda x:x['frames'][0].__setitem__('rgb_timestamp', float('nan')),
                     lambda x:x['files'][0].__setitem__('source_path', '../outside'),
                     lambda x:x['files'][0].__setitem__('bytes', True),
                     lambda x:x['files'][0].__setitem__('sha256', 'x'*64),
                     lambda x:x['files'].pop()]
        for change in mutations:
            altered = copy.deepcopy(original)
            change(altered)
            self.assertNotEqual(A.digest(json.dumps(altered, sort_keys=True).encode()),
                                A.digest(json.dumps(original, sort_keys=True).encode()))
            with self.assertRaises(ValueError):
                A.verify_manifest(altered)
        altered = copy.deepcopy(original)
        altered['frames'][0]['depth_timestamp'] += .001
        altered['frames'][0]['pair_gap_seconds'] = abs(altered['frames'][0]['depth_timestamp'] - altered['frames'][0]['rgb_timestamp'])
        A.verify_manifest(altered)
        with self.assertRaisesRegex(ValueError, 'original timestamp/window'):
            Q.qualify_metadata(altered, lambda name: tables()[name])

    def test_failed_metadata_is_preserved_before_any_selected_image_copy(self):
        bad = tables()
        lines = bad['groundtruth.txt'].splitlines()
        lines[-1] = lines[-2]
        bad['groundtruth.txt'] = b'\n'.join(lines) + b'\n'
        changed = self.directory / 'invalid-gt.tgz'
        archive(changed, fixture_entries(bad))
        output, log = self.directory/'failure-output', self.directory/'failure-log.json'
        with self.assertRaises(ValueError):
            A.acquire(changed, A.DESIGN_PATH, output, log)
        failure = json.loads(log.with_name(log.name + '.failure.json').read_text())
        self.assertEqual(failure['stage'], 'metadata_qualification')
        self.assertFalse(failure['passed'])
        self.assertFalse(failure['source_or_window_reselected'])
        self.assertTrue(failure['compressed_images_acquired'])
        self.assertTrue(failure['archive_payload_decompressed'])
        self.assertEqual({p.name for p in (output/'raw').iterdir()}, set(A.METADATA_ROLES))
        self.assertFalse((output/'manifest.json').exists())
        self.assertFalse(log.exists())
        with self.assertRaises(FileExistsError):
            A.acquire(changed, A.DESIGN_PATH, output, log)

    def test_synthetic_pipeline_proof_and_original_member_integrity(self):
        output, log = self.directory/'success-output', self.directory/'success-log.json'
        # Isolate archive mechanics from the separately tested pinned camera boundary.
        with mock.patch.object(A, 'camera_bytes', return_value=b'C'*A.CALIBRATION['bytes']):
            result = A.acquire(self.source, A.DESIGN_PATH, output, log)
            self.assertTrue(result['metadata_qualified_before_selected_image_copy'])
            self.assertEqual(result['acquisition_helper_sha256'], A.digest((ROOT/'scripts/fetch-independent-dataset.py').read_bytes()))
            m = json.loads((output/'manifest.json').read_text())
            proof = Q.qualify((output/'manifest.json').read_bytes(), output/'raw')
            self.assertEqual(proof['helper_sha256'], A.digest((ROOT/'scripts/qualify-rgbd-independent.py').read_bytes()))
            self.assertEqual(proof['official_archive'], m['official_archive'])
            self.assertFalse(proof['pixels_read'])
            self.assertFalse(proof['ground_truth_pose_values_parsed'])
            self.assertTrue(result['compressed_images_acquired'])
            self.assertTrue(result['archive_payload_decompressed'])
            selected = next(i for i in m['files'] if i['role']=='depth_frame')
            raw = output/'raw'/selected['file']
            data = raw.read_bytes()
            raw.write_bytes(bytes([data[0]^1])+data[1:])
            with self.assertRaisesRegex(ValueError, 'original archive member'):
                A.verify_inputs(m, output/'raw', self.source)
            changed = copy.deepcopy(m)
            next(i for i in changed['files'] if i['file']==selected['file'])['sha256'] = A.digest(raw.read_bytes())
            with self.assertRaisesRegex(ValueError, 'original archive member'):
                A.verify_inputs(changed, output/'raw', self.source)

    def test_raw_extra_symlink_and_camera_sidecar_boundary(self):
        m, scanned, _ = manifest(self.source)
        raw = self.directory/'proof-raw'
        raw.mkdir()
        for item in m['files']:
            content = scanned['metadata'].get(item['file'], b'opaque')
            (raw/item['file']).write_bytes(content)
        (raw/A.CALIBRATION['file']).write_bytes(b'not the pinned camera')
        proof = Q.qualify((json.dumps(m)+'\n').encode(), raw)
        self.assertTrue(proof['passed'])  # Metadata-only proof does not read the sidecar or PNGs.
        with self.assertRaises(ValueError):
            A.camera_bytes(raw/A.CALIBRATION['file'])
        (raw/'unexpected.txt').write_text('extra')
        with self.assertRaisesRegex(ValueError, 'extra or missing'):
            Q.qualify(json.dumps(m).encode(), raw)
        (raw/'unexpected.txt').unlink()
        name = m['frames'][0]['rgb_file']
        (raw/name).unlink()
        (raw/name).symlink_to(raw/'rgb.txt')
        with self.assertRaisesRegex(ValueError, 'unsafe actual raw'):
            Q.qualify(json.dumps(m).encode(), raw)

    def test_final_evidence_failure_cannot_leave_stale_success(self):
        original_writer = A.write_json_new
        for publish_log_before_failure in (False, True):
            with self.subTest(publish_log_before_failure=publish_log_before_failure):
                output = self.directory/('publish-output-'+str(publish_log_before_failure))
                log = self.directory/('publish-log-'+str(publish_log_before_failure)+'.json')
                def fail_final(path, value):
                    if path == log:
                        self.assertTrue((output/'manifest.json').exists())
                        if publish_log_before_failure:
                            original_writer(path, value)
                        raise OSError('synthetic final evidence publication failure')
                    original_writer(path, value)
                with mock.patch.object(A, 'camera_bytes', return_value=b'C'*A.CALIBRATION['bytes']), mock.patch.object(A, 'write_json_new', side_effect=fail_final):
                    with self.assertRaises(OSError):
                        A.acquire(self.source, A.DESIGN_PATH, output, log)
                self.assertFalse(log.exists())
                self.assertFalse((output/'manifest.json').exists())
                failure = json.loads(log.with_name(log.name+'.failure.json').read_text())
                self.assertFalse(failure['passed'])
                self.assertEqual(failure['stage'], 'final_evidence_publication')
                self.assertTrue(failure['manifest_was_written'])
                self.assertFalse(failure['success_manifest_retained'])
                self.assertFalse(failure['success_log_retained'])
                self.assertTrue(failure['compressed_images_acquired'])
                self.assertTrue(failure['archive_payload_decompressed'])

    def test_partial_transfer_failure_preserves_unknown_exposure_and_byte_observation(self):
        archive_path = self.directory/'partial-source.tgz'
        output, log = self.directory/'partial-output', self.directory/'partial-log.json'
        partial_bytes = b'opaque transferred bytes before synthetic curl failure'
        def transfer(command, check):
            headers = Path(command[command.index('--dump-header')+1])
            if '--head' in command:
                headers.write_text('HTTP/2 302\nLocation: '+A.FINAL_URL+'\n')
            else:
                headers.write_text('HTTP/2 200\n')
                Path(command[command.index('--output')+1]).write_bytes(partial_bytes)
                raise A.subprocess.CalledProcessError(56, 'synthetic partial transfer')
        with mock.patch.object(A.subprocess, 'run', side_effect=transfer) as calls:
            with self.assertRaises(A.subprocess.CalledProcessError):
                A.acquire(archive_path, A.DESIGN_PATH, output, log)
            self.assertEqual(calls.call_count, 2)
        failure = json.loads(log.with_name(log.name+'.failure.json').read_text())
        self.assertTrue(failure['compressed_acquisition_attempted'])
        self.assertEqual(failure['compressed_archive_bytes_observed'], len(partial_bytes))
        self.assertIsNone(failure['compressed_images_acquired'])
        self.assertFalse(failure['decompression_attempted'])
        self.assertFalse(failure['archive_payload_decompressed'])
        self.assertFalse(failure['manifest_was_written'])
        self.assertFalse(archive_path.exists())
        self.assertFalse(archive_path.with_suffix('.download').exists())
        self.assertFalse(log.exists())
        self.assertFalse((output/'manifest.json').exists())

    def test_identity_failure_reports_existing_bytes_without_decompression(self):
        output, log = self.directory/'identity-output', self.directory/'identity-log.json'
        with mock.patch.object(A, 'archive_identity', side_effect=ValueError('synthetic identity failure')), mock.patch.object(A, 'scan_archive') as scanner:
            with self.assertRaisesRegex(ValueError, 'synthetic identity failure'):
                A.acquire(self.source, A.DESIGN_PATH, output, log)
            scanner.assert_not_called()
        failure = json.loads(log.with_name(log.name+'.failure.json').read_text())
        self.assertTrue(failure['compressed_acquisition_attempted'])
        self.assertEqual(failure['compressed_archive_bytes_observed'], self.source.stat().st_size)
        self.assertIsNone(failure['compressed_images_acquired'])
        self.assertFalse(failure['decompression_attempted'])
        self.assertFalse(failure['archive_payload_decompressed'])
        self.assertFalse(failure['success_manifest_retained'])
        self.assertFalse(log.exists())
        self.assertFalse((output/'manifest.json').exists())

    def test_scan_failure_distinguishes_attempt_from_established_decompression(self):
        for before_gzip in (False, True):
            with self.subTest(before_gzip=before_gzip):
                source = self.source
                if not before_gzip:
                    source = self.directory/'invalid-gzip.tgz'
                    source.write_bytes(b'not a gzip stream; receipt still hashes these bytes')
                    self.assertNotEqual(A.archive_identity(source), A.archive_identity(self.source))
                output = self.directory/('scan-output-'+str(before_gzip))
                log = self.directory/('scan-log-'+str(before_gzip)+'.json')
                original_scan = A.scan_archive
                def scan(*args, **kwargs):
                    if before_gzip:
                        raise ValueError('synthetic scanner identity failure before gzip opening')
                    return original_scan(*args, **kwargs)
                with mock.patch.object(A, 'scan_archive', side_effect=scan):
                    with self.assertRaises((ValueError, OSError)):
                        A.acquire(source, A.DESIGN_PATH, output, log)
                failure = json.loads(log.with_name(log.name+'.failure.json').read_text())
                self.assertTrue(failure['compressed_acquisition_attempted'])
                self.assertTrue(failure['decompression_attempted'])
                self.assertIsNone(failure['compressed_images_acquired'])
                self.assertIsNone(failure['archive_payload_decompressed'])
                self.assertEqual(failure['compressed_archive_bytes_observed'], source.stat().st_size)
                self.assertFalse(failure['manifest_was_written'])
                self.assertFalse(log.exists())
                self.assertFalse((output/'manifest.json').exists())

    def test_download_follows_only_preregistered_https_redirect(self):
        destination = self.directory/'downloaded.tgz'
        calls = []
        def transfer(command, check):
            calls.append(command)
            self.assertIn('--proto', command)
            self.assertEqual(command[command.index('--proto')+1], '=https')
            self.assertNotIn('--location', command)
            self.assertNotIn('--insecure', command)
            headers = Path(command[command.index('--dump-header')+1])
            if '--head' in command:
                self.assertIn(A.OFFICIAL_URL, command)
                headers.write_text('HTTP/1.1 200 Connection established\n\nHTTP/2 302\nLocation: '+A.FINAL_URL+'\n')
            else:
                self.assertIn(A.FINAL_URL, command)
                headers.write_text('HTTP/2 200\n')
                Path(command[command.index('--output')+1]).write_bytes(self.source.read_bytes())
        with mock.patch.object(A.subprocess, 'run', side_effect=transfer):
            A.download_archive(destination)
        self.assertEqual(len(calls), 2)
        self.assertEqual(A.archive_identity(destination), A.archive_identity(self.source))
        bad_destination = self.directory/'forbidden-redirect.tgz'
        def forbidden_redirect(command, check):
            self.assertIn('--head', command)
            Path(command[command.index('--dump-header')+1]).write_text('HTTP/2 302\nLocation: https://another.example/archive.tgz\n')
        with mock.patch.object(A.subprocess, 'run', side_effect=forbidden_redirect) as network:
            with self.assertRaisesRegex(ValueError, 'redirect differs'):
                A.download_archive(bad_destination)
            self.assertEqual(network.call_count, 1)
        self.assertFalse(bad_destination.exists())


if __name__ == '__main__':
    unittest.main()
