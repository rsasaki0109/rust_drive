#!/usr/bin/env python3
"""Exercise acquisition rejection and preservation without external downloads."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location('fetch_datasets', ROOT/'scripts/fetch-datasets.py')
fetch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fetch)


class DatasetAcquisitionFailures(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='rustdrive-data-test-')
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.payload = b'original measured fixture\n'
        self.entry = {'file': 'cloud.pcd', 'source_path': 'terrain/cloud.pcd', 'bytes': len(self.payload),
                      'sha256': hashlib.sha256(self.payload).hexdigest()}
        self.manifest = {'dataset': 'isprs-terrain', 'repository': 'PointCloudLibrary/data',
                         'revision': '5c26bdd0591ba150b91858b5c9fe5e91cb39ae86', 'files': [self.entry]}
        self.manifest_path = self.root/'data/isprs-terrain/manifest.json'
        self.manifest_path.parent.mkdir(parents=True)
        self.raw = self.manifest_path.parent/'raw'
        self.raw.mkdir()
        self.target = self.raw/'cloud.pcd'
        self.patch = mock.patch.object(fetch, 'ROOT', self.root)
        self.patch.start()
        self.addCleanup(self.patch.stop)
        self.save()

    def save(self):
        self.manifest_path.write_text(json.dumps(self.manifest), encoding='utf-8')

    def run_main(self, *extra):
        with mock.patch('sys.argv', ['fetch-datasets.py', '--dataset', 'isprs-terrain', *extra]):
            return fetch.main()

    def test_same_size_raw_tampering_is_rejected_without_download_or_changes(self):
        self.target.write_bytes(self.payload)
        self.assertEqual(self.run_main('--verify-only'), 0)
        changed = b'X'+self.payload[1:]
        self.target.write_bytes(changed)
        with mock.patch.object(fetch.subprocess, 'run') as download:
            with self.assertRaisesRegex(ValueError, 'pinned SHA/size'):
                self.run_main('--verify-only')
            download.assert_not_called()
        self.assertEqual(self.target.read_bytes(), changed)

    def test_wrong_download_never_replaces_existing_raw_file(self):
        previous = b'previous local data remains recoverable\n'
        self.target.write_bytes(previous)
        def corrupt_download(command, **kwargs):
            Path(command[-1]).write_bytes(b'X'+self.payload[1:])
        with mock.patch.object(fetch.subprocess, 'run', side_effect=corrupt_download):
            with self.assertRaisesRegex(ValueError, 'downloaded bytes differ'):
                self.run_main()
        self.assertEqual(self.target.read_bytes(), previous)
        self.assertFalse(self.target.with_name('cloud.pcd.download').exists())

    def test_invalid_paths_pins_and_negative_sizes_fail_before_any_download(self):
        for field, bad in [('file', '../escape.pcd'), ('file', '/absolute.pcd'),
                           ('file', 'C:\\escape.pcd'), ('source_path', 'terrain/../cloud.pcd'),
                           ('source_path', '/terrain/cloud.pcd'), ('source_path', 'terrain//cloud.pcd'),
                           ('bytes', -1), ('bytes', True), ('bytes', fetch.MAX_FILE_BYTES+1),
                           ('sha256', 'not-a-hash'), ('sha256', int('1'*64))]:
            original = self.entry[field]
            self.entry[field] = bad
            self.save()
            with self.subTest(field=field, bad=bad), mock.patch.object(fetch.subprocess, 'run') as download:
                with self.assertRaises(ValueError):
                    self.run_main()
                download.assert_not_called()
            self.entry[field] = original
        for field, bad in [('dataset', '../escape'), ('repository', 'org/repo/../../other'),
                           ('revision', 'main')]:
            original = self.manifest[field]
            self.manifest[field] = bad
            self.save()
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.run_main()
            self.manifest[field] = original

    def test_positive_individual_sizes_cannot_exceed_total_budget(self):
        self.manifest['files'] = [dict(self.entry, file=f'cloud{i}.pcd', source_path=f'terrain/cloud{i}.pcd',
            bytes=fetch.MAX_FILE_BYTES) for i in range(8)]
        self.save()
        with mock.patch.object(fetch.subprocess, 'run') as download:
            with self.assertRaisesRegex(ValueError, '15 MB'):
                self.run_main()
            download.assert_not_called()

    def test_case_insensitive_destination_collisions_are_rejected(self):
        self.manifest['files'].append(dict(self.entry, file='CLOUD.pcd', source_path='terrain/CLOUD.pcd'))
        self.save()
        with self.assertRaisesRegex(ValueError, 'duplicate name'):
            self.run_main('--verify-only')


if __name__ == '__main__':
    unittest.main()
