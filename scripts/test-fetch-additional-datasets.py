#!/usr/bin/env python3
"""Verify additional-dataset acquisition boundaries without a network request."""
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parent.parent
SPEC = importlib.util.spec_from_file_location('fetch_extra', ROOT/'scripts/fetch-additional-datasets.py')
fetch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fetch)


class AcquisitionBoundaries(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory(prefix='rustdrive-additional-data-')
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name)
        self.path = self.root/'data/pdal-autzen/manifest.json'
        self.path.parent.mkdir(parents=True)
        self.payload = b'recorded terrain fixture\n'
        self.entry = {'file': 'terrain.las', 'source_path': 'terrain/terrain.las',
                      'bytes': len(self.payload), 'sha256': hashlib.sha256(self.payload).hexdigest()}
        self.manifest = {'dataset': 'pdal-autzen', 'repository': 'PDAL/PDAL',
                         'revision': '1'*40, 'redistribute_raw': False, 'files': [self.entry]}
        patch = mock.patch.object(fetch, 'ROOT', self.root)
        patch.start()
        self.addCleanup(patch.stop)
        self.save()

    def save(self):
        self.path.write_text(json.dumps(self.manifest), encoding='utf-8')

    def run_main(self, *args):
        with mock.patch('sys.argv', ['fetch-extra', '--dataset', 'pdal-autzen', *args]):
            return fetch.main()

    def test_same_size_corruption_is_rejected_without_network(self):
        raw = self.path.parent/'raw/terrain.las'
        raw.parent.mkdir()
        raw.write_bytes(self.payload)
        self.assertEqual(self.run_main('--verify-only'), 0)
        raw.write_bytes(b'X'+self.payload[1:])
        with mock.patch.object(fetch.subprocess, 'run') as download:
            with self.assertRaisesRegex(ValueError, 'pinned SHA/size'):
                self.run_main('--verify-only')
            download.assert_not_called()

    def test_corrupt_download_preserves_existing_data(self):
        raw = self.path.parent/'raw/terrain.las'
        raw.parent.mkdir()
        previous = b'preserve prior data'
        raw.write_bytes(previous)
        def corrupt(command, **kwargs):
            Path(command[-1]).write_bytes(b'X'+self.payload[1:])
        with mock.patch.object(fetch.subprocess, 'run', side_effect=corrupt):
            with self.assertRaisesRegex(ValueError, 'downloaded bytes differ'):
                self.run_main()
        self.assertEqual(raw.read_bytes(), previous)
        self.assertFalse(raw.with_name('terrain.las.download').exists())

    def test_unsafe_paths_and_false_size_types_rejected_before_network(self):
        for field, value in [('file', '../escape'), ('file', 'C:\\escape'),
                             ('source_path', 'terrain/../terrain.las'),
                             ('source_path', '/terrain/terrain.las'),
                             ('bytes', True), ('bytes', fetch.MAX_FILE_BYTES+1)]:
            original = self.entry[field]
            self.entry[field] = value
            self.save()
            with self.subTest(field=field, value=value), mock.patch.object(fetch.subprocess, 'run') as download:
                with self.assertRaises(ValueError):
                    self.run_main()
                download.assert_not_called()
            self.entry[field] = original

    def test_bounded_aggregate_and_no_unproven_redistribution(self):
        self.manifest['files'] = [dict(self.entry, file=f'terrain{i}.las',
            source_path=f'terrain/terrain{i}.las', bytes=fetch.MAX_FILE_BYTES) for i in range(8)]
        self.save()
        with self.assertRaisesRegex(ValueError, '30 MB'):
            self.run_main()
        self.manifest['files'] = [self.entry]
        self.manifest['redistribute_raw'] = True
        self.save()
        with self.assertRaisesRegex(ValueError, 'redistribution policy'):
            self.run_main()


if __name__ == '__main__':
    unittest.main()
