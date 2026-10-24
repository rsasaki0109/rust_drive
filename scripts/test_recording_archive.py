"""Retained recordings use portable receipt keys without changing payload bytes."""
import hashlib
import json
import lzma
from pathlib import Path
import tempfile
import unittest

from check_gnss_antenna import archive
from check_observed_detours import stored_bytes


class RecordingArchiveTests(unittest.TestCase):
    def test_nested_receipts_round_trip_with_platform_independent_names(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            folder = root / 'episode'
            payloads = {
                'run/run.json': b'{"frames": [1, 2], "accepted": false}\n',
                'run/sensors.jsonl': b'{"stamp": 0.0}\n{"stamp": 0.05}\n',
                'replay/outputs.jsonl': b'{"verified": true}\n',
                'other folder/road.json': '{"road": "道路"}\n'.encode('utf-8'),
            }
            for name, payload in payloads.items():
                path = folder / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(payload)
            log = folder / 'run-process.log'
            log.write_bytes(b'process exit: 0\n')
            storage = archive(folder)
            self.assertEqual(set(storage), set(payloads))
            row = {'recordings': 'episode', 'storage': storage}
            for name, payload in payloads.items():
                with self.subTest(name=name):
                    receipt = storage[name]
                    self.assertEqual(receipt['path'], name + '.xz')
                    self.assertEqual(receipt['encoding'], 'xz')
                    self.assertEqual(receipt['sha256'], hashlib.sha256(payload).hexdigest())
                    retained = folder / receipt['path']
                    self.assertEqual(receipt['stored_sha256'],
                                     hashlib.sha256(retained.read_bytes()).hexdigest())
                    self.assertEqual(lzma.decompress(retained.read_bytes()), payload)
                    self.assertEqual(stored_bytes(root, row, name), payload)
                    self.assertFalse((folder / name).exists())
            self.assertEqual(log.read_bytes(), b'process exit: 0\n')
            # The same receipt can be persisted and read by another platform.
            persisted = json.loads(json.dumps(row))
            self.assertEqual(stored_bytes(root, persisted, 'replay/outputs.jsonl'),
                             payloads['replay/outputs.jsonl'])

    def test_changed_compressed_evidence_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            folder = root / 'episode' / 'replay'
            folder.mkdir(parents=True)
            (folder / 'outputs.jsonl').write_bytes(b'{"verified": true}\n')
            storage = archive(root / 'episode')
            row = {'recordings': 'episode', 'storage': storage}
            (root / 'episode' / storage['replay/outputs.jsonl']['path']).write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'stored recording changed'):
                stored_bytes(root, row, 'replay/outputs.jsonl')


if __name__ == '__main__':
    unittest.main()
