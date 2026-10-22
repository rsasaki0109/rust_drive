"""Corruption controls for same-binary, unchanged-input observation baselines."""
import copy
import hashlib
import json
import lzma
from pathlib import Path
import tempfile
import unittest

from comparison_control import Control, sha


class ComparisonControlTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='comparison-control-')
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name) / 'control'
        self.base.mkdir()
        self.binary = Path(self.temporary.name) / 'binary'
        self.binary.write_bytes(b'current executable')
        self.expected = dict(duration=45., sensor_timing=dict(gnss_delay_ticks=2))
        self.input = copy.deepcopy(self.expected)
        self.identity = dict(plant='reference-2d', world='curve', mode='default', seed=7)
        raw = b'{"frames":[],"summary":{"passed":false,"steps":2}}\n'
        data = lzma.compress(raw)
        path = self.base / 'episodes/case/run.json.xz'
        path.parent.mkdir(parents=True)
        path.write_bytes(data)
        self.row = dict(self.identity, scenario='frozen/input.json', recordings='episodes/case',
                        summary=dict(passed=False, steps=2), run_exit_code=1, replay_exit_code=0,
                        replay=dict(verified=True, ticks=2), storage={'run/run.json':
                            dict(path='run.json.xz', sha256=hashlib.sha256(raw).hexdigest(),
                                 stored_sha256=sha(path))})
        self.raw = raw
        self.report = dict(complete=True, binaries={'reference-2d':
                           dict(path=str(self.binary), sha256=sha(self.binary))}, episodes=[self.row])
        self.save()

    def save(self):
        source = self.base / self.row['scenario']
        source.parent.mkdir(parents=True, exist_ok=True)
        source.write_text(json.dumps(self.input))
        self.row['sha256'] = sha(source)
        fields = [*self.identity, 'scenario', 'sha256']
        manifest = dict(episodes=[{k: row[k] for k in fields} for row in self.report['episodes']])
        path = self.base / 'frozen-manifest.json'
        path.write_text(json.dumps(manifest))
        self.report['manifest_sha256'] = sha(path)
        (self.base / 'report.json').write_text(json.dumps(self.report))

    def control(self):
        return Control(self.base, {'reference-2d': self.binary})

    def test_current_control_keeps_failed_physical_acceptance_explicit(self):
        control = self.control()
        row = control.episode(self.identity, self.expected)
        self.assertFalse(row['summary']['passed'])
        self.assertEqual(control.archive(row, 'run/run.json')[0], self.raw)
        self.assertEqual(control.binaries['reference-2d'], sha(self.binary))

    def test_different_executable_is_rejected(self):
        self.binary.write_bytes(b'different executable')
        with self.assertRaises(ValueError):
            self.control()
        self.binary.write_bytes(b'current executable')
        self.identity['plant'] = self.row['plant'] = 'rne-dynamic'
        self.save()
        with self.assertRaises(ValueError):
            self.control().episode(self.identity, self.expected)

    def test_changed_input_cannot_be_hidden_by_rebinding_its_hashes(self):
        self.input['duration'] = 46.
        self.save()
        with self.assertRaises(ValueError):
            self.control().episode(self.identity, self.expected)

    def test_incomplete_and_duplicate_controls_are_rejected(self):
        self.report['complete'] = False
        self.save()
        with self.assertRaises(ValueError):
            self.control()
        self.report['complete'] = True
        self.report['episodes'].append(copy.deepcopy(self.row))
        self.save()
        with self.assertRaises(ValueError):
            self.control().episode(self.identity, self.expected)

    def test_compressed_and_decoded_payload_corruption_are_both_rejected(self):
        control = self.control()
        row = control.episode(self.identity, self.expected)
        path = self.base / 'episodes/case/run.json.xz'
        path.write_bytes(lzma.compress(b'changed decoded evidence'))
        with self.assertRaises(ValueError):
            control.archive(row, 'run/run.json')
        row['storage']['run/run.json']['stored_sha256'] = sha(path)
        with self.assertRaises(ValueError):
            control.archive(row, 'run/run.json')

    def test_failed_replay_or_independent_audit_cannot_supply_a_control(self):
        self.row['replay']['verified'] = False
        self.save()
        with self.assertRaises(ValueError):
            self.control().episode(self.identity, self.expected)
        self.row['replay']['verified'] = True
        self.row['audit_error'] = 'invalid measured inputs'
        self.save()
        with self.assertRaises(ValueError):
            self.control().episode(self.identity, self.expected)

    def test_archive_paths_cannot_escape_the_control_directory(self):
        control = self.control()
        row = control.episode(self.identity, self.expected)
        row['storage']['run/run.json']['path'] = '../../../foreign.xz'
        with self.assertRaises(ValueError):
            control.archive(row, 'run/run.json')


if __name__ == '__main__':
    unittest.main()
