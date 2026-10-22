#!/usr/bin/env python3
"""Refuse changed shared math and preserve existing staging destinations."""
import importlib.util
import io
from pathlib import Path
import tarfile
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    'rgbd_regression', Path(__file__).with_name('prepare-rgbd-regression.py'))
staging = importlib.util.module_from_spec(spec)
spec.loader.exec_module(staging)


class SourceIntegrity(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        snapshots = [staging.ROOT / 'integrations/rgbd/baselines' / name / 'sources'
                     for name in ('reprojection-fr2-desk-first-v1',
                                  'submap-office-first-v8', 'public-keyframes-v6')]
        archive = io.BytesIO()
        with tarfile.open(fileobj=archive, mode='w') as source:
            for name in (*staging.KERNELS, 'crates/core/src/lib.rs'):
                snapshot = next(root / name for root in snapshots if (root / name).is_file())
                source.add(snapshot, arcname=name)
        cls.archive = archive.getvalue()

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='rgbd-staging-test-')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / 'source'
        for name in (*staging.KERNELS, 'crates/core/src/lib.rs'):
            path = self.root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes((staging.ROOT / name).read_bytes())
        self.output = Path(self.temporary.name) / 'regression'

    def refuse(self):
        with patch.object(staging, 'ROOT', self.root), \
                patch.object(staging.subprocess, 'check_output', return_value=self.archive):
            with self.assertRaises(ValueError):
                staging.prepare(self.output)
        self.assertFalse(self.output.exists())

    def test_each_changed_active_kernel_is_rejected_before_staging(self):
        for name in staging.KERNELS:
            with self.subTest(kernel=name):
                path = self.root / name
                original = path.read_bytes()
                path.write_bytes(original + b'\n// changed source\n')
                self.refuse()
                path.write_bytes(original)

    def test_changed_vec3_numeric_validation_is_rejected(self):
        path = self.root / 'crates/core/src/lib.rs'
        path.write_bytes(path.read_bytes().replace(
            b'self.x.is_finite() && self.y.is_finite() && self.z.is_finite()', b'true'))
        self.refuse()

    def test_duplicate_vec3_source_is_rejected(self):
        path = self.root / 'crates/core/src/lib.rs'
        path.write_bytes(path.read_bytes() + staging.vec3(path.read_bytes()))
        self.refuse()

    def test_existing_destination_keeps_foreign_bytes(self):
        self.output.mkdir()
        foreign = self.output / 'keep'
        foreign.write_bytes(b'foreign data')
        with self.assertRaises(ValueError):
            staging.prepare(self.output)
        self.assertEqual(foreign.read_bytes(), b'foreign data')
        self.assertEqual(list(self.output.iterdir()), [foreign])


if __name__ == '__main__':
    unittest.main()
