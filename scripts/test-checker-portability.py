#!/usr/bin/env python3
"""Check pinned OSM bytes through a real autocrlf checkout and reject tampering."""
import hashlib
import importlib.util
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parent.parent
PINNED = ('german-road-extract.osm', 'german-road-extract.json')


class OsmCheckoutPortability(unittest.TestCase):
    def test_autocrlf_preserves_pinned_sources_and_tampering_fails(self):
        spec = importlib.util.spec_from_file_location('portable_osm', ROOT/'scripts/check-osm-scenes.py')
        osm = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(osm)
        with tempfile.TemporaryDirectory(prefix='rustdrive-checkout-') as directory:
            fixture = Path(directory)/'fixture'
            fixture.mkdir()
            shutil.copyfile(ROOT/'.gitattributes', fixture/'.gitattributes')
            shutil.copytree(ROOT/'maps/osm', fixture/'maps/osm')
            (fixture/'unprotected.txt').write_bytes(b'first\nsecond\n')
            def git(*args):
                subprocess.run(['git', '-C', str(fixture), *args], check=True,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            git('init', '--quiet')
            # Store LF blobs, then exercise the Windows checkout transformation.
            git('-c', 'core.autocrlf=false', 'add', '.')
            checkout = Path(directory)/'checkout'
            checkout.mkdir()
            git('-c', 'core.autocrlf=true', '-c', 'core.eol=crlf', 'checkout-index', '--all',
                '--prefix='+checkout.resolve().as_posix()+'/')
            self.assertEqual((checkout/'unprotected.txt').read_bytes(), b'first\r\nsecond\r\n')
            for name in PINNED:
                actual = (checkout/'maps/osm'/name).read_bytes()
                self.assertEqual(hashlib.sha256(actual).hexdigest(), osm.DATA_HASHES[name])
                self.assertEqual(actual, (ROOT/'maps/osm'/name).read_bytes())
            osm.ROOT = checkout
            # The real source/geometry oracle must still accept the checkout.
            osm.map_contract()
            for name in PINNED:
                path = checkout/'maps/osm'/name
                original = path.read_bytes()
                for mutated in (original.replace(b'\n', b'\r\n'), original+b' '):
                    path.write_bytes(mutated)
                    with self.assertRaisesRegex(ValueError, 'pinned real OSM source changed'):
                        osm.map_contract()
                path.write_bytes(original)


if __name__ == '__main__':
    unittest.main()
