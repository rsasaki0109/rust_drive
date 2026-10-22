"""Bind observational comparisons to complete controls from the same binary."""
import hashlib
import json
import lzma
from pathlib import Path


def sha(path):
    with Path(path).open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


class Control:
    def __init__(self, folder, binaries):
        self.folder = Path(folder).resolve()
        self.report = json.loads((self.folder / 'report.json').read_text())
        manifest = self.folder / 'frozen-manifest.json'
        if not self.report['complete'] or sha(manifest) != self.report['manifest_sha256']:
            raise ValueError('control qualification is incomplete or its manifest changed')
        self.manifest = json.loads(manifest.read_text())
        self.binaries = {}
        for plant, binary in binaries.items():
            recorded = self.report['binaries'][plant]
            recorded = recorded['sha256'] if isinstance(recorded, dict) else recorded
            actual = sha(binary)
            if recorded != actual:
                raise ValueError('control was produced by a different binary: ' + plant)
            self.binaries[plant] = actual

    def path(self, relative):
        path = (self.folder / relative).resolve()
        if not path.is_relative_to(self.folder):
            raise ValueError('control evidence escapes its directory')
        return path

    def episode(self, original, expected_input):
        fields = ['plant', 'world', 'mode', 'seed']
        if 'cohort' in original:
            fields.append('cohort')
        identity = {name: original[name] for name in fields}
        rows = [row for row in self.report['episodes']
                if all(row.get(name) == value for name, value in identity.items())]
        if len(rows) != 1:
            raise ValueError('missing or duplicate control episode')
        row = rows[0]
        if row['plant'] not in self.binaries:
            raise ValueError('control plant has no validated executable')
        records = [entry for entry in self.manifest['episodes']
                   if entry['scenario'] == row['scenario'] and entry['sha256'] == row['sha256']
                   and all(entry.get(name) == value for name, value in identity.items())]
        source = self.path(row['scenario'])
        if len(records) != 1 or sha(source) != row['sha256']:
            raise ValueError('control input is not bound to its frozen manifest')
        if json.loads(source.read_text()) != expected_input:
            raise ValueError('control changed the original world, sensing or budgets')
        if (row.get('audit_error') or row['replay_exit_code'] != 0
                or row['replay']['verified'] is not True
                or row['replay']['ticks'] != row['summary']['steps']
                or row['run_exit_code'] != (0 if row['summary']['passed'] else 1)):
            raise ValueError('control evidence failed acquisition, replay or independent audit')
        return row

    def archive(self, row, name):
        record = row['storage'][name]
        path = self.path(str(Path(row['recordings']) / record['path']))
        if sha(path) != record['stored_sha256']:
            raise ValueError('compressed control evidence changed')
        with lzma.open(path, 'rb') as stream:
            raw = stream.read()
        if hashlib.sha256(raw).hexdigest() != record['sha256']:
            raise ValueError('decoded control evidence changed')
        return raw, record
