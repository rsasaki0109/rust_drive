#!/usr/bin/env python3
"""Independent source-contract tests for metadata qualification and read ordering."""
import copy
import hashlib
import importlib.util
import math
from pathlib import Path
import unittest

SPEC = importlib.util.spec_from_file_location('qualification',
    Path(__file__).with_name('qualify-rgbd-temporal.py'))
q = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(q)


def item(name, content):
    blob = b'blob ' + str(len(content)).encode() + b'\0' + content
    return dict(file=name, bytes=len(content), sha256=hashlib.sha256(content).hexdigest(),
                git_blob_sha1=hashlib.sha1(blob).hexdigest())


def fixture():
    depth = [f'{i * .0335:.6f}' for i in range(300)]
    rgb = [f'{i * .0335 + .001:.6f}' for i in range(300)]
    content = {
        'depth.txt': ''.join(f'{t} depth/{t}.png\n' for t in depth).encode(),
        'rgb.txt': ''.join(f'{t} rgb/{t}.png\n' for t in rgb).encode(),
        # Pose tokens intentionally cannot be parsed as numbers. Qualification
        # must treat them as opaque; the post-fit numerical reader rejects them.
        'groundtruth.txt': ''.join(f'{i * .01:.6f} opaque pose values must stay unread here\n'
                                   for i in range(1201)).encode(),
    }
    frames = [dict(source_index=i, rgb_source_index=i, depth_timestamp=float(depth[i]),
                   rgb_timestamp=float(rgb[i]), depth_file='depth-' + depth[i] + '.png',
                   rgb_file='rgb-' + rgb[i] + '.png',
                   split='initialization' if i == 100 else 'viewed_prefix' if i < 136 else 'unviewed_extension') for i in range(100, 280)]
    manifest = dict(files=[item(name, body) for name, body in content.items()], frames=frames)
    return manifest, content


class MetadataQualification(unittest.TestCase):
    def test_source_metadata_is_only_input_and_pose_columns_remain_opaque(self):
        manifest, content = fixture()
        reads = []
        def reader(name):
            reads.append(name)
            return content[name]
        result = q.qualify_metadata(manifest, reader)
        self.assertEqual(reads, ['depth.txt', 'rgb.txt', 'groundtruth.txt'])
        self.assertEqual(result['timestamps']['unique_rgb_acquisitions'], 180)
        self.assertEqual(result['timestamps']['duplicate_rgb_associations'], 0)
        self.assertEqual(result['timestamps']['maximum_pair_gap_us'], 1000)
        self.assertEqual(result['timestamps']['ground_truth_rows'], 1201)

    def test_conflicting_later_timestamp_is_rejected_for_whole_source(self):
        manifest, content = fixture()
        # The selected interval ends at 9.3465; the duplicate is later than it.
        content['groundtruth.txt'] += b'12.000000 other opaque pose tokens still stay unread\n'
        manifest['files'][2] = item('groundtruth.txt', content['groundtruth.txt'])
        with self.assertRaisesRegex(ValueError, 'nonincreasing timestamp'):
            q.qualify_metadata(manifest, content.__getitem__)

    def test_first_acquisition_and_last_acquisition_need_supported_brackets(self):
        manifest, content = fixture()
        for stamp in (0., .01, .02):
            self.assertEqual(q.bracket_width([0., .01, .02], stamp), 0.)
        for stamp in (-1e-9, .02 + 1e-9):
            with self.assertRaisesRegex(ValueError, 'extrapolation'):
                q.bracket_width([0., .01, .02], stamp)
        self.assertAlmostEqual(q.bracket_width([0., .02], .01), .02)
        with self.assertRaisesRegex(ValueError, 'exceeds'):
            q.bracket_width([0., .0200000001], .01)

    def test_bad_full_source_times_and_arity_do_not_get_deduplicated(self):
        for body in (b'0 a b c d e f g\n0 a b c d e f g\n',
                     b'0 a b c d e f g\nnan a b c d e f g\n',
                     b'1 a b c d e f g\n0 a b c d e f g\n',
                     b'0 a b c d e f\n1 a b c d e f g\n'):
            with self.assertRaises(ValueError):
                q.timestamp_rows(body, 8, 'groundtruth.txt', 30000)

    def test_wrong_hash_corrupt_bytes_wrong_association_and_window_are_rejected(self):
        manifest, content = fixture()
        corrupt = dict(content, **{'groundtruth.txt':content['groundtruth.txt'] + b' '})
        with self.assertRaisesRegex(ValueError, 'bytes changed'):
            q.qualify_metadata(manifest, corrupt.__getitem__)
        for field, value in [('rgb_source_index', 99), ('source_index', 99),
                             ('split', 'calibration'), ('depth_timestamp', math.nan)]:
            changed = copy.deepcopy(manifest)
            changed['frames'][0][field] = value
            with self.assertRaises(ValueError):
                q.qualify_metadata(changed, content.__getitem__)

    def test_nearest_rgb_has_fixed_earlier_tie_and_duplicates_are_retained(self):
        self.assertEqual(q.nearest_index([0., .01, .02], .005), 0)
        manifest, content = fixture()
        frames = manifest['frames']
        # Make two consecutive depth frames share a source RGB acquisition.
        # This is accepted metadata, not permission to reuse image features.
        times = [f'{i * .0335 + .001:.6f}' for i in range(300)]
        times[100] = '3.367000'
        del times[101]
        rgb_content = ''.join(f'{t} rgb/{t}.png\n' for t in times).encode()
        content['rgb.txt'] = rgb_content
        manifest['files'][1] = item('rgb.txt', rgb_content)
        for frame in frames:
            ordinal = 100 if frame['source_index'] in (100, 101) else frame['source_index'] - 1
            frame.update(rgb_source_index=ordinal, rgb_timestamp=float(times[ordinal]),
                         rgb_file='rgb-' + times[ordinal] + '.png')
        proof = q.qualify_metadata(manifest, content.__getitem__)
        self.assertEqual(proof['window']['frames'], 180)
        self.assertEqual(proof['timestamps']['unique_rgb_acquisitions'], 179)
        self.assertEqual(proof['timestamps']['duplicate_rgb_associations'], 1)


class ContinuousWindowContract(unittest.TestCase):
    def test_tail_cannot_be_dropped_reinitialized_or_reclassified(self):
        manifest, content = fixture()
        proof = q.qualify_metadata(manifest, content.__getitem__)
        self.assertEqual(proof['window'], dict(first_depth_index=100, last_depth_index=279,
                                             frames=180, updates=179))
        self.assertEqual(sum(f['split'] == 'initialization' for f in manifest['frames']), 1)
        self.assertEqual(sum(f['split'] == 'viewed_prefix' for f in manifest['frames']), 35)
        self.assertEqual(sum(f['split'] == 'unviewed_extension' for f in manifest['frames']), 144)
        for mode in ('drop_tail', 'reset_tail', 'claim_fresh_prefix', 'hide_tail_role'):
            changed = copy.deepcopy(manifest)
            if mode == 'drop_tail':
                changed['frames'].pop()
            else:
                index, role = {'reset_tail': (36, 'initialization'),
                               'claim_fresh_prefix': (1, 'unviewed_extension'),
                               'hide_tail_role': (179, 'viewed_prefix')}[mode]
                changed['frames'][index]['split'] = role
            with self.assertRaises(ValueError, msg=mode):
                q.qualify_metadata(changed, content.__getitem__)


if __name__ == '__main__':
    unittest.main()
